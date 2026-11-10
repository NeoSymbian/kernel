// SPDX-License-Identifier: Apache-2.0
//
// qcom_ipa.cpp -- the SDM670's IPA v3.5.1 (DTB ipa@1e40000 "qcom,sdm670-ipa"), the packet engine between the modem
// and the AP. The modem needs it running even with no data call: on LTE data an idle IPA lets the modem's rings fill
// and it asserts (telephony C5: "IPA Assert: new_free_space > 0").
// The bring-up, in Linux's order (C5b L1): power (interconnect BCMs, the RPMh IPA clock); the GSI firmware through
// TrustZone PAS 15; an identity SMMU context for IPA's streams; ipa_config (hardware registers, SRAM canaries, hash
// config, resource groups); GSI (event rings 0/1, channels 4 AP_COMMAND_TX and 5 AP_LAN_RX, endpoints 5 and 9); the
// immediate commands that init the header/processing-context/modem SRAM and the route/filter tables; LAN_RX buffers
// and the default route; then a 1 ms poll recycles LAN_RX and answers the modem's SMP2P "ipa" clock query. The
// modem's IPA QMI handshake is MODEMD's (telephony modem-proto ipa.rs). No IRQs: everything is polled.
// Facts (offsets, values, layouts) from Linux drivers/net/ipa: ipa_data-v3.5.1.c, reg/ipa_reg-v3.5.1.c,
// reg/gsi_reg-v3.5.1.c, ipa_main.c, ipa_mem.c, ipa_table.c, ipa_endpoint.c, ipa_resource.c, gsi.c, gsi_trans.c,
// ipa_cmd.c, ipa_smp2p.c; clk-rpmh.c (IP0 = rate / (unit x 1000)) and the sdm670 interconnect data.
// shortcut: no mobile data endpoints yet (AP_MODEM_TX/RX, C10/L3); LAN_RX packets are counted and dropped.
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_rpmh.h"
#include "qcom_pas.h"
#include "qcom_smem.h"
#include "qcom_smmu.h"
#include "qcom_smp2p.h"
#include "qcom_ipa.h"
#include "a64_device.h"

const TUint32 KIpa = 0x01e40000, KShared = 0x01e47000, KGsi = 0x01e04000;
const TUint32 KIpaFwPhys = 0x97a00000, KIpaFwSize = 0x10000;	// DTB ipa-fw carve-out
const TUint32 KCoreHz = 75000000;

static TBool Powered, Loaded, Ready;
static TUint8* Fw;
static TUint32 FwLen, FwMdtLen;
static TInt Step;											// how far Ipa_Start got (the monitor's "ipa")
static TUint32 LanEvents, Queries;

static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KIpa + aOff); }
static volatile TUint32& S(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KShared + aOff); }
static volatile TUint32& G(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KGsi + aOff); }

// ---- DMA memory: physically contiguous, mapped non-cacheable (IPA isn't coherent with our caches)
struct SDma { TPhysAddr iPhys; TUint8* iLin; };
static TInt DmaAlloc(SDma& aDma, TUint32 aSize, TInt aAlignLog2)
	{
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(aSize, aDma.iPhys, aAlignLog2);
	DPlatChunkHw* c = NULL;
	if (r == KErrNone)
		r = DPlatChunkHw::New(c, aDma.iPhys, aSize, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return r;
	aDma.iLin = (TUint8*)c->LinearAddress();
	memclr(aDma.iLin, aSize);
	return KErrNone;
	}

// ---- power: the interconnects first, then the core clock (Linux ipa_power_enable)
TInt Ipa_PowerOn()
	{
	if (Powered)
		return KErrNone;
	// ipa_data-v3.5.1.c bandwidths; BCMs from the sdm670 interconnect data: IPA's own on-ramp qxm_ipa -> aggre2_noc
	// (SN10) and IMEM (SN1). Not the shared ones (SN0, SH0, MC0 = DDR, CN0): the apps RSC keeps one vote per resource
	// for the whole AP (Linux sums CPU, display, IPA...), so IPA's small numbers alone would lower DDR and the memory NoC
	// under the running CPUs. C5b: a data abort right after the SH0 vote. They are already up for the CPUs.
	// shortcut: no interconnect aggregation; IPA's DDR bandwidth rides on what the CPUs keep voted.
	struct SPath { const char* iBcm; TUint32 iAvg, iPeak, iWidth; };
	static const SPath KPaths[] = { { "SN10", 80000, 600000, 8 }, { "SN1", 0, 350000, 8 } };
	TInt r = KErrNone;
	for (TUint i = 0; i < sizeof(KPaths) / sizeof(KPaths[0]) && r == KErrNone; ++i)
		r = Bcm_Vote(KPaths[i].iBcm, KPaths[i].iAvg, KPaths[i].iPeak, KPaths[i].iWidth);
	if (r != KErrNone)
		return r;
	TUint32 addr = 0, aux[2];
	r = CmdDb_Find("IP0", addr, aux);
	if (r != KErrNone || !aux[0])
		return r != KErrNone ? r : KErrNotFound;
	TUint32 y = KCoreHz / (aux[0] * 1000);
	y = y < 1 ? 1 : y > 0x3fff ? 0x3fff : y;
	r = Rpmh_Set("IP0", 0, (1u << 30) | (1u << 29) | y);		// BCM_TCS_CMD(commit, enable, 0, rate / unit)
	Kern::Printf("[ipa] core clock: IP0 at %05x unit %d kHz -> vote %d -> %d", addr, aux[0], y, r);
	if (r == KErrNone)
		{
		Powered = ETrue;
		Kern::NanoWait(100000);
		}
	return r;
	}

TInt Ipa_SetFirmware(const TUint8* aImage, TUint32 aLen, TUint32 aMdtLen)
	{
	NKern::ThreadEnterCS();
	if (Fw)
		Kern::Free(Fw);
	Fw = (TUint8*)Kern::Alloc(aLen);
	NKern::ThreadLeaveCS();
	if (!Fw)
		return KErrNoMemory;
	memcpy(Fw, aImage, aLen);
	FwLen = aLen;
	FwMdtLen = aMdtLen;
	return KErrNone;
	}

// ---- GSI (EE 0, gsi_reg-v3.5.1.c)
enum { EChCntxt0 = 0x1c000, EChQos = 0x1c05c, EChScratch0 = 0x1c060, EEvCntxt0 = 0x1d000, EEvCntxt4 = 0x1d010,
	EEvCntxt8 = 0x1d020, EChDoorbell = 0x1e000, EEvDoorbell = 0x1e100, EGsiStatus = 0x1f000, EChCmd = 0x1f008,
	EEvCmd = 0x1f010, EHwParam2 = 0x1f040, ETypeIrqMsk = 0x1f088, ESrcChIrqMsk = 0x1f098, ESrcEvIrqMsk = 0x1f09c,
	EIeobIrqMsk = 0x1f0b8, EGlobIrqEn = 0x1f108, EGsiIrqEn = 0x1f120, EIntSet = 0x1f180, EErrorLog = 0x1f200,
	EInterEeChMsk = 0xc020, EInterEeEvMsk = 0xc024 };
static TUint32 ChReg(TInt aCh, TUint32 aOff) { return aOff + 0x80 * aCh; }
static TUint32 State(TUint32 aCntxt0) { return (aCntxt0 >> 20) & 0xf; }

static TInt WaitState(TUint32 aReg, TUint32 aState)		// Linux waits 50 ms for the command's IRQ
	{
	for (TInt i = 0; i < 500; ++i)
		{
		if (State(G(aReg)) == aState)
			return KErrNone;
		Kern::NanoWait(100000);
		}
	Kern::Printf("[ipa] gsi %05x: state %d, wanted %d", aReg, State(G(aReg)), aState);
	return KErrTimedOut;
	}

// one ring of 16-byte elements, aligned to its size; iNext = the next index software writes (TREs) or reads (events)
struct SRing { SDma iMem; TUint32 iCount, iNext; };
static TUint32 RingAddr(const SRing& aRing, TUint32 aIndex) { return aRing.iMem.iPhys + 16 * (aIndex % aRing.iCount); }

static TInt EvRingSetup(TInt aEv, SRing& aRing, TUint32 aCount)
	{
	aRing.iCount = aCount;
	aRing.iNext = 0;
	TInt r = DmaAlloc(aRing.iMem, aCount * 16, aCount == 512 ? 13 : 12);
	if (r != KErrNone)
		return r;
	if (State(G(ChReg(aEv, EEvCntxt0))) != 0)
		return KErrInUse;
	G(EEvCmd) = (0u << 24) | aEv;								// ALLOCATE
	if ((r = WaitState(ChReg(aEv, EEvCntxt0), 1)) != KErrNone)
		return r;
	G(ChReg(aEv, EEvCntxt0)) = 2 | (1u << 16) | (16u << 24);	// GPI, IRQ type, element 16 bytes
	G(ChReg(aEv, EEvCntxt0 + 4)) = aCount * 16;					// length
	G(ChReg(aEv, EEvCntxt0 + 8)) = aRing.iMem.iPhys;			// ring IOVA (identity)
	G(ChReg(aEv, EEvCntxt0 + 12)) = 0;
	G(ChReg(aEv, EEvCntxt8)) = 0x00010020;						// MODT 32 (1 ms), MODC 1
	for (TUint32 o = 0x24; o <= 0x34; o += 4)					// CNTXT_9..13: no MSI
		G(ChReg(aEv, EEvCntxt0 + o)) = 0;
	G(EEvDoorbell + 8 * aEv) = RingAddr(aRing, aCount - 1);		// "processed" up to the element before 0
	return KErrNone;
	}

static TInt ChannelSetup(TInt aCh, SRing& aRing, TUint32 aCount, TBool aToIpa, TInt aEv, TUint32 aQos, TUint32 aTlv)
	{
	aRing.iCount = aCount;
	aRing.iNext = 0;
	TInt r = DmaAlloc(aRing.iMem, aCount * 16, aCount == 512 ? 13 : 12);
	if (r != KErrNone)
		return r;
	if (State(G(ChReg(aCh, EChCntxt0))) != 0)
		return KErrInUse;
	G(EChCmd) = (0u << 24) | aCh;								// ALLOCATE
	if ((r = WaitState(ChReg(aCh, EChCntxt0), 1)) != KErrNone)
		return r;
	G(ChReg(aCh, EChCntxt0)) = 2 | (aToIpa ? 1u << 3 : 0) | ((TUint32)aEv << 14) | (16u << 24);
	G(ChReg(aCh, EChCntxt0 + 4)) = aCount * 16;
	G(ChReg(aCh, EChCntxt0 + 8)) = aRing.iMem.iPhys;
	G(ChReg(aCh, EChCntxt0 + 12)) = 0;
	G(ChReg(aCh, EChQos)) = aQos;
	G(ChReg(aCh, EChScratch0)) = 0;
	G(ChReg(aCh, EChScratch0 + 4)) = 0;
	G(ChReg(aCh, EChScratch0 + 8)) = (aTlv * 16) << 16;		// max outstanding TRE bytes
	TUint32 s3 = G(ChReg(aCh, EChScratch0 + 12));
	G(ChReg(aCh, EChScratch0 + 12)) = (s3 & 0xffff) | (0x20u << 16);	// outstanding threshold 2 x 16
	return KErrNone;
	}

static TInt ChannelStart(TInt aCh)
	{
	G(EChCmd) = (1u << 24) | aCh;								// START
	return WaitState(ChReg(aCh, EChCntxt0), 2);
	}

// ---- immediate commands on AP_COMMAND_TX (channel 4, event ring 0): one TRE per command, one transaction
const TInt KCmdCh = 4, KCmdEv = 0, KLanCh = 5, KLanEv = 1;
static SRing CmdRing, CmdEvRing, LanRing, LanEvRing;
static SDma Pool, Zero, Table;
struct SCmd { TUint16 iOpcode; TUint8 iPayload[24]; };

// before a doorbell: the TREs/payloads (non-cacheable RAM) must reach memory before GSI reads them. dmb ish orders
// only for the CPUs; Linux puts a dma_wmb/dsb before each doorbell
static void Dsb() { __asm__ __volatile__("dsb sy" ::: "memory"); }

static void Put64(TUint8* aP, TUint64 aV) { memcpy(aP, &aV, 8); }
static void Put32(TUint8* aP, TUint32 aV) { memcpy(aP, &aV, 4); }
static void Put16(TUint8* aP, TUint16 aV) { memcpy(aP, &aV, 2); }

// events up to the hardware's write pointer (EV_CH_E_CNTXT_4): returns how many, acknowledged through the doorbell
static TUint32 EvDrain(TInt aEv, SRing& aRing, TUint32* aLastXfer = NULL)
	{
	TUint32 hw = (G(ChReg(aEv, EEvCntxt4)) - (TUint32)aRing.iMem.iPhys) / 16 % aRing.iCount;
	TUint32 n = 0;
	while (aRing.iNext % aRing.iCount != hw)
		{
		const TUint8* e = aRing.iMem.iLin + 16 * (aRing.iNext % aRing.iCount);
		if (aLastXfer)
			memcpy(aLastXfer, e, 4);							// xfer_ptr low 32 bits: the transaction's last TRE
		++aRing.iNext;
		++n;
		}
	if (n)
		G(EEvDoorbell + 8 * aEv) = RingAddr(aRing, aRing.iNext + aRing.iCount - 1);
	return n;
	}

// aType 3 = immediate commands; 4 = NOP TREs (GSI completes them without IPA: a check of GSI's TRE fetch)
static TInt Command(const SCmd* aCmds, TInt aCount, TUint32 aType = 3)
	{
	for (TInt i = 0; i < aCount; ++i)
		{
		TUint32 idx = CmdRing.iNext + i;
		TUint8* payload = Pool.iLin + 32 * (idx % 128);
		memcpy(payload, aCmds[i].iPayload, 24);
		TUint8* tre = CmdRing.iMem.iLin + 16 * (idx % CmdRing.iCount);
		Put64(tre, Pool.iPhys + 32 * (idx % 128));
		Put16(tre + 8, aCmds[i].iOpcode);
		Put16(tre + 10, 0);
		Put32(tre + 12, aType != 3 ? aType << 16 | 0x200 : i + 1 < aCount ? 0x00030001 : 0x00030600);	// IMMD_CMD + CHAIN, last: IMMD_CMD + IEOT + BEI
		}
	TUint32 first = CmdRing.iNext;
	TUint32 last = RingAddr(CmdRing, CmdRing.iNext + aCount - 1);
	CmdRing.iNext += aCount;
	TUint32 pre[8];												// ch4 RP/WP and ev0 RP/WP before the doorbell
	for (TInt i = 0; i < 4; ++i)
		{
		pre[i] = G(ChReg(KCmdCh, EChCntxt0) + 0x10 + 4 * i);
		pre[4 + i] = G(ChReg(KCmdEv, EEvCntxt4) + 4 * i);
		}
	TUint32 errBefore = G(EErrorLog);
	TUint32 db = RingAddr(CmdRing, CmdRing.iNext);					// the first unfilled TRE
	Dsb();
	G(EChDoorbell + 8 * KCmdCh) = db;
	for (TInt t = 0; t < 1000; ++t)
		{
		TUint32 xfer = 0;
		if (EvDrain(KCmdEv, CmdEvRing, &xfer) && xfer == last)
			return KErrNone;
		Kern::NanoWait(100000);
		}
	TUint32 e = G(EErrorLog);
	Kern::Printf("[ipa] command transaction (%d) timed out; GSI error log %08x (before %08x): type %d idx %d code %d "
		"args %x %x %x", aCount, e, errBefore, (e >> 24) & 0xf, (e >> 19) & 0x1f, (e >> 12) & 0xf, (e >> 8) & 0xf,
		(e >> 4) & 0xf, e & 0xf);
	TUint32 c[8];
	for (TInt i = 0; i < 8; ++i)
		c[i] = G(ChReg(KCmdCh, EChCntxt0) + 4 * i);
	Kern::Printf("[ipa] ch4 cntxt0-7 %08x %08x %08x %08x %08x %08x %08x %08x qos %08x", c[0], c[1], c[2], c[3], c[4],
		c[5], c[6], c[7], G(ChReg(KCmdCh, EChQos)));
	Kern::Printf("[ipa] ch4 scratch %08x %08x %08x %08x; before db: rp %08x %08x wp %08x %08x; db %08x",
		G(ChReg(KCmdCh, EChScratch0)), G(ChReg(KCmdCh, EChScratch0 + 4)), G(ChReg(KCmdCh, EChScratch0 + 8)),
		G(ChReg(KCmdCh, EChScratch0 + 12)), pre[0], pre[1], pre[2], pre[3], db);
	TUint32 v[4];
	for (TInt i = 0; i < 4; ++i)
		v[i] = G(ChReg(KCmdEv, EEvCntxt0) + 4 * i);
	Kern::Printf("[ipa] ev0 cntxt0-3 %08x %08x %08x %08x; 4-7 %08x %08x %08x %08x (before %08x %08x %08x %08x)", v[0],
		v[1], v[2], v[3], G(ChReg(KCmdEv, EEvCntxt4)), G(ChReg(KCmdEv, EEvCntxt4) + 4), G(ChReg(KCmdEv, EEvCntxt4) + 8),
		G(ChReg(KCmdEv, EEvCntxt4) + 12), pre[4], pre[5], pre[6], pre[7]);
	for (TInt i = 0; i < aCount; ++i)
		{
		TUint32 idx = (first + i) % CmdRing.iCount;
		const TUint32* t = (const TUint32*)(CmdRing.iMem.iLin + 16 * idx);
		const TUint32* p = (const TUint32*)(Pool.iLin + 32 * ((first + i) % 128));
		Kern::Printf("[ipa] tre %d @%08x: %08x %08x %08x %08x; payload %08x %08x %08x %08x %08x %08x", i,
			RingAddr(CmdRing, idx), t[0], t[1], t[2], t[3], p[0], p[1], p[2], p[3], p[4], p[5]);
		}
	TUint32 ep[17];
	for (TInt i = 0; i < 17; ++i)
		ep[i] = R(0x800 + 0x70 * 5 + 4 * i);
	Kern::Printf("[ipa] ep5 800-840 %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x", ep[0], ep[1], ep[2], ep[3],
		ep[4], ep[5], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11], ep[12], ep[13], ep[14], ep[15], ep[16]);
	for (TUint32 o = 0; o < 0x300; o += 0x40)					// IPA core, to diff with Linux's live registers
		{
		TUint32 w[16];
		for (TInt i = 0; i < 16; ++i)
			w[i] = R(o + 4 * i);
		Kern::Printf("[ipa] core %03x: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x", o, w[0], w[1], w[2], w[3],
			w[4], w[5], w[6], w[7], w[8], w[9], w[10], w[11], w[12], w[13], w[14], w[15]);
		}
	TUint32 fsr = 0, fsynr = 0;
	TUint64 far = 0;
	Smmu_Fault(fsr, far, fsynr);
	Kern::Printf("[ipa] smmu fsr %08x far %08x fsynr %08x; ch4 ring %08x", fsr, (TUint32)far, fsynr,
		(TUint32)CmdRing.iMem.iPhys);
	TUint32 gfsr = 0, gfsynr = 0;
	TUint64 gfar = 0;
	Smmu_GlobalFault(gfsr, gfar, gfsynr);
	Kern::Printf("[ipa] smmu global: gfsr %08x gfar %08x%08x gfsynr0 %08x", gfsr, (TUint32)(gfar >> 32), (TUint32)gfar,
		gfsynr);
	return KErrTimedOut;
	}

static SCmd TableInit(TUint16 aOp, TUint32 aSrc, TUint32 aSize, TUint32 aHash, TUint32 aNhash)
	{
	SCmd c = { aOp, { 0 } };
	TUint64 flags = (TUint64)aSize | (TUint64)aHash << 12 | (TUint64)aSize << 28 | (TUint64)aNhash << 40;
	Put64(c.iPayload, aSrc);
	Put64(c.iPayload + 8, flags);
	Put64(c.iPayload + 16, aSrc);
	return c;
	}

static SCmd DmaZero(TUint32 aSram, TUint32 aSize)
	{
	SCmd c = { 0x13, { 0 } };									// DMA_SHARED_MEM, write into IPA SRAM
	Put16(c.iPayload + 2, (TUint16)aSize);
	Put16(c.iPayload + 4, (TUint16)aSram);
	Put64(c.iPayload + 8, Zero.iPhys);
	return c;
	}

// ---- LAN_RX: buffers posted, recycled as their events come
const TInt KLanBufs = 32;
const TUint32 KLanBufSize = 8192;
static SDma LanBuf[KLanBufs];

static void LanPost(TInt aBuf)
	{
	TUint8* tre = LanRing.iMem.iLin + 16 * (LanRing.iNext % LanRing.iCount);
	Put64(tre, LanBuf[aBuf].iPhys);
	Put16(tre + 8, (TUint16)KLanBufSize);
	Put16(tre + 10, 0);
	Put32(tre + 12, 0x00020200);								// XFER + IEOT
	++LanRing.iNext;
	}

static void Config()
	{
	R(0x1d0) = 0x3b;											// BCR (backward compatibility)
	R(0x74) = 0x48;												// QSB max writes
	R(0x78) = 0xc0;												// QSB max reads: what Linux writes on v3.5.1
	R(0x1f0) = 0xf0;											// COUNTER_CFG: aggregation granularity 500 us
	R(0x220) = 0x100;											// IDLE_INDICATION_CFG
	static const TUint32 KCanaries[] = { 0x280, 0x284, 0x300, 0x304, 0x380, 0x384, 0x400, 0x404, 0x480, 0x484, 0x500,
		0x504, 0x580, 0x584, 0x600, 0x604, 0x680, 0x684, 0x7c8, 0x7cc, 0x1bfc };
	for (TUint i = 0; i < sizeof(KCanaries) / sizeof(KCanaries[0]); ++i)
		S(KCanaries[i]) = 0xdeadbeef;
	R(0x300c) = 0;												// IPA_IRQ_EN: then the uC's two (polled anyway)
	R(0x300c) = 0xc;
	for (TInt n = 0; n <= 14; ++n)								// filter/route hash masks off
		R(0x85c + 0x70 * n) &= ~((0x7fu) | (0x7fu << 16));
	static const TUint32 KRsrc[][2] = { { 0x400, 0x3f013f01 }, { 0x404, 0x3f010000 }, { 0x420, 0x0a0a0a0a },
		{ 0x424, 0x08080000 }, { 0x440, 0x0e0e0c0c }, { 0x444, 0x08080000 }, { 0x460, 0x3f003f00 },
		{ 0x464, 0x3f003f00 }, { 0x480, 0x14140e0e }, { 0x484, 0x0e0e0000 }, { 0x500, 0x04040404 },
		{ 0x504, 0x00000303 }, { 0x520, 0x3f013f02 }, { 0x524, 0x00000201 } };
	for (TUint i = 0; i < sizeof(KRsrc) / sizeof(KRsrc[0]); ++i)
		R(KRsrc[i][0]) = KRsrc[i][1];
	}

static void Endpoints()
	{
	const TUint32 e5 = 0x70 * 5, e9 = 0x70 * 9;
	R(0x800 + e5) &= ~2u;										// CTRL: no delay
	R(0x808 + e5) = 0;
	R(0x80c + e5) = 0;											// NAT bypass
	R(0x810 + e5) = 0;
	R(0x814 + e5) = 0;
	R(0x820 + e5) = 0x93;										// MODE: DMA to endpoint 9
	R(0x824 + e5) = 0;
	R(0x834 + e5) = 0;
	R(0x838 + e5) = 1;											// SRC_UL_DL
	R(0x83c + e5) = 0;
	R(0x840 + e5) = 0;
	R(0x800 + e9) &= ~1u;										// CTRL: no suspend
	R(0x808 + e9) = 0;
	R(0x810 + e9) = 0;
	R(0x814 + e9) = 0x800;										// HDR_EXT: pad to 4 bytes
	R(0x818 + e9) = 0;
	R(0x824 + e9) = 0x4d1;										// AGGR: generic, 6 KB / 500 us
	R(0x82c + e9) = 0;											// no HOLB drop
	R(0x838 + e9) = 1;											// DST_UL_DL_DPL
	R(0x840 + e9) = 1;											// STATUS
	}

// ---- the 1 ms poll: LAN_RX recycled, the modem's clock query answered
static TDynamicDfcQue* PollQ;
static void PollFn(TAny*);
static TDfc PollDfc(PollFn, NULL, 1);
static void PollTick(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTick, NULL);
static TUint8 LanOf[256];										// ring index -> buffer

static void PollFn(TAny*)
	{
	TUint32 before = LanRing.iNext;
	TUint32 done = LanEvRing.iNext;
	TUint32 n = EvDrain(KLanEv, LanEvRing);
	for (TUint32 i = 0; i < n; ++i)								// in order: event k completes TRE k of the ring
		{
		TInt buf = LanOf[(done + i) % LanRing.iCount];
		LanOf[LanRing.iNext % LanRing.iCount] = (TUint8)buf;
		LanPost(buf);
		}
	LanEvents += n;
	if (LanRing.iNext != before)
		{
		Dsb();
		G(EChDoorbell + 8 * KLanCh) = RingAddr(LanRing, LanRing.iNext);
		}
	TUint32 in = 0;
	if (Smp2p_In(KSmemHostModem, "ipa", in) == KErrNone && (in & 1) && !Queries++)
		{
		Smp2p_Set(KSmemHostModem, "ipa", 2, 2);					// clock enabled, then valid (Linux's order)
		Smp2p_Set(KSmemHostModem, "ipa", 1, 1);
		Kern::Printf("[ipa] the modem asked for the IPA clock: answered on");
		}
	PollTimer.OneShot(NKern::TimerTicks(1));
	}

TInt Ipa_Start()
	{
	if (Ready)
		return KErrNone;
	if (!Fw)
		return KErrNotReady;
	Step = 1;
	TInt r = Ipa_PowerOn();
	if (r != KErrNone)
		return r;
	TUint32 shared = R(0x54);									// SHARED_MEM_SIZE: offset (bits 31:16) must be 0
	if (shared >> 16)
		return KErrNotSupported;								// every SRAM offset below assumes it
	Step = 2;
	// IPA's streams 0x720 and 0x722, one exact match each as Linux (C5b: one masked match 0x720/2 wasn't used)
	if ((r = Smmu_IdentityMap(0x720, 0)) != KErrNone || (r = Smmu_IdentityMap(0x722, 0)) != KErrNone)
		return r;
	r = Smem_Alloc(KSmemHostModem, 497, 0x2000);				// SMEM_IPA_FILTER_TABLE, for the modem
	if (r != KErrNone && r != KErrAlreadyExists)
		return r;
	Step = 3;
	Config();													// before the firmware, as Linux (ipa_config, then load)
	Step = 4;
	if (!Loaded)
		{
		r = Pas_Load(KPasIpa, Fw, FwLen, FwMdtLen, KIpaFwPhys, KIpaFwSize);
		Loaded = r == KErrNone;
		if (r != KErrNone)
			return r;
		}
	TUint32 flavor = R(0x210);
	Step = 5;
	// DMA: the zero buffer, the command payloads, the route/filter table (zero rule, filter bitmap, 15 pointers)
	if ((r = DmaAlloc(Zero, 0x2000, 12)) != KErrNone || (r = DmaAlloc(Pool, 0x1000, 12)) != KErrNone
		|| (r = DmaAlloc(Table, 0x1000, 12)) != KErrNone)
		return r;
	Put64(Table.iLin + 8, 0x98);								// filtered endpoints {2,3,6}, shifted by 1
	for (TInt i = 0; i < 15; ++i)
		Put64(Table.iLin + 0x10 + 8 * i, Table.iPhys);			// every slot -> the zero rule
	Step = 6;
	if (!(G(EGsiStatus) & 1))
		return KErrNotReady;
	G(EIntSet) = 1;
	G(ETypeIrqMsk) = 0;
	G(ESrcChIrqMsk) = 0;
	G(ESrcEvIrqMsk) = 0;
	G(EGlobIrqEn) = 0;
	G(EIeobIrqMsk) = 0;
	G(EInterEeChMsk) = 0;
	G(EInterEeEvMsk) = 0;
	G(EGsiIrqEn) = 0;
	TUint32 hw2 = G(EHwParam2);
	G(EErrorLog) = 0;
	Step = 7;
	if ((r = EvRingSetup(KCmdEv, CmdEvRing, 256)) != KErrNone
		|| (r = ChannelSetup(KCmdCh, CmdRing, 512, ETrue, KCmdEv, 0x20f, 20)) != KErrNone
		|| (r = EvRingSetup(KLanEv, LanEvRing, 256)) != KErrNone
		|| (r = ChannelSetup(KLanCh, LanRing, 256, EFalse, KLanEv, 0x200, 8)) != KErrNone)
		return r;
	Step = 8;
	Endpoints();
	if ((r = ChannelStart(KCmdCh)) != KErrNone)
		return r;
	Step = 9;
	SCmd nop = { 0, { 0 } };									// C5b: does GSI fetch our TREs at all?
	r = Command(&nop, 1, 4);
	Kern::Printf("[ipa] NOP TRE -> %d", r);
	if (r != KErrNone)
		return r;
	// ipa_mem_setup: the modem header table from zeros, the processing contexts and the modem's SRAM zeroed
	SCmd mem[4];
	mem[0].iOpcode = 0x9;										// HDR_INIT_LOCAL: 0x140 bytes at 0x688
	memclr(mem[0].iPayload, 24);
	Put64(mem[0].iPayload, Zero.iPhys);
	Put32(mem[0].iPayload + 8, 0x140 | (0x688u << 12));
	mem[1] = DmaZero(0x7d0, 0x200);
	mem[2] = DmaZero(0x9d0, 0x200);
	mem[3] = DmaZero(0xbd8, 0x1024);
	if ((r = Command(mem, 4)) != KErrNone)
		return r;
	R(0x1e8) = 0x7d0;											// LOCAL_PKT_PROC_CNTXT
	Step = 10;
	// ipa_table_setup: routes (15 x the zero rule) and filters (bitmap + 3 endpoints), unused filter tails zeroed
	SCmd tbl[8];
	tbl[0] = TableInit(0x7, Table.iPhys + 0x10, 0x78, 0x488, 0x508);
	tbl[1] = TableInit(0x8, Table.iPhys + 0x10, 0x78, 0x588, 0x608);
	tbl[2] = TableInit(0x3, Table.iPhys + 0x08, 0x20, 0x288, 0x308);
	tbl[3] = DmaZero(0x328, 0x58);
	tbl[4] = DmaZero(0x2a8, 0x58);
	tbl[5] = TableInit(0x4, Table.iPhys + 0x08, 0x20, 0x388, 0x408);
	tbl[6] = DmaZero(0x428, 0x58);
	tbl[7] = DmaZero(0x3a8, 0x58);
	if ((r = Command(tbl, 8)) != KErrNone)
		return r;
	Step = 11;
	if ((r = ChannelStart(KLanCh)) != KErrNone)
		return r;
	for (TInt i = 0; i < KLanBufs && r == KErrNone; ++i)
		r = DmaAlloc(LanBuf[i], KLanBufSize, 12);
	if (r != KErrNone)
		return r;
	for (TInt i = 0; i < KLanBufs; ++i)
		{
		LanOf[LanRing.iNext % LanRing.iCount] = (TUint8)i;
		LanPost(i);
		}
	Dsb();
	G(EChDoorbell + 8 * KLanCh) = RingAddr(LanRing, LanRing.iNext);
	R(0x48) = 0x01120052;										// ROUTE: default to LAN_RX (endpoint 9)
	Step = 12;
	if (!PollQ && (r = Kern::DynamicDfcQCreate(PollQ, 25, _L8("IpaPoll"))) != KErrNone)
		return r;
	PollDfc.SetDfcQ(PollQ);
	PollTimer.OneShot(NKern::TimerTicks(1));
	Ready = ETrue;
	Kern::Printf("[ipa] running: flavor %08x, gsi hw %08x, GSI error log %08x", flavor, hw2, G(EErrorLog));
	return KErrNone;
	}

TInt Ipa_Reg(TUint32 aOffset, TUint32& aValue)
	{
	if (!Powered || aOffset >= 0x7000 || aOffset & 3)
		return KErrNotReady;
	aValue = R(aOffset);
	return KErrNone;
	}

TInt Ipa_Status(TUint32& aGsiStatus)
	{
	if (!Powered || !Loaded)
		return KErrNotReady;									// an unclocked or firmware-less GSI hangs the bus
	aGsiStatus = G(EGsiStatus);
	return KErrNone;
	}

void Ipa_Report(void (*aSay)(const char*, ...))
	{
	aSay("ipa: step %d ready %d; LAN_RX events %d, clock queries %d", Step, Ready, LanEvents, Queries);
	if (Loaded)
		aSay("ipa: GSI error log %08x, ch4 %08x ch5 %08x ev0 %08x ev1 %08x", G(EErrorLog), G(ChReg(4, EChCntxt0)),
			G(ChReg(5, EChCntxt0)), G(ChReg(0, EEvCntxt0)), G(ChReg(1, EEvCntxt0)));
	TUint32 fsr = 0, fsynr = 0;
	TUint64 far = 0;
	if (Smmu_Fault(fsr, far, fsynr) == KErrNone)
		aSay("ipa: smmu fsr %08x far %08x fsynr %08x", fsr, (TUint32)far, fsynr);
	}

static TA64DevState IpaStatus(TDes8& aDetail)
	{
	if (Ready)
		{
		A64Device_Format(aDetail, "running, no data endpoints yet; LAN_RX events %d, clock queries %d", LanEvents, Queries);
		return EA64DevPartial;
		}
	if (!Step)
		{
		A64Device_Format(aDetail, "not started (MODEMD starts it)");
		return EA64DevOff;
		}
	A64Device_Format(aDetail, "stopped at step %d of 12", Step);	// Ipa_Start runs inside one HAL call
	return EA64DevError;
	}
A64_DEVICE("ipa", "Packet engine (IPA)", "qcom_ipa", IpaStatus);
