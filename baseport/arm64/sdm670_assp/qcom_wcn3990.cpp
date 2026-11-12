// SPDX-License-Identifier: Apache-2.0
//
// qcom_wcn3990.cpp -- the WCN3990 Wi-Fi data path, bring-up stage W2 (docs wifi.md): the copy engines (CE, polled, no
// IRQs yet), HTC (ready, connect HTT + WMI, setup complete) and WMI (service ready -> init -> ready: the MAC).
// The firmware is already up: MODEMD ran the QMI WLFW handshake, WLAN cfg and mission mode (telephony wlfw.rs), then
// asks for this with HAL WlanStart (qcom_telhal.cpp 0x4e1d). Everything runs on our own DFC thread.
// Register layout, ring and message formats are facts from Linux ath10k (ISC: snoc.c, ce.c/h, hw.c, htc.c/h,
// wmi-tlv.c/h, wmi.c/h); private/wifi/w2-spec.md cites each one. This file is a new implementation.
// W3a: after READY, a first passive scan (PDEV_SET_PARAM chain masks, SCAN_CHAN_LIST, VDEV_CREATE, START_SCAN; beacons
// from MGMT_RX logged), facts in private/wifi/w3a-scan-spec.md.
// shortcut: polled for a fixed window, a few rx buffers per ring, no HTT; IRQs and the WHA PDD come with W3.
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "a64_device.h"
#include "qcom_smmu.h"
#include "qcom_smem.h"
#include "a64_dbgmon.h"
#include "qcom_wcn3990.h"

const TUint32 KWlan = 0x18800000;
static inline TUint32 CeReg(TInt aCe, TUint32 aOff) { return KWlan + 0x240000 + 0x1000 * aCe + aOff; }
static volatile TUint32& R(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }
enum	// per CE (w2-spec.md 1)
	{
	ESrBaseLo = 0x00, ESrBaseHi = 0x04, ESrSize = 0x08, EDrBaseLo = 0x0c, EDrBaseHi = 0x10, EDrSize = 0x14,
	ECtrl1 = 0x18, EHostIe = 0x2c, EHostIs = 0x30, ESrWrIdx = 0x3c, EDstWrIdx = 0x40, ESrri = 0x44, EDrri = 0x48,
	ESrcWm = 0x4c, EDstWm = 0x50
	};
const TUint32 KRriLow = KWlan + 0x24c004, KRriHigh = KWlan + 0x24c008;
static inline TUint32 Shadow(TInt aCe) { return KWlan + 0x32000 + 4 * aCe; }	// the source write index doorbell
const TUint32 KCtrl1SrcSwap = 1u << 17, KCtrl1DstSwap = 1u << 18, KCtrl1Rri = 1u << 19;

// the host's copy engines (ath10k snoc.c host_ce_config_wlan): source entries / max bytes, destination entries
struct SCeCfg { TUint16 iSrcN, iMax, iDstN; };
const SCeCfg KCe[12] = {
	{ 16, 2048, 0 },	// 0 HTC control (EP0) to the target
	{ 0, 2048, 512 },	// 1 HTT + HTC from the target
	{ 0, 2048, 64 },	// 2 WMI from the target
	{ 32, 2048, 0 },	// 3 WMI to the target
	{ 2048, 256, 0 },	// 4 HTT to the target (no interrupt)
	{ 0, 512, 512 },	// 5 HTT from the target
	{ 0, 0, 0 },		// 6 the target's own
	{ 2, 2048, 2 },		// 7 diagnostics
	{ 0, 2048, 128 },	// 8 target to uMC (posted, never reaped)
	{ 0, 2048, 512 },	// 9 HTT
	{ 0, 2048, 512 },	// 10 HTT
	{ 0, 2048, 512 },	// 11 pktlog
	};
const TInt KRxPost = 8;			// rx buffers posted per destination ring (RAM: our bank is ~87 MB)
const TInt KDesc = 16;			// struct ce_desc_64

struct SDesc { TUint32 iAddrLo, iAddrHi; TUint16 iNbytes, iFlags; TUint32 iHash; };
struct SRing { SDesc* iDesc; TPhysAddr iPhys; TUint32 iMask, iSw, iWr, iSlots; TUint8* iBuf; TPhysAddr iBufPhys; };
static SRing Src[12], Dst[12];

// The pool is mapped as device memory: only aligned 32-bit accesses (the compiler merged two descriptor fields into an
// 8-byte store at a 4-byte boundary, and that faulted). Descriptors are written word by word, buffers copied in words.
static void DescWrite(SDesc* aDesc, TUint32 aAddr, TUint16 aNbytes, TUint16 aFlags)
	{
	volatile TUint32* w = (volatile TUint32*)aDesc;
	w[0] = aAddr;
	w[1] = 0;												// address bits 36:32 (our RAM is below 4 GB), no gather
	w[2] = aNbytes | (TUint32)aFlags << 16;
	w[3] = 0;
	}
static TUint16 DescNbytes(const SDesc* aDesc) { return (TUint16)(((volatile const TUint32*)aDesc)[2] & 0xffff); }
static TUint32 DescAddr(const SDesc* aDesc) { return ((volatile const TUint32*)aDesc)[0]; }
static void DescClearNbytes(SDesc* aDesc) { volatile TUint32* w = (volatile TUint32*)aDesc; w[2] &= 0xffff0000u; }
static void ToDev(TUint8* aDst, const TUint8* aSrc, TInt aLen)	// aSrc 4-byte aligned, aLen rounded up
	{
	volatile TUint32* d = (volatile TUint32*)aDst;
	const TUint32* s = (const TUint32*)aSrc;
	for (TInt i = 0; i < (aLen + 3) / 4; ++i)
		d[i] = s[i];
	}
static void FromDev(TUint8* aDst, const TUint8* aSrc, TInt aLen)
	{
	volatile const TUint32* s = (volatile const TUint32*)aSrc;
	TUint32* d = (TUint32*)aDst;
	for (TInt i = 0; i < (aLen + 3) / 4; ++i)
		d[i] = s[i];
	}
static volatile TUint32* Rri;	// 12 words the hardware mirrors its read indices into (bits 15:0 SRRI, 31:16 DRRI)
static TUint8* Pool;
static TPhysAddr PoolPhys;
static TUint32 PoolUsed, PoolSize;

// ---- state, for the device registry and the log
static TInt Step;				// 0 idle, 1 rings, 2 HTC ready, 3 connected, 4 service ready, 5 init sent, 6 ready,
								// 7 scanning, 8 scanned
static TInt Error;
static TUint16 CreditSize, Credits;
static TUint8 HttEid = 0xff, WmiEid = 0xff, Seq[8];
static TUint32 AbiVer0, AbiVer1, RfChains, MemReqs, BoardId, PhyCap, EepromRd, Low5g, High5g;
static TUint8 Mac[6];
static TUint8 HostMac[6] = { 0x02, 0x4e, 0x53, 0x57, 0x46, 0x01 };	// our station address (PickMac)
static TUint32 Svc[32];			// the service bitmap: word id / 4, bit id % 4 (ath10k WMI_SERVICE_IS_ENABLED)
static TBool Svc174, Svc179;

static TInt Fail(TInt aErr, const char* aWhat)
	{
	Error = aErr;
	Kern::Printf("[wifi] step %d: %s failed (%d)", Step, aWhat, aErr);
	return aErr;
	}

// one uncached, physically contiguous pool for rings, rri and buffers (the WCN3990 is not coherent: no dma-coherent)
static TAny* Carve(TUint32 aSize, TPhysAddr& aPhys)
	{
	TUint32 at = (PoolUsed + 127) & ~127u;						// 128: the HTT rx buffers' alignment
	if (at + aSize > PoolSize)
		return NULL;
	PoolUsed = at + aSize;
	aPhys = PoolPhys + at;
	return Pool + at;
	}

static TUint32 Pow2(TUint32 n) { TUint32 p = 1; while (p < n) p <<= 1; return p; }

// ---- copy engines (ath10k ce.c, w2-spec.md 2)
static TInt InitRings()
	{
	TPhysAddr rriPhys;
	Rri = (volatile TUint32*)Carve(12 * 4, rriPhys);
	if (!Rri)
		return KErrNoMemory;
	R(KRriLow) = rriPhys;
	R(KRriHigh) = 0;
	for (TInt c = 0; c < 12; ++c)
		R(CeReg(c, ECtrl1)) |= KCtrl1Rri;
	for (TInt c = 0; c < 12; ++c)
		{
		const SCeCfg& cfg = KCe[c];
		if (cfg.iSrcN)
			{
			SRing& s = Src[c];
			TUint32 n = Pow2(cfg.iSrcN);
			s.iDesc = (SDesc*)Carve(n * KDesc, s.iPhys);
			// tx buffers where we send: HTC control (CE0), WMI (CE3), HTT (CE4, 64 slots: all 2048 would be 512 KB)
			TBool tx = c == 0 || c == 3 || c == 4;
			s.iSlots = Min(n, 64u);
			s.iBuf = tx ? (TUint8*)Carve(s.iSlots * cfg.iMax, s.iBufPhys) : NULL;
			if (!s.iDesc || (tx && !s.iBuf))
				return KErrNoMemory;
			s.iMask = n - 1;
			s.iSw = R(CeReg(c, ESrri)) & s.iMask;		// adopt the hardware's indices (they survive sessions)
			s.iWr = R(CeReg(c, ESrWrIdx)) & s.iMask;
			R(CeReg(c, ESrBaseLo)) = s.iPhys;
			R(CeReg(c, ESrBaseHi)) = 0;
			R(CeReg(c, ESrSize)) = n;
			R(CeReg(c, ECtrl1)) = (R(CeReg(c, ECtrl1)) & ~(0xffffu | KCtrl1SrcSwap)) | cfg.iMax;
			R(CeReg(c, ESrcWm)) = n;						// low 0, high n
			}
		if (cfg.iDstN)
			{
			SRing& d = Dst[c];
			TUint32 n = Pow2(cfg.iDstN);
			TInt post = Min((TInt)n - 1, KRxPost);
			d.iDesc = (SDesc*)Carve(n * KDesc, d.iPhys);
			d.iBuf = (TUint8*)Carve(post * cfg.iMax, d.iBufPhys);
			if (!d.iDesc || !d.iBuf)
				return KErrNoMemory;
			d.iMask = n - 1;
			d.iSw = R(CeReg(c, EDrri)) & d.iMask;
			d.iWr = R(CeReg(c, EDstWrIdx)) & d.iMask;
			R(CeReg(c, EDrBaseLo)) = d.iPhys;
			R(CeReg(c, EDrBaseHi)) &= ~0x1fu;
			R(CeReg(c, EDrSize)) = n;
			R(CeReg(c, ECtrl1)) &= ~KCtrl1DstSwap;
			R(CeReg(c, EDstWm)) = n;
			}
		R(CeReg(c, EHostIe)) &= ~0x1fu;					// polled: no copy-complete or watermark interrupts
		}
	return KErrNone;
	}

// hand rx buffer aPhys to destination ring aCe (ath10k __ath10k_ce_rx_post_buf_64)
static void PostRx(TInt aCe, TPhysAddr aPhys)
	{
	SRing& d = Dst[aCe];
	DescWrite(&d.iDesc[d.iWr], aPhys, 0, 0);
	__e32_memory_barrier();
	d.iWr = (d.iWr + 1) & d.iMask;
	R(CeReg(aCe, EDstWrIdx)) = d.iWr;
	}

static void PostAllRx()
	{
	for (TInt c = 0; c < 12; ++c)
		if (KCe[c].iDstN)
			for (TInt i = 0; i < Min((TInt)Dst[c].iMask, KRxPost); ++i)
				PostRx(c, Dst[c].iBufPhys + i * KCe[c].iMax);
	}

// send aLen bytes on source ring aCe, tagged with transfer id aEid (ath10k _ath10k_ce_send_nolock_64)
static TInt Send(TInt aCe, TUint8 aEid, const TUint8* aData, TInt aLen)
	{
	SRing& s = Src[aCe];
	if (!s.iDesc || !s.iBuf || aLen > KCe[aCe].iMax)
		return KErrArgument;
	TUint32 sw = Rri[aCe] & 0xffff & s.iMask;
	if (((s.iWr + 1) & s.iMask) == sw)
		return KErrOverflow;									// full: one slot always stays empty
	TUint32 slot = s.iWr % s.iSlots;							// shortcut: CE4 reuses 64 slots (fine while sends are rare)
	ToDev(s.iBuf + slot * KCe[aCe].iMax, aData, aLen);
	DescWrite(&s.iDesc[s.iWr], s.iBufPhys + slot * KCe[aCe].iMax, (TUint16)aLen, (TUint16)((aEid << 4) & 0xfff0));
	__e32_memory_barrier();
	s.iWr = (s.iWr + 1) & s.iMask;
	R(Shadow(aCe)) = s.iWr;
	return KErrNone;
	}

// ---- HTC (ath10k htc.c, w2-spec.md 4)
static TInt HtcSend(TUint8 aEid, const TUint8* aPayload, TInt aLen)
	{
	static TUint32 fw[512];										// the DFC thread's stack is small; word aligned
	TUint8* f = (TUint8*)fw;
	if (aLen + 8 > (TInt)sizeof(fw))
		return KErrArgument;
	TBool wmi = aEid == WmiEid;
	f[0] = aEid;
	f[1] = wmi ? 0x01 : 0;										// NEED_CREDIT_UPDATE on the credit-flow endpoint
	f[2] = (TUint8)aLen;
	f[3] = (TUint8)(aLen >> 8);
	f[4] = 0;
	f[5] = Seq[aEid & 7]++;
	f[6] = f[7] = 0;
	memcpy(f + 8, aPayload, aLen);
	TInt ce = aEid == 0 ? 0 : wmi ? 3 : 4;
	if (wmi)
		{
		if (!Credits)
			return KErrNotReady;
		--Credits;
		}
	return Send(ce, aEid, f, (aLen + 8 + 3) & ~3);
	}

static void Le16(TUint8* p, TUint16 v) { p[0] = (TUint8)v; p[1] = (TUint8)(v >> 8); }
static void Le32(TUint8* p, TUint32 v) { Le16(p, (TUint16)v); Le16(p + 2, (TUint16)(v >> 16)); }
static TUint16 G16(const TUint8* p) { return (TUint16)(p[0] | p[1] << 8); }
static TUint32 G32(const TUint8* p) { return G16(p) | (TUint32)G16(p + 2) << 16; }

static TInt Connect(TUint16 aService, TUint16 aFlags)
	{
	TUint8 m[8];
	Le16(m, 2);													// CONNECT_SERVICE
	Le16(m + 2, aService);
	Le16(m + 4, aFlags);
	m[6] = m[7] = 0;
	return HtcSend(0, m, sizeof(m));
	}

static void HtcControl(const TUint8* p, TInt n)
	{
	if (n < 2)
		return;
	TUint16 id = G16(p);
	if (id == 1 && n >= 8 && Step == 1)							// READY
		{
		TUint16 count = G16(p + 2);
		CreditSize = G16(p + 4);
		Credits = 1;											// use_fw_tx_credits = false: one credit in total
		Kern::Printf("[wifi] HTC ready: %d credits of %d bytes, %d endpoints%s", count, CreditSize, p[6],
			n >= 12 ? (p[8] ? ", HTC 2.1" : ", HTC 2.0") : "");
		if (!count || !CreditSize)
			{
			Fail(KErrCorrupt, "HTC ready");
			return;
			}
		Step = 2;
		}
	else if (id == 3 && n >= 8)									// CONNECT_SERVICE_RESP
		{
		TUint16 svc = G16(p + 2);
		Kern::Printf("[wifi] HTC service %04x -> status %d, endpoint %d, max %d", svc, p[4], p[5], G16(p + 6));
		if (p[4])
			Fail(KErrNotReady, "HTC connect");
		else if (svc == 0x0300)
			HttEid = p[5];
		else if (svc == 0x0100)
			WmiEid = p[5];
		}
	}

// ---- WMI (TLV ABI, ath10k wmi-tlv.c, w2-spec.md 5)
const TUint32 KAbi[6] = { 0x01000000, 53, 0x5f414351, 0x00004c4d, 0, 0 };
struct SChunk { TUint32 iReq, iPhys, iSize; };
static SChunk Chunks[16];
static TInt NumChunks;

// the next top-level TLV of aP[0..aN) from aAt: its tag, value and length; EFalse at the end
static TBool NextTlv(const TUint8* aP, TInt aN, TInt& aAt, TInt& aTag, const TUint8*& aV, TInt& aLen)
	{
	if (aAt + 4 > aN)
		return EFalse;
	TUint32 h = G32(aP + aAt);
	aLen = h & 0xffff;
	aTag = h >> 16;
	if (aAt + 4 + aLen > aN)
		return EFalse;
	aV = aP + aAt + 4;
	aAt += 4 + aLen;
	return ETrue;
	}

// a host memory request (tag 34: req_id, unit_size, num_unit_info, num_units): one chunk of the pool
static void MemReq(const TUint8* r)
	{
	if (NumChunks >= 16)
		return;
	TUint32 req = G32(r), unit = G32(r + 4), info = G32(r + 8), units = G32(r + 12);
	if (info & 1)
		units = 4 + 1;											// vdevs
	else if (info & 6)
		units = 33 + 1;											// peers
	TUint32 size = units * ((unit + 3) & ~3u);
	TPhysAddr phys;
	if (!size || !Carve(size, phys))
		{
		Kern::Printf("[wifi] host memory request %d: %d x %d bytes not allocated", req, units, unit);
		return;
		}
	Chunks[NumChunks].iReq = req;
	Chunks[NumChunks].iPhys = phys;
	Chunks[NumChunks].iSize = size;
	++NumChunks;
	}

static void ServiceReady(const TUint8* p, TInt n)
	{
	TBool first = ETrue;
	TInt at = 0, tag, len;
	const TUint8* v;
	while (NextTlv(p, n, at, tag, v, len))
		{
		if (tag == 32 && len >= 84)								// SERVICE_READY_EVENT
			{
			AbiVer0 = G32(v + 4);
			AbiVer1 = G32(v + 8);
			PhyCap = G32(v + 28);
			RfChains = G32(v + 36);
			MemReqs = G32(v + 72);
			BoardId = G32(v + 80);
			}
		else if (tag == 33 && len >= 36)						// HAL_REG_CAPABILITIES
			{
			EepromRd = G32(v);
			Low5g = G32(v + 28);
			High5g = G32(v + 32);
			}
		else if (tag == 16 && first)							// the service bitmap (ARRAY_UINT32)
			{
			first = EFalse;
			for (TInt i = 0; i < 32 && i * 4 + 4 <= len; ++i)
				Svc[i] = G32(v + 4 * i);
			}
		else if (tag == 18)										// mem_reqs: nested tag-34 TLVs
			{
			TInt in = 0, t, l;
			const TUint8* r;
			while (NextTlv(v, len, in, t, r, l))
				if (t == 34 && l >= 16)
					MemReq(r);
			}
		}
	Kern::Printf("[wifi] WMI service ready: ABI %d.%d.%d.%d, %d RF chains, %d memory requests (%d allocated), board %x",
		AbiVer0 >> 24, AbiVer0 & 0xffffff, AbiVer1 >> 16, AbiVer1 & 0xffff, RfChains, MemReqs, NumChunks, BoardId);
	Kern::Printf("[wifi] reg caps: eeprom_rd %x, phy caps %x (2.4 GHz %d, 5 GHz %d), 5 GHz channels %d-%d", EepromRd,
		PhyCap, (PhyCap >> 1) & 1, PhyCap & 1, Low5g, High5g);
	Step = 4;
	}

static TBool Svc65() { return (Svc[65 / 4] >> (65 % 4)) & 1; }	// RX_FULL_REORDER
static TBool Svc76() { return (Svc[76 / 4] >> (76 % 4)) & 1; }	// MGMT_TX_WMI (w4-join-spec.md 0)

static TInt SendInit()
	{
	TUint8 m[4 + 32 + 180 + 4 + 16 * 20];
	TInt at = 0;
	Le32(m, 1);													// WMI header: INIT_CMDID
	at = 4;
	Le32(m + at, 74 << 16 | 28);								// TLV INIT_CMD: abi + chunk count
	for (TInt i = 0; i < 6; ++i)
		Le32(m + at + 4 + 4 * i, KAbi[i]);
	Le32(m + at + 28, NumChunks);
	at += 32;
	TBool reorder = Svc65();
	const TUint32 cfg[44] = {
		4, 33, reorder ? 4u : 0u, reorder ? 4u : 0u, 2, 66, 16, 7, 7, 100, 100, 100, 40, 1, 4, 4, 4, 8, 0, 0, 0,
		0x400, 2, 0, 0, 0, 0, 2, 1056, 2, 1, 0x20, 2, 5, 22, 6, 0, 1, 1, 0, 0, 0, 0,
		(1u << 9) | (Svc174 ? 1u << 18 : 0u) };
	Le32(m + at, 75 << 16 | 176);								// TLV RESOURCE_CONFIG
	for (TInt i = 0; i < 44; ++i)
		Le32(m + at + 4 + 4 * i, cfg[i]);
	at += 180;
	Le32(m + at, 18 << 16 | (NumChunks * 20));					// ARRAY_STRUCT of WLAN_HOST_MEMORY_CHUNK
	at += 4;
	for (TInt i = 0; i < NumChunks; ++i, at += 20)
		{
		Le32(m + at, 76 << 16 | 16);
		Le32(m + at + 4, Chunks[i].iReq);
		Le32(m + at + 8, Chunks[i].iPhys);
		Le32(m + at + 12, Chunks[i].iSize);
		Le32(m + at + 16, 0);
		}
	if (at + 8 > CreditSize)
		return Fail(KErrOverflow, "WMI init bigger than one credit");
	return HtcSend(WmiEid, m, at);
	}

// ---- W3a: the first scan (private/wifi/w3a-scan-spec.md)
// iIes: the beacon's elements after timestamp, interval and capability (W4 joins with its rates and RSN element)
const TInt KIes = 384;
struct SBss { TUint8 iBssid[6]; TUint8 iSsidLen; TUint8 iSsid[32]; TUint8 iChannel; TInt8 iDbm; TUint8 iSecured;
	TUint16 iCap, iBcnInt, iIeLen; TUint8 iIes[KIes]; };
static SBss Bss[32];
static TInt NumBss, Beacons, ChanInfos, OtherFrames, HttFrames, HttMgmt;
static TBool ScanDone;

static void ScanEvent(const TUint8* p, TInt n)
	{
	TInt at = 0, tag, len;
	const TUint8* v;
	while (NextTlv(p, n, at, tag, v, len))
		if (tag == 36 && len >= 24)
			{
			TUint32 type = G32(v), reason = G32(v + 4);
			if (type & 0x1)
				Kern::Printf("[wifi] scan started (tick %d)", NKern::TickCount());
			if (type & 0x40)
				Kern::Printf("[wifi] scan start failed (reason %d)", reason);
			if (type & 0x42)
				ScanDone = ETrue;
			if (type & 0x2)
				Kern::Printf("[wifi] scan completed (reason %d, tick %d): %d networks from %d beacons; %d channel reports, "
					"HTT %d frames (%d mgmt), %d other HTT messages", reason, NKern::TickCount(), NumBss, Beacons, ChanInfos,
					HttFrames, HttMgmt, OtherFrames);
			}
	}

// the element aId in aIes[0..aLen) (id, length, body), or NULL
static const TUint8* FindIe(const TUint8* aIes, TInt aLen, TUint8 aId)
	{
	for (TInt ie = 0; ie + 2 <= aLen && ie + 2 + aIes[ie + 1] <= aLen; ie += 2 + aIes[ie + 1])
		if (aIes[ie] == aId)
			return aIes + ie;
	return NULL;
	}

// ---- W4: the link (docs wifi.md W4, private/wifi/w4-join-spec.md). The user side (neoui's supplicant thread) asks
// through HAL WlanLink 0x4e1f; this thread does the 802.11 join and the WMI/HTT work. The WPA2 handshake itself is
// the user side's: it gets the AP's EAPOL frames from Eapol[], sends its own through a request, and installs keys.
enum TLink { ELinkIdle, ELinkJoining, ELinkHandshake, ELinkConnected, ELinkLost, ELinkError };
static NFastMutex LinkLock;										// guards what the HAL reads: the state, Eapol[], Req
static TInt Link;
static TUint16 LinkAid, LinkStatus, LinkReason;
static SBss Target;												// the network being joined (a copy of its scan entry)
static TUint8 OurRsn[64];										// the user side's RSN element, for the Association Request
static TInt OurRsnLen;
static TUint32 RxData, RxIp;									// data frames received, and how many were ARP/IPv4/IPv6
const TInt KEapolSlots = 4, KEapolMax = 512;
static TUint8 Eapol[KEapolSlots][KEapolMax];
static TUint16 EapolLen[KEapolSlots];
static TInt EapolHead, EapolCount;
// set by the rx handlers (this thread), read by Join/Teardown
static TBool StartRespGot, VdevStoppedGot, PeerMapped, PeerUnmapped, AuthDone, AssocDone, LostWanted, AbortWanted;
static TInt StartStatus;
static TUint16 AuthStatus, AssocStatus, AssocAid, AssocCap, LostReason, PeerId;
static TUint32 KeyAcks;
// pending tx frames (management and data), each in its own DMA slot until the chip reports it sent
const TInt KTxSlots = 8, KTxSlot = 512;
static TUint8* TxBuf;
static TPhysAddr TxBufPhys;
static TBool TxBusy[KTxSlots], TxAuto[KTxSlots];
static TInt TxStat[KTxSlots];									// -1 pending, else the chip's status (0 = acked)

static void TxDone(TUint32 aId, TInt aStatus)
	{
	if (aId >= (TUint32)KTxSlots || !TxBusy[aId])
		return;
	TxStat[aId] = aStatus;
	if (TxAuto[aId])
		TxBusy[aId] = EFalse;									// a management frame: nobody waits for it
	}

static void SetLink(TInt aState, TUint16 aStatus, TUint16 aReason)
	{
	NKern::FMWait(&LinkLock);
	Link = aState;
	LinkStatus = aStatus;
	LinkReason = aReason;
	if (aState == ELinkIdle || aState == ELinkJoining)
		EapolCount = 0;
	NKern::FMSignal(&LinkLock);
	static const char* const KNames[] = { "idle", "joining", "handshake", "connected", "lost", "error" };
	Kern::Printf("[wifi] link %s (status %d, reason %d)", KNames[aState], aStatus, aReason);
	}

// an EAPOL frame from the AP (802.1X header onward) for the supplicant
static void EapolIn(const TUint8* aFrame, TInt aLen)
	{
	if (aLen < 4 || aLen > KEapolMax)
		return;
	NKern::FMWait(&LinkLock);
	if (EapolCount < KEapolSlots)
		{
		TInt slot = (EapolHead + EapolCount++) % KEapolSlots;
		memcpy(Eapol[slot], aFrame, aLen);
		EapolLen[slot] = (TUint16)aLen;
		}
	NKern::FMSignal(&LinkLock);
	Kern::Printf("[wifi] EAPOL in, %d bytes", aLen);
	}

// one beacon or probe response: aHdr = its 24-byte 802.11 header, aBody = timestamp, interval, capability, IEs
static void AddBss(const TUint8* aHdr, const TUint8* aBody, TInt aBodyLen, TInt aChannel, TInt aDbm, const char* aVia)
	{
	if ((aHdr[0] != 0x80 && aHdr[0] != 0x50) || aBodyLen < 12)	// beacons and probe responses only
		return;
	++Beacons;
	const TUint8* bssid = aHdr + 16;
	for (TInt i = 0; i < NumBss; ++i)
		if (!memcompare(Bss[i].iBssid, 6, bssid, 6))
			return;												// seen already
	if (NumBss >= 32)
		return;
	SBss& b = Bss[NumBss++];
	memcpy(b.iBssid, bssid, 6);
	b.iSsidLen = 0;
	// the AP's channel comes from its beacon: the firmware's header gives the channel it was tuned to, which can be a
	// neighbour (a 5 GHz network showed on 2.4 GHz)
	TInt own = 0;
	TInt tuned = aChannel;
	aChannel = 0;
	for (TInt ie = 12; ie + 2 <= aBodyLen && ie + 2 + aBody[ie + 1] <= aBodyLen; ie += 2 + aBody[ie + 1])
		if (aBody[ie] == 0 && aBody[ie + 1] <= 32 && !b.iSsidLen)
			{
			b.iSsidLen = aBody[ie + 1];
			memcpy(b.iSsid, aBody + ie + 2, b.iSsidLen);
			}
		else if (aBody[ie] == 3 && aBody[ie + 1] == 1)
			aChannel = aBody[ie + 2];							// DS parameter set: the AP's own channel (2.4 GHz)
		else if (aBody[ie] == 61 && aBody[ie + 1] >= 1 && !own)
			own = aBody[ie + 2];								// HT operation: the primary channel (5 GHz has no DS IE)
	if (!aChannel)
		aChannel = own ? own : tuned;
	b.iChannel = (TUint8)aChannel;
	b.iDbm = (TInt8)aDbm;
	b.iSecured = (aBody[10] & 0x10) ? 1 : 0;
	b.iCap = G16(aBody + 10);
	b.iBcnInt = G16(aBody + 8);
	b.iIeLen = (TUint16)Min(aBodyLen - 12, KIes);					// shortcut: a longer beacon's tail is cut
	memcpy(b.iIes, aBody + 12, b.iIeLen);
	TBuf8<33> ssid;
	for (TInt i = 0; i < b.iSsidLen; ++i)
		ssid.Append(b.iSsid[i] >= 0x20 && b.iSsid[i] < 0x7f ? b.iSsid[i] : '?');
	if (!b.iSsidLen)
		ssid.Copy((const TUint8*)"(hidden)", 8);
	Kern::Printf("[wifi] scan: %S  %02x:%02x:%02x:%02x:%02x:%02x  ch %d  %d dBm%s (%s)", &ssid, bssid[0], bssid[1],
		bssid[2], bssid[3], bssid[4], bssid[5], aChannel, aDbm, aBody[10] & 0x10 ? "  secured" : "", aVia);
	}

static void MgmtRx(const TUint8* p, TInt n)
	{
	TInt at = 0, tag, len;
	const TUint8* v;
	const TUint8* hdr = NULL;
	const TUint8* frame = NULL;
	TInt flen = 0;
	while (NextTlv(p, n, at, tag, v, len))
		{
		if (tag == 44 && len >= 40)
			hdr = v;
		else if (tag == 17)
			frame = v, flen = len;
		}
	if (!hdr || !frame)
		return;
	TUint32 channel = G32(hdr), snr = G32(hdr + 4), buflen = G32(hdr + 16), status = G32(hdr + 20);
	if (status & 0x29 || (TInt)buflen > flen || buflen < 24)
		return;
	if (Link >= ELinkJoining && Link <= ELinkConnected && !memcompare(frame + 10, 6, Target.iBssid, 6) &&
		(!memcompare(frame + 4, 6, HostMac, 6) || frame[4] == 0xff))
		{
		const TUint8* body = frame + 24;						// W4: the AP's answers to our join, and its goodbyes
		TInt blen = buflen - 24;
		if (frame[0] == 0xb0 && blen >= 6 && G16(body + 2) == 2)	// Authentication, transaction 2
			AuthStatus = G16(body + 4), AuthDone = ETrue;
		else if ((frame[0] == 0x10 || frame[0] == 0x30) && blen >= 6)	// (Re)Association Response
			AssocCap = G16(body), AssocStatus = G16(body + 2), AssocAid = G16(body + 4) & 0x7ff, AssocDone = ETrue;
		else if ((frame[0] == 0xc0 || frame[0] == 0xa0) && blen >= 2)	// Deauthentication, Disassociation
			LostReason = G16(body), LostWanted = ETrue;
		if (frame[0] != 0x80 && frame[0] != 0x50)
			Kern::Printf("[wifi] mgmt %02x from the AP, %d bytes", frame[0], buflen);
		}
	if (buflen < 36)
		return;
	AddBss(frame, frame + 24, buflen - 24, channel, (TInt)snr - 95, "WMI");
	}

// ---- W3b: the HTT rx ring (private/wifi/w3b-htt-rx-spec.md). The firmware's MAC rx ring is fed from these buffers;
// without them it receives (CHAN_INFO shows it) but has nowhere to put a frame, beacons included.
const TInt KHttRing = 2048, KHttBufs = 128, KHttBuf = 2048;
static volatile TUint32* HttRing;		// KHttRing x le64 buffer addresses
static volatile TUint32* HttIdx;		// alloc_idx: the firmware reads it
static TPhysAddr HttRingPhys, HttIdxPhys, HttBufPhys, BankPhys;
static TUint8* Bank;											// the frag-desc bank: one 72-byte tx descriptor per msdu id
static TUint8* HttBufs;

static void HttPost(TPhysAddr aPhys)
	{
	TUint32 i = *HttIdx & (KHttRing - 1);
	((volatile TUint32*)(HttBufs + (aPhys - HttBufPhys)))[1] = 0;	// the attention word: not done
	HttRing[2 * i] = aPhys;
	HttRing[2 * i + 1] = 0;
	__e32_memory_barrier();
	*HttIdx = (i + 1) & (KHttRing - 1);
	}

static TInt HttSetup()
	{
	if (!HttRing)
		{
		Bank = (TUint8*)Carve(1056 * 72, BankPhys);				// tx MSDU extension descriptors (data tx, W4)
		HttRing = (volatile TUint32*)Carve(KHttRing * 8, HttRingPhys);
		HttIdx = (volatile TUint32*)Carve(4, HttIdxPhys);
		HttBufs = (TUint8*)Carve(KHttBufs * KHttBuf, HttBufPhys);
		TxBuf = (TUint8*)Carve(KTxSlots * KTxSlot, TxBufPhys);	// W4: frames the chip reads by DMA until it sent them
		if (!Bank || !HttRing || !HttIdx || !HttBufs || !TxBuf)
			return Fail(KErrNoMemory, "HTT rx memory");
		*HttIdx = 0;
		for (TInt i = 0; i < KHttBufs; ++i)
			HttPost(HttBufPhys + i * KHttBuf);
		}
	static TUint32 mw[16];
	TUint8* m = (TUint8*)mw;
	memclr(m, 64);												// FRAG_DESC_BANK_CFG (64-bit)
	m[0] = 6;
	m[2] = 1;													// num_banks
	m[3] = 72;													// desc_size
	Le32(m + 4, BankPhys);										// bank_base_addrs[0], le64
	Le16(m + 38, 1055);											// bank_id[0] max
	m[60] = 1;													// record_size
	TInt r = HtcSend(HttEid, m, 64);
	memclr(m, 48);												// RX_RING_CFG (64-bit), one ring
	m[0] = 2;
	m[1] = 1;
	Le32(m + 4, HttIdxPhys);									// fw_idx_shadow_reg_paddr, le64
	Le32(m + 12, HttRingPhys);									// rx_ring_base_paddr, le64
	Le16(m + 20, KHttRing);
	Le16(m + 22, KHttBuf);
	Le16(m + 24, 0xffff);										// every part of the descriptor, mgmt frames included
	Le16(m + 26, (TUint16)*HttIdx);								// fw_idx_init_val: the buffers posted
	static const TUint16 KOffsets[10] = { 74, 90, 27, 37, 4, 26, 7, 12, 1, 2 };	// wcn3990 htt_rx_desc_v2, in words
	for (TInt i = 0; i < 10; ++i)
		Le16(m + 28 + 2 * i, KOffsets[i]);
	if (r == KErrNone)
		r = HtcSend(HttEid, m, 48);
	const TUint8 aggr[4] = { 5, 64, 3, 0 };						// AGGR_CFG v2: 64 A-MPDU, 3 A-MSDU
	if (r == KErrNone)
		r = HtcSend(HttEid, aggr, 4);
	Kern::Printf("[wifi] HTT rx ring: %d entries, %d buffers posted at %08x -> %d", KHttRing, KHttBufs, HttBufPhys, r);
	return r == KErrNone ? r : Fail(r, "HTT setup");
	}

// a native-wifi data frame from the AP: 24-byte header (no QoS), LLC/SNAP, payload (w4-join-spec.md 4). EAPOL goes to the
// supplicant; the rest is only counted until there is a network interface (W5)
static void DataIn(const TUint8* f, TInt aLen)
	{
	static const TUint8 KSnap[6] = { 0xaa, 0xaa, 0x03, 0, 0, 0 };
	if (aLen < 32 || Link < ELinkHandshake || Link > ELinkConnected || (f[0] & 0x0c) != 0x08 ||
		memcompare(f + 10, 6, Target.iBssid, 6) || memcompare(f + 24, 6, KSnap, 6))
		return;
	TUint16 type = (TUint16)(f[30] << 8 | f[31]);
	++RxData;
	if (type == 0x888e)
		EapolIn(f + 32, aLen - 32);
	else if (type == 0x0806 || type == 0x0800 || type == 0x86dd)
		++RxIp;
	}

// one HTT message from the target (CE1/9/10 by HTC endpoint, CE5 with its 8-byte header stripped)
static void HttRx(const TUint8* p, TInt n)
	{
	if (n < 4)
		return;
	if (p[0] == 0x03 && n >= 10)								// PEER_MAP {type, vdev, peer_id, addr} (W4)
		{
		if (p[1] == 0 && !memcompare(p + 4, 6, Target.iBssid, 6))
			PeerMapped = ETrue, PeerId = G16(p + 2);
		return;
		}
	if (p[0] == 0x04)											// PEER_UNMAP
		{
		PeerUnmapped = ETrue;
		return;
		}
	if (p[0] == 0x07)											// TX_COMPL_IND: one status for the listed msdu ids
		{
		for (TInt i = 0; i < p[2] && 4 + 2 * i + 2 <= n; ++i)
			TxDone(G16(p + 4 + 2 * i), p[1] & 7);
		if (Link == ELinkHandshake)								// W4 diagnosis: msg4's report
			Kern::Printf("[wifi] tx done: %d msdus from %d, status %d", p[2], n >= 6 ? G16(p + 4) : -1, p[1] & 7);
		return;
		}
	if (p[0] == 0x0b)											// SEC_IND: a key was installed
		{
		++KeyAcks;
		return;
		}
	if (p[0] != 0x12)											// RX_IN_ORD_PADDR_IND is what full-reorder firmware sends
		{
		if (++OtherFrames <= 5)
			Kern::Printf("[wifi] HTT message %x, %d bytes", p[0], n);
		return;
		}
	TBool offload = p[1] & 0x20;
	TInt count = G16(p + 6);
	for (TInt i = 0; i < count && 8 + 12 * i + 12 <= n; ++i)
		{
		TPhysAddr phys = G32(p + 8 + 12 * i);
		TInt len = G16(p + 16 + 12 * i);
		if (phys < HttBufPhys || phys >= HttBufPhys + KHttBufs * KHttBuf || (phys - HttBufPhys) % KHttBuf)
			continue;											// not one of ours
		++HttFrames;
		static TUint32 bw[KHttBuf / 4];
		TUint8* b = (TUint8*)bw;
		FromDev(b, HttBufs + (phys - HttBufPhys), KHttBuf);
		TUint32 attention = G32(b + 4), decap = (G32(b + 36) >> 8) & 3;
		if (offload && G16(b) + 7 <= KHttBuf)					// {msdu_len, peer_id, vdev, tid, fw_desc} + the frame
			DataIn(b + 7, G16(b));
		else if (!offload && (attention >> 31) && !(attention & 0x100) && decap == 1 && len <= KHttBuf - 360)
			DataIn(b + 360, len);								// a native-wifi data frame (W4)
		if (!offload && (attention >> 31) && (attention & 0x100) && len >= 24 && len <= KHttBuf - 360)
			{
			++HttMgmt;
			TInt dbm = (TInt)b[124] - 95;						// ppdu_start rssi_comb
			if (decap == 0)										// RAW: the whole MPDU, 4-byte FCS at the end
				AddBss(b + 360, b + 384, len - 28, 0, dbm, "HTT");
			else if (decap == 1)								// native wifi: the real header is in rx_hdr_status
				AddBss(b + 296, b + 360 + 24, len - 24, 0, dbm, "HTT");
			}
		HttPost(phys);
		}
	}

static void WmiEvent(const TUint8* p, TInt n)
	{
	if (n < 4)
		return;
	TUint32 id = G32(p) & 0xffffff;
	p += 4;
	n -= 4;
	TInt at = 0, tag, len;
	const TUint8* v;
	if (id == 1)
		ServiceReady(p, n);
	else if (id == 3)											// SERVICE_AVAILABLE: ext services (id - 128)
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 559 && len >= 12)
				{
				Svc174 = (G32(v + 4 + 4 * ((174 - 128) / 32)) >> ((174 - 128) % 32)) & 1;
				Svc179 = (G32(v + 4 + 4 * ((179 - 128) / 32)) >> ((179 - 128) % 32)) & 1;
				}
		}
	else if (id == 2)											// READY
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 35 && len >= 36)
				{
				memcpy(Mac, v + 24, 6);
				Kern::Printf("[wifi] WMI ready: MAC %02x:%02x:%02x:%02x:%02x:%02x, status %d (W2 done)", Mac[0], Mac[1],
					Mac[2], Mac[3], Mac[4], Mac[5], G32(v + 32));
				Step = 6;
				}
		}
	else if (id == 0x3001)										// SCAN_EVENT
		ScanEvent(p, n);
	else if (id == 0x7001)										// MGMT_RX: beacons and probe responses
		MgmtRx(p, n);
	else if (id == 0x5001)										// VDEV_START_RESP {vdev, req, type, status} (W4)
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 40 && len >= 16)
				StartStatus = (TInt)G32(v + 12), StartRespGot = ETrue;
		}
	else if (id == 0x5002)										// VDEV_STOPPED
		VdevStoppedGot = ETrue;
	else if (id == 0x6001)										// PEER_STA_KICKOUT {mac}: the AP stopped acking
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 43 && len >= 6 && !memcompare(v, 6, Target.iBssid, 6))
				LostReason = 34, LostWanted = ETrue;			// 802.11 reason 34: frames not acknowledged
		}
	else if (id == 0xb001)										// ROAM {vdev, reason, rssi}: 2 = beacon miss
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 48 && len >= 12 && G32(v + 4) == 2 && Link >= ELinkHandshake)
				LostReason = 4, LostWanted = ETrue;				// 802.11 reason 4 (inactivity) stands in for it
		}
	else if (id == 0x7006)										// MGMT_TX_COMPLETION {desc_id, status, ...}
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 423 && len >= 8)
				TxDone(G32(v), (TInt)G32(v + 4));
		}
	else if (id == 0x7007)										// MGMT_TX_BUNDLE_COMPLETION: arrays of ids, statuses
		{
		const TUint8* ids = NULL;
		TInt idsLen = 0;
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 16 && !ids)
				ids = v, idsLen = len;
			else if (tag == 16)
				for (TInt i = 0; 4 * i + 4 <= Min(len, idsLen); ++i)
					TxDone(G32(ids + 4 * i), (TInt)G32(v + 4 * i));
		}
	else if (id == 0x4002)										// CHAN_INFO: one per channel, counted
		{
		TInt at = 0, tag, len;
		const TUint8* v;
		while (++ChanInfos <= 6 && NextTlv(p, n, at, tag, v, len))
			if (tag == 38 && len >= 36)							// err, freq, flags, noise floor, rx clear, cycles, rx frames
				Kern::Printf("[wifi] channel %d MHz: err %d flags %x noise %d dBm, rx clear %d / %d cycles, %d frames",
					G32(v + 4), G32(v), G32(v + 8), (TInt)G32(v + 12), G32(v + 16), G32(v + 20), G32(v + 32));
		}
	else
		Kern::Printf("[wifi] WMI event %x, %d bytes", id, n);
	}

// one HTC frame from a destination ring: route by endpoint, take the credits from its trailer
static void HtcRx(const TUint8* f, TInt n)
	{
	if (n < 8)
		return;
	TUint8 eid = f[0], flags = f[1];
	TInt len = Min((TInt)G16(f + 2), n - 8);
	const TUint8* p = f + 8;
	if (flags & 0x02)											// TRAILER_PRESENT: records at the end
		{
		TInt tl = Min((TInt)f[4], len);
		const TUint8* t = p + len - tl;
		for (TInt at = 0; at + 4 <= tl; )						// records: {id, len, pad, pad} + len bytes
			{
			TInt id = t[at], rl = t[at + 1];
			if (at + 4 + rl > tl)
				break;
			if (id == 1)										// CREDITS: {eid, credits, pad, pad} each
				for (TInt i = 0; i + 4 <= rl; i += 4)
					if (t[at + 4 + i] == WmiEid)
						Credits += t[at + 4 + i + 1];
			at += 4 + rl;
			}
		len -= tl;
		}
	if (!len)
		return;
	if (eid == 0)
		HtcControl(p, len);
	else if (eid == WmiEid)
		WmiEvent(p, len);
	else if (eid == HttEid)
		HttRx(p, len);
	}

// reap every destination ring once: frames whose length the hardware filled in
static void Poll()
	{
	for (TInt c = 0; c < 12; ++c)
		{
		SRing& d = Dst[c];
		if (!d.iDesc || c == 8)									// CE8's buffers are the target's (never reaped)
			continue;
		for (TInt guard = 0; guard < 64; ++guard)
			{
			SDesc* e = &d.iDesc[d.iSw];
			TUint16 nbytes = DescNbytes(e);
			if (!nbytes)
				break;
			TPhysAddr phys = DescAddr(e);
			DescClearNbytes(e);
			d.iSw = (d.iSw + 1) & d.iMask;
			static TUint32 rx[512];								// a cached copy to parse (byte access is fine there)
			if (nbytes <= KCe[c].iMax && nbytes <= (TInt)sizeof(rx) && phys >= d.iBufPhys)
				{
				FromDev((TUint8*)rx, d.iBuf + (phys - d.iBufPhys), nbytes);
				if (c == 5)										// HTT rx on CE5 skips HTC parsing (ath10k snoc.c)
					HttRx((const TUint8*)rx + 8, nbytes - 8);
				else
					HtcRx((const TUint8*)rx, nbytes);
				}
			PostRx(c, phys);									// the same buffer goes back
			}
		}
	}

static TBool WaitFor(TInt aStep, TInt aMs)
	{
	for (TInt t = 0; t < aMs && Step < aStep && !Error; ++t)
		{
		Poll();
		NKern::Sleep(NKern::TimerTicks(1));
		}
	return Step >= aStep;
	}

// one WMI command: wait for the single credit (a CREDITS trailer returns it), then send
static TUint32 CmdBuf[400];
static TUint8* const Cmd = (TUint8*)CmdBuf;
static TInt CmdLen;
static void CmdStart(TUint32 aId) { Le32(Cmd, aId); CmdLen = 4; }
static void CmdTlv(TInt aTag, TInt aLen) { Le32(Cmd + CmdLen, (TUint32)aTag << 16 | aLen); CmdLen += 4; }
static void Cmd32(TUint32 aValue) { Le32(Cmd + CmdLen, aValue); CmdLen += 4; }
static TInt CmdSend(const char* aWhat)
	{
	for (TInt t = 0; t < 1000 && !Credits && !Error; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if (!Credits)
		return Fail(KErrTimedOut, aWhat);
	TInt r = HtcSend(WmiEid, Cmd, CmdLen);
	return r == KErrNone ? r : Fail(r, aWhat);
	}

// channels: 2.4 GHz 1-11 active, 12-13 passive; 5 GHz UNII-1 and UNII-3 passive (no DFS yet). Word 3 = mode | flags
// (11G = 1, 11A = 0; ALLOW_HT 0x800, ALLOW_VHT 0x1000, ADHOC 0x100, PASSIVE 0x80); powers in 0.5 dBm (20 / 30 dBm)
struct SChan { TUint16 iMhz; TUint16 iWord; };
static const SChan KChans[] = {
	{ 2412, 0x1901 }, { 2417, 0x1901 }, { 2422, 0x1901 }, { 2427, 0x1901 }, { 2432, 0x1901 }, { 2437, 0x1901 },
	{ 2442, 0x1901 }, { 2447, 0x1901 }, { 2452, 0x1901 }, { 2457, 0x1901 }, { 2462, 0x1901 }, { 2467, 0x1881 },
	{ 2472, 0x1881 },
	{ 5180, 0x1880 }, { 5200, 0x1880 }, { 5220, 0x1880 }, { 5240, 0x1880 },
	{ 5745, 0x1880 }, { 5765, 0x1880 }, { 5785, 0x1880 }, { 5805, 0x1880 }, { 5825, 0x1880 },
	};
const TInt KNumChans = sizeof(KChans) / sizeof(KChans[0]);
// The WCN3990 stores no MAC: a locally administered 02:4e:<the SoC serial> (SMEM socinfo, item 137, serial_num at +96),
// stable per phone; a fixed one if socinfo is too old to carry it
// (HostMac: declared with the driver state above)
static void PickMac()
	{
	TUint32 len = 0;
	const TUint8* soc = (const TUint8*)Smem_Get(KSmemHostGlobal, 137, &len);
	if (!soc || len < 100)
		return;
	TUint32 serial = G32(soc + 96);
	for (TInt i = 0; i < 4; ++i)
		HostMac[2 + i] = (TUint8)(serial >> (24 - 8 * i));
	}

// once after READY: chain masks, the channel list, the regulatory domain, the station vdev
static TInt ScanSetup()
	{
	TUint32 chains = (1u << Min(RfChains, 4u)) - 1;
	for (TUint32 param = 1; param <= 2; ++param)				// PDEV_SET_PARAM TX / RX_CHAIN_MASK
		{
		CmdStart(0x4003);
		CmdTlv(82, 8);
		Cmd32(param);
		Cmd32(chains);
		if (CmdSend("chain mask") != KErrNone)
			return Error;
		}
	CmdStart(0x3003);											// SCAN_CHAN_LIST
	CmdTlv(79, 4);
	Cmd32(KNumChans);
	CmdTlv(18, KNumChans * 28);
	for (TInt i = 0; i < KNumChans; ++i)
		{
		TBool g = KChans[i].iMhz < 3000;
		CmdTlv(80, 24);
		Cmd32(KChans[i].iMhz);
		Cmd32(KChans[i].iMhz);
		Cmd32(0);
		Cmd32(KChans[i].iWord);
		Cmd32(g ? 0x00282800 : 0x003c3c00);
		Cmd32(g ? 0x00002800 : 0x00003c00);
		}
	if (CmdSend("channel list") != KErrNone)
		return Error;
	// PDEV_SET_REGDOMAIN as ath10k (mac.c ath10k_regd_update): eeprom_rd 0 = US FCC3 0x3a, ctl 0x10; else the value as is
	// with no CTL (world 0x60-0x64: 0xff)
	TUint32 regd = EepromRd ? EepromRd & 0x7fff : 0x3a, ctl = EepromRd ? 0xff : 0x10;
	if (!regd)
		regd = 0x64;
	CmdStart(0x4001);
	CmdTlv(81, 24);
	Cmd32(0);													// pdev_id
	Cmd32(regd);
	Cmd32(regd);
	Cmd32(regd);
	Cmd32(ctl);
	Cmd32(ctl);
	if (CmdSend("regdomain") != KErrNone)
		return Error;
	PickMac();
	CmdStart(0x5001);											// VDEV_CREATE: vdev 0, station
	CmdTlv(86, 20);
	Cmd32(0);
	Cmd32(2);
	Cmd32(0);
	Cmd32(HostMac[0] | HostMac[1] << 8 | HostMac[2] << 16 | (TUint32)HostMac[3] << 24);
	Cmd32(HostMac[4] | HostMac[5] << 8);
	if (CmdSend("vdev create") != KErrNone)
		return Error;
	CmdStart(0x5008);											// VDEV_SET_PARAM TX_ENCAP_TYPE = 1, native wifi (W4)
	CmdTlv(95, 12);
	Cmd32(0);
	Cmd32(0x2c);
	Cmd32(1);
	if (CmdSend("tx encap") != KErrNone)
		return Error;
	CmdStart(0x9001);											// STA_POWERSAVE_MODE disabled: the firmware stays awake
	CmdTlv(108, 8);
	Cmd32(0);
	Cmd32(0);
	if (CmdSend("power save off") != KErrNone)
		return Error;
	Kern::Printf("[wifi] management tx over %s (WMI service 76 %s)", Svc76() ? "WMI" : "HTT", Svc76() ? "on" : "off");
	Kern::Printf("[wifi] regdomain %x, ctl %x; station %02x:%02x:%02x:%02x:%02x:%02x", regd, ctl, HostMac[0], HostMac[1],
		HostMac[2], HostMac[3], HostMac[4], HostMac[5]);
	return KErrNone;
	}

static TInt ScanStart(TBool aActive)
	{
	CmdStart(0x3001);											// START_SCAN
	CmdTlv(77, 100);
	TUint32 scan[25] = { 0xa000, 0xa001, 0, 1, 0x11f, 50, 150, 50, 500, 0, 0, 0, 20000, 5,
		// active 0x30 / passive 0x31: CHAN_STAT_EVENT | FILTER_PROBE_REQ (inverted on TLV firmware) | PASSIVE
		aActive ? 0x30u : 0x31u, 0, (TUint32)KNumChans, 1, aActive ? 1u : 0u, aActive ? 16u : 0u, 3, 0, 0, 0, 0 };
	for (TInt i = 0; i < 25; ++i)
		Cmd32(scan[i]);
	CmdTlv(16, KNumChans * 4);									// the channels, MHz
	for (TInt i = 0; i < KNumChans; ++i)
		Cmd32(KChans[i].iMhz);
	CmdTlv(19, aActive ? 36 : 0);								// SSIDs: the wildcard (length 0) for an active scan
	for (TInt i = 0; aActive && i < 9; ++i)
		Cmd32(0);
	CmdTlv(19, 8);												// BSSIDs: the broadcast address
	Cmd32(0xffffffff);
	Cmd32(0x0000ffff);
	CmdTlv(17, aActive ? 16 : 0);								// probe request IEs: supported + extended rates
	if (aActive)
		{
		Cmd32(0x84820801);										// 01 08 82 84 8b 96 0c 12 18 24 32 04 30 48 60 6c
		Cmd32(0x120c968b);
		Cmd32(0x04322418);
		Cmd32(0x6c604830);
		}
	NumBss = Beacons = ChanInfos = OtherFrames = HttFrames = HttMgmt = 0;
	ScanDone = EFalse;
	TInt r = CmdSend("start scan");
	if (r == KErrNone)
		Step = 7;
	return r;
	}

// ---- the scan results for the user side (HAL WlanScan): copied under a lock when a scan completes
static NFastMutex ResultLock;
static SBss Results[32];
static TInt NumResults, Scans;
static TBool ScanWanted;
static TUint32 ScanStartedAt;

static void PublishResults()
	{
	NKern::FMWait(&ResultLock);
	memcpy(Results, Bss, sizeof(Bss));
	NumResults = NumBss;
	++Scans;
	NKern::FMSignal(&ResultLock);
	}

// one request from the HAL (or the monitor) to this thread at a time: join, leave, send EAPOL, install a key, authorize.
// The thread copies it out when it starts it and completes it by sequence number, so one the caller gave up on (3 s)
// can't complete the next.
enum { EReqNone, EReqJoin = 1, EReqLeave = 2, EReqSend = 5, EReqKey = 6, EReqAuthorize = 7 };
static struct { TInt iOp; TUint32 iSeq; TBool iDone; TInt iResult; TInt iLen; TUint8 iData[KEapolMax]; } Req;

static TInt PostReq(TInt aOp, const TUint8* aData, TInt aLen)
	{
	NKern::FMWait(&LinkLock);
	if (Req.iOp != EReqNone)
		{
		NKern::FMSignal(&LinkLock);
		return KErrInUse;
		}
	memcpy(Req.iData, aData, aLen);
	Req.iLen = aLen;
	if (aOp == EReqLeave)
		AbortWanted = ETrue;									// a join in progress gives up at its next wait
	Req.iDone = EFalse;
	Req.iOp = aOp;
	TUint32 seq = ++Req.iSeq;
	NKern::FMSignal(&LinkLock);
	TInt r = KErrTimedOut;
	for (TInt ms = 0; ms < 3000; ms += 10)
		{
		NKern::Sleep(NKern::TimerTicks(10));
		NKern::FMWait(&LinkLock);
		TBool done = Req.iDone && Req.iSeq == seq;
		if (done)
			r = Req.iResult;
		NKern::FMSignal(&LinkLock);
		if (done)
			break;
		}
	NKern::FMWait(&LinkLock);
	Req.iOp = EReqNone;
	++Req.iSeq;													// a late completion of this one now misses
	NKern::FMSignal(&LinkLock);
	return r;
	}

// HAL WlanLink (0x4e1f): a1 = the op, a2 = TDes8*. Ops 1 join {bssid 6, our RSN element}, 2 leave, 5 send EAPOL
// {frame}, 6 install key {0 pairwise / 1 group, key index, 16-byte CCMP key}, 7 authorize: carried out by the driver
// thread, the caller waits (3 s at most). 3 status -> {version 1, state, aid, status, reason (le16), bssid, our MAC,
// rx data, rx ARP/IP (le32), AP RSN length, AP RSN element}; 4 the next EAPOL frame (KErrNotReady if none).
TInt Wcn3990_LinkHal(TAny* a1, TAny* a2)
	{
	TInt op = (TInt)(TLinAddr)a1;
	DThread* t = &Kern::CurrentThread();
	if (Step < 7)
		return KErrNotReady;									// not up (or not on this board)
	if (op != 3 && op != 4 && op != EReqJoin && op != EReqLeave && op != EReqSend && op != EReqKey &&
		op != EReqAuthorize)
		return KErrArgument;
	NKern::ThreadEnterCS();
	TUint8* buf = (TUint8*)Kern::Alloc(KEapolMax);				// not on the caller's small supervisor stack
	NKern::ThreadLeaveCS();
	if (!buf)
		return KErrNoMemory;
	TInt r = KErrNone, n = 0;
	if (op == 3 || op == 4)
		{
		NKern::FMWait(&LinkLock);
		if (op == 3)
			{
			buf[0] = 1;
			buf[1] = (TUint8)Link;
			Le16(buf + 2, LinkAid);
			Le16(buf + 4, LinkStatus);
			Le16(buf + 6, LinkReason);
			memcpy(buf + 8, Target.iBssid, 6);
			memcpy(buf + 14, HostMac, 6);
			Le32(buf + 20, RxData);
			Le32(buf + 24, RxIp);
			buf[28] = 0;
			n = 29;
			const TUint8* rsn = Link != ELinkIdle ? FindIe(Target.iIes, Target.iIeLen, 0x30) : NULL;
			if (rsn && rsn[1] <= 62)
				{
				buf[28] = (TUint8)(2 + rsn[1]);
				memcpy(buf + 29, rsn, buf[28]);
				n += buf[28];
				}
			}
		else if (EapolCount)
			{
			n = EapolLen[EapolHead];
			memcpy(buf, Eapol[EapolHead], n);
			EapolHead = (EapolHead + 1) % KEapolSlots;
			--EapolCount;
			}
		NKern::FMSignal(&LinkLock);
		r = op == 4 && !n ? KErrNotReady : Kern::ThreadDesWrite(t, a2, TPtrC8(buf, n), 0, KChunkShiftBy0, NULL);
		}
	else
		{
		n = a2 ? Kern::ThreadGetDesLength(t, a2) : 0;
		TPtr8 des(buf, KEapolMax);
		if (n < 0 || n > KEapolMax || (n && Kern::ThreadDesRead(t, a2, des, 0, KChunkShiftBy0) != KErrNone))
			r = KErrArgument;
		else if ((op == EReqJoin && n != 6 && (n < 6 + 2 || n > 6 + 64)) || (op == EReqSend && n < 4) ||
			(op == EReqKey && n != 18))										// a join with no RSN element: an open network
			r = KErrArgument;
		else
			r = PostReq(op, buf, n);
		}
	NKern::ThreadEnterCS();
	Kern::Free(buf);
	NKern::ThreadLeaveCS();
	return r;
	}

// the monitor's "wifi" (state and the scan list), "wifi join N" (N = an index into that list; joins with the RSN
// element the user side uses, WPA2-PSK CCMP: auth + assoc only, no keys), "wifi leave". No secrets cross the monitor.
void Wcn3990_Monitor(TInt aOp, TInt aArg)
	{
	static const TUint8 KJoin[6 + 22] = { 0, 0, 0, 0, 0, 0, 0x30, 20, 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac,
		4, 1, 0, 0x00, 0x0f, 0xac, 2, 0, 0 };
	if (aOp == 0)
		{
		A64Mon_Say("wifi: step %d error %d; link %d aid %d status %d reason %d; rx data %d ip %d; eapol queued %d", Step,
			Error, Link, LinkAid, LinkStatus, LinkReason, RxData, RxIp, EapolCount);
		for (TInt i = 0; i < NumResults; ++i)
			A64Mon_Say("  %d: %02x:%02x:%02x:%02x:%02x:%02x ch %d %d dBm%s", i, Results[i].iBssid[0],
				Results[i].iBssid[1], Results[i].iBssid[2], Results[i].iBssid[3], Results[i].iBssid[4],
				Results[i].iBssid[5], Results[i].iChannel, Results[i].iDbm, Results[i].iSecured ? " secured" : "");
		return;
		}
	if (Step < 7 || (aOp == 1 && (aArg < 0 || aArg >= NumResults)))
		{
		A64Mon_Say("wifi: not up, or no network %d (see \"wifi\")", aArg);
		return;
		}
	TUint8 join[sizeof(KJoin)];
	memcpy(join, KJoin, sizeof(KJoin));
	if (aOp == 1)
		memcpy(join, Results[aArg].iBssid, 6);
	TInt r = aOp == 1 ? PostReq(EReqJoin, join, sizeof(join)) : PostReq(EReqLeave, NULL, 0);
	A64Mon_Say("wifi %s -> %d", aOp == 1 ? "join" : "leave", r);
	}

// ---- W4: sending frames and joining (private/wifi/w4-join-spec.md)
static void CmdMac(const TUint8* aMac)							// wmi_mac_addr: 6 bytes + 2 pad
	{
	Cmd32(aMac[0] | aMac[1] << 8 | aMac[2] << 16 | (TUint32)aMac[3] << 24);
	Cmd32(aMac[4] | aMac[5] << 8);
	}
static void CmdBytes(const TUint8* aData, TInt aLen)			// an ARRAY_BYTE body, zero-padded to 4
	{
	memcpy(Cmd + CmdLen, aData, aLen);
	memclr(Cmd + CmdLen + aLen, -aLen & 3);
	CmdLen += (aLen + 3) & ~3;
	}

// two descriptors on CE4 as one transfer (ath10k _ath10k_ce_send_nolock_64 with GATHER, spec 3.4): aHead copied into a
// ring slot, then aLen1 bytes the chip reads at aPhys1; the doorbell only after the last
static TInt SendGather(const TUint8* aHead, TInt aLen0, TPhysAddr aPhys1, TInt aLen1)
	{
	SRing& s = Src[4];
	if (!s.iDesc || !s.iBuf)
		return KErrNotReady;
	TUint32 sw = Rri[4] & 0xffff & s.iMask;
	if (((s.iWr + 1) & s.iMask) == sw || ((s.iWr + 2) & s.iMask) == sw)
		return KErrOverflow;
	TUint32 slot = s.iWr % s.iSlots;
	ToDev(s.iBuf + slot * KCe[4].iMax, aHead, aLen0);
	volatile TUint32* w = (volatile TUint32*)&s.iDesc[s.iWr];
	w[0] = s.iBufPhys + slot * KCe[4].iMax;
	w[1] = 0x80010000;											// WCN3990 gather bit 31 (+ ath10k's CE_SEND_FLAG_GATHER)
	w[2] = aLen0 | 1u << 16;									// flags: GATHER, transfer id 0
	w[3] = 0;
	s.iWr = (s.iWr + 1) & s.iMask;
	DescWrite(&s.iDesc[s.iWr], aPhys1, (TUint16)aLen1, 0);
	__e32_memory_barrier();
	s.iWr = (s.iWr + 1) & s.iMask;
	R(Shadow(4)) = s.iWr;
	return KErrNone;
	}

static TInt TxAlloc(TBool aAuto)
	{
	for (TInt i = 0; i < KTxSlots; ++i)
		if (!TxBusy[i])
			{
			TxBusy[i] = ETrue;
			TxAuto[i] = aAuto;
			TxStat[i] = -1;
			return i;
			}
	return -1;
	}

// HTT TX_FRM (spec 3.2) for the frame in tx slot aId: aFlags0 = MAC_HDR_PRESENT | pkt type << 5 (| NO_ENCRYPT), ext tid
// 16 data / 17 mgmt. Its first bytes go as the gather's second item (prefetch); the chip fetches the rest from aFrags.
static TInt HttTx(TInt aId, TUint8 aFlags0, TInt aTid, TPhysAddr aFrags, TInt aLen)
	{
	TUint32 mw[7];
	TUint8* m = (TUint8*)mw;
	TInt prefetch = (Min(50, aLen) + 3) & ~3;
	m[0] = HttEid;												// HTC header: no flags, seq 0 (as ath10k for TX_FRM)
	m[1] = 0;
	Le16(m + 2, (TUint16)(20 + prefetch));
	Le32(m + 4, 0);
	m[8] = 0x01;												// TX_FRM
	m[9] = aFlags0;
	Le16(m + 10, (TUint16)((aTid << 6) | 0x800));				// vdev 0, ext tid, POSTPONED
	Le16(m + 12, (TUint16)aLen);
	Le16(m + 14, (TUint16)aId);
	Le32(m + 16, aFrags);
	Le32(m + 20, 0);
	Le16(m + 24, 0xffff);										// peer id: invalid
	Le16(m + 26, 0);											// freq: the home channel
	return SendGather(m, 28, TxBufPhys + aId * KTxSlot, prefetch);
	}

// a management frame (802.11 header onward, aFrame word aligned): over WMI by reference with service 76, else HTT;
// freed when the chip reports it sent
static TInt MgmtTx(const TUint8* aFrame, TInt aLen)
	{
	TInt id = TxAlloc(ETrue);
	if (id < 0 || aLen > KTxSlot)
		return KErrOverflow;
	ToDev(TxBuf + id * KTxSlot, aFrame, aLen);
	__e32_memory_barrier();
	if (!Svc76())
		return HttTx(id, 0x61, 17, TxBufPhys + id * KTxSlot, aLen);
	TInt head = Min(aLen, 64);
	CmdStart(0x7008);											// MGMT_TX_SEND
	CmdTlv(422, 28);
	Cmd32(0);													// vdev
	Cmd32(id);													// desc_id
	Cmd32(0);													// chanfreq: the home channel
	Cmd32(TxBufPhys + id * KTxSlot);							// paddr, le64
	Cmd32(0);
	Cmd32(aLen);
	Cmd32((head + 3) & ~3);
	CmdTlv(17, (head + 3) & ~3);
	CmdBytes(aFrame, head);
	return CmdSend("mgmt tx");
	}

// a data frame (native wifi, aFrame word aligned): its ext descriptor in the bank, then TX_FRM; aNoEncrypt before the
// pairwise key is installed. The slot is freed when the chip reports the frame sent (or discarded).
static TInt DataTx(const TUint8* aFrame, TInt aLen, TBool aNoEncrypt)
	{
	TInt id = TxAlloc(ETrue);
	if (id < 0 || aLen > KTxSlot)
		return KErrOverflow;
	ToDev(TxBuf + id * KTxSlot, aFrame, aLen);
	volatile TUint32* ext = (volatile TUint32*)(Bank + 72 * id);	// htt_msdu_ext_desc_64 (spec 3.3)
	for (TInt i = 0; i < 18; ++i)
		ext[i] = 0;
	ext[6] = TxBufPhys + id * KTxSlot;							// frags[0]: address, then {address bits 47:32, length}
	ext[7] = (TUint32)aLen << 16;
	__e32_memory_barrier();
	TInt r = HttTx(id, (TUint8)(0x21 | (aNoEncrypt ? 4 : 0)), 16, BankPhys + 72 * id, aLen);
	if (r != KErrNone)
		{
		TxBusy[id] = EFalse;
		return r;
		}
	return id;
	}

static TBool PeerUp, VdevStarted, VdevUp, PtkInstalled;
static TUint32 FrameW[KTxSlot / 4];								// frames are built here (word aligned for ToDev)
static TUint8* const Frame = (TUint8*)FrameW;

// a management header to the AP: FC, duration 0, AP, us, AP, sequence 0 (the firmware numbers it)
static TInt MgmtHdr(TUint8 aType)
	{
	memclr(Frame, 24);
	Frame[0] = aType;
	memcpy(Frame + 4, Target.iBssid, 6);
	memcpy(Frame + 10, HostMac, 6);
	memcpy(Frame + 16, Target.iBssid, 6);
	return 24;
	}

// poll until aFlag is set: aMs at most, and (aAbortable) not past a deauth or a leave
static TBool WaitFlag(volatile TBool& aFlag, TInt aMs, TBool aAbortable)
	{
	for (TInt t = 0; t < aMs && !aFlag && !Error && !(aAbortable && (LostWanted || AbortWanted)); ++t)
		{
		Poll();
		NKern::Sleep(NKern::TimerTicks(1));
		}
	return aFlag;
	}

// the rates both sides have, in 500 kb/s units without the basic bit, ours in table order (spec 5.2)
static TInt CommonRates(TUint8* aOut)
	{
	static const TUint8 KRates[] = { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 };
	TBool g = Target.iChannel <= 14;
	TInt n = 0;
	for (TUint i = g ? 0 : 4; i < sizeof(KRates); ++i)
		for (TUint8 id = 1; id <= 50; id += 49)					// Supported Rates, Extended Supported Rates
			{
			const TUint8* ie = FindIe(Target.iIes, Target.iIeLen, id);
			TBool has = EFalse;
			for (TInt k = 0; ie && k < ie[1]; ++k)
				has |= (ie[2 + k] & 0x7f) == KRates[i];
			if (has)
				{
				aOut[n++] = KRates[i];
				break;
				}
			}
	return n;
	}

// leave the network: deauth (if asked), PEER_DELETE, VDEV_DOWN, VDEV_STOP (ath10k's order, spec 1), then aState
static void Teardown(TInt aState, TUint16 aStatus, TUint16 aReason, TBool aDeauth)
	{
	if (aDeauth && PeerUp)
		{
		TInt n = MgmtHdr(0xc0);
		Le16(Frame + n, 3);										// reason 3: the station is leaving
		MgmtTx(Frame, n + 2);
		for (TInt t = 0; t < 50; ++t)							// let it go out before the peer goes
			Poll(), NKern::Sleep(NKern::TimerTicks(1));
		}
	if (PeerUp)
		{
		PeerUnmapped = EFalse;
		CmdStart(0x6002);										// PEER_DELETE
		CmdTlv(98, 12);
		Cmd32(0);
		CmdMac(Target.iBssid);
		if (CmdSend("peer delete") == KErrNone)
			WaitFlag(PeerUnmapped, 1000, EFalse);
		PeerUp = PeerMapped = EFalse;
		}
	if (VdevUp)
		{
		CmdStart(0x5007);										// VDEV_DOWN
		CmdTlv(94, 4);
		Cmd32(0);
		CmdSend("vdev down");
		VdevUp = EFalse;
		}
	if (VdevStarted)
		{
		VdevStoppedGot = EFalse;
		CmdStart(0x5006);										// VDEV_STOP
		CmdTlv(93, 4);
		Cmd32(0);
		if (CmdSend("vdev stop") == KErrNone)
			WaitFlag(VdevStoppedGot, 2000, EFalse);
		VdevStarted = EFalse;
		}
	PtkInstalled = LostWanted = AbortWanted = EFalse;
	for (TInt i = 0; i < KTxSlots; ++i)
		TxBusy[i] = EFalse;
	SetLink(aState, aStatus, aReason);
	}

// local reasons for a failed join (above the 802.11 reason codes, so the UI never mistakes them for one)
enum { EJoinNoRates = 0x101, EJoinStart, EJoinPeer, EJoinAuth, EJoinAssoc, EJoinTx };

static void JoinFail(TUint16 aStatus, TUint16 aReason)
	{
	if (LostWanted)												// the AP sent us away mid-join: that is the reason
		aReason = LostReason;
	Teardown(ELinkError, aStatus, aReason, !LostWanted && !AbortWanted);
	}

// send aLen bytes of Frame up to 3 times until aDone (the AP's answer, set by MgmtRx), 1 s each
static TBool MgmtExchange(TInt aLen, volatile TBool& aDone)
	{
	for (TInt i = 0; i < 3 && !aDone && !LostWanted && !AbortWanted && !Error; ++i)
		if (MgmtTx(Frame, aLen) == KErrNone)
			WaitFlag(aDone, 1000, ETrue);
	return aDone;
	}

// the whole join, on this thread (spec 1): vdev start, peer, auth, assoc, peer assoc, vdev up. A scan finishes first.
static void Join()
	{
	for (TInt t = 0; t < 8000 && Step == 7 && !ScanDone; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if (Step == 7)
		{
		PublishResults();
		Step = 8;
		}
	TUint8 rates[12];
	TInt nr = CommonRates(rates);
	TBool g = Target.iChannel <= 14, ofdm = EFalse;
	for (TInt i = 0; i < nr; ++i)
		ofdm |= rates[i] != 2 && rates[i] != 4 && rates[i] != 11 && rates[i] != 22;
	if (!nr)
		return JoinFail(0, EJoinNoRates);
	TUint32 mhz = g ? (Target.iChannel == 14 ? 2484 : 2407 + 5 * Target.iChannel) : 5000 + 5 * Target.iChannel;
	TUint32 phymode = g ? (ofdm ? 1 : 2) : 0;					// 11G, 11B, 11A (legacy: no HT yet)
	const TUint8* tim = FindIe(Target.iIes, Target.iIeLen, 5);
	TUint32 dtim = tim && tim[1] >= 2 && tim[3] ? tim[3] : 1;

	StartRespGot = EFalse;
	CmdStart(0x5003);											// VDEV_START_REQUEST + channel + empty NoA array
	CmdTlv(88, 72);
	Cmd32(0);													// vdev
	Cmd32(0);													// requestor
	Cmd32(Target.iBcnInt ? Target.iBcnInt : 100);
	Cmd32(dtim);
	for (TInt i = 0; i < 14; ++i)								// flags, ssid {len, 32}, beacon rate/power, NoA count,
		Cmd32(0);												// disable_hw_ack: all 0 for a station
	CmdTlv(80, 24);												// wmi_channel: mhz, centre 1, centre 2, mode (no flags)
	Cmd32(mhz);
	Cmd32(mhz);
	Cmd32(0);
	Cmd32(phymode);
	Cmd32(g ? 0x00282800 : 0x003c3c00);
	Cmd32(g ? 0x00002800 : 0x00003c00);
	CmdTlv(18, 0);
	if (CmdSend("vdev start") != KErrNone)
		return JoinFail(0, EJoinStart);
	VdevStarted = ETrue;
	if (!WaitFlag(StartRespGot, 5000, ETrue) || StartStatus)
		return JoinFail(0, EJoinStart);

	PeerMapped = EFalse;
	CmdStart(0x6001);											// PEER_CREATE: the AP
	CmdTlv(97, 16);
	Cmd32(0);
	CmdMac(Target.iBssid);
	Cmd32(0);
	if (CmdSend("peer create") != KErrNone)
		return JoinFail(0, EJoinPeer);
	PeerUp = ETrue;
	if (!WaitFlag(PeerMapped, 3000, ETrue))
		return JoinFail(0, EJoinPeer);

	AuthDone = EFalse;											// Authentication, open system, transaction 1
	TInt n = MgmtHdr(0xb0);
	Le16(Frame + n, 0);
	Le16(Frame + n + 2, 1);
	Le16(Frame + n + 4, 0);
	if (!MgmtExchange(n + 6, AuthDone))
		return JoinFail(0, EJoinAuth);
	if (AuthStatus)
		return JoinFail(AuthStatus, 0);

	AssocDone = EFalse;											// Association Request (spec 5.2)
	n = MgmtHdr(0x00);
	TUint16 cap = (TUint16)((g ? 0x0421 : 0x0001) | (Target.iCap & 0x10));	// ESS, privacy as the AP; 2.4 GHz short
	Le16(Frame + n, cap);										// preamble + short slot
	Le16(Frame + n + 2, 5);										// listen interval
	n += 4;
	Frame[n++] = 0;
	Frame[n++] = Target.iSsidLen;
	memcpy(Frame + n, Target.iSsid, Target.iSsidLen);
	n += Target.iSsidLen;
	Frame[n++] = 1;
	Frame[n++] = (TUint8)Min(nr, 8);
	memcpy(Frame + n, rates, Min(nr, 8));
	n += Min(nr, 8);
	if (nr > 8)
		{
		Frame[n++] = 50;
		Frame[n++] = (TUint8)(nr - 8);
		memcpy(Frame + n, rates + 8, nr - 8);
		n += nr - 8;
		}
	memcpy(Frame + n, OurRsn, OurRsnLen);
	n += OurRsnLen;
	if (!MgmtExchange(n, AssocDone))
		return JoinFail(0, EJoinAssoc);
	if (AssocStatus)
		return JoinFail(AssocStatus, 0);

	CmdStart(0x6005);											// PEER_ASSOC_COMPLETE, legacy (spec 2.5)
	CmdTlv(101, 76);
	CmdMac(Target.iBssid);
	Cmd32(0);													// vdev
	Cmd32(1);													// new_assoc
	Cmd32(AssocAid);
	Cmd32(OurRsnLen ? 0x5 : 0x1);								// AUTH (| NEED_PTK_4_WAY)
	Cmd32(AssocCap);
	Cmd32(1);													// listen interval: ath10k forces 1
	for (TInt i = 0; i < 4; ++i)								// ht caps, max mpdu, mpdu density, rate caps
		Cmd32(0);
	Cmd32(1);													// nss
	Cmd32(0);													// vht caps
	Cmd32(phymode);
	Cmd32(0);													// ht info
	Cmd32(0);
	Cmd32(nr);
	Cmd32(0);													// HT rates
	TUint8 legacy[12];
	for (TInt i = 0; i < nr; ++i)								// bit 7 marks CCK rates here, not basic ones
		legacy[i] = (TUint8)(rates[i] | (rates[i] == 2 || rates[i] == 4 || rates[i] == 11 || rates[i] == 22 ? 0x80 : 0));
	CmdTlv(17, (nr + 3) & ~3);
	CmdBytes(legacy, nr);
	CmdTlv(17, 0);
	CmdTlv(102, 16);											// VHT rate set: none
	for (TInt i = 0; i < 4; ++i)
		Cmd32(0);
	if (CmdSend("peer assoc") != KErrNone)
		return JoinFail(0, EJoinAssoc);

	CmdStart(0x5005);											// VDEV_UP {vdev, aid, bssid}
	CmdTlv(92, 16);
	Cmd32(0);
	Cmd32(AssocAid);
	CmdMac(Target.iBssid);
	if (CmdSend("vdev up") != KErrNone)
		return JoinFail(0, EJoinAssoc);
	VdevUp = ETrue;
	LinkAid = AssocAid;
	Kern::Printf("[wifi] associated: aid %d, %d rates, channel %d (%d MHz)", AssocAid, nr, Target.iChannel, mhz);
	SetLink(OurRsnLen ? ELinkHandshake : ELinkConnected, 0, 0);	// an open network has no handshake
	}

// one request (Req, from the HAL or the monitor), on this thread; returns its result
static TInt DoReq(TInt aOp, const TUint8* aData, TInt aLen)
	{
	TBool linked = Link >= ELinkJoining && Link <= ELinkConnected;
	switch (aOp)
		{
	case EReqJoin:
		{
		if (linked)
			return KErrInUse;
		TBool found = EFalse;
		NKern::FMWait(&ResultLock);
		for (TInt i = 0; i < NumResults && !found; ++i)
			if (!memcompare(Results[i].iBssid, 6, aData, 6))
				Target = Results[i], found = ETrue;
		NKern::FMSignal(&ResultLock);
		if (!found)
			return KErrNotFound;
		OurRsnLen = aLen - 6;
		memcpy(OurRsn, aData + 6, OurRsnLen);
		RxData = RxIp = 0;
		LinkAid = 0;
		LostWanted = AbortWanted = EFalse;
		SetLink(ELinkJoining, 0, 0);
		return KErrNone;										// the join itself runs after this completes
		}
	case EReqLeave:
		if (linked)
			Teardown(ELinkIdle, 0, 0, ETrue);
		else
			SetLink(ELinkIdle, 0, 0);
		return KErrNone;
	case EReqSend:
		{
		if (Link != ELinkHandshake && Link != ELinkConnected)
			return KErrNotReady;
		if (32 + aLen > KTxSlot)
			return KErrArgument;
		memclr(Frame, 32);										// native wifi: data, to the AP; LLC/SNAP EAPOL
		Frame[0] = 0x08;
		Frame[1] = (TUint8)(0x01 | (PtkInstalled ? 0x40 : 0));
		memcpy(Frame + 4, Target.iBssid, 6);
		memcpy(Frame + 10, HostMac, 6);
		memcpy(Frame + 16, Target.iBssid, 6);
		static const TUint8 KSnapEapol[8] = { 0xaa, 0xaa, 0x03, 0, 0, 0, 0x88, 0x8e };
		memcpy(Frame + 24, KSnapEapol, 8);
		memcpy(Frame + 32, aData, aLen);
		TInt id = DataTx(Frame, 32 + aLen, !PtkInstalled);
		if (id < 0)
			return id;
		// queued, not waited for: the firmware holds msg4 until the PTK is installed (phone, 2026-11-12: no report in
		// 2 s, the AP resending msg3, then DISCARD at teardown), as Linux's supplicant installs it right after msg4.
		// NO_ENCRYPT was fixed at queue time, so msg4 still goes out in the clear.
		Kern::Printf("[wifi] EAPOL out, %d bytes, msdu %d", aLen, id);
		return KErrNone;
		}
	case EReqKey:
		{
		if (Link != ELinkHandshake && Link != ELinkConnected)
			return KErrNotReady;
		TBool group = aData[0] != 0;
		TUint32 acks = KeyAcks;
		CmdStart(0x5009);										// VDEV_INSTALL_KEY, CCMP (spec 2.7)
		CmdTlv(96, 92);
		Cmd32(0);
		CmdMac(Target.iBssid);									// the AP for both keys on a station
		Cmd32(group ? aData[1] & 3 : 0);
		Cmd32(group ? 1 : 0);									// WMI_KEY_GROUP / PAIRWISE
		Cmd32(4);												// AES_CCM
		for (TInt i = 0; i < 14; ++i)							// replay counters: 0, as ath10k
			Cmd32(0);
		Cmd32(16);
		Cmd32(0);
		Cmd32(0);
		CmdTlv(17, 16);
		CmdBytes(aData + 2, 16);
		TInt r = CmdSend("install key");
		memclr(Cmd, CmdLen);									// no key stays in the command buffer
		if (r != KErrNone)
			return r;
		for (TInt t = 0; t < 2000 && KeyAcks == acks && !Error; ++t)	// HTT SEC_IND acknowledges it
			Poll(), NKern::Sleep(NKern::TimerTicks(1));
		if (KeyAcks == acks)
			return KErrTimedOut;
		if (!group)
			PtkInstalled = ETrue;
		Kern::Printf("[wifi] %s key installed", group ? "group" : "pairwise");
		return KErrNone;
		}
	case EReqAuthorize:
		{
		if (Link != ELinkHandshake && Link != ELinkConnected)
			return KErrNotReady;
		CmdStart(0x6004);										// PEER_SET_PARAM AUTHORIZE = 1
		CmdTlv(100, 20);
		Cmd32(0);
		CmdMac(Target.iBssid);
		Cmd32(3);
		Cmd32(1);
		TInt r = CmdSend("authorize");
		if (r == KErrNone)
			SetLink(ELinkConnected, 0, 0);
		return r;
		}
		}
	return KErrArgument;
	}

// take the pending request, if it is new, carry it out, complete it; a join then runs here
static void ServeReq()
	{
	static TUint32 lastSeq;
	static TUint8 data[KEapolMax];
	NKern::FMWait(&LinkLock);
	TInt op = Req.iOp != EReqNone && !Req.iDone && Req.iSeq != lastSeq ? Req.iOp : EReqNone;
	TUint32 seq = Req.iSeq;
	TInt len = Req.iLen;
	if (op)
		memcpy(data, Req.iData, len);
	NKern::FMSignal(&LinkLock);
	if (!op)
		return;
	lastSeq = seq;
	TInt r = DoReq(op, data, len);
	memclr(data, len);											// keys don't linger
	NKern::FMWait(&LinkLock);
	if (Req.iSeq == seq)
		Req.iResult = r, Req.iDone = ETrue;
	NKern::FMSignal(&LinkLock);
	if (op == EReqJoin && r == KErrNone)
		Join();
	}

// after bring-up: poll for good, scan when asked (polled; shortcut: no IRQs, 10 ms latency when idle)
static void Serve()
	{
	for (;;)
		{
		Poll();
		if (Step == 7 && (ScanDone || NKern::TickCount() - ScanStartedAt > 25000))
			{
			PublishResults();
			Step = 8;
			}
		ServeReq();
		if (LostWanted && Link >= ELinkJoining && Link <= ELinkConnected)
			Teardown(ELinkLost, 0, LostReason, EFalse);
		TBool linked = Link >= ELinkJoining && Link <= ELinkConnected;
		if (ScanWanted && Step == 8 && !Error && !linked)		// shortcut: no scans while joined (off-channel)
			{
			ScanWanted = EFalse;
			ScanStartedAt = NKern::TickCount();
			if (ScanStart(EFalse) != KErrNone)					// passive: on this phone active scans missed 2.4 GHz
				return;
			}
		NKern::Sleep(NKern::TimerTicks(Step == 7 || Link == ELinkHandshake ? 1 : 10));
		}
	}

// HAL WlanScan (0x4e1e): a1 = TDes8* (>= 4 + 32 x 42 bytes) -> {version 1, state, count, scans % 256} then per network
// {bssid 6, channel, dBm, flags (1 secured), ssid length, ssid 32}; a2 = 1 asks for a new scan.
// State: 0 off, 1 starting, 2 ready (results valid), 3 scanning, 4 error.
TInt Wcn3990_ScanHal(TAny* a1, TAny* a2)
	{
	if (a2)
		ScanWanted = ETrue;
	NKern::ThreadEnterCS();
	TUint8* out = (TUint8*)Kern::Alloc(4 + 32 * 42);				// not on the caller's small supervisor stack
	NKern::ThreadLeaveCS();
	if (!out)
		return KErrNoMemory;
	NKern::FMWait(&ResultLock);
	TInt n = NumResults;
	for (TInt i = 0; i < n; ++i)
		{
		TUint8* e = out + 4 + 42 * i;
		const SBss& b = Results[i];
		memcpy(e, b.iBssid, 6);
		e[6] = b.iChannel;
		e[7] = (TUint8)b.iDbm;
		e[8] = b.iSecured;
		e[9] = b.iSsidLen;
		memclr(e + 10, 32);
		memcpy(e + 10, b.iSsid, b.iSsidLen);
		}
	TInt scans = Scans;
	NKern::FMSignal(&ResultLock);
	out[0] = 1;
	out[1] = Error ? 4 : Step == 7 ? 3 : Step == 8 ? 2 : Step ? 1 : 0;
	out[2] = (TUint8)n;
	out[3] = (TUint8)scans;
	TPtrC8 des(out, 4 + 42 * n);
	TInt r = Kern::ThreadDesWrite(&Kern::CurrentThread(), a1, des, 0, KChunkShiftBy0, NULL);
	NKern::ThreadEnterCS();
	Kern::Free(out);
	NKern::ThreadLeaveCS();
	return r;
	}

// ---- the bring-up, on its own thread
static TInt Run()
	{
	Step = 0;
	Error = KErrNone;
	if (!Pool)
		{
		PoolSize = 0x100000;									// CE rings + buffers ~280 KB, HTT (W3b) ~350 KB
		NKern::ThreadEnterCS();
		TInt r = Epoc::AllocPhysicalRam(PoolSize, PoolPhys, 12);
		DPlatChunkHw* c = NULL;
		if (r == KErrNone)
			r = DPlatChunkHw::New(c, PoolPhys, PoolSize, EMapAttrSupRw | EMapAttrBufferedNC);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return Fail(r, "DMA memory");
		Pool = (TUint8*)c->LinearAddress();
		memclr(Pool, PoolSize);
		}
	PoolUsed = 0;
	NumChunks = 0;
	HttEid = WmiEid = 0xff;
	Credits = 0;
	memclr(Seq, sizeof(Seq));
	TInt r = Smmu_IdentityMap(0x40, 0x1);						// DTB iommus <&apps_smmu 0x40 0x1>
	Kern::Printf("[wifi] SMMU streams 40/1 -> %d; DMA pool %08x + %x", r, PoolPhys, PoolSize);
	if (r != KErrNone && r != KErrInUse)
		return Fail(r, "SMMU");
	Step = 1;
	if ((r = InitRings()) != KErrNone)
		return Fail(r, "rings");
	PostAllRx();
	Kern::Printf("[wifi] copy engines up (%d bytes of the pool); waiting for HTC ready", PoolUsed);
	if (!WaitFor(2, 3000))
		return Fail(KErrTimedOut, "HTC ready");
	if ((r = Connect(0x0300, 0x0008)) != KErrNone)				// HTT first, as ath10k, no credit flow
		return Fail(r, "HTT connect");
	for (TInt t = 0; t < 1000 && HttEid == 0xff && !Error; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if ((r = Connect(0x0100, 0x0100)) != KErrNone)				// WMI control: credit flow, RECV_ALLOC 1
		return Fail(r, "WMI connect");
	for (TInt t = 0; t < 1000 && WmiEid == 0xff && !Error; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if (HttEid == 0xff || WmiEid == 0xff)
		return Fail(KErrTimedOut, "HTC connect");
	Step = 3;
	TUint8 done[12];
	memclr(done, sizeof(done));
	Le16(done, 5);												// SETUP_COMPLETE_EX, all zero on SNOC
	if ((r = HtcSend(0, done, sizeof(done))) != KErrNone)
		return Fail(r, "setup complete");
	if (!WaitFor(4, 5000))
		return Fail(KErrTimedOut, "WMI service ready");
	for (TInt t = 0; t < 200; ++t)								// SERVICE_AVAILABLE follows within ms
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if ((r = SendInit()) != KErrNone)
		return Error ? Error : Fail(r, "WMI init");
	Step = 5;
	if (!WaitFor(6, 5000))
		return Fail(KErrTimedOut, "WMI ready");
	TUint8 ver[4] = { 0, 0, 0, 0 };								// HTT_H2T VERSION_REQ (ath10k htt_tx.c), the reply on CE1
	r = HtcSend(HttEid, ver, sizeof(ver));
	Kern::Printf("[wifi] HTT version request -> %d", r);
	for (TInt t = 0; t < 300; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if ((r = HttSetup()) != KErrNone)
		return r;
	for (TInt t = 0; t < 100; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if ((r = ScanSetup()) != KErrNone)
		return r;
	ScanStartedAt = NKern::TickCount();
	if ((r = ScanStart(EFalse)) != KErrNone)					// a first list right away
		return r;
	Serve();
	return Error;
	}

static TDynamicDfcQue* Queue;
static void RunDfcFn(TAny*) { Run(); }
static TDfc RunDfc(RunDfcFn, NULL, 1);

TInt Wcn3990_Start()
	{
	if (Step && !Error)
		return KErrInUse;										// up (or coming up): one bring-up per boot
	if (!Queue)
		{
		TInt r = Kern::DynamicDfcQCreate(Queue, 24, _L8("WlanCe"));
		if (r != KErrNone)
			return r;
		RunDfc.SetDfcQ(Queue);
		}
	RunDfc.Enque();
	return KErrNone;
	}

static TA64DevState WifiStatus(TDes8& aDetail)
	{
	static const char* const KSteps[] = { "idle", "rings", "HTC ready", "connected", "service ready", "init sent",
		"ready", "scanning", "scanned" };
	A64Device_Format(aDetail, "WCN3990 %s", KSteps[Step]);
	if (Error)
		A64Device_Format(aDetail, ", error %d", Error);
	if (Step >= 7)
		A64Device_Format(aDetail, ", %d scans, %d networks", Scans, NumResults);
	if (Step == 6)
		A64Device_Format(aDetail, ", MAC %02x:%02x:%02x:%02x:%02x:%02x", Mac[0], Mac[1], Mac[2], Mac[3], Mac[4],
			Mac[5]);
	return Error ? EA64DevError : Step >= 6 ? EA64DevPartial : Step ? EA64DevStarting : EA64DevOff;
	}
A64_DEVICE("wifi", "WCN3990 Wi-Fi", "qcom_wcn3990", WifiStatus);
