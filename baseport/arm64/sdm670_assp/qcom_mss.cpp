// SPDX-License-Identifier: Apache-2.0
//
// qcom_mss.cpp -- boots the SDM670's modem (MSS, DTB remoteproc@4080000 "qcom,sdm670-mss-pil") the way the
// SDM845 family does it, with the modem boot authenticator (MBA):
//   1. AOP load_state on, RPMh votes for the cx/mx/mss rails, the proxy and reset clocks, the "alt" reset
//      (PDC sync reset 9 + RMB alt-reset + AOSS mss_restart), the active clocks
//   2. MBA.MBN into the mba carve-out, mpss + mba handed to the modem's VMID (TrustZone assign_mem); the QDSP6's
//      boot FSM starts; the PBL and then the MBA report through the RMB registers (status 1/2 = XPU unlocked)
//   3. MODEM.MDT (ELF headers + hash) in a contiguous buffer the modem may read: MBA checks it (status 3)
//   4. each loadable segment MODEM.Bnn copied into mpss (all relocatable, from 0x8b000000), the RMB told the code
//      start and length as they grow; mpss handed back to the modem; MBA authenticates it all (status 4)
//   5. the modem says "ready" and "handover" on SMP2P (slave-kernel bits 1, 2); the mba carve-out comes back
// The firmware is read by the kernel itself from the modem_a partition (qcom_fat16.cpp). The boot runs on its own
// DFC queue: it takes seconds and the debug console's queue must keep the watchdog petted.
// Written from the sequence in Linux drivers/remoteproc/qcom_q6v5_mss.c (sdm845_mss data, q6v5_mba_load,
// q6v5_mpss_init_image, q6v5_mpss_load), qcom_q6v5.c and drivers/soc/qcom/mdt_loader.c; registers of the clocks
// and resets from Linux gcc-sdm845.c, reset-qcom-aoss.c and reset-qcom-pdc.c. Telephony stage C3 (docs/telephony.md).
// Before all that, the Pixel's board record goes into SMEM item 134 (BoardInfo): the modem's RF task reads it to
// pick the RF card, and faults without it (C4: ":0:[rf]EX:kernel:0:rf:171" 0.5 s after ready).
// At handover the proxy votes go, as Linux's qcom_msa_handover: kept at their highest, they pinned the modem's rails
// and its RPMh code asserted after voice calls ("rpmh_image_os.c:162", C6 2026-11-09).
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_scm.h"
#include "qcom_smem.h"
#include "qcom_smp2p.h"
#include "qcom_qmp.h"
#include "qcom_rpmh.h"
#include "qcom_fat16.h"
#include "qcom_glink.h"
#include "qcom_llcc.h"
#include "qcom_telhal.h"
#include "qcom_mss.h"

const TUint32 KQdsp6 = 0x04080000, KRmb = 0x04180000;
const TUint32 KMbaPhys = 0x95600000, KMbaSize = 0x00200000, KMpssPhys = 0x8b000000, KMpssSize = 0x09800000;
const TUint32 KHaltQ6 = 0x01f63000, KHaltModem = 0x01f65000, KHaltNc = 0x01f64000;	// TCSR + 0x3000/0x5000/0x4000
const TUint32 KAossMssRestart = 0x0c2b0000, KPdcSyncReset = 0x0b2e0100, KPdcModemBit = 1u << 9;
enum { ERmbMbaImage = 0x00, ERmbPblStatus = 0x04, ERmbCommand = 0x08, ERmbStatus = 0x0c, ERmbMetaData = 0x10,
	ERmbCodeStart = 0x14, ERmbCodeLength = 0x18, ERmbMssStatus = 0x40, ERmbAltReset = 0x44 };
enum { EQ6Sleep = 0x3c, EQ6BootCoreStart = 0x400, EQ6BootCmd = 0x404 };
const TUint32 KCmdMetaDataReady = 1, KCmdLoadReady = 2;
const TInt KStatusMetaOk = 3, KStatusAuthComplete = 4;
const TUint32 KWindow = 0x01000000;						// mpss is copied through 16 MB mappings

// GCC branch clocks (Linux gcc-sdm845.c): enable register + bit, and the CBCR whose bit 31 says "off" (0: none)
struct SClk { const char* iName; TUint32 iEnable, iBit, iHalt; };
static const SClk KProxyClks[] = { { "prng", 0x152004, 13, 0x134004 } };
static const SClk KResetClks[] = { { "iface", 0x18a000, 0, 0x18a000 }, { "snoc_axi", 0x18a150, 0, 0x18a150 } };
static const SClk KActiveClks[] = { { "bus", 0x18a154, 0, 0x18a154 }, { "mem", 0x152004, 10, 0x138004 },
	{ "gpll0_mss", 0x152004, 17, 0 }, { "mnoc_axi", 0x18a004, 0, 0x18a004 } };

static volatile TUint32& Reg(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }

// ---- state the monitor can read
static TInt Step, Error;
static TUint64 MpssPerm = 1ull << KScmVmidHlos, MbaPerm = 1ull << KScmVmidHlos;

static TInt Fail(TInt aStep, TInt aErr, const char* aWhat)
	{
	Error = aErr;
	Kern::Printf("[mss] step %d FAILED: %s (%d); RMB pbl %d status %d mss %x", aStep, aWhat, aErr,
		(TInt)Reg(KRmb + ERmbPblStatus), (TInt)Reg(KRmb + ERmbStatus), Reg(KRmb + ERmbMssStatus));
	return aErr;
	}

static TInt ClocksOn(const SClk* aClk, TInt aCount)
	{
	for (TInt i = 0; i < aCount; ++i)
		{
		Reg(aClk[i].iEnable) |= 1u << aClk[i].iBit;
		if (!aClk[i].iHalt)
			{
			Kern::NanoWait(1000);
			continue;
			}
		TInt n = 0;
		while ((Reg(aClk[i].iHalt) >> 31) && ++n < 500)
			Kern::NanoWait(1000);
		if (n >= 500)
			Kern::Printf("[mss] clock %s still off (cbcr %08x)", aClk[i].iName, Reg(aClk[i].iHalt));
		}
	return KErrNone;
	}

// Linux qcom_msa_handover: the proxy clock off and the proxy rails back. Our RPMh votes are one per resource, not
// summed per driver, so CX goes back to the eMMC's own level (qcom_sdhci.cpp, DDR52) rather than 0.
// shortcut: no per-driver vote aggregation in qcom_rpmh.cpp; add it when a third CX user appears.
static void ReleaseProxy()
	{
	for (TUint i = 0; i < sizeof(KProxyClks) / sizeof(SClk); ++i)
		Reg(KProxyClks[i].iEnable) &= ~(1u << KProxyClks[i].iBit);
	TInt r = Arc_Vote("mss.lvl", 0);
	r |= Arc_Vote("mx.lvl", 0);
	r |= Arc_Vote("cx.lvl", 128);
	r |= Rpmh_Set("xo.lvl", 0, 0);
	Kern::Printf("[mss] handover: proxy votes released (mss/mx 0, cx 128, xo off) -> %d", r);
	}

// Linux q6v5_xfer_mem_ownership: HLOS (RWX) and/or the modem's MSA (RW)
static TInt Xfer(TUint64& aPerm, TBool aLocal, TBool aRemote, TUint32 aPhys, TUint32 aSize)
	{
	if (aLocal == ((aPerm >> KScmVmidHlos) & 1) && aRemote == ((aPerm >> KScmVmidMssMsa) & 1))
		return KErrNone;
	SScmVmPerm next[2];
	TInt n = 0;
	if (aLocal)
		{
		next[n].iVmid = KScmVmidHlos;
		next[n++].iPerm = KScmPermRwx;
		}
	if (aRemote)
		{
		next[n].iVmid = KScmVmidMssMsa;
		next[n++].iPerm = KScmPermRw;
		}
	return Scm_AssignMem(aPhys, (aSize + 0xfff) & ~0xfffu, aPerm, next, n);
	}

static TInt RmbWait(TInt aStatus, TInt aMs)					// Linux q6v5_rmb_mba_wait: aStatus 0 = any nonzero
	{
	for (TInt t = 0; t < aMs; ++t)
		{
		TInt v = (TInt)Reg(KRmb + ERmbStatus);
		if (v < 0 || (!aStatus && v) || (aStatus && v == aStatus))
			return v;
		NKern::Sleep(NKern::TimerTicks(1));
		}
	return KErrTimedOut;
	}

// physical memory outside the kernel's RAM, mapped non-cacheable for the copy (the modem reads it from DRAM)
static TUint8* Map(TUint32 aPhys, TUint32 aSize, DPlatChunkHw*& aChunk)
	{
	aChunk = NULL;
	NKern::ThreadEnterCS();
	TInt r = DPlatChunkHw::New(aChunk, aPhys, (aSize + 0xfff) & ~0xfffu, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	return r == KErrNone ? (TUint8*)aChunk->LinearAddress() : NULL;
	}
static void Unmap(DPlatChunkHw* aChunk)
	{
	__e32_memory_barrier();
	NKern::ThreadEnterCS();
	if (aChunk)
		aChunk->Close(NULL);
	NKern::ThreadLeaveCS();
	}

// the alt reset (Linux q6v5_reset_deassert, has_alt_reset)
static void AltReset()
	{
	Reg(KPdcSyncReset) |= KPdcModemBit;
	Reg(KRmb + ERmbAltReset) = 1;
	Reg(KAossMssRestart) = 1;
	Kern::NanoWait(300000);								// 6 cycles of the 32 kHz sleep clock
	Reg(KAossMssRestart) = 0;
	Kern::NanoWait(300000);
	Reg(KRmb + ERmbAltReset) = 0;
	Reg(KPdcSyncReset) &= ~KPdcModemBit;
	}

static void HaltAxi(TUint32 aBase)							// Linux q6v5proc_halt_axi_port
	{
	if (Reg(aBase + 8))										// already idle
		return;
	Reg(aBase) = 1;
	for (TInt i = 0; i < 100 && !Reg(aBase + 4); ++i)
		NKern::Sleep(NKern::TimerTicks(1));
	if (!Reg(aBase + 8))
		Kern::Printf("[mss] AXI port %08x failed to halt", aBase);
	Reg(aBase) = 0;
	}

static void Stop()											// halt the ports, reset, and take the memory back
	{
	HaltAxi(KHaltQ6);
	HaltAxi(KHaltModem);
	HaltAxi(KHaltNc);
	AltReset();
	Xfer(MpssPerm, ETrue, EFalse, KMpssPhys, KMpssSize);
	Xfer(MbaPerm, ETrue, EFalse, KMbaPhys, KMbaSize);
	}

struct SPhdr { TUint32 iType, iOffset, iVaddr, iPaddr, iFilesz, iMemsz, iFlags, iAlign; };
const TUint32 KTypeMask = 7u << 24, KTypeHash = 2u << 24, KRelocatable = 1u << 27;
static TBool Loadable(const SPhdr& aP) { return aP.iType == 1 && (aP.iFlags & KTypeMask) != KTypeHash && aP.iMemsz; }

static TInt LoadSegment(TInt aIndex, const SPhdr& aP, TUint32 aOffset)
	{
	SFatFile f = { 0, 0 };
	if (aP.iFilesz)
		{
		TBuf8<24> name((const TUint8*)"IMAGE/MODEM.Bxx");
		name[13] = (TUint8)('0' + aIndex / 10);
		name[14] = (TUint8)('0' + aIndex % 10);
		name.Append(0);
		TInt r = Fat_Find((const char*)name.Ptr(), f);
		if (r != KErrNone || f.iSize != aP.iFilesz)
			return r != KErrNone ? r : KErrCorrupt;
		}
	for (TUint32 done = 0; done < aP.iMemsz; )
		{
		TUint32 n = Min(aP.iMemsz - done, KWindow);
		DPlatChunkHw* c;
		TUint8* p = Map(KMpssPhys + aOffset + done, n, c);
		if (!p)
			return KErrNoMemory;
		TUint32 file = done < aP.iFilesz ? Min(n, aP.iFilesz - done) : 0;
		TInt r = file ? Fat_Read(f, done, p, file) : KErrNone;
		if (r == KErrNone && file < n)
			memclr(p + file, n - file);
		Unmap(c);
		if (r != KErrNone)
			return r;
		done += n;
		}
	return KErrNone;
	}

// The Pixel board record in SMEM item 134 (SMEM_ID_VENDOR0, global partition, 32 bytes): { version, modem_flag,
// major_id, minor_id, subtype, platform, efs_magic, ftm_magic }, filled from socinfo (item 137). Facts from what the
// postmarketOS sdm670 kernel's "google,modemsmem" driver writes (DTB modemsmem { qcom,smem = <134>; }); the flags and
// magics stay 0 as there.
static TInt BoardInfo()
	{
	TInt r = Smem_Alloc(KSmemHostGlobal, 134, 32);
	if (r != KErrNone && r != KErrAlreadyExists)
		return r;
	TUint32 len = 0, ilen = 0;
	TUint32* info = (TUint32*)Smem_Get(KSmemHostGlobal, 134, &ilen);
	const TUint8* soc = (const TUint8*)Smem_Get(KSmemHostGlobal, 137, &len);
	if (!info || ilen < 32)
		return KErrCorrupt;
	if (!soc || len < 0x44)
		{
		Kern::Printf("[mss] board info: socinfo missing or short (%d bytes); smem 134 left zero", len);
		return KErrNone;
		}
	TUint32 platform = *(const TUint32*)(soc + 0x34), platVer = *(const TUint32*)(soc + 0x38);
	TUint32 subtype = *(const TUint32*)(soc + 0x40);
	info[0] = 0;
	info[2] = platVer >> 16 & 0xff;
	info[3] = platVer & 0xff;
	info[4] = subtype;
	info[5] = platform;
	__e32_memory_barrier();
	Kern::Printf("[mss] board info (smem 134): platform %d subtype %d version %d.%d", platform, subtype, info[2], info[3]);
	return KErrNone;
	}

static TInt Boot()
	{
	TInt r;
	Error = KErrNone;
	Step = 1;												// firmware files and the SMEM/SMP2P side
	if ((r = Smp2p_Start(KSmemHostModem)) != KErrNone)
		return Fail(Step, r, "smp2p");
	if ((r = BoardInfo()) != KErrNone)
		return Fail(Step, r, "smem 134");
	if ((r = Fat_Mount("modem_a")) != KErrNone)
		return Fail(Step, r, "mount modem_a");
	SFatFile mba, mdt;
	if ((r = Fat_Find("IMAGE/MBA.MBN", mba)) != KErrNone || (r = Fat_Find("IMAGE/MODEM.MDT", mdt)) != KErrNone)
		return Fail(Step, r, "firmware files");
	if (mba.iSize > KMbaSize || mdt.iSize > 0x3000 || mdt.iSize < 0x34)
		return Fail(Step, KErrTooBig, "firmware sizes");
	NKern::ThreadEnterCS();
	TUint8* hdr = (TUint8*)Kern::Alloc(mdt.iSize);
	NKern::ThreadLeaveCS();
	if (!hdr || (r = Fat_Read(mdt, 0, hdr, mdt.iSize)) != KErrNone)
		return Fail(Step, hdr ? r : KErrNoMemory, "read MODEM.MDT");
	TUint32 phoff = *(TUint32*)(hdr + 28), phnum = *(TUint16*)(hdr + 44);
	if (memcompare(hdr, 4, (const TUint8*)"\x7f" "ELF", 4) != 0 || phoff + phnum * sizeof(SPhdr) > mdt.iSize)
		return Fail(Step, KErrCorrupt, "MODEM.MDT header");
	const SPhdr* ph = (const SPhdr*)(hdr + phoff);
	TUint32 minAddr = 0xffffffff;
	TBool reloc = EFalse;
	for (TUint32 i = 0; i < phnum; ++i)
		if (Loadable(ph[i]))
			{
			minAddr = Min(minAddr, ph[i].iPaddr);
			reloc |= (ph[i].iFlags & KRelocatable) != 0;
			}
	Kern::Printf("[mss] firmware: MBA %d bytes, MDT %d bytes, %d segments from %08x%s", mba.iSize, mdt.iSize, phnum,
		minAddr, reloc ? " (relocatable)" : "");

	Step = 2;												// MBA into its carve-out (before the modem owns it)
	DPlatChunkHw* c;
	TUint8* p = Map(KMbaPhys, KMbaSize, c);
	if (!p)
		return Fail(Step, KErrNoMemory, "map mba");
	r = Fat_Read(mba, 0, p, mba.iSize);
	Unmap(c);
	if (r != KErrNone)
		return Fail(Step, r, "read MBA.MBN");

	Step = 3;												// power, clocks, reset
	if ((r = Qmp_Send("{class: image, res: load_state, name: modem, val: on}")) != KErrNone)
		return Fail(Step, r, "qmp load_state");
	Llcc_Init();											// the system cache's slices, the modem's included
	Arc_Vote("mx.lvl", 0xffff);
	Arc_Vote("cx.lvl", 0xffff);
	Arc_Vote("mss.lvl", 0xffff);
	Rpmh_Set("xo.lvl", 0, 3);								// the "xo" proxy clock (Linux clk-rpmh bi_tcxo, on = 3)
	ClocksOn(KProxyClks, sizeof(KProxyClks) / sizeof(SClk));
	ClocksOn(KResetClks, sizeof(KResetClks) / sizeof(SClk));
	AltReset();
	ClocksOn(KActiveClks, sizeof(KActiveClks) / sizeof(SClk));

	Step = 4;												// memory to the modem, then the MBA
	if ((r = Xfer(MpssPerm, EFalse, ETrue, KMpssPhys, KMpssSize)) != KErrNone)
		return Fail(Step, r, "assign mpss to the modem");
	if ((r = Xfer(MbaPerm, EFalse, ETrue, KMbaPhys, KMbaSize)) != KErrNone)
		return Fail(Step, r, "assign mba to the modem");
	Reg(KRmb + ERmbMbaImage) = KMbaPhys;
	Reg(KQdsp6 + EQ6Sleep) |= 1;							// the QDSP6 sleep clock
	for (TInt i = 0; (Reg(KQdsp6 + EQ6Sleep) >> 31) && i < 200; ++i)
		Kern::NanoWait(1000);
	Reg(KQdsp6 + EQ6BootCoreStart) = 1;
	Reg(KQdsp6 + EQ6BootCmd) = 1;							// the boot FSM
	TInt i = 0;
	for (; !(Reg(KRmb + ERmbMssStatus) & 1) && i < 1000; ++i)
		Kern::NanoWait(10000);
	if (i >= 1000)
		{
		Stop();
		return Fail(Step, KErrTimedOut, "boot FSM");
		}
	for (i = 0; !Reg(KRmb + ERmbPblStatus) && i < 1000; ++i)
		NKern::Sleep(NKern::TimerTicks(1));
	if (Reg(KRmb + ERmbPblStatus) != 1)
		{
		Stop();
		return Fail(Step, KErrGeneral, "PBL");
		}
	r = RmbWait(0, 5000);
	if (r != 1 && r != 2)
		{
		Stop();
		return Fail(Step, r < 0 ? r : KErrGeneral, "MBA boot");
		}
	Kern::Printf("[mss] MBA up (status %d)", r);

	Step = 5;												// the headers and hash, checked by the MBA
	Reg(KRmb + ERmbCodeLength) = 0;
	TPhysAddr md;
	NKern::ThreadEnterCS();
	r = Epoc::AllocPhysicalRam(0x3000, md);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		{
		Stop();
		return Fail(Step, r, "metadata buffer");
		}
	if ((p = Map(md, 0x3000, c)) != NULL)
		{
		memcpy(p, hdr, mdt.iSize);
		Unmap(c);
		}
	TUint64 mdPerm = 1ull << KScmVmidHlos;
	r = Xfer(mdPerm, EFalse, ETrue, md, mdt.iSize);
	if (r == KErrNone)
		{
		Reg(KRmb + ERmbMetaData) = md;
		Reg(KRmb + ERmbCommand) = KCmdMetaDataReady;
		r = RmbWait(KStatusMetaOk, 1000);
		Xfer(mdPerm, ETrue, EFalse, md, mdt.iSize);
		}
	NKern::ThreadEnterCS();
	Epoc::FreePhysicalRam(md, 0x3000);
	NKern::ThreadLeaveCS();
	if (r != KStatusMetaOk)
		{
		Stop();
		return Fail(Step, r < 0 ? r : KErrGeneral, "MDT authentication");
		}
	Kern::Printf("[mss] headers authenticated");

	Step = 6;												// the segments, shared with the modem while loading
	Xfer(MpssPerm, ETrue, EFalse, KMpssPhys, KMpssSize);
	if ((r = Xfer(MpssPerm, ETrue, ETrue, KMpssPhys, KMpssSize)) != KErrNone)
		{
		Stop();
		return Fail(Step, r, "share mpss");
		}
	TUint32 base = reloc ? minAddr : KMpssPhys, size = 0, t0 = NKern::TickCount();
	for (TUint32 s = 0; s < phnum; ++s)
		{
		const SPhdr& h = ph[s];
		if (!Loadable(h))
			continue;
		TUint32 off = h.iPaddr - base;
		if (h.iPaddr < base || off + h.iMemsz > KMpssSize || h.iFilesz > h.iMemsz)
			{
			Stop();
			return Fail(Step, KErrCorrupt, "segment outside mpss");
			}
		if ((r = LoadSegment(s, h, off)) != KErrNone)
			{
			Stop();
			Kern::Printf("[mss] segment %d", s);
			return Fail(Step, r, "load segment");
			}
		size += h.iMemsz;
		if (!Reg(KRmb + ERmbCodeLength))
			{
			Reg(KRmb + ERmbCodeStart) = reloc ? KMpssPhys : minAddr;
			Reg(KRmb + ERmbCommand) = KCmdLoadReady;
			}
		Reg(KRmb + ERmbCodeLength) = size;
		if ((TInt)Reg(KRmb + ERmbStatus) < 0)
			{
			Stop();
			return Fail(Step, (TInt)Reg(KRmb + ERmbStatus), "segment authentication");
			}
		}
	Kern::Printf("[mss] %d bytes loaded in %d ms", size, (TInt)(NKern::TickCount() - t0) * NKern::TickPeriod() / 1000);
	if ((r = Xfer(MpssPerm, EFalse, ETrue, KMpssPhys, KMpssSize)) != KErrNone)
		{
		Stop();
		return Fail(Step, r, "assign mpss");
		}
	r = RmbWait(KStatusAuthComplete, 10000);
	if (r != KStatusAuthComplete)
		{
		Stop();
		return Fail(Step, r < 0 ? r : KErrGeneral, "MPSS authentication");
		}
	Kern::Printf("[mss] MPSS authenticated");

	Step = 7;												// the modem runs: wait for ready and handover
	TUint32 v = 0;
	for (i = 0; i < 5000; ++i)
		{
		Smp2p_Poll(KSmemHostModem);
		if (Smp2p_In(KSmemHostModem, "slave-kernel", v) == KErrNone && (v & (KMssReady | KMssFatal)))
			break;
		NKern::Sleep(NKern::TimerTicks(1));
		}
	if (!(v & KMssReady))
		{
		TUint32 len = 0;
		const char* why = (const char*)Smem_Get(KSmemHostGlobal, 421, &len);	// SMEM_SSR_REASON_MSS0
		Kern::Printf("[mss] slave-kernel %08x; crash reason: %s", v, why && len ? why : "(none)");
		return Fail(Step, v & KMssFatal ? KErrDied : KErrTimedOut, "ready");
		}
	// GLINK right away, as Linux starts the rproc's glink subdevice at "up"; handover comes after (~1 s there)
	r = Glink_Start(KSmemHostModem);
	TInt ch = r == KErrNone ? Glink_Open(KSmemHostModem, "IPCRTR", Qrtr_Rx, NULL) : r;
	if (ch > 0)
		Qrtr_SetChannel(ch);								// MODEMD's QRTR router reads it (qcom_telhal.cpp)
	Xfer(MbaPerm, ETrue, EFalse, KMbaPhys, KMbaSize);
	NKern::ThreadEnterCS();
	Kern::Free(hdr);
	NKern::ThreadLeaveCS();
	Step = 8;
	Kern::Printf("[mss] modem running (slave-kernel %08x); glink start %d, IPCRTR %d", v, r, ch);
	for (i = 0; !(v & (KMssHandover | KMssFatal)) && i < 30000; ++i)
		{
		NKern::Sleep(NKern::TimerTicks(1));
		Smp2p_Poll(KSmemHostModem);
		Smp2p_In(KSmemHostModem, "slave-kernel", v);
		}
	Kern::Printf("[mss] after %d ms: slave-kernel %08x%s", i, v, v & KMssHandover ? " (handover)" : "");
	if (v & KMssHandover)
		ReleaseProxy();
	return KErrNone;
	}

// ---- the boot's own thread
static TDynamicDfcQue* Queue;
static void BootDfcFn(TAny*) { Boot(); }
static TDfc BootDfc(BootDfcFn, NULL, 1);

TInt Mss_StartBoot()
	{
	if (Step && Step < 8 && !Error)
		return KErrInUse;
	if (!Queue)
		{
		TInt r = Kern::DynamicDfcQCreate(Queue, 24, _L8("ModemBoot"));
		if (r != KErrNone)
			return r;
		BootDfc.SetDfcQ(Queue);
		}
	Step = 0;
	BootDfc.Enque();
	return KErrNone;
	}

void Mss_State(TInt& aStep, TInt& aError, TUint32 aRmb[4])
	{
	aStep = Step;
	aError = Error;
	aRmb[0] = Reg(KRmb + ERmbPblStatus);
	aRmb[1] = Reg(KRmb + ERmbStatus);
	aRmb[2] = Reg(KRmb + ERmbMssStatus);
	aRmb[3] = Reg(KRmb + ERmbCodeLength);
	}
