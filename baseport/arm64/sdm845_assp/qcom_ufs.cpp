// SPDX-License-Identifier: Apache-2.0
//
// qcom_ufs.cpp -- the UFS host (DTB ufshc@1d84000, "qcom,ufshc", UFSHCI 2.x; QMP UFS PHY v3 at 0x1d87000), READ
// ONLY (docs fajita.md OP6: Ubuntu Touch owns the phone's storage). ABL reads UFS on every boot; this driver takes
// over the host and link ABL leaves up ("adopt"): our own request list, then SCSI commands through UTP transfer
// requests, polled, one slot. Written from JESD223 (UFSHCI) and JESD220 (UFS); register and clock offsets from
// mainline Linux (ufshci.h, gcc-sdm845.c, phy-qcom-qmp-ufs.c).
// - No write path: Command() refuses every opcode outside a read-only allow-list; there is no A64BlkDev_Write (the
//   weak default refuses), no query UPIU, no task management.
// - A64BlkDev_Init says KErrNotSupported, so C: stays the RAM drive; the monitor's "blk init" brings it up.
// - The block layer's sectors are 512 bytes; the LUNs' blocks are 4096: reads go through a bounce buffer.
// shortcut: adopt only; if ABL leaves UFS off this returns KErrNotReady (the full bring-up: supplies, GDSC, clocks,
// QMP PHY tables, HCE, link start-up, comes when a boot needs it).
#include <kernel/kern_priv.h>
#include <kernel/cache.h>
#include "board.h"
#include "a64_blk.h"
#include "a64_dbgmon.h"
#include "a64_device.h"
#include "qcom_rpmh.h"
#include "qcom_smmu.h"

const TUint32 KHc = 0x01d84000, KPhy = 0x01d87000, KGcc = 0x00100000, KSmmu = 0x15000000;
// UFSHCI registers
enum { ECap = 0x00, EVer = 0x08, EIs = 0x20, EIe = 0x24, EHcs = 0x30, EHce = 0x34, EUtrlba = 0x50, EUtrlbau = 0x54,
	EUtrldbr = 0x58, EUtrlclr = 0x5c, EUtrlrsr = 0x60, EUtmrlba = 0x70, EUtmrlbau = 0x74, EUtmrldbr = 0x78,
	EUtmrlrsr = 0x80, EUicCmd = 0x90, EUicArg1 = 0x94, EUicArg2 = 0x98, EUicArg3 = 0x9c };
enum { EHcsDp = 1, EHcsUtrlRdy = 2, EHcsUtmrlRdy = 4, EHcsUcRdy = 8, EIsUtrcs = 1, EIsUccs = 1 << 10 };
// GCC (gcc-sdm845.c): the GDSC, then clock branches (bit 31 = CLK_OFF)
enum { EGdsc = 0x77004, EAxi = 0x7700c, EAhb = 0x77010, ETxSym0 = 0x77014, ERxSym0 = 0x77018, EUnipro = 0x77054,
	EIce = 0x77058, EAux = 0x7708c, ERxSym1 = 0x770a8, EAggreAxi = 0x82024, EClkref = 0x8c000 };
// the QMP PHY's PCS (at +0xc00): start, power-down control, ready status (phy-qcom-qmp-pcs-ufs-v3.h)
enum { EPcsStart = 0xc00, EPcsPwrDn = 0xc04, EPcsReady = 0xd60 };

static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KHc) + aOff); }
static volatile TUint32& Gcc(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KGcc + aOff); }
static volatile TUint32& Phy(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KPhy) + aOff); }

static TBool On(TUint32 aBranch) { return !(Gcc(aBranch) >> 31); }
// the host's registers answer only with its power domain and interface clocks on; touching them otherwise is a
// secure bus error, which on this phone means QUALCOMM CrashDump
static TBool HostClocked() { return (Gcc(EGdsc) >> 31) && On(EAhb) && On(EAxi) && On(EUnipro); }

// one request at a time: the monitor, and later the media driver, share the slot and the buffers
static DMutex* Lock;
const TUint8 KMutexOrdUfs = 0xe0;
class TLock
	{
public:
	TLock() { NKern::ThreadEnterCS(); if (Lock) Kern::MutexWait(*Lock); }
	~TLock() { if (Lock) Kern::MutexSignal(*Lock); NKern::ThreadLeaveCS(); }
	};

// memory: one non-cached 8 KB block (the transfer list at 0 (1 KB aligned), the task list at 0x400, the command
// descriptor at 0x800: command UPIU, response UPIU at +0x200, the PRDT at +0x400), and a cached data buffer
const TInt KXfer = 256 * 1024;							// bytes per request: one PRDT entry (its 18-bit count)
const TUint32 KUcd = 0x800, KResp = 0x200, KPrdt = 0x400;
static TPhysAddr DescPhys, DataPhys;
static volatile TUint32* Desc;
static TUint8* Data;

struct TLun { TUint32 iBlocks, iBlockSize; };
const TInt KLuns = 8;
static TLun Luns[KLuns];
static TInt Cur;										// the LUN A64BlkDev_Read reads ("blk lun N")
static TInt InitResult = KErrNotReady;
static TBuf8<200> Report[24];
static TInt NReport;

static void Log(const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<200> b;
	Kern::AppendFormat(b, aFmt, list);
	VA_END(list);
	Kern::Printf("[ufs] %S", &b);
	if (NReport < (TInt)(sizeof(Report) / sizeof(Report[0])))
		Report[NReport++] = b;
	}

static TUint32 Be32(const TUint8* p) { return (TUint32)p[0] << 24 | (TUint32)p[1] << 16 | (TUint32)p[2] << 8 | p[3]; }
static void PutBe32(TUint8* p, TUint32 v) { p[0] = (TUint8)(v >> 24); p[1] = (TUint8)(v >> 16); p[2] = (TUint8)(v >> 8); p[3] = (TUint8)v; }

// a UIC DME_GET (read a UniPro attribute): the value, or -1
static TInt DmeGet(TUint32 aAttr)
	{
	if (!(R(EHcs) & EHcsUcRdy))
		return -1;
	R(EIs) = EIsUccs;
	R(EUicArg1) = aAttr << 16;
	R(EUicArg2) = 0;
	R(EUicArg3) = 0;
	R(EUicCmd) = 0x01;
	for (TInt i = 0; i < 10000 && !(R(EIs) & EIsUccs); ++i)
		Kern::NanoWait(10000);
	if (!(R(EIs) & EIsUccs))
		return -1;
	R(EIs) = EIsUccs;
	return (R(EUicArg2) & 0xff) ? -1 : (TInt)R(EUicArg3);
	}

// "blk info" (and Volume Up): what ABL left, read only
void A64BlkDump()
	{
	Kern::Printf("[ufs] gcc: gdsc %08x ahb %08x axi %08x unipro %08x ice %08x aux %08x tx0 %08x rx0 %08x rx1 %08x "
		"aggre %08x clkref %08x", Gcc(EGdsc), Gcc(EAhb), Gcc(EAxi), Gcc(EUnipro), Gcc(EIce), Gcc(EAux), Gcc(ETxSym0),
		Gcc(ERxSym0), Gcc(ERxSym1), Gcc(EAggreAxi), Gcc(EClkref));
	// the stream matches that cover UFS's streams 0x100-0x10f (mainline: iommus <0x100 0xf>)
	TInt smrs = *(volatile TUint32*)A64_Io(KSmmu + 0x20) & 0xff;
	for (TInt n = 0; n < smrs; ++n)
		{
		TUint32 smr = *(volatile TUint32*)A64_Io(KSmmu + 0x800 + 4 * n);
		TUint32 id = smr & 0x7fff, mask = (smr >> 16) & 0x7fff;
		if (!(smr >> 31) || ((id ^ 0x100) & ~(mask | 0xf) & 0x7fff))
			continue;
		TUint32 s2cr = *(volatile TUint32*)A64_Io(KSmmu + 0xc00 + 4 * n);
		Kern::Printf("[ufs] smmu: smr %d = %08x, s2cr %08x", n, smr, s2cr);
		if ((s2cr >> 16) & 3)
			continue;										// not a translating context
		// the context bank it translates through (qcom_smmu.cpp's layout)
		TUint32 idr1 = *(volatile TUint32*)A64_Io(KSmmu + 0x24);
		TUint32 page = (idr1 >> 31) ? 0x10000 : 0x1000, pages = 1u << (((idr1 >> 28) & 7) + 1), cb = s2cr & 0xff;
		volatile TUint32* c = (volatile TUint32*)A64_Io(KSmmu + pages * page + cb * page);
		Kern::Printf("[ufs] smmu: cb %d cbar %08x cba2r %08x sctlr %08x tcr %08x tcr2 %08x ttbr0 %08x:%08x mair0 %08x "
			"fsr %08x", cb, *(volatile TUint32*)A64_Io(KSmmu + page + 4 * cb),
			*(volatile TUint32*)A64_Io(KSmmu + page + 0x800 + 4 * cb), c[0], c[0x30 / 4], c[0x10 / 4], c[0x24 / 4],
			c[0x20 / 4], c[0x38 / 4], c[0x58 / 4]);
		}
	if (!HostClocked())
		{
		Kern::Printf("[ufs] host unclocked: not touched");
		}
	else
		{
		Kern::Printf("[ufs] host: cap %08x ver %08x hcs %08x hce %08x is %08x ie %08x", R(ECap), R(EVer), R(EHcs),
			R(EHce), R(EIs), R(EIe));
		Kern::Printf("[ufs] lists: utrl %08x:%08x run %d bell %08x; utmrl %08x run %d bell %08x", R(EUtrlbau),
			R(EUtrlba), R(EUtrlrsr), R(EUtrldbr), R(EUtmrlba), R(EUtmrlrsr), R(EUtmrldbr));
		if (On(EAux))
			Kern::Printf("[ufs] phy: start %08x pwrdn %08x ready %08x", Phy(EPcsStart), Phy(EPcsPwrDn), Phy(EPcsReady));
		// PA_PWRMODE (rx<<4 | tx: 1 fast, 2 slow, 4 fast auto, 5 slow auto), gears, lanes, HS series
		Kern::Printf("[ufs] link: pwrmode %x tx gear %d rx gear %d lanes %d/%d series %d", DmeGet(0x1571),
			DmeGet(0x1568), DmeGet(0x1583), DmeGet(0x1560), DmeGet(0x1580), DmeGet(0x156a));
		}
	for (TInt i = 0; i < NReport; ++i)
		Kern::Printf("[ufs] %S", &Report[i]);
	}

// one request in slot 0: a NOP OUT (aCdb NULL) or a read-only SCSI command, aBytes into Data. KErrNone and the SCSI
// status (0 good, 2 check condition; then aSense = key << 16 | asc << 8 | ascq)
static TInt Command(TInt aLun, const TUint8* aCdb, TUint32 aBytes, TInt& aStatus, TUint32& aSense)
	{
	aStatus = 0;
	aSense = 0;
	if (aCdb)
		switch (aCdb[0])
			{
		case 0x00:	// TEST UNIT READY
		case 0x03:	// REQUEST SENSE
		case 0x12:	// INQUIRY
		case 0x25:	// READ CAPACITY(10)
		case 0x28:	// READ(10)
			break;
		default:
			return KErrAccessDenied;							// the no-write rule, in one place
			}
	if (aBytes > (TUint32)KXfer)
		return KErrArgument;
	volatile TUint32* utrd = Desc;
	TUint8* ucd = (TUint8*)Desc + KUcd;
	memclr(ucd, KPrdt + 16);
	if (aCdb)
		{
		ucd[0] = 0x01;									// COMMAND UPIU
		ucd[1] = aBytes ? 0x40 : 0;						// R: data in; simple task
		ucd[2] = (TUint8)aLun;
		PutBe32(ucd + 12, aBytes);						// expected data transfer length
		memcpy(ucd + 16, aCdb, 16);
		}												// else NOP OUT: all zero
	if (aBytes)
		{
		volatile TUint32* prd = (volatile TUint32*)(ucd + KPrdt);
		prd[0] = (TUint32)DataPhys;
		prd[1] = 0;
		prd[2] = 0;
		prd[3] = aBytes - 1;
		Cache::SyncMemoryBeforeDmaRead((TLinAddr)Data, aBytes);
		}
	// UTRD: command type 1 (UFS storage), data direction 2 (device to host) or 0, interrupt on completion
	utrd[0] = (1u << 28) | (aBytes ? (2u << 25) : 0) | (1u << 24);
	utrd[1] = 0;
	utrd[2] = 0x0f;										// OCS: invalid until the host writes it
	utrd[3] = 0;
	utrd[4] = (TUint32)(DescPhys + KUcd);
	utrd[5] = 0;
	utrd[6] = ((KResp / 4) << 16) | (KResp / 4);		// response UPIU offset, length (dwords)
	utrd[7] = ((KPrdt / 4) << 16) | (aBytes ? 1 : 0);	// PRDT offset (dwords), entries
	__e32_memory_barrier();
	R(EIs) = EIsUtrcs;
	R(EUtrldbr) = 1;
	TInt i = 0;
	for (; i < 100000 && (R(EUtrldbr) & 1); ++i)		// up to 1 s
		Kern::NanoWait(10000);
	if (R(EUtrldbr) & 1)
		{
		R(EUtrlclr) = ~1u;								// writing 0 clears the slot
		Log("slot 0 timed out: cdb %02x lun %d, is %08x hcs %08x", aCdb ? aCdb[0] : 0xff, aLun, R(EIs), R(EHcs));
		return KErrTimedOut;
		}
	R(EIs) = EIsUtrcs;
	if (aBytes)
		Cache::SyncMemoryAfterDmaRead((TLinAddr)Data, aBytes);
	const TUint8* resp = ucd + KResp;
	TUint32 ocs = utrd[2] & 0xff;
	if (ocs != 0 || resp[0] != (aCdb ? 0x21 : 0x20) || resp[6] != 0)
		{
		Log("cdb %02x lun %d: ocs %x, response %02x/%02x", aCdb ? aCdb[0] : 0xff, aLun, ocs, resp[0], resp[6]);
		return KErrGeneral;
		}
	aStatus = resp[7];
	if (aStatus == 2 && (resp[32] << 8 | resp[33]) >= 14)	// fixed-format sense data after its length
		aSense = (resp[34 + 2] & 0xf) << 16 | resp[34 + 12] << 8 | resp[34 + 13];
	return KErrNone;
	}

// a SCSI command with up to three retries on a check condition (UNIT ATTENTION after a reset, ...)
static TInt Scsi(TInt aLun, const TUint8* aCdb, TUint32 aBytes)
	{
	TInt status = 0, r = KErrNone;
	TUint32 sense = 0;
	for (TInt k = 0; k < 4; ++k)
		{
		r = Command(aLun, aCdb, aBytes, status, sense);
		if (r != KErrNone || status != 2)
			break;
		}
	if (r == KErrNone && status != 0)
		{
		if (aCdb[0] != 0x00)								// TUR on an absent LUN: expected, not logged
			Log("cdb %02x lun %d: status %d sense %06x", aCdb[0], aLun, status, sense);
		r = KErrNotReady;
		}
	return r;
	}

static TInt ReadBlocks(TInt aLun, TUint32 aBlock, TUint32 aCount)
	{
	TUint8 cdb[16] = { 0x28 };
	PutBe32(cdb + 2, aBlock);
	cdb[7] = (TUint8)(aCount >> 8);
	cdb[8] = (TUint8)aCount;
	return Scsi(aLun, cdb, aCount * Luns[aLun].iBlockSize);
	}

static TInt Adopt()
	{
	if (!Lock)
		{
		_LIT(KLockName, "Ufs");
		NKern::ThreadEnterCS();
		TInt r = Kern::MutexCreate(Lock, KLockName, KMutexOrdUfs);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return r;
		}
	TLock lock;
	if (!HostClocked())
		{
		Log("ABL left the host unclocked (gdsc %08x ahb %08x): the full bring-up isn't written yet", Gcc(EGdsc),
			Gcc(EAhb));
		return KErrNotReady;
		}
	TUint32 hcs = R(EHcs);
	if (!(R(EHce) & 1) || (hcs & (EHcsDp | EHcsUtrlRdy)) != (EHcsDp | EHcsUtrlRdy))
		{
		Log("ABL left no link: hce %08x hcs %08x", R(EHce), hcs);
		return KErrNotReady;
		}
	if (!Desc)
		{
		NKern::ThreadEnterCS();
		TInt r = Epoc::AllocPhysicalRam(0x2000, DescPhys, 12);
		if (r == KErrNone)
			r = Epoc::AllocPhysicalRam(KXfer, DataPhys, 12);
		DPlatChunkHw* d = NULL;
		DPlatChunkHw* c = NULL;
		if (r == KErrNone)
			r = DPlatChunkHw::New(d, DescPhys, 0x2000, EMapAttrSupRw | EMapAttrBufferedNC);
		if (r == KErrNone)
			r = DPlatChunkHw::New(c, DataPhys, KXfer, EMapAttrSupRw | EMapAttrCachedMax);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return r;
		Desc = (volatile TUint32*)d->LinearAddress();
		Data = (TUint8*)c->LinearAddress();
		memclr((TAny*)Desc, 0x2000);
		}
	// the data path through aggre1 NoC to DDR (BCM SN9, 16 bytes wide; MC0/SH0 are voted by USB)
	Log("bus vote SN9 -> %d", Bcm_Vote("SN9", 300000, 600000, 16));
	TInt sm = Smmu_IdentityMap(0x100, 0);
	Log("smmu stream 0x100 -> %d (%d: routed already)", sm, KErrInUse);
	if (sm != KErrNone && sm != KErrInUse)
		return sm;
	// our lists in place of ABL's: stop both, point them at our memory, start; interrupts masked (we poll)
	Log("ABL's lists: utrl %08x run %d bell %08x; utmrl %08x run %d; ie %08x", R(EUtrlba), R(EUtrlrsr), R(EUtrldbr),
		R(EUtmrlba), R(EUtmrlrsr), R(EIe));
	R(EIe) = 0;
	R(EUtrlrsr) = 0;
	R(EUtmrlrsr) = 0;
	R(EUtrlba) = (TUint32)DescPhys;
	R(EUtrlbau) = 0;
	R(EUtmrlba) = (TUint32)DescPhys + 0x400;
	R(EUtmrlbau) = 0;
	R(EUtmrlrsr) = 1;
	R(EUtrlrsr) = 1;
	TInt status;
	TUint32 sense;
	TInt r = Command(0, NULL, 0, status, sense);
	Log("NOP OUT -> %d", r);
	if (r != KErrNone)
		return r;
	TInt found = 0;
	for (TInt lun = 0; lun < KLuns; ++lun)
		{
		Luns[lun].iBlocks = Luns[lun].iBlockSize = 0;
		const TUint8 tur[16] = { 0x00 };
		if (Scsi(lun, tur, 0) != KErrNone)
			continue;
		const TUint8 cap[16] = { 0x25 };
		if (Scsi(lun, cap, 8) != KErrNone)
			continue;
		TUint32 last = Be32(Data), size = Be32(Data + 4);
		if (size < KA64BlkSector || size > (TUint32)KXfer || size % KA64BlkSector)
			{
			Log("lun %d: block size %d not usable", lun, size);
			continue;
			}
		Luns[lun].iBlocks = last + 1;
		Luns[lun].iBlockSize = size;
		Log("lun %d: %d blocks of %d (%d MB)", lun, last + 1, size, (TInt)((TUint64)(last + 1) * size >> 20));
		++found;
		}
	const TUint8 inq[16] = { 0x12, 0, 0, 0, 36 };
	if (found && Scsi(0, inq, 36) == KErrNone)
		{
		TBuf8<32> id;
		id.Copy(Data + 8, 28);								// vendor, product, revision
		Log("device: %S", &id);
		}
	Cur = 0;
	return found ? KErrNone : KErrNotFound;
	}

// C: stays the RAM drive in OP6: no mount at boot
TInt A64BlkDev_Init(TUint64&) { return KErrNotSupported; }

TInt A64BlkDev_Probe()									// "blk init"
	{
	NReport = 0;
	return InitResult = Adopt();
	}

// 512-byte sectors of the selected LUN, through the bounce buffer
TInt A64BlkDev_Read(TUint64 aLba, TInt aCount, TUint8* aBuf)
	{
	if (InitResult != KErrNone)
		return KErrNotReady;
	TLock lock;
	const TLun& l = Luns[Cur];
	const TUint32 per = l.iBlockSize / KA64BlkSector;
	if (!l.iBlocks || aCount <= 0 || aLba + aCount > (TUint64)l.iBlocks * per)
		return KErrArgument;
	while (aCount > 0)
		{
		TUint32 block = (TUint32)(aLba / per), off = (TUint32)(aLba % per) * KA64BlkSector;
		TUint32 want = off + aCount * KA64BlkSector;
		TUint32 n = Min((want + l.iBlockSize - 1) / l.iBlockSize, (TUint32)KXfer / l.iBlockSize);
		TInt r = ReadBlocks(Cur, block, n);
		if (r != KErrNone)
			return r;
		TInt sectors = Min((TInt)((n * l.iBlockSize - off) / KA64BlkSector), aCount);
		memcpy(aBuf, Data + off, sectors * KA64BlkSector);
		aBuf += sectors * KA64BlkSector;
		aLba += sectors;
		aCount -= sectors;
		}
	return KErrNone;
	}

// one LUN's GPT, in its own block size: a line per partition
static void ListGpt(TInt aLun)
	{
	const TLun& l = Luns[aLun];
	A64Mon_Say("lun %d: %d blocks of %d, %d MB", aLun, l.iBlocks, l.iBlockSize, (TInt)((TUint64)l.iBlocks * l.iBlockSize >> 20));
	if (ReadBlocks(aLun, 1, 1) != KErrNone || memcompare(Data, 8, (const TUint8*)"EFI PART", 8) != 0)
		{
		A64Mon_Say("lun %d: no GPT", aLun);
		return;
		}
	TUint32 lba = (TUint32)*(TUint64*)(Data + 72), n = *(TUint32*)(Data + 80), size = *(TUint32*)(Data + 84);
	if (size < 128 || l.iBlockSize % size)
		{
		A64Mon_Say("lun %d: GPT entry size %d", aLun, size);
		return;
		}
	const TUint32 per = l.iBlockSize / size;
	for (TUint32 i = 0; i < n && i < 256; ++i)
		{
		if (i % per == 0 && ReadBlocks(aLun, lba + i / per, 1) != KErrNone)
			return;
		const TUint8* p = Data + (i % per) * size;
		TUint64 first = *(const TUint64*)(p + 32), last = *(const TUint64*)(p + 40);
		if (!first)
			continue;
		TBuf8<40> name;
		for (TInt c = 0; c < 36 && p[56 + 2 * c]; ++c)
			name.Append(p[56 + 2 * c]);
		A64Mon_Say("lun %d #%d %S %d..%d %d MB", aLun, i, &name, (TInt)first, (TInt)last,
			(TInt)((last - first + 1) * l.iBlockSize >> 20));
		}
	}

static TBool Is(const TDesC8& aWord, const char* aLit)
	{
	TInt n = 0;
	while (aLit[n])
		++n;
	return aWord.Length() == n && memcompare(aWord.Ptr(), n, (const TUint8*)aLit, n) == 0;
	}

TInt A64BlkDev_Monitor(TInt aArgc, const TPtrC8* aArg, const TUint64* aNum, TDes8& aOut)
	{
	if (InitResult != KErrNone)
		{
		A64Device_Format(aOut, "ufs: not up (%d; blk init)", InitResult);
		return KErrNotReady;
		}
	if (Is(aArg[1], "luns"))
		{
		TLock lock;
		for (TInt lun = 0; lun < KLuns; ++lun)
			if (Luns[lun].iBlocks)
				ListGpt(lun);
		aOut.Append((const TUint8*)"ufs: luns done", 14);
		return KErrNone;
		}
	if (Is(aArg[1], "lun") && aArgc >= 3)
		{
		TInt lun = (TInt)aNum[2];
		if (lun < 0 || lun >= KLuns || !Luns[lun].iBlocks)
			return KErrNotFound;
		Cur = lun;
		A64Device_Format(aOut, "ufs: blk read/dump use lun %d", lun);
		return KErrNone;
		}
	if (Is(aArg[1], "speed"))						// 64 MB from LUN 0's start, raw (no copy out of Data)
		{
		TLock lock;
		const TUint32 n = KXfer / Luns[0].iBlockSize, total = (64u << 20) / Luns[0].iBlockSize;
		TInt r = KErrNone;
		TUint32 t0 = NKern::TickCount();
		for (TUint32 b = 0; b < total && r == KErrNone; b += n)
			r = ReadBlocks(0, b, n);
		TInt ms = Max(1, (TInt)((NKern::TickCount() - t0) * NKern::TickPeriod() / 1000));
		A64Device_Format(aOut, "ufs: read 64 MB in %d ms (%d KB/s)", ms, 65536 * 1000 / ms);
		return r;
		}
	aOut.Append((const TUint8*)"blk luns | blk lun <n> | blk speed", 34);
	return KErrArgument;
	}

static TA64DevState UfsStatus(TDes8& aDetail)
	{
	if (InitResult == KErrNotReady && !Desc)
		{
		A64Device_Format(aDetail, "read-only; not started (blk init)");
		return EA64DevOff;
		}
	if (InitResult != KErrNone)
		{
		A64Device_Format(aDetail, "init failed: %d", InitResult);
		return EA64DevError;
		}
	TInt luns = 0;
	for (TInt i = 0; i < KLuns; ++i)
		luns += Luns[i].iBlocks != 0;
	A64Device_Format(aDetail, "read-only, %d LUNs, ABL's link adopted, polled", luns);
	return EA64DevPartial;
	}
A64_DEVICE("ufs", "Storage (UFS)", "qcom_ufs", UfsStatus);
