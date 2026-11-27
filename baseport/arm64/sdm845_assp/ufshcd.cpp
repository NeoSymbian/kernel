// SPDX-License-Identifier: Apache-2.0
//
// ufshcd.cpp -- the UFS host core: UFSHCI 2.1 (JESD223), UniPro (UIC) commands, UPIUs (JESD220), the LUNs, and the
// block backend (a64_blk.h). It follows the design and order of Linux's ufshcd.c (docs fajita/ufs-linux-reference.md),
// written anew; the SoC parts are the vendor hooks (ufshcd.h, qcom_ufs.cpp).
// - Two ways up: Adopt() takes over the host and link the bootloader leaves running ("blk init"); FullInit() brings
//   them up itself, from the core reset on ("blk reset"). Gear() then moves the link to HS-G3, 2 lanes ("blk gear").
// - Writes (EMMC_C builds) reach only system_b on LUN 0, checked twice: A64BlkDev_Write against the partition, and
//   Build() against the WRITE(10)'s own LUN and blocks. Every other opcode outside the allow-list is refused; the
//   only query that writes is SET_FLAG fDeviceInit.
// - The block layer's sectors are 512 bytes; the LUNs' blocks are 4096: reads go through bounce buffers, writes of
//   part of a block read it first.
// - Requests: block reads keep up to KQueue slots in flight, each with its own 256 KB buffer; everything else runs
//   in slot 0, one at a time. Completion is the host's interrupt (SPI 265) or, with "blk irq off", polling.
#include <kernel/kern_priv.h>
#include <kernel/cache.h>
#include "board.h"
#include "a64_blk.h"
#include "a64_dbgmon.h"
#include "a64_device.h"
#include "a64_platform.h"
#include <sha256.h>
#include "ufshcd.h"

const TUint32 KHc = 0x01d84000;
// UFSHCI registers
enum { ECap = 0x00, EVer = 0x08, EIs = 0x20, EIe = 0x24, EHcs = 0x30, EHce = 0x34, EUecpa = 0x38, EUecdl = 0x3c,
	EUecn = 0x40, EUect = 0x44, EUecdme = 0x48, EUtriacr = 0x4c, EUtrlba = 0x50, EUtrlbau = 0x54, EUtrldbr = 0x58,
	EUtrlclr = 0x5c, EUtrlrsr = 0x60, EUtmrlba = 0x70, EUtmrlbau = 0x74, EUtmrldbr = 0x78, EUtmrlrsr = 0x80,
	EUicCmd = 0x90, EUicArg1 = 0x94, EUicArg2 = 0x98, EUicArg3 = 0x9c };
enum { EHcsDp = 1, EHcsUtrlRdy = 2, EHcsUtmrlRdy = 4, EHcsUcRdy = 8 };
enum { EIsUtrcs = 1, EIsUe = 4, EIsUpms = 0x10, EIsUccs = 0x400, EIsErrors = 0x71a84 };
// UIC opcodes and UniPro attributes
enum { EDmeGet = 0x01, EDmeSet = 0x02, EDmePeerGet = 0x03, EDmePeerSet = 0x04, EDmeLinkStartup = 0x16 };
enum { EPaActiveTxLanes = 0x1560, EPaConnectedTxLanes = 0x1561, EPaTxGear = 0x1568, EPaTxTermination = 0x1569,
	EPaHsSeries = 0x156a, EPaPwrMode = 0x1571, EPaActiveRxLanes = 0x1580, EPaConnectedRxLanes = 0x1581,
	EPaRxGear = 0x1583, EPaRxTermination = 0x1584, EPaMaxRxHsGear = 0x1587, EPaPwrModeUserData0 = 0x15b0,
	EDmeFc0Protection = 0xd041, EDmeTc0Replay = 0xd042, EDmeAfc0Req = 0xd043 };

volatile TUint32& Ufs_Reg(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KHc) + aOff); }
#define R Ufs_Reg

// one request at a time: the monitor, and later the media driver, share the slot and the buffers
static DMutex* Lock;
const TUint8 KMutexOrdUfs = 0xe0;
class TLock
	{
public:
	TLock() { NKern::ThreadEnterCS(); if (Lock) Kern::MutexWait(*Lock); }
	~TLock() { if (Lock) Kern::MutexSignal(*Lock); NKern::ThreadLeaveCS(); }
	};

// memory, non-cached: the transfer list (32 slots x 32 bytes, 1 KB aligned) at 0, the task list at 0x400, then a
// command descriptor per slot from 0x800, KUcdSize apart (command UPIU, response UPIU at +0x200, PRDT at +0x400);
// and a cached data buffer per queued slot
const TInt KSlots = 32, KXfer = 256 * 1024;			// bytes per request: one PRDT entry (its 18-bit count)
const TInt KQueue = 8;									// slots in flight for reads (2 MB of buffers)
const TUint32 KUcd0 = 0x800, KUcdSize = 0x480, KResp = 0x200, KPrdt = 0x400;
const TUint32 KDescBytes = KUcd0 + KSlots * KUcdSize;	// 37 KB
static TPhysAddr DescPhys, DataPhys;
static volatile TUint32* Desc;
static TUint8* Data;

// EMMC_C=1: C: (and E:) on system_b at boot; EMMC_C=2: writes from the monitor only (C: stays the RAM drive);
// unset: read-only. The name is the Pixel's flag, which build/neo already passes through.
#ifdef A64_EMMC_C
const TBool KWritable = ETrue;
const TBool KBootDrive = A64_EMMC_C == 1;
#else
const TBool KWritable = EFalse;
const TBool KBootDrive = EFalse;
#endif
const TInt KWriteLun = 0;								// system_b's LUN: writes never follow "blk lun"

// LUN 0's partitions by GPT name, each found exactly once, in 512-byte sectors; only system_b is writable (the
// owner's choice, 2026-11-26: slot b's 1 GB, its contents given up). The others: "blk hash" before and after writes
struct TPart { const char* iName; TBool iWritable; TInt iFound; TUint64 iFirst, iLast; };
static TPart Parts[] = { { "system_b", ETrue }, { "system_a", EFalse }, { "userdata", EFalse } };
const TInt KParts = sizeof(Parts) / sizeof(Parts[0]);

static TBool WriteAllowed(TUint64 aLba, TInt aCount)
	{
	const TPart& p = Parts[0];
	return KWritable && p.iFound == 1 && aCount > 0 && aLba >= p.iFirst && aLba + aCount - 1 <= p.iLast;
	}

struct TLun { TUint32 iBlocks, iBlockSize; };
const TInt KLuns = 8;
const TInt KWlunDevice = 0xd0;					// the UFS device well-known LUN (UPIU LUN field)
static TLun Luns[KLuns];
static TInt Cur;										// the LUN A64BlkDev_Read reads ("blk lun N")
static TInt InitResult = KErrNotReady;
static TBool Ours;										// brought up by FullInit (not adopted)
static TUint32 Vendor, Spec;							// the device descriptor's manufacturer ID and spec version
static TBuf8<200> Report[32];
static TInt NReport;
static TUint32 UicErrors, FatalErrors, Timeouts;

// completion by interrupt: the ISR masks the level line and adds an IDFC that wakes the waiting thread; the thread
// clears IS.UTRCS and unmasks (as qcom_wcn3990.cpp's copy engines). A 10-ms timer bounds every wait.
const TUint32 KIntId = 32 + 265;						// DTB ufshc@1d84000: GIC_SPI 265, level high
static TBool IrqBound, IrqWanted = ETrue, IrqOn;
static TUint32 Irqs, Waits;
static NFastSemaphore DoneSem;
static void DoneIdfcFn(TAny*) { DoneSem.Signal(); }	// IDFC: the kernel is locked
static TDfc DoneIdfc(DoneIdfcFn, NULL);
static void DoneTimerFn(TAny*) { DoneIdfc.Add(); }
static NTimer DoneTimer(DoneTimerFn, NULL);
static void UfsIsr(TAny*)
	{
	A64_MaskSpi(KIntId, ETrue);
	++Irqs;
	DoneIdfc.Add();
	}

void Ufs_Log(const char* aFmt, ...)
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
#define Log Ufs_Log

void Ufs_Delay(TInt aUs)
	{
	if (aUs < 1000)
		Kern::NanoWait(aUs * 1000);
	else
		NKern::Sleep(NKern::TimerTicks((aUs + 999) / 1000));
	}

// poll until (reg & mask) == want, every aStepUs for up to aUs
static TBool Poll(TUint32 aOff, TUint32 aMask, TUint32 aWant, TInt aUs, TInt aStepUs)
	{
	for (TInt t = 0; ; t += aStepUs)
		{
		if ((R(aOff) & aMask) == aWant)
			return ETrue;
		if (t >= aUs)
			return EFalse;
		Ufs_Delay(aStepUs);
		}
	}

static TUint32 Be16(const TUint8* p) { return (TUint32)p[0] << 8 | p[1]; }
static TUint32 Be32(const TUint8* p) { return (TUint32)p[0] << 24 | (TUint32)p[1] << 16 | (TUint32)p[2] << 8 | p[3]; }
static void PutBe32(TUint8* p, TUint32 v) { p[0] = (TUint8)(v >> 24); p[1] = (TUint8)(v >> 16); p[2] = (TUint8)(v >> 8); p[3] = (TUint8)v; }

// the error status bits, counted and cleared (UE: the UniPro layer's error code registers, read to clear)
static void TakeErrors()
	{
	TUint32 is = R(EIs) & EIsErrors;
	if (!is)
		return;
	R(EIs) = is;
	if (is & EIsUe)
		{
		TUint32 pa = R(EUecpa), dl = R(EUecdl), n = R(EUecn), t = R(EUect), dme = R(EUecdme);
		if (++UicErrors <= 8)
			Log("uic error: pa %08x dl %08x n %08x t %08x dme %08x", pa, dl, n, t, dme);
		}
	if (is & ~EIsUe)
		{
		if (++FatalErrors <= 8)
			Log("error status %08x, hcs %08x", is, R(EHcs));
		}
	}

// one UIC command: KErrNone and ARG3 (the value of a DME_GET), or an error. A power-mode change completes on
// IS.UPMS instead of IS.UCCS (aPwrMode)
static TInt Uic(TUint32 aCmd, TUint32 aArg1, TUint32 aArg3, TUint32& aOut, TBool aPwrMode = EFalse, TInt aUs = 500000)
	{
	if (!Poll(EHcs, EHcsUcRdy, EHcsUcRdy, 500000, 500))
		return KErrNotReady;
	R(EIs) = EIsUccs | EIsUpms;
	R(EUicArg1) = aArg1;
	R(EUicArg2) = 0;
	R(EUicArg3) = aArg3;
	R(EUicCmd) = aCmd;
	const TUint32 done = aPwrMode ? EIsUpms : EIsUccs;
	if (!Poll(EIs, done, done, aUs, aUs > 10000 ? 1000 : 10))
		{
		Log("uic %02x %08x timed out: is %08x", aCmd, aArg1, R(EIs));
		return KErrTimedOut;
		}
	TUint32 result = R(EUicArg2) & 0xff;
	aOut = R(EUicArg3);
	R(EIs) = EIsUccs | EIsUpms;
	if (result)
		{
		if (aCmd != EDmeGet || aArg1 >> 16 != 0x0041)		// a TX FSM poll may fail while the PHY settles
			Log("uic %02x %08x: result %d", aCmd, aArg1, result);
		return KErrGeneral;
		}
	return KErrNone;
	}

TInt Ufs_DmeGet(TUint32 aAttr, TUint32 aSel, TUint32& aVal, TBool aPeer)
	{
	return Uic(aPeer ? EDmePeerGet : EDmeGet, aAttr << 16 | aSel, 0, aVal);
	}

TInt Ufs_DmeSet(TUint32 aAttr, TUint32 aSel, TUint32 aVal, TBool aPeer)
	{
	TUint32 ignored;
	return Uic(aPeer ? EDmePeerSet : EDmeSet, aAttr << 16 | aSel, aVal, ignored);
	}

static TInt DmeGetOr(TUint32 aAttr)
	{
	TUint32 v;
	return Ufs_DmeGet(aAttr, 0, v) == KErrNone ? (TInt)v : -1;
	}

// ---- requests (slot 0)

static TUint8* Ucd(TInt aSlot = 0) { return (TUint8*)Desc + KUcd0 + aSlot * KUcdSize; }
static TUint8* Buf(TInt aSlot = 0) { return Data + aSlot * KXfer; }

// the transfer request in slot aSlot (its UCD filled), rung
static void Start(TInt aSlot, TUint32 aDd, TUint32 aBytes)
	{
	volatile TUint32* utrd = Desc + aSlot * 8;
	if (aBytes)
		{
		volatile TUint32* prd = (volatile TUint32*)(Ucd(aSlot) + KPrdt);
		prd[0] = (TUint32)DataPhys + aSlot * KXfer;
		prd[1] = 0;
		prd[2] = 0;
		prd[3] = aBytes - 1;
		if (aDd == 1)
			Cache::SyncMemoryBeforeDmaWrite((TLinAddr)Buf(aSlot), aBytes);
		else
			Cache::SyncMemoryBeforeDmaRead((TLinAddr)Buf(aSlot), aBytes);
		}
	// UTRD: command type 1 (UFS storage), data direction, interrupt on completion
	TUint32 ucd = (TUint32)DescPhys + KUcd0 + aSlot * KUcdSize;
	utrd[0] = (1u << 28) | (aDd << 25) | (1u << 24);
	utrd[1] = 0;
	utrd[2] = 0x0f;										// OCS: invalid until the host writes it
	utrd[3] = 0;
	utrd[4] = ucd;
	utrd[5] = 0;
	utrd[6] = ((KResp / 4) << 16) | (KResp / 4);		// response UPIU offset, length (dwords)
	utrd[7] = ((KPrdt / 4) << 16) | (aBytes ? 1 : 0);	// PRDT offset (dwords), entries
	__e32_memory_barrier();
	R(EUtrldbr) = 1u << aSlot;
	}

// until the doorbell bits aMask are clear or aUs passed: the bits still set (cleared from the list, counted)
static TUint32 WaitSlots(TUint32 aMask, TInt aUs, const char* aWhat, TInt aCode)
	{
	const TUint32 t0 = NKern::TickCount(), limit = aUs / NKern::TickPeriod() + 2;
	TUint32 left;
	while ((left = R(EUtrldbr) & aMask) != 0 && NKern::TickCount() - t0 < limit)
		{
		if (!IrqOn)
			{
			Ufs_Delay(10);
			continue;
			}
		DoneSem.SetOwner(NULL);							// this thread (the monitor's or the file server's)
		DoneTimer.OneShot(NKern::TimerTicks(10));
		NKern::FSWait(&DoneSem);
		DoneTimer.Cancel();
		++Waits;
		R(EIs) = EIsUtrcs;								// before the doorbell is read again: a later completion re-raises it
		A64_MaskSpi(KIntId, EFalse);
		}
	if (left)
		{
		R(EUtrlclr) = ~left;							// writing 0 clears a slot
		Poll(EUtrldbr, left, 0, 1000000, 100);
		++Timeouts;
		Log("%s %02x timed out: slots %08x is %08x hcs %08x", aWhat, aCode, left, R(EIs), R(EHcs));
		}
	if (!IrqOn)
		R(EIs) = EIsUtrcs;
	TakeErrors();
	return left;
	}

// a completed slot: KErrNone and the response UPIU's transaction code checked (aBytes: read data to sync)
static TInt Finish(TInt aSlot, TUint32 aBytes, TUint8 aRespCode, const char* aWhat, TInt aCode)
	{
	if (aBytes)
		Cache::SyncMemoryAfterDmaRead((TLinAddr)Buf(aSlot), aBytes);
	const TUint8* resp = Ucd(aSlot) + KResp;
	TUint32 ocs = Desc[aSlot * 8 + 2] & 0xff;
	// a SCSI command's outcome is its status byte (Linux reads only that): with CHECK CONDITION this device also sets
	// the response field to "target failure"
	if (ocs != 0 || resp[0] != aRespCode || (aRespCode != 0x21 && resp[6] != 0))
		{
		Log("%s %02x: ocs %x, response %02x/%02x status %02x; sense %02x %02x %02x", aWhat, aCode, ocs, resp[0],
			resp[6], resp[7], resp[34 + 2], resp[34 + 12], resp[34 + 13]);
		return KErrGeneral;
		}
	return KErrNone;
	}

// slot 0, prepared, issued and waited for
static TInt Issue(TUint32 aDd, TUint32 aBytes, TUint8 aRespCode, const char* aWhat, TInt aCode, TInt aUs = 1000000)
	{
	Start(0, aDd, aBytes);
	if (WaitSlots(1, aUs, aWhat, aCode))
		return KErrTimedOut;
	return Finish(0, aDd == 2 ? aBytes : 0, aRespCode, aWhat, aCode);
	}

// the host's interrupt for transfer completions on (bound once, IE.UTRCE) or off (polled)
static void SetIrq(TBool aOn)
	{
	if (aOn && !IrqBound)
		IrqBound = A64_BindSpi(KIntId, UfsIsr, NULL) == KErrNone;
	IrqOn = aOn && IrqBound;
	R(EIs) = EIsUtrcs;
	R(EIe) = IrqOn ? EIsUtrcs : 0;
	if (IrqBound)
		A64_MaskSpi(KIntId, !IrqOn);
	}

static TInt Nop()
	{
	memclr(Ucd(), KPrdt + 16);								// NOP OUT: all zero (tag 0)
	return Issue(0, 0, 0x20, "nop", 0);
	}

// a QUERY REQUEST: read a descriptor (into Data), an attribute or a flag (aValue); the only write is SET_FLAG
// fDeviceInit (idn 1)
enum { EQReadDesc = 1, EQReadAttr = 3, EQReadFlag = 5, EQSetFlag = 6 };
static TInt Query(TUint8 aOp, TUint8 aIdn, TUint8 aIndex, TUint32& aValue)
	{
	if (aOp != EQReadDesc && aOp != EQReadAttr && aOp != EQReadFlag && !(aOp == EQSetFlag && aIdn == 1))
		return KErrAccessDenied;
	TUint8* ucd = Ucd();
	memclr(ucd, KPrdt + 16);
	ucd[0] = 0x16;
	ucd[5] = aOp == EQSetFlag ? 0x81 : 0x01;				// standard write / read request
	ucd[12] = aOp;
	ucd[13] = aIdn;
	ucd[14] = aIndex;
	if (aOp == EQReadDesc)
		ucd[19] = 0xff;									// length: up to 255 bytes
	TInt r = Issue(0, 0, 0x36, "query", aOp << 4 | (aIdn & 0xf), 1500000);
	if (r != KErrNone)
		return r;
	const TUint8* resp = ucd + KResp;
	if (aOp == EQReadDesc)
		{
		aValue = Be16(resp + 10);							// the descriptor's length, data from resp + 32
		memcpy(Data, resp + 32, Min(aValue, (TUint32)255));
		}
	else
		aValue = aOp == EQReadAttr ? Be32(resp + 20) : resp[23] & 1;
	return KErrNone;
	}

// a SCSI command's UPIU in slot aSlot's UCD, if the allow-list lets it through
static TInt Build(TInt aSlot, TInt aLun, const TUint8* aCdb, TUint32 aBytes)
	{
	switch (aCdb[0])
		{
	case 0x00:	// TEST UNIT READY
	case 0x03:	// REQUEST SENSE
	case 0x12:	// INQUIRY
	case 0x25:	// READ CAPACITY(10)
	case 0x28:	// READ(10)
		break;
	case 0x2a:	// WRITE(10): on LUN 0, inside the writable partition, the length matching the blocks
		{
		const TUint32 bs = Luns[KWriteLun].iBlockSize, per = bs / KA64BlkSector;
		if (aLun == KWriteLun && per && aBytes == Be16(aCdb + 7) * bs &&
			WriteAllowed((TUint64)Be32(aCdb + 2) * per, (TInt)(Be16(aCdb + 7) * per)))
			break;
		return KErrAccessDenied;
		}
	case 0x1b:	// START STOP UNIT: only the UFS device W-LUN's power condition (no media access)
		if (aLun == KWlunDevice)
			break;
		return KErrAccessDenied;
	default:
		return KErrAccessDenied;							// the no-write rule, in one place
		}
	if (aBytes > (TUint32)KXfer)
		return KErrArgument;
	TUint8* ucd = Ucd(aSlot);
	memclr(ucd, KPrdt + 16);
	ucd[0] = 0x01;										// COMMAND UPIU
	ucd[1] = !aBytes ? 0 : aCdb[0] == 0x2a ? 0x20 : 0x40;	// W: data out, R: data in; simple task
	ucd[2] = (TUint8)aLun;
	ucd[3] = (TUint8)aSlot;								// task tag
	PutBe32(ucd + 12, aBytes);							// expected data transfer length
	memcpy(ucd + 16, aCdb, 16);
	return KErrNone;
	}

// a SCSI command in slot 0; aSense = key << 16 | asc << 8 | ascq after a check condition
static TInt Command(TInt aLun, const TUint8* aCdb, TUint32 aBytes, TInt& aStatus, TUint32& aSense)
	{
	aStatus = 0;
	aSense = 0;
	TInt r = Build(0, aLun, aCdb, aBytes);
	if (r == KErrNone)
		r = Issue(!aBytes ? 0 : aCdb[0] == 0x2a ? 1 : 2, aBytes, 0x21, "cdb", aCdb[0]);
	if (r != KErrNone)
		return r;
	const TUint8* resp = Ucd() + KResp;
	aStatus = resp[7];
	if (aStatus == 2 && Be16(resp + 32) >= 14)			// fixed-format sense data after its length
		aSense = (resp[34 + 2] & 0xf) << 16 | resp[34 + 12] << 8 | resp[34 + 13];
	return KErrNone;
	}

// with up to three retries on a check condition (UNIT ATTENTION after a reset, ...)
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

// bytes [aFrom, aFrom + aBytes) of a LUN into aBuf (NULL: read only), up to KQueue READ(10)s in flight. A slot that
// fails (a check condition, a timeout) is read again alone through Scsi()'s retries.
static TInt ReadSpan(TInt aLun, TUint64 aFrom, TUint32 aBytes, TUint8* aBuf)
	{
	const TUint32 bs = Luns[aLun].iBlockSize, per = KXfer / bs;
	TUint32 b = (TUint32)(aFrom / bs);
	const TUint32 end = (TUint32)((aFrom + aBytes + bs - 1) / bs);
	while (b < end)
		{
		const TUint32 first = b;
		TInt q = 0;
		for (; q < KQueue && b < end; ++q, b += per)
			{
			TUint32 n = Min(end - b, per);
			TUint8 cdb[16] = { 0x28 };
			PutBe32(cdb + 2, b);
			cdb[7] = (TUint8)(n >> 8);
			cdb[8] = (TUint8)n;
			TInt r = Build(q, aLun, cdb, n * bs);
			if (r != KErrNone)
				return r;
			Start(q, 2, n * bs);
			}
		TUint32 left = WaitSlots((1u << q) - 1, 1000000, "read", q);
		b = first;
		for (TInt s = 0; s < q; ++s, b += per)
			{
			TUint32 n = Min(end - b, per);
			const TUint8* src = Buf(s);
			if ((left >> s & 1) || Finish(s, n * bs, 0x21, "read", s) != KErrNone || Ucd(s)[KResp + 7] != 0)
				{
				TInt r = ReadBlocks(aLun, b, n);			// slot 0's buffer: slot 0 is copied out already
				if (r != KErrNone)
					return r;
				src = Buf(0);
				}
			if (!aBuf)
				continue;
			TUint64 lo = Max((TUint64)b * bs, aFrom), hi = Min((TUint64)(b + n) * bs, aFrom + aBytes);
			memcpy(aBuf + (TUint32)(lo - aFrom), src + (TUint32)(lo - (TUint64)b * bs), (TUint32)(hi - lo));
			}
		}
	return KErrNone;
	}

// ---- bring-up

static TInt Prepare()
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
	if (!Desc)
		{
		NKern::ThreadEnterCS();
		TInt r = Epoc::AllocPhysicalRam(KDescBytes, DescPhys, 12);
		if (r == KErrNone)
			r = Epoc::AllocPhysicalRam(KQueue * KXfer, DataPhys, 12);
		DPlatChunkHw* d = NULL;
		DPlatChunkHw* c = NULL;
		if (r == KErrNone)
			r = DPlatChunkHw::New(d, DescPhys, KDescBytes, EMapAttrSupRw | EMapAttrBufferedNC);
		if (r == KErrNone)
			r = DPlatChunkHw::New(c, DataPhys, KQueue * KXfer, EMapAttrSupRw | EMapAttrCachedMax);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return r;
		Desc = (volatile TUint32*)d->LinearAddress();
		Data = (TUint8*)c->LinearAddress();
		memclr((TAny*)Desc, KDescBytes);
		}
	return KErrNone;
	}

// our lists in place of whatever is there: stop both, point them at our memory, start (HCS must say ready)
static TInt MakeOperational()
	{
	SetIrq(EFalse);
	R(EUtriacr) = 0;
	R(EUtrlrsr) = 0;
	R(EUtmrlrsr) = 0;
	R(EUtrlba) = (TUint32)DescPhys;
	R(EUtrlbau) = 0;
	R(EUtmrlba) = (TUint32)DescPhys + 0x400;
	R(EUtmrlbau) = 0;
	if ((R(EHcs) & 0xe) != 0xe)
		{
		Log("lists not ready: hcs %08x", R(EHcs));
		return KErrNotReady;
		}
	R(EUtmrlrsr) = 1;
	R(EUtrlrsr) = 1;
	SetIrq(IrqWanted);
	return KErrNone;
	}

// HCE off, the vendor's pre-change (core reset, PHY), HCE on, the vendor's post-change (Linux ufshcd_hba_enable)
static TInt HbaEnable()
	{
	if (R(EHce) & 1)
		{
		R(EHce) = 0;
		if (!Poll(EHce, 1, 0, 1000, 10))
			return KErrTimedOut;
		}
	TInt r = UfsVop_HcePre();
	if (r != KErrNone)
		{
		Log("vendor pre-HCE -> %d", r);
		return r;
		}
	R(EHce) = 1;
	Ufs_Delay(1000);
	if (!Poll(EHce, 1, 1, 50000, 1000))
		{
		Log("HCE stays 0");
		return KErrTimedOut;
		}
	r = UfsVop_HcePost();
	if (r != KErrNone)
		Log("vendor post-HCE -> %d", r);
	return r;
	}

static TInt LinkStartup()
	{
	TInt r = KErrGeneral;
	for (TInt k = 0; k < 4; ++k)
		{
		if (k && (r = HbaEnable()) != KErrNone)				// a failed start-up starts over from HCE
			continue;
		if ((r = UfsVop_LinkPre()) != KErrNone)
			continue;
		TUint32 v;
		r = Uic(EDmeLinkStartup, 0, 0, v);
		if (r == KErrNone && !(R(EHcs) & EHcsDp))
			{
			Log("link up but no device (hcs %08x)", R(EHcs));
			return KErrNotFound;
			}
		if (r == KErrNone)
			break;
		Log("link start-up try %d -> %d", k + 1, r);
		}
	if (r != KErrNone)
		return r;
	Log("link up: connected lanes tx %d rx %d", DmeGetOr(EPaConnectedTxLanes), DmeGetOr(EPaConnectedRxLanes));
	r = UfsVop_LinkPost();
	(void)R(EUecpa);										// clear the line reset the start-up leaves
	R(EIs) = EIsUe;
	return r;
	}

// every LUN that answers TEST UNIT READY and READ CAPACITY(10); the device's name
static TInt ScanLuns()
	{
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

// LUN 0's GPT (in its own block size): the allow-list's partitions, in 512-byte sectors
static void FindParts()
	{
	for (TInt p = 0; p < KParts; ++p)
		Parts[p].iFound = 0;
	const TLun& l = Luns[KWriteLun];
	if (!l.iBlocks || ReadBlocks(KWriteLun, 1, 1) != KErrNone || memcompare(Data, 8, (const TUint8*)"EFI PART", 8) != 0)
		{
		Log("lun %d: no GPT, nothing writable", KWriteLun);
		return;
		}
	TUint32 lba = (TUint32)*(TUint64*)(Data + 72), n = *(TUint32*)(Data + 80), size = *(TUint32*)(Data + 84);
	if (size < 128 || l.iBlockSize % size)
		return;
	const TUint32 per = l.iBlockSize / size, sectors = l.iBlockSize / KA64BlkSector;
	for (TUint32 i = 0; i < n && i < 256; ++i)
		{
		if (i % per == 0 && ReadBlocks(KWriteLun, lba + i / per, 1) != KErrNone)
			{
			for (TInt p = 0; p < KParts; ++p)
				Parts[p].iFound = 0;
			return;
			}
		const TUint8* e = Data + (i % per) * size;
		for (TInt p = 0; p < KParts; ++p)
			{
			const char* name = Parts[p].iName;
			TBool match = ETrue;
			for (TInt c = 0; match; ++c)					// the letters and the terminating 0, as UTF-16
				{
				match = e[56 + 2 * c] == (TUint8)name[c] && e[57 + 2 * c] == 0;
				if (!name[c])
					break;
				}
			TUint64 first = *(const TUint64*)(e + 32), last = *(const TUint64*)(e + 40);
			if (match && ++Parts[p].iFound == 1)
				{
				Parts[p].iFirst = first * sectors;
				Parts[p].iLast = (last + 1) * sectors - 1;
				if (first < 6 || last < first || last >= l.iBlocks)	// past the GPT, inside the LUN
					Parts[p].iFound = 99;
				}
			}
		}
	for (TInt p = 0; p < KParts; ++p)
		Log("allow-list: %s sectors %d..%d (found %d), %s", Parts[p].iName, (TInt)Parts[p].iFirst,
			(TInt)Parts[p].iLast, Parts[p].iFound, !Parts[p].iWritable ? "read-only" : WriteAllowed(Parts[p].iFirst, 1) ?
			(KBootDrive ? "writable, C: + E:" : "writable (monitor only)") : "not writable");
	}

// the device descriptor: manufacturer (for the vendor's quirks), spec version, LUN count
static void ReadDevice()
	{
	TUint32 len = 0, ref = 0;
	if (Query(EQReadDesc, 0, 0, len) == KErrNone && len >= 0x1a)
		{
		Vendor = Be16(Data + 0x18);
		Spec = Be16(Data + 0x10);
		Log("device descriptor: manufacturer %04x, UFS %x.%02x, %d LUNs, queue depth %d", Vendor, Spec >> 8,
			Spec & 0xff, Data[0x06], Data[0x21]);
		}
	if (Query(EQReadAttr, 0x0a, 0, ref) == KErrNone)		// bRefClkFreq: 0 = 19.2 MHz
		Log("bRefClkFreq %d", ref);
	TUint32 mode = 0;
	if (Query(EQReadAttr, 0x02, 0, mode) == KErrNone)		// bCurrentPowerMode: 0x11 active, 0x22 sleep
		Log("bCurrentPowerMode %02x (bInitPowerMode %02x)", mode, len >= 0x0b ? Data[0x0a] : 0xff);
	}

// after a reset: REQUEST SENSE clears the device W-LUN's unit attention, START STOP UNIT (power condition active)
// wakes the device if bInitPowerMode left it asleep (Linux: ufshcd_set_dev_pwr_mode)
static void WakeDevice()
	{
	TInt status;
	TUint32 sense;
	const TUint8 rs[16] = { 0x03, 0, 0, 0, 20 };
	TInt r = Command(KWlunDevice, rs, 20, status, sense);
	Log("device w-lun: request sense -> %d status %d, key %x asc %02x", r, status, Data[2] & 0xf, Data[12]);
	TUint32 mode = 0;
	if (Query(EQReadAttr, 0x02, 0, mode) == KErrNone && mode == 0x11)
		return;												// active already
	const TUint8 ssu[16] = { 0x1b, 0, 0, 0, 0x10 };			// power condition 1: active
	for (TInt k = 0; k < 3; ++k)
		{
		r = Command(KWlunDevice, ssu, 0, status, sense);
		if (r != KErrNone || status != 2)
			break;
		}
	Query(EQReadAttr, 0x02, 0, mode);
	Log("device w-lun: start stop unit (active) -> %d status %d sense %06x; power mode now %02x", r, status, sense, mode);
	}

static TInt Adopt()
	{
	if (!UfsVop_Clocked())
		{
		Log("the host is unclocked: nothing to adopt (blk reset brings it up)");
		return KErrNotReady;
		}
	TUint32 hcs = R(EHcs);
	if (!(R(EHce) & 1) || (hcs & (EHcsDp | EHcsUtrlRdy)) != (EHcsDp | EHcsUtrlRdy))
		{
		Log("no link to adopt: hce %08x hcs %08x", R(EHce), hcs);
		return KErrNotReady;
		}
	Log("adopting: utrl %08x run %d bell %08x; utmrl %08x run %d; ie %08x", R(EUtrlba), R(EUtrlrsr), R(EUtrldbr),
		R(EUtmrlba), R(EUtmrlrsr), R(EIe));
	TInt r = UfsVop_PowerUp();								// our votes, so nothing ABL left can lapse
	if (r == KErrNone)
		r = MakeOperational();
	if (r == KErrNone)
		r = Nop();
	if (r == KErrNone)
		ReadDevice();
	if (r == KErrNone)
		r = ScanLuns();
	Ours = EFalse;
	if (r == KErrNone)
		FindParts();
	return r;
	}

// our own bring-up, from the core reset on (Linux: ufshcd_hba_init, ufshcd_hba_enable, ufshcd_link_startup,
// ufshcd_verify_dev_init, ufshcd_complete_dev_init)
static TInt FullInit()
	{
	Log("full init: power");
	TInt r = UfsVop_PowerUp();
	if (r != KErrNone)
		return r;
	Log("full init: hw %08x cap %08x ver %08x", R(0xe4), R(ECap), R(EVer));
	SetIrq(EFalse);
	R(EIs) = R(EIs);
	UfsVop_DeviceReset();
	Log("full init: HCE");
	if ((r = HbaEnable()) != KErrNone)
		return r;
	Log("full init: link");
	if ((r = LinkStartup()) != KErrNone)
		return r;
	if ((r = MakeOperational()) != KErrNone)
		return r;
	for (TInt k = 0; k < 10; ++k)							// the device may need a moment after the link
		if ((r = Nop()) == KErrNone)
			break;
	Log("full init: nop -> %d", r);
	if (r != KErrNone)
		return r;
	TUint32 flag = 1;
	r = Query(EQSetFlag, 1, 0, flag);						// fDeviceInit, then wait for the device to clear it
	for (TInt t = 0; r == KErrNone && t < 1500; t += 2)
		{
		if ((r = Query(EQReadFlag, 1, 0, flag)) != KErrNone || !flag)
			break;
		Ufs_Delay(2000);
		}
	Log("full init: fDeviceInit -> %d (flag %d)", r, flag);
	if (r != KErrNone || flag)
		return r != KErrNone ? r : KErrTimedOut;
	ReadDevice();
	WakeDevice();
	r = ScanLuns();
	Ours = r == KErrNone;
	if (r == KErrNone)
		FindParts();
	return r;
	}

// HS-G3, Rate B, both lanes, fast mode (Linux ufshcd_change_power_mode with ufs-qcom's limits)
static TInt Gear()
	{
	TInt lanes = Min(DmeGetOr(EPaConnectedTxLanes), DmeGetOr(EPaConnectedRxLanes));
	TInt gear = Min(DmeGetOr(EPaMaxRxHsGear), 3);
	TUint32 peerGear = 0;
	if (Ufs_DmeGet(EPaMaxRxHsGear, 0, peerGear, ETrue) == KErrNone)
		gear = Min(gear, (TInt)peerGear);
	if (lanes < 1 || gear < 1)
		{
		Log("gear: lanes %d gear %d", lanes, gear);
		return KErrNotSupported;
		}
	TInt r = UfsVop_GearPre(Vendor);
	const struct { TUint16 iAttr; TUint16 iVal; } set[] = {
		{ EPaRxGear, (TUint16)gear }, { EPaActiveRxLanes, (TUint16)lanes }, { EPaRxTermination, 1 },
		{ EPaTxGear, (TUint16)gear }, { EPaActiveTxLanes, (TUint16)lanes }, { EPaTxTermination, 1 },
		{ EPaHsSeries, 2 },
		{ EPaPwrModeUserData0 + 0, 8191 }, { EPaPwrModeUserData0 + 1, 65535 }, { EPaPwrModeUserData0 + 2, 32767 },
		{ EPaPwrModeUserData0 + 3, 8191 }, { EPaPwrModeUserData0 + 4, 65535 }, { EPaPwrModeUserData0 + 5, 32767 },
		{ EDmeFc0Protection, 8191 }, { EDmeTc0Replay, 65535 }, { EDmeAfc0Req, 32767 } };
	for (TUint i = 0; r == KErrNone && i < sizeof(set) / sizeof(set[0]); ++i)
		r = Ufs_DmeSet(set[i].iAttr, 0, set[i].iVal);
	if (r != KErrNone)
		return r;
	TUint32 v;
	r = Uic(EDmeSet, EPaPwrMode << 16, 0x11, v, ETrue);		// rx fast << 4 | tx fast
	TUint32 upmcrs = (R(EHcs) >> 8) & 7;
	Log("gear: HS-G%d x%d -> %d, power mode status %d (1 = done)", gear, lanes, r, upmcrs);
	return r == KErrNone && upmcrs != 1 ? KErrGeneral : r;
	}

// ---- the block backend (a64_blk.h) and the monitor

// at boot only with EMMC_C=1: our own bring-up, HS-G3, and LUN 0 as the device (system_b its window)
TInt A64BlkDev_Init(TUint64& aSectors)
	{
	if (!KBootDrive)
		return KErrNotSupported;
	TInt r = Prepare();
	if (r != KErrNone)
		return InitResult = r;
	TLock lock;
	if ((r = InitResult = FullInit()) != KErrNone)
		return r;
	Log("boot: gear -> %d", Gear());						// a slow link still works
	aSectors = (TUint64)Luns[KWriteLun].iBlocks * (Luns[KWriteLun].iBlockSize / KA64BlkSector);
	return KErrNone;
	}

TInt A64BlkBoard_Window(TUint64, TUint64& aFirst, TUint64& aCount)
	{
	const TPart& p = Parts[0];
	if (!KBootDrive || p.iFound != 1)
		return KErrNotFound;
	aFirst = p.iFirst;
	aCount = p.iLast - p.iFirst + 1;
	return KErrNone;
	}

TUint32 A64BlkDev_EraseBlock() { return Luns[KWriteLun].iBlockSize; }	// FAT's clusters on whole 4 KB blocks

static TInt WriteBlocks(TUint32 aBlock, TUint32 aCount)	// from Buf(0)
	{
	// shortcut: FUA on every write (no flush at shutdown yet); SYNCHRONIZE CACHE at power-down would be faster
	TUint8 cdb[16] = { 0x2a, 0x08 };
	PutBe32(cdb + 2, aBlock);
	cdb[7] = (TUint8)(aCount >> 8);
	cdb[8] = (TUint8)aCount;
	return Scsi(KWriteLun, cdb, aCount * Luns[KWriteLun].iBlockSize);
	}

// 512-byte sectors of LUN 0, inside system_b only; a part-block is read first (one slot at a time)
TInt A64BlkDev_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf)
	{
	if (InitResult != KErrNone)
		return KErrNotReady;
	if (!WriteAllowed(aLba, aCount))					// a second check under a64_blk.cpp's window, whoever asks
		return KErrAccessDenied;
	TLock lock;
	const TUint32 bs = Luns[KWriteLun].iBlockSize;
	TUint64 from = aLba * KA64BlkSector;
	TUint32 bytes = (TUint32)aCount * KA64BlkSector;
	while (bytes)
		{
		const TUint32 b = (TUint32)(from / bs), off = (TUint32)(from % bs);
		const TUint32 n = Min((off + bytes + bs - 1) / bs, (TUint32)KXfer / bs), len = Min(n * bs - off, bytes);
		TInt r = off || len < n * bs ? ReadBlocks(KWriteLun, b, n) : KErrNone;
		if (r != KErrNone)
			return r;
		memcpy(Buf(0) + off, aBuf, len);
		if ((r = WriteBlocks(b, n)) != KErrNone)
			return r;
		from += len;
		aBuf += len;
		bytes -= len;
		}
	return KErrNone;
	}

// the first system_b sector a partition of medblk's MBR uses, or 2048 without one: the sectors before it are free
static TUint64 GapEnd()
	{
	static TUint8 s[KA64BlkSector];
	TUint64 end = 2048;
	if (A64BlkDev_Read(Parts[0].iFirst, 1, s) == KErrNone && s[510] == 0x55 && s[511] == 0xaa && s[0] != 0xeb &&
		s[0] != 0xe9)
		for (TInt i = 0; i < 4; ++i)
			{
			const TUint8* e = s + 446 + 16 * i;
			TUint64 base = e[8] | (e[9] << 8) | (e[10] << 16) | ((TUint32)e[11] << 24);
			if (e[4] && base)
				end = Min(end, base);
			}
	return end;
	}

// "blk wtest": a pattern on system_b sectors 1..8 (part of block 0, all of block 1: both write paths), read back,
// compared, then zeroed
TInt A64BlkDev_SelfTest()
	{
	if (!WriteAllowed(Parts[0].iFirst, 1) || Cur != KWriteLun || GapEnd() < 16)
		return KErrNotReady;
	static TUint8 b[8 * KA64BlkSector], c[8 * KA64BlkSector];
	for (TInt i = 0; i < 8 * KA64BlkSector; ++i)
		b[i] = (TUint8)(i % KA64BlkSector < 8 ? "NEOSYMB"[i % 8] : i / KA64BlkSector + i);
	const TUint64 at = Parts[0].iFirst + 1;
	TInt r = A64BlkDev_Write(at, 8, b);
	Log("wtest: write 8 at %d -> %d", (TInt)at, r);
	if (r == KErrNone)
		r = A64BlkDev_Read(at, 8, c);
	TInt bad = r == KErrNone ? memcompare(b, sizeof(b), c, sizeof(c)) : -1;
	Log("wtest: read back -> %d, %s", r, bad ? "MISMATCH" : "matches");
	if (r != KErrNone || bad)
		return r != KErrNone ? r : KErrCorrupt;
	memclr(b, sizeof(b));
	r = A64BlkDev_Write(at, 8, b);
	Log("wtest: zeroed them again -> %d", r);
	return r;
	}

TInt A64BlkDev_Probe()									// "blk init": adopt the bootloader's link
	{
	NReport = 0;
	TInt r = Prepare();
	if (r != KErrNone)
		return InitResult = r;
	TLock lock;
	return InitResult = Adopt();
	}

TInt A64BlkDev_Read(TUint64 aLba, TInt aCount, TUint8* aBuf)	// 512-byte sectors of the selected LUN
	{
	if (InitResult != KErrNone)
		return KErrNotReady;
	TLock lock;
	const TLun& l = Luns[Cur];
	const TUint32 per = l.iBlockSize / KA64BlkSector;
	if (!l.iBlocks || aCount <= 0 || aLba + aCount > (TUint64)l.iBlocks * per)
		return KErrArgument;
	return ReadSpan(Cur, aLba * KA64BlkSector, aCount * KA64BlkSector, aBuf);
	}

// "blk info" (and Volume Up): the state, read only
void A64BlkDump()
	{
	UfsVop_Dump();
	if (UfsVop_Clocked())
		{
		Kern::Printf("[ufs] host: hw %08x cap %08x ver %08x hcs %08x hce %08x is %08x ie %08x", R(0xe4), R(ECap),
			R(EVer), R(EHcs), R(EHce), R(EIs), R(EIe));
		Kern::Printf("[ufs] lists: utrl %08x:%08x run %d bell %08x; utmrl %08x run %d", R(EUtrlbau), R(EUtrlba),
			R(EUtrlrsr), R(EUtrldbr), R(EUtmrlba), R(EUtmrlrsr));
		// PA_PWRMODE (rx << 4 | tx: 1 fast, 2 slow, 4 fast auto, 5 slow auto), gears, lanes, HS series
		Kern::Printf("[ufs] link: pwrmode %x tx gear %d rx gear %d lanes %d/%d (connected %d/%d) series %d",
			DmeGetOr(EPaPwrMode), DmeGetOr(EPaTxGear), DmeGetOr(EPaRxGear), DmeGetOr(EPaActiveTxLanes),
			DmeGetOr(EPaActiveRxLanes), DmeGetOr(EPaConnectedTxLanes), DmeGetOr(EPaConnectedRxLanes),
			DmeGetOr(EPaHsSeries));
		}
	Kern::Printf("[ufs] %s; manufacturer %04x; errors: uic %d fatal %d timeouts %d; %s, irqs %d waits %d",
		Ours ? "our own bring-up" : "adopted or not up", Vendor, UicErrors, FatalErrors, Timeouts,
		IrqOn ? "interrupts" : "polled", Irqs, Waits);
	Kern::Printf("[ufs] writes: %s; system_b sectors %d..%d (found %d)", !KWritable ? "off (read-only build)" :
		KBootDrive ? "C: + E: on system_b" : "monitor only", (TInt)Parts[0].iFirst, (TInt)Parts[0].iLast,
		Parts[0].iFound);
	for (TInt i = 0; i < NReport; ++i)
		Kern::Printf("[ufs] %S", &Report[i]);
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
	if (Is(aArg[1], "reset"))							// our own bring-up, whatever state the host is in
		{
		NReport = 0;
		TInt r = Prepare();
		if (r == KErrNone)
			{
			TLock lock;
			r = InitResult = FullInit();
			}
		A64Device_Format(aOut, "ufs: full init -> %d (blk info for the steps)", r);
		return r;
		}
	if (InitResult != KErrNone)
		{
		A64Device_Format(aOut, "ufs: not up (%d; blk init or blk reset)", InitResult);
		return KErrNotReady;
		}
	if (Is(aArg[1], "gear"))
		{
		TLock lock;
		TInt r = Gear();
		A64Device_Format(aOut, "ufs: gear -> %d", r);
		return r;
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
		if (KBootDrive && lun != KWriteLun)					// C: reads through the selected LUN
			{
			A64Device_Format(aOut, "ufs: C: is on lun %d, it stays selected", KWriteLun);
			return KErrInUse;
			}
		Cur = lun;
		A64Device_Format(aOut, "ufs: blk read/dump use lun %d", lun);
		return KErrNone;
		}
	if (Is(aArg[1], "speed"))							// "speed [MB]" (64) from LUN 0's start, raw (no copy out of Data)
		{
		TLock lock;
		const TUint32 mb = aArgc >= 3 && aNum[2] >= 1 && aNum[2] <= 1024 ? (TUint32)aNum[2] : 64;
		TUint32 t0 = NKern::TickCount(), irqs = Irqs;
		TInt r = ReadSpan(0, 0, mb << 20, NULL);
		TInt ms = Max(1, (TInt)((NKern::TickCount() - t0) * NKern::TickPeriod() / 1000));
		A64Device_Format(aOut, "ufs: read %d MB in %d ms (%d KB/s) -> %d; %s, %d irqs", mb, ms,
			(TInt)((TUint64)mb * 1024 * 1000 / ms), r, IrqOn ? "interrupts" : "polled", Irqs - irqs);
		return r;
		}
	if (Is(aArg[1], "guard") && aArgc >= 3)				// no I/O: what the write check says for a LUN 0 sector
		{
		const TPart* p = NULL;
		for (TInt i = 0; i < KParts; ++i)
			if (Parts[i].iFound == 1 && aNum[2] >= Parts[i].iFirst && aNum[2] <= Parts[i].iLast)
				p = &Parts[i];
		A64Device_Format(aOut, "lun 0 sector %d: write %s (%s)", (TInt)aNum[2], WriteAllowed(aNum[2], 1) ? "allowed" :
			"denied", p ? p->iName : "outside the allow-list");
		return KErrNone;
		}
	if (Is(aArg[1], "wspeed"))							// "wspeed [MB]" (64): system_b from 1 MB in, C: not on it
		{
		const TUint32 mb = aArgc >= 3 && aNum[2] >= 1 && aNum[2] <= 512 ? (TUint32)aNum[2] : 64;
		const TUint64 at = Parts[0].iFirst + 2048;
		if (KBootDrive || !WriteAllowed(at, (TInt)(mb << 11)))
			return KErrAccessDenied;
		NKern::ThreadEnterCS();
		TUint8* src = (TUint8*)Kern::Alloc(KXfer);
		TUint8* back = (TUint8*)Kern::Alloc(KXfer);
		NKern::ThreadLeaveCS();
		TInt r = src && back ? KErrNone : KErrNoMemory;
		for (TInt i = 0; src && i < KXfer; ++i)
			src[i] = (TUint8)(i * 7 + (i >> 12));
		TUint32 t0 = NKern::TickCount();
		const TInt per = KXfer / KA64BlkSector;
		for (TUint32 i = 0; i < mb * 4 && r == KErrNone; ++i)	// 4 x 256 KB per MB
			r = A64BlkDev_Write(at + i * per, per, src);
		TInt ms = Max(1, (TInt)((NKern::TickCount() - t0) * NKern::TickPeriod() / 1000));
		TInt same = -1;										// the last piece, read back
		if (r == KErrNone && (r = A64BlkDev_Read(at + (mb * 4 - 1) * per, per, back)) == KErrNone)
			same = memcompare(src, KXfer, back, KXfer) == 0;
		if (r == KErrNone && same != 1)
			r = KErrCorrupt;
		NKern::ThreadEnterCS();
		Kern::Free(src);
		Kern::Free(back);
		NKern::ThreadLeaveCS();
		A64Device_Format(aOut, "ufs: wrote %d MB at %d in %d ms (%d KB/s), read back %s -> %d", mb, (TInt)at, ms,
			(TInt)((TUint64)mb * 1024 * 1000 / ms), same == 1 ? "matches" : "MISMATCH", r);
		return r;
		}
	if (Is(aArg[1], "hash") && aArgc >= 3)				// "hash <part> [MB]" (64): SHA-256 of a partition's start
		{
		const TPart* p = NULL;
		for (TInt i = 0; i < KParts; ++i)
			if (Is(aArg[2], Parts[i].iName) && Parts[i].iFound == 1)
				p = &Parts[i];
		if (!p)
			return KErrNotFound;
		const TUint32 mb = aArgc >= 4 && aNum[3] >= 1 && aNum[3] <= 1024 ? (TUint32)aNum[3] : 64;
		if (((TUint64)mb << 11) > p->iLast - p->iFirst + 1)
			return KErrArgument;
		NKern::ThreadEnterCS();
		SHA256* sha = new SHA256;							// heap: a static would need a C++ constructor run
		TUint8* buf = (TUint8*)Kern::Alloc(1 << 20);
		NKern::ThreadLeaveCS();
		TInt r = sha && buf ? KErrNone : KErrNoMemory;
		for (TUint32 i = 0; r == KErrNone && i < mb; ++i)		// 1 MB at a time (4 reads in flight)
			{
			TLock lock;
			if ((r = ReadSpan(KWriteLun, p->iFirst * KA64BlkSector + ((TUint64)i << 20), 1 << 20, buf)) == KErrNone)
				sha->Update(buf, 1 << 20);
			}
		A64Device_Format(aOut, "ufs: %s first %d MB sha256 ", p->iName, mb);
		if (r == KErrNone)
			{
			const TUint8* h = sha->Final().Ptr();
			for (TInt i = 0; i < KSHA256HashSize; ++i)
				A64Device_Format(aOut, "%02x", h[i]);
			}
		A64Device_Format(aOut, " -> %d", r);
		NKern::ThreadEnterCS();
		delete sha;
		Kern::Free(buf);
		NKern::ThreadLeaveCS();
		return r;
		}
	if (Is(aArg[1], "irq") && aArgc >= 3)					// "irq on" / "irq off" (polled)
		{
		TLock lock;
		IrqWanted = Is(aArg[2], "on");
		SetIrq(IrqWanted);
		A64Device_Format(aOut, "ufs: %s", IrqOn ? "interrupts" : "polled");
		return IrqOn == IrqWanted ? KErrNone : KErrNotSupported;
		}
	aOut.Append((const TUint8*)"blk init | reset | gear | luns | lun <n> | speed [MB] | irq on|off | guard <sector> | "
		"wspeed [MB] | hash <part> [MB] | wtest", 124);
	return KErrArgument;
	}

static TA64DevState UfsStatus(TDes8& aDetail)
	{
	if (InitResult == KErrNotReady && !Desc)
		{
		A64Device_Format(aDetail, "read-only; not started (blk init or blk reset)");
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
	A64Device_Format(aDetail, "read-only, %d LUNs, %s, polled", luns, Ours ? "own bring-up" : "ABL's link adopted");
	return EA64DevPartial;
	}
A64_DEVICE("ufs", "Storage (UFS)", "ufshcd", UfsStatus);
