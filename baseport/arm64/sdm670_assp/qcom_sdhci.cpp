// SPDX-License-Identifier: Apache-2.0
//
// qcom_sdhci.cpp -- the Pixel's eMMC (DTB mmc@7c4000, sdhci-msm v5) as the A64BlkDev backend (a64_blk.h). ABL loaded
// our boot image from this eMMC, so the host is powered and clocked and the card is selected in transfer state:
// it is taken over as is -- no re-init, no tuning -- with standard SDHCI commands in PIO mode (no DMA, so no SMMU
// or cache maintenance): CMD13 finds the card's RCA, CMD8 reads EXT_CSD, CMD18/CMD25 move data with auto-CMD12.
// The window (a64_blk.cpp) is the GPT partition "userdata": nothing outside it can be read or written above.
// Writes stay off unless built with EMMC_C=1 (-DA64_EMMC_C): without it, the backend only reports (Volume Up) and
// C: stays the RAM drive.
// shortcut: polled PIO, bounded spins; upgrade to ADMA2 (through the SMMU) if file throughput matters.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_blk.h"

const TUint32 KHc = 0x007c4000, KCqhci = 0x007c5000;
enum
	{
	EBlockSizeCount = 0x04, EArgument = 0x08, EModeCommand = 0x0c, EResponse0 = 0x10, EBuffer = 0x20,
	EPresentState = 0x24, EHostCtrl1 = 0x28, EClockCtrl = 0x2c, EIntStatus = 0x30, EIntStatusEn = 0x34,
	ECaps = 0x40, ESlotVersion = 0xfc,
	};
enum { EPsCmdInhibit = 1 << 0, EPsDatInhibit = 1 << 1 };
enum { EIntCmd = 1 << 0, EIntXfer = 1 << 1, EIntBufWr = 1 << 4, EIntBufRd = 1 << 5, EIntErr = 1 << 15 };
enum { EModeCount = 1 << 1, EModeAutoCmd12 = 1 << 2, EModeRead = 1 << 4, EModeMulti = 1 << 5 };
enum { ERespR1 = 0x1a, ERespR1Data = 0x3a };			// 48-bit response, CRC + index check (+ data present)
const TInt KSpin = 20000000;

#ifdef A64_EMMC_C
const TBool KWritable = ETrue;
#else
const TBool KWritable = EFalse;
#endif

static TUint32 Rca;
static TUint64 Sectors;
static TBuf8<100> Report[8];
static TInt NReport;

static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KHc) + aOff); }

static void Log(const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<100> b;
	Kern::AppendFormat(b, aFmt, list);
	VA_END(list);
	Kern::Printf("[emmc] %S", &b);
	if (NReport < (TInt)(sizeof(Report) / sizeof(Report[0])))
		Report[NReport++] = b;
	}

void A64BlkDump()										// a64_keys.cpp: on Volume Up
	{
	for (TInt i = 0; i < NReport; ++i)
		Kern::Printf("[emmc] %S", &Report[i]);
	}

static TBool WaitClear(TUint32 aOff, TUint32 aMask)
	{
	for (TInt i = 0; i < KSpin; ++i)
		if (!(R(aOff) & aMask))
			return ETrue;
	return EFalse;
	}

// the interrupt status bit aBit (or an error): KErrNone, KErrTimedOut or the error status in the upper half
static TInt WaitInt(TUint32 aBit)
	{
	for (TInt i = 0; i < KSpin; ++i)
		{
		TUint32 st = R(EIntStatus);
		if (st & EIntErr)
			{
			R(EIntStatus) = st;
			return KErrGeneral;
			}
		if (st & aBit)
			{
			R(EIntStatus) = aBit;
			return KErrNone;
			}
		}
	return KErrTimedOut;
	}

static void ResetLines()									// after an error: reset the CMD and DAT circuits
	{
	volatile TUint8* reset = (volatile TUint8*)(A64_Io(KHc) + EClockCtrl + 3);
	*reset = 0x06;
	for (TInt i = 0; i < KSpin && (*reset & 0x06); ++i)
		{}
	R(EIntStatus) = 0xffffffff;
	}

// one command; aBlocks > 0 starts a data transfer of that many blocks with transfer mode aMode
static TInt Command(TUint aIndex, TUint32 aArg, TUint32 aFlags, TUint32 aMode = 0, TInt aBlocks = 0)
	{
	if (!WaitClear(EPresentState, EPsCmdInhibit | (aBlocks ? EPsDatInhibit : 0)))
		return KErrNotReady;
	R(EIntStatus) = 0xffffffff;
	if (aBlocks)
		R(EBlockSizeCount) = (aBlocks << 16) | KA64BlkSector;
	R(EArgument) = aArg;
	R(EModeCommand) = (aIndex << 24) | (aFlags << 16) | aMode;	// the command half triggers it
	TInt r = WaitInt(EIntCmd);
	if (r != KErrNone)
		ResetLines();
	return r;
	}

static TInt ReadBlocks(TUint aCmd, TUint32 aArg, TInt aCount, TUint8* aBuf)
	{
	TUint32 mode = EModeRead | (aCount > 1 ? EModeMulti | EModeCount | EModeAutoCmd12 : 0);
	TInt r = Command(aCmd, aArg, ERespR1Data, mode, aCount);
	for (TInt b = 0; r == KErrNone && b < aCount; ++b)
		{
		r = WaitInt(EIntBufRd);
		TUint32* w = (TUint32*)(aBuf + b * KA64BlkSector);
		for (TInt i = 0; r == KErrNone && i < KA64BlkSector / 4; ++i)
			w[i] = R(EBuffer);
		}
	if (r == KErrNone)
		r = WaitInt(EIntXfer);
	if (r != KErrNone)
		ResetLines();
	return r;
	}

TInt A64BlkDev_Read(TUint64 aLba, TInt aCount, TUint8* aBuf)
	{
	if (!Rca || aLba + aCount > Sectors)
		return KErrArgument;
	return ReadBlocks(aCount > 1 ? 18 : 17, (TUint32)aLba, aCount, aBuf);	// sector addressing (> 2 GB)
	}

TInt A64BlkDev_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf)
	{
	if (!KWritable)
		return KErrAccessDenied;
	if (!Rca || aLba + aCount > Sectors)
		return KErrArgument;
	TUint32 mode = aCount > 1 ? EModeMulti | EModeCount | EModeAutoCmd12 : 0;
	TInt r = Command(aCount > 1 ? 25 : 24, (TUint32)aLba, ERespR1Data, mode, aCount);
	for (TInt b = 0; r == KErrNone && b < aCount; ++b)
		{
		r = WaitInt(EIntBufWr);
		const TUint32* w = (const TUint32*)(aBuf + b * KA64BlkSector);
		for (TInt i = 0; r == KErrNone && i < KA64BlkSector / 4; ++i)
			R(EBuffer) = w[i];
		}
	if (r == KErrNone)
		r = WaitInt(EIntXfer);							// includes the card's busy time after the last block
	if (r != KErrNone)
		ResetLines();
	return r;
	}

TInt A64BlkDev_Init(TUint64& aSectors)
	{
	R(EIntStatusEn) = 0xffff0033;						// make the status bits we poll visible (no signals)
	Log("host v%x caps %08x present %08x hostctl %08x clock %08x cqhci cfg %08x",
		R(ESlotVersion) >> 16, R(ECaps), R(EPresentState), R(EHostCtrl1), R(EClockCtrl),
		*(volatile TUint32*)(A64_Io(KCqhci) + 0x08));
	if (*(volatile TUint32*)(A64_Io(KCqhci) + 0x08) & 1)
		{
		Log("command queueing is on: not taken over");
		return KErrNotSupported;
		}
	for (TUint32 rca = 1; rca <= 2 && !Rca; ++rca)		// ABL's RCA: CMD13 SEND_STATUS answers only to the right one
		if (Command(13, rca << 16, ERespR1) == KErrNone && ((R(EResponse0) >> 9) & 0xf) == 4)	// state 4 = tran
			Rca = rca;
	if (!Rca)
		{
		Log("no card in transfer state at RCA 1 or 2 (status %08x)", R(EResponse0));
		return KErrNotReady;
		}
	static TUint8 ext[KA64BlkSector];
	Sectors = ~0ull;										// let CMD8 through ReadBlocks' bounds
	TInt r = ReadBlocks(8, 0, 1, ext);					// SEND_EXT_CSD
	if (r != KErrNone)
		{
		Log("EXT_CSD read -> %d", r);
		Rca = 0;
		return r;
		}
	Sectors = ext[212] | (ext[213] << 8) | (ext[214] << 16) | ((TUint32)ext[215] << 24);	// SEC_COUNT
	Log("RCA %d, %d sectors (%d MB), cache size %d ctrl %d, partition config %02x, rev %d", Rca, (TInt)Sectors,
		(TInt)(Sectors >> 11), ext[249] | (ext[250] << 8) | (ext[251] << 16) | (ext[252] << 24), ext[33], ext[179],
		ext[192]);
	if (ext[179] & 7)										// PARTITION_ACCESS: a boot or RPMB area is selected
		{
		Log("not the user area (partition access %d)", ext[179] & 7);
		Rca = 0;
		return KErrNotSupported;
		}
	aSectors = Sectors;
	return KErrNone;
	}

// the GPT partition "userdata" (UTF-16 name in the entry): the only window
TInt A64BlkBoard_Window(TUint64 aSectors, TUint64& aFirst, TUint64& aCount)
	{
	static TUint8 s[KA64BlkSector];
	TInt r = A64BlkDev_Read(1, 1, s);
	if (r != KErrNone || memcompare(s, 8, (const TUint8*)"EFI PART", 8) != 0)
		{
		Log("no GPT header (%d)", r);
		return KErrCorrupt;
		}
	TUint64 entries = *(TUint64*)(s + 72);
	TUint32 n = *(TUint32*)(s + 80), size = *(TUint32*)(s + 84);
	TInt found = 0;
	TUint64 first = 0, last = 0;
	for (TUint32 i = 0; i < n && size >= 128 && size <= KA64BlkSector; ++i)
		{
		TUint32 off = i * size;
		if (off % KA64BlkSector == 0 && A64BlkDev_Read(entries + off / KA64BlkSector, 1, s) != KErrNone)
			return KErrCorrupt;
		const TUint8* e = s + off % KA64BlkSector;
		static const char KName[] = "userdata";
		TBool match = ETrue;
		for (TInt c = 0; c < 9 && match; ++c)				// 8 letters and the terminating 0, as UTF-16
			match = e[56 + 2 * c] == (TUint8)KName[c] && e[57 + 2 * c] == 0;
		if (match)
			{
			++found;
			first = *(const TUint64*)(e + 32);
			last = *(const TUint64*)(e + 40);
			}
		}
	if (found != 1 || last < first || last >= aSectors)
		{
		Log("GPT: %d entries, userdata found %d times (%d..%d): no window", n, found, (TInt)first, (TInt)last);
		return KErrNotFound;
		}
	Log("GPT: %d entries; userdata = sectors %d..%d (%d MB)%s", n, (TInt)first, (TInt)last,
		(TInt)((last - first + 1) >> 11), KWritable ? "" : "; read-only build: C: stays the RAM drive");
	aFirst = first;
	aCount = last - first + 1;
	return KWritable ? KErrNone : KErrNotSupported;		// read-only build: no block drive yet
	}
