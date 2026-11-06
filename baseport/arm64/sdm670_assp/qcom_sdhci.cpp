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
	ESdmaAddr = 0x00, EBlockSizeCount = 0x04, EArgument = 0x08, EModeCommand = 0x0c, EResponse0 = 0x10, EBuffer = 0x20,
	EPresentState = 0x24, EHostCtrl1 = 0x28, EClockCtrl = 0x2c, EIntStatus = 0x30, EIntStatusEn = 0x34,
	ECaps = 0x40, ESlotVersion = 0xfc,
	};
enum { EPsCmdInhibit = 1 << 0, EPsDatInhibit = 1 << 1, EPsBufWrEn = 1 << 10, EPsBufRdEn = 1 << 11 };
enum { EIntCmd = 1 << 0, EIntXfer = 1 << 1, EIntBufWr = 1 << 4, EIntBufRd = 1 << 5, EIntErr = 1 << 15 };
enum { EModeDma = 1 << 0, EModeCount = 1 << 1, EModeAutoCmd12 = 1 << 2, EModeRead = 1 << 4, EModeMulti = 1 << 5 };
enum { ERespR1 = 0x1a, ERespR1Data = 0x3a };			// 48-bit response, CRC + index check (+ data present)
const TInt KSpin = 2000000;							// ~0.2 s of register reads: fail fast, never stall boot

#ifdef A64_EMMC_C
const TBool KWritable = ETrue;
#else
const TBool KWritable = EFalse;
#endif

static TUint32 Rca;
static TUint64 Sectors;
// data moves by SDMA through a bounce buffer: this host's PIO data port never drains (found on the phone; Linux
// sdhci-msm only ever uses DMA). The SDHC sits behind the SMMU, which passes unconfigured streams through.
const TInt KDmaMax = 128;								// sectors per transfer: 64 KB, inside one 512 KB boundary
static TPhysAddr DmaPhys;
static TUint8* DmaBuf;
static TBuf8<200> Report[24];
static TInt NReport;

static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KHc) + aOff); }
// this host answers 8- and 16-bit accesses with an asynchronous SError (found on the phone): sub-word registers are
// read-modify-written through their 32-bit word
static TUint32 Get(TUint32 aOff, TInt aBits) { return (R(aOff & ~3u) >> ((aOff & 3) * 8)) & ((1u << aBits) - 1); }
static void Set(TUint32 aOff, TInt aBits, TUint32 aVal)
	{
	TUint32 sh = (aOff & 3) * 8, mask = ((1u << aBits) - 1) << sh;
	R(aOff & ~3u) = (R(aOff & ~3u) & ~mask) | ((aVal << sh) & mask);
	}

// sdhci-msm v5 (Linux sdhci-msm.c sdhci_msm_v5_offset): the vendor register and the power-control handshake
enum { EVendorSpec = 0x20c, EPwrStatus = 0x240, EPwrMask = 0x244, EPwrClear = 0x248, EPwrCtl = 0x24c };
enum { EPowerCtrl = 0x29, ETimeoutCtrl = 0x2e, EHostCtrl2 = 0x3e };
const TUint32 KGcc = 0x00100000;
enum { ESdcc1Apps = 0x26004, ESdcc1Ahb = 0x26008, ESdcc1RcgCmd = 0x26028, ESdcc1RcgCfg = 0x2602c };
static volatile TUint32& Gcc(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KGcc + aOff); }	// only its pages are mapped

static void Log(const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<200> b;
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

static TInt WaitSet(TUint32 aOff, TUint32 aMask)
	{
	for (TInt i = 0; i < KSpin; ++i)
		if (R(aOff) & aMask)
			return KErrNone;
	return KErrTimedOut;
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
	Set(EClockCtrl + 3, 8, 0x06);						// SOFTWARE_RESET: CMD and DAT
	for (TInt i = 0; i < KSpin && (Get(EClockCtrl + 3, 8) & 0x06); ++i)
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
		R(EBlockSizeCount) = (aBlocks << 16) | (7u << 12) | KA64BlkSector;	// SDMA boundary 512 KB
		R(ESdmaAddr) = DmaPhys;
	R(EArgument) = aArg;
	R(EModeCommand) = (aIndex << 24) | (aFlags << 16) | aMode;	// the command half triggers it
	TInt r = WaitInt(EIntCmd);
	if (r != KErrNone)
		ResetLines();
	return r;
	}

// one SDMA transfer of aCount (<= KDmaMax) blocks between the card and DmaBuf
static TInt Transfer(TUint aCmd, TUint32 aArg, TInt aCount, TBool aRead)
	{
	TUint32 mode = EModeDma | (aRead ? EModeRead : 0) | (aCount > 1 ? EModeMulti | EModeCount | EModeAutoCmd12 : 0);
	TInt r = Command(aCmd, aArg, ERespR1Data, mode, aCount);
	if (r == KErrNone)
		r = WaitInt(EIntXfer);							// all data moved (a write: and the card's busy time)
	if (r != KErrNone)
		{
		Log("CMD%d x%d failed: %d, int %08x present %08x resp %08x", aCmd, aCount, r, R(EIntStatus),
			R(EPresentState), R(EResponse0));
		ResetLines();
		}
	return r;
	}

static TInt ReadBlocks(TUint aCmd, TUint32 aArg, TInt aCount, TUint8* aBuf)
	{
	TInt r = Transfer(aCmd, aArg, aCount, ETrue);
	if (r == KErrNone)
		memcpy(aBuf, DmaBuf, aCount * KA64BlkSector);
	return r;
	}

TInt A64BlkDev_Read(TUint64 aLba, TInt aCount, TUint8* aBuf)
	{
	if (!Rca || aLba + aCount > Sectors)
		return KErrArgument;
	for (TInt done = 0; done < aCount; )
		{
		TInt n = Min(aCount - done, KDmaMax);
		TInt r = ReadBlocks(n > 1 ? 18 : 17, (TUint32)(aLba + done), n, aBuf + done * KA64BlkSector);	// sector addressing
		if (r != KErrNone)
			return r;
		done += n;
		}
	return KErrNone;
	}

TInt A64BlkDev_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf)
	{
	if (!KWritable)
		return KErrAccessDenied;
	if (!Rca || aLba + aCount > Sectors)
		return KErrArgument;
	for (TInt done = 0; done < aCount; )
		{
		TInt n = Min(aCount - done, KDmaMax);
		memcpy(DmaBuf, aBuf + done * KA64BlkSector, n * KA64BlkSector);
		TInt r = Transfer(n > 1 ? 25 : 24, (TUint32)(aLba + done), n, EFalse);
		if (r != KErrNone)
			return r;
		done += n;
		}
	return KErrNone;
	}

// msm raises a power interrupt for each bus/IO power change and holds it until the driver acks; there are no
// regulator calls to make here (RPMh keeps the eMMC supplies on), so ack success, as sdhci_msm_handle_pwr_irq does
static TUint32 PwrAck()
	{
	TUint32 st = 0;
	for (TInt i = 0; i < 200 && !st; ++i)
		{
		st = R(EPwrStatus);
		if (!st)
			NKern::Sleep(NKern::TimerTicks(1));
		}
	if (st)
		{
		R(EPwrClear) = st;
		R(EPwrCtl) = ((st & 3) ? 1u : 0) | ((st & 0xc) ? 4u : 0);	// BUS_SUCCESS, IO_SUCCESS
		}
	return st;
	}

// SD clock = 19.2 MHz (XO, the GCC root) / (2 * aDiv), aDiv 0 = undivided
static void SdClock(TUint aDiv)
	{
	Set(EClockCtrl, 16, 0);
	Set(EClockCtrl, 16, ((aDiv & 0xff) << 8) | ((aDiv >> 8) << 6) | 1);	// internal clock on
	for (TInt i = 0; i < KSpin && !(Get(EClockCtrl, 16) & 2); ++i)
		{}
	Set(EClockCtrl, 16, Get(EClockCtrl, 16) | 4);			// card clock on
	}

// ABL powers the host down before it starts a kernel (bus power off, card clock off, 1-bit): bring the card up
// again from scratch -- CMD0 also undoes any HS200/HS400 timing the card was left in -- at a legacy 19.2 MHz
// the core (apps) clock first: ABL switched it off, and the msm vendor registers hang the bus without it
static void ClocksOn()
	{
	Gcc(ESdcc1RcgCfg) = 1;									// source 0 = XO, pre-divider 1 (encoded 2*1-1)
	Gcc(ESdcc1RcgCmd) |= 1;
	for (TInt i = 0; i < KSpin && (Gcc(ESdcc1RcgCmd) & 1); ++i)
		{}
	Gcc(ESdcc1Ahb) |= 1;
	Gcc(ESdcc1Apps) |= 1;
	for (TInt i = 0; i < KSpin && ((Gcc(ESdcc1Apps) | Gcc(ESdcc1Ahb)) & (1u << 31)); ++i)	// CLK_OFF clears
		{}
	}

static TInt Reinit()
	{
	// as Linux at probe: reset the whole SDHCI core (ABL leaves HS400 state behind), the vendor register to its
	// power-on value with the I/O pads switchable, then bus power and 1.8 V signalling, each acked (sdhci-msm.c)
	Log("step: reset all");
	R(EPwrMask) = 0xf;
	Set(EClockCtrl + 3, 8, 0x01);							// SOFTWARE_RESET_ALL
	for (TInt i = 0; i < KSpin && (Get(EClockCtrl + 3, 8) & 1); ++i)
		{}
	TUint32 pwrOff = R(EPwrStatus) ? PwrAck() : 0;
	R(EVendorSpec) = 0xa9c | (1u << 15) | (1u << 16);		// POR value + IO_PAD_PWR_SWITCH_EN + 1.8 V pads
	R(EIntStatusEn) = 0xffff0033;							// the reset cleared the status enables
	R(EPwrMask) = 0xf;
	Log("step: power (reset pwr irq %x)", pwrOff);
	Set(EPowerCtrl, 8, 0x0b);									// bus power on, 1.8 V
	TUint32 pwr = PwrAck();
	Set(EHostCtrl2, 16, 1u << 3);							// 1.8 V signalling: msm switches the pads (IO_LOW)
	pwr |= PwrAck() << 8;
	Log("step: clock 400k (pwr irq %x)", pwr);
	SdClock(24);											// 400 kHz for identification
	Set(ETimeoutCtrl, 8, 0x0e);
	Set(EHostCtrl1, 8, 0);										// 1-bit, SDR
	NKern::Sleep(NKern::TimerTicks(2));
	Log("step: cmd0");
	Command(0, 0, 0);										// GO_IDLE_STATE (no response)
	NKern::Sleep(NKern::TimerTicks(2));
	TUint32 ocr = 0;
	for (TInt i = 0; i < 1000 && !(ocr & 0x80000000u); ++i)	// SEND_OP_COND: sector mode, 1.8 V + 2.7-3.6 V
		{
		if (Command(1, 0x40ff8080, 0x02) == KErrNone)
			ocr = R(EResponse0);
		if (!(ocr & 0x80000000u))
			NKern::Sleep(NKern::TimerTicks(1));
		}
	TInt r2 = (ocr & 0x80000000u) ? Command(2, 0, 0x09) : KErrNotReady;	// ALL_SEND_CID (R2)
	TInt r3 = r2 == KErrNone ? Command(3, 1 << 16, ERespR1) : r2;		// SET_RELATIVE_ADDR = 1
	if (r3 == KErrNone)
		SdClock(0);											// identification done: 19.2 MHz
	TInt r7 = r3 == KErrNone ? Command(7, 1 << 16, 0x1b) : r3;			// SELECT_CARD (R1b)
	if (r7 == KErrNone)
		WaitInt(EIntXfer);									// the card's busy time
	TInt r13 = r7 == KErrNone ? Command(13, 1 << 16, ERespR1) : r7;
	TUint32 state = (R(EResponse0) >> 9) & 0xf;
	Log("reinit: pwr irq %x, ocr %08x, cid %d, rca %d, select %d, status %d state %d", pwr, ocr, r2, r3, r7, r13,
		state);
	if (r13 != KErrNone || state != 4)
		return KErrNotReady;
	Rca = 1;
	return KErrNone;
	}

static TInt DoInit(TUint64& aSectors);

// at boot only in the EMMC_C build; else the monitor's "blk init" (a hang then stops only the console's thread)
TInt A64BlkDev_Init(TUint64& aSectors)
	{
	if (!KWritable)
		return KErrNotSupported;
	return DoInit(aSectors);
	}

TInt A64BlkDev_Probe()
	{
	TUint64 sectors = 0, first = 0, count = 0;
	TInt r = DoInit(sectors);
	if (r == KErrNone)
		r = A64BlkBoard_Window(sectors, first, count);
	return r;
	}

static TInt DoInit(TUint64& aSectors)
	{
	if (!DmaBuf)
		{
		NKern::ThreadEnterCS();
		TInt r = Epoc::AllocPhysicalRam(KDmaMax * KA64BlkSector, DmaPhys, 12);
		DPlatChunkHw* chunk = NULL;
		if (r == KErrNone)
			r = DPlatChunkHw::New(chunk, DmaPhys, KDmaMax * KA64BlkSector, EMapAttrSupRw | EMapAttrBufferedNC);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return r;
		DmaBuf = (TUint8*)chunk->LinearAddress();
		}
	Log("host v%x caps %08x present %08x hostctl %08x clock %08x; gcc apps %08x ahb %08x", R(ESlotVersion) >> 16,
		R(ECaps), R(EPresentState), R(EHostCtrl1), R(EClockCtrl), Gcc(ESdcc1Apps), Gcc(ESdcc1Ahb));
	Log("step: gcc clocks");
	ClocksOn();
	Log("step: int enable");
	R(EIntStatusEn) = 0xffff0033;						// make the status bits we poll visible (no signals)
	Log("clocks on: gcc apps %08x ahb %08x; vendor %08x pwr %08x cqhci %08x", Gcc(ESdcc1Apps), Gcc(ESdcc1Ahb),
		R(EVendorSpec), R(EPwrStatus), *(volatile TUint32*)(A64_Io(KCqhci) + 0x08));
	*(volatile TUint32*)(A64_Io(KCqhci) + 0x08) &= ~1u;	// command queueing off: plain SDHCI commands
	TInt r = Reinit();
	if (r != KErrNone)
		return r;
	static TUint8 ext[KA64BlkSector];
	Sectors = ~0ull;										// let CMD8 through ReadBlocks' bounds
	r = ReadBlocks(8, 0, 1, ext);						// SEND_EXT_CSD
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
