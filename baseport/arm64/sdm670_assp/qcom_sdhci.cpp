// SPDX-License-Identifier: Apache-2.0
//
// qcom_sdhci.cpp -- the Pixel's eMMC (DTB mmc@7c4000, sdhci-msm v5) as the A64BlkDev backend (a64_blk.h). ABL loaded
// our boot image from this eMMC, so the host is powered and clocked and the card is selected in transfer state:
// but ABL shuts the host down before it starts a kernel, so it is re-initialised (Reinit: legacy 19.2 MHz, 1-bit, no
// tuning), with the RPMh votes Linux makes at probe (CX, the NoC link). CMD8 reads EXT_CSD, CMD18/CMD25 move data
// with auto-CMD12, by ADMA2 through a non-cached bounce buffer.
// The window (a64_blk.cpp) is the GPT partition "userdata": nothing outside it can be read or written above.
// Writes stay off unless built with EMMC_C=1 (-DA64_EMMC_C): without it, the backend only reports (Volume Up) and
// C: stays the RAM drive.
// shortcut: polled, bounded spins, 1-bit 19.2 MHz (~2 MB/s); HS200/8-bit with tuning if file throughput matters.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_blk.h"
#include "qcom_rpmh.h"

const TUint32 KHc = 0x007c4000, KCqhci = 0x007c5000;
enum
	{
	EBlockSizeCount = 0x04, EArgument = 0x08, EModeCommand = 0x0c, EResponse0 = 0x10, EBuffer = 0x20,
	EPresentState = 0x24, EHostCtrl1 = 0x28, EClockCtrl = 0x2c, EIntStatus = 0x30, EIntStatusEn = 0x34,
	ECaps = 0x40, EAdmaError = 0x54, EAdmaAddr = 0x58, ESlotVersion = 0xfc,
	};
enum { EPsCmdInhibit = 1 << 0, EPsDatInhibit = 1 << 1, EPsBufWrEn = 1 << 10, EPsBufRdEn = 1 << 11 };
enum { EIntCmd = 1 << 0, EIntXfer = 1 << 1, EIntBufWr = 1 << 4, EIntBufRd = 1 << 5, EIntErr = 1 << 15 };
enum { EModeDma = 1 << 0, EModeCount = 1 << 1, EModeAutoCmd12 = 1 << 2, EModeRead = 1 << 4, EModeMulti = 1 << 5 };
enum { ERespR1 = 0x1a, ERespR1Data = 0x3a };			// 48-bit response, CRC + index check (+ data present)
const TInt KSpin = 2000000;							// ~0.2 s of register reads: fail fast, never stall boot

// EMMC_C=1: C: is the window at boot; EMMC_C=2: writes allowed, but only the monitor touches the eMMC (C: stays
// the RAM drive); unset: read-only
#ifdef A64_EMMC_C
const TBool KWritable = ETrue;
const TBool KBootDrive = A64_EMMC_C == 1;
#else
const TBool KWritable = EFalse;
const TBool KBootDrive = EFalse;
#endif
// C: = the first 2000 MB of userdata: ELOCAL.FSY is still the old sfat (FAT12/16), whose 32 KB clusters top out
// just under 2 GB (docs/filesystem.md). shortcut: grows to 4-8 GB (FAT32) with FS1's sfat32
const TUint64 KCSectors = (2000ull << 20) / KA64BlkSector;
static TUint64 WinFirst, WinCount;						// the window, once the GPT is read: the only writable range

static TUint32 Rca;
static TUint64 Sectors;
// data moves by ADMA2 through a bounce buffer. This host has no SDMA (CAPS bit 22 is 0: an SDMA transfer leaves the
// block in the FIFO and the stuck master resets the SoC) and its PIO data port never drains (both found on the
// phone); Linux sdhci.c picks ADMA2 here too. The SDHC's SMMU stream (0x140) is set up as pass-through (CB0, M = 0).
const TInt KDmaMax = 128;								// sectors per transfer: 64 KB
const TInt KAdmaChunk = 32 * 1024;						// bytes per descriptor (the 16-bit length field)
static TPhysAddr DmaPhys;								// KDmaMax sectors of data, then the descriptor table
static TUint8* DmaBuf;
static volatile TUint32* Desc;
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
		{
		R(EBlockSizeCount) = (aBlocks << 16) | KA64BlkSector;
		R(EAdmaAddr) = DmaPhys + KDmaMax * KA64BlkSector;
		R(EAdmaAddr + 4) = 0;
		}
	R(EArgument) = aArg;
	R(EModeCommand) = (aIndex << 24) | (aFlags << 16) | aMode;	// the command half triggers it
	TInt r = WaitInt(EIntCmd);
	if (r != KErrNone)
		ResetLines();
	return r;
	}

// one ADMA2 transfer of aCount (<= KDmaMax) blocks between the card and DmaBuf: 32-bit descriptors (sdhci.c
// __sdhci_adma_write_desc) {cmd = valid | transfer [| end], length, address}, 32 KB each
static TInt Transfer(TUint aCmd, TUint32 aArg, TInt aCount, TBool aRead)
	{
	TInt bytes = aCount * KA64BlkSector, nd = 0;
	for (TInt off = 0; off < bytes; off += KAdmaChunk, ++nd)
		{
		TInt len = Min(bytes - off, KAdmaChunk);
		Desc[2 * nd] = (TUint32)len << 16 | 0x21 | (off + len >= bytes ? 0x2 : 0);	// ADMA2_TRAN_VALID [| ADMA2_END]
		Desc[2 * nd + 1] = (TUint32)DmaPhys + off;
		}
	__e32_memory_barrier();
	TUint32 mode = EModeDma | (aRead ? EModeRead : 0) | (aCount > 1 ? EModeMulti | EModeCount | EModeAutoCmd12 : 0);
	TInt r = Command(aCmd, aArg, ERespR1Data, mode, aCount);
	if (r != KErrNone)
		{
		Log("CMD%d x%d: command failed %d", aCmd, aCount, r);
		return r;
		}
	r = KErrTimedOut;
	for (TInt i = 0; i < KSpin && r == KErrTimedOut; ++i)	// all data moved (a write: and the card's busy time)
		{
		TUint32 st = R(EIntStatus);
		if (st & EIntErr)
			r = KErrGeneral;
		else if (st & EIntXfer)
			{
			R(EIntStatus) = EIntXfer;
			r = KErrNone;
			}
		}
	if (r != KErrNone)		// the data phase failed: leave the host as it is (a DAT reset under a stuck DMA may hang)
		Log("CMD%d x%d failed %d: int %08x present %08x resp %08x adma %08x (table %08x) err %08x hostctl %08x "
			"ctrl2 %08x", aCmd, aCount, r, R(EIntStatus), R(EPresentState), R(EResponse0), R(EAdmaAddr),
			(TUint32)DmaPhys + KDmaMax * KA64BlkSector, R(EAdmaError), R(EHostCtrl1), R(0x3c));
	if (r != KErrNone)		// the msm core's own view (sdhci-msm.c v5 offsets): FIFO count, MCI status, data count, debug
		Log("  mci fifo %08x status %08x data cnt %08x debug %08x generics %08x", R(0x308), R(0x324), R(0x35c),
			R(0x358), R(0x320));
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
	// a second check under a64_blk.cpp's: nothing outside the window userdata gave, whoever asks
	if (!KWritable || !WinCount || aCount <= 0 || aLba < WinFirst || aLba + aCount > WinFirst + WinCount)
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
	R(EIntStatusEn) = 0xffff003f;							// the reset cleared the status enables
	R(EPwrMask) = 0xf;
	Log("step: power (reset pwr irq %x)", pwrOff);
	Set(EPowerCtrl, 8, 0x0b);									// bus power on, 1.8 V
	TUint32 pwr = PwrAck();
	Set(EHostCtrl2, 16, 1u << 3);							// 1.8 V signalling: msm switches the pads (IO_LOW)
	pwr |= PwrAck() << 8;
	Log("step: clock 400k (pwr irq %x)", pwr);
	SdClock(24);											// 400 kHz for identification
	Set(ETimeoutCtrl, 8, 0x0e);
	Set(EHostCtrl1, 8, 0x10);								// 1-bit, SDR, DMA select = ADMA2 32-bit
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
	if (!KBootDrive)
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
		const TInt size = KDmaMax * KA64BlkSector + 4096;	// + one page of descriptors
		TInt r = Epoc::AllocPhysicalRam(size, DmaPhys, 12);
		DPlatChunkHw* chunk = NULL;
		if (r == KErrNone)
			r = DPlatChunkHw::New(chunk, DmaPhys, size, EMapAttrSupRw | EMapAttrBufferedNC);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return r;
		DmaBuf = (TUint8*)chunk->LinearAddress();
		Desc = (volatile TUint32*)(DmaBuf + KDmaMax * KA64BlkSector);
		}
	Log("host v%x caps %08x present %08x hostctl %08x clock %08x; gcc apps %08x ahb %08x", R(ESlotVersion) >> 16,
		R(ECaps), R(EPresentState), R(EHostCtrl1), R(EClockCtrl), Gcc(ESdcc1Apps), Gcc(ESdcc1Ahb));
	// ABL dropped the eMMC's votes; Linux makes them at probe from the DT opp-20000000: CX low_svs, and the bandwidth
	// through qnm_aggre1_noc (16 bytes wide, BCM SN8)
	Log("step: CX low_svs -> %d", Arc_Vote("cx.lvl", 64));	// the DT opp-20000000 required-opps
	Log("step: bus vote -> %d", Bcm_Vote("SN8", 52286, 80000, 16));
	Log("step: gcc clocks");
	ClocksOn();
	Log("step: int enable");
	R(EIntStatusEn) = 0xffff003f;						// make the status bits we poll visible (no signals)
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
	aFirst = WinFirst = first;
	aCount = WinCount = Min(last - first + 1, KCSectors);
	Log("GPT: %d entries; userdata = sectors %d..%d (%d MB); window %d + %d, %s", n, (TInt)first, (TInt)last,
		(TInt)((last - first + 1) >> 11), (TInt)WinFirst, (TInt)WinCount,
		!KWritable ? "read-only" : KBootDrive ? "writable, C:" : "writable (monitor only)");
	return KErrNone;
	}

// the monitor's "blk wtest" (EMMC_C builds, after "blk init"): write, read back and compare a pattern on the
// window's last 8 sectors, then zero the window's first 34 and last 64 sectors -- ext4's superblock and any stale
// GPT go, so the next mount sees an unformatted disk (ztOS FIXES.md #5)
TInt A64BlkDev_SelfTest()
	{
	if (!KWritable || !WinCount || WinCount < 128)
		return KErrNotReady;
	static TUint8 b[8 * KA64BlkSector];
	const TUint64 end = WinFirst + WinCount;
	for (TInt i = 0; i < 8 * KA64BlkSector; ++i)
		b[i] = (TUint8)(i % KA64BlkSector < 8 ? "NEOSYMB"[i % 8] : i / KA64BlkSector + i);
	TInt r = A64BlkDev_Write(end - 8, 8, b);
	Log("wtest: write 8 at %d -> %d", (TInt)(end - 8), r);
	static TUint8 c[8 * KA64BlkSector];
	if (r == KErrNone)
		r = A64BlkDev_Read(end - 8, 8, c);
	TInt bad = r == KErrNone ? memcompare(b, sizeof(b), c, sizeof(c)) : -1;
	Log("wtest: read back -> %d, %s", r, bad ? "MISMATCH" : "matches");
	if (r != KErrNone || bad)
		return r != KErrNone ? r : KErrCorrupt;
	memclr(b, sizeof(b));
	for (TInt i = 0; i < 64 && r == KErrNone; i += 8)		// the last 64
		r = A64BlkDev_Write(end - 64 + i, 8, b);
	for (TInt i = 0; i < 40 && r == KErrNone; i += 8)		// the first 34 (40)
		r = A64BlkDev_Write(WinFirst + i, 8, b);
	Log("wtest: zeroed the window's first 40 and last 64 sectors -> %d", r);
	return r;
	}
