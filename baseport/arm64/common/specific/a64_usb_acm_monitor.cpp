// SPDX-License-Identifier: Apache-2.0
//
// a64_usb_acm_monitor.cpp -- the debug console's USB function: a CDC-ACM serial port (the Mac's /dev/cu.usbmodem*)
// on the board's device controller core (a64_usb_function.h; on the Pixel sdm670_assp/qcom_dwc3.cpp). Its bytes in
// are lines for the debug monitor (a64_dbgmon.cpp); its bytes out are the kernel log ring and the monitor's answers.
// tools/phone.py drives it.
// IDs 0x1209:0x0001 (pid.codes test ID, private development use).
// shortcut: one ACM port; a second (fshell) comes in roadmap T8.
#include <kernel/kern_priv.h>
#include "a64_dbgmon.h"
#include "a64_usb_function.h"

enum { EPhysBulkOut = 2, EPhysBulkIn = 3, EPhysIntrIn = 5 };
const TInt KBulkMax = 512, KInMax = 4096;
// a 4 KB HS transfer takes ~70 us on the wire: the writer polls for its completion up to 200 us (fast counter)
// before it sleeps a tick (a tick per 4 KB held "shot" at ~1 MB/s)
const TInt KSpinPerSecond = 5000;

// ---- descriptors
static const TUint8 KDevice[] = {
	18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, 64, 0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 1, 2, 3, 1 };
static const TUint8 KQualifier[] = { 10, 6, 0x00, 0x02, 0x02, 0x00, 0x00, 64, 1, 0 };
static const TUint8 KFragment[] = {						// the core's header: 1 configuration, bus powered 100 mA
	9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,						// interface 0: CDC communication, ACM, AT commands
	5, 0x24, 0x00, 0x10, 0x01,								// header 1.10
	5, 0x24, 0x01, 0x00, 0x01,								// call management: data on interface 1
	4, 0x24, 0x02, 0x02,									// ACM: line coding + control line state
	5, 0x24, 0x06, 0x00, 0x01,								// union: 0 controls 1
	7, 5, 0x82, 0x03, 16, 0, 9,								// EP2 IN interrupt, 16 bytes
	9, 4, 1, 0, 2, 0x0a, 0x00, 0x00, 0,						// interface 1: CDC data
	7, 5, 0x01, 0x02, 0x00, 0x02, 0,						// EP1 OUT bulk 512
	7, 5, 0x81, 0x02, 0x00, 0x02, 0,						// EP1 IN bulk 512
	};
static const char* const KStrings[] = { NULL, "NeoSymbian", "Symbian Pixel debug", "0001" };
static const SA64UsbEndpoint KEndpoints[] = {				// in this order: each gets its resources and buffer
	{ EPhysBulkOut, EA64UsbBulk, KBulkMax, 0, 0x1000 },		// the raw "blk put" transfers are 4 KB
	{ EPhysBulkIn, EA64UsbBulk, KBulkMax, 0, KInMax },
	{ EPhysIntrIn, EA64UsbInterrupt, 16, 9, 0 },			// never used: ACM notifications
	};

// ---- state
static TBool Dtr, InBusy, OutBusy, AbortIn;
static TUint8 LineCoding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0, 8 };	// 115200 8N1, as the host likes to see
static TUint32 LogPos;
static TBuf8<160> Line;
static TBool LineReady;
// "blk put" raw input (A64Usb_RawIn below): the stage, how much of it is filled, what is still to come
// A transfer never splits: the stage has a transfer's worth of slack past 64 KB, and what isn't a whole sector stays
// for the next stage. Raw transfers are 4 KB (the OUT area), lines take one packet at a time.
const TInt KRawStage = 64 * 1024, KRawXfer = 0x1000, KSector = 512;
static TUint8* RawStage;									// KRawStage + KRawXfer
static TInt RawFill, RawLeft;
static TBool RawFull;
static TInt InStarts, InDone, InFail, OutLines;
static TUint32 Generation;								// a new host session: a writer still sending drops the rest

static TInt ArmOut()
	{
	return A64Usb_Start(EPhysBulkOut, RawLeft > 0 ? KRawXfer : KBulkMax);
	}

// ---- the core's callbacks
static TInt ClassSetup(const TUint8* aSetup, TUint8* aBuf)	// CDC ACM, to interface 0
	{
	const TUint8 type = aSetup[0];
	const TUint16 index = aSetup[4] | (aSetup[5] << 8), length = aSetup[6] | (aSetup[7] << 8);
	if (index != 0)
		return KA64UsbStall;
	switch (aSetup[1])
		{
	case 0x20:												// SET_LINE_CODING: 7 bytes follow
		return type == 0x21 && length == 7 ? KA64UsbDataOut : KA64UsbStall;
	case 0x21:												// GET_LINE_CODING
		if (type != 0xa1 || length != 7)
			return KA64UsbStall;
		memcpy(aBuf, LineCoding, 7);
		return 7;
	case 0x22:												// SET_CONTROL_LINE_STATE: bit 0 = DTR (port open)
		if (type != 0x21 || length != 0)
			return KA64UsbStall;
		Dtr = aSetup[2] & 1;
		AbortIn = InBusy;									// a closed port never collects its last piece: abort it
															// (later: End Transfer stalls during a control transfer)
		if (Dtr)
			LogPos = 0;										// a new session sees the whole log
		return KA64UsbNoData;
	default: return KA64UsbStall;
		}
	}

static void ClassOutData(const TUint8* aSetup, const TUint8* aData)
	{
	if (aSetup[1] == 0x20)
		memcpy(LineCoding, aData, 7);
	}

static TInt Ready()
	{
	InBusy = EFalse;
	OutBusy = ArmOut() == KErrNone;
	return OutBusy ? KErrNone : KErrGeneral;
	}

static void Reset()											// the session ended: forget its state
	{
	++Generation;
	Dtr = InBusy = OutBusy = AbortIn = EFalse;
	RawLeft = RawFill = 0;
	RawFull = LineReady = EFalse;
	Line.Zero();
	}

static void XferComplete(TInt aPhysEp, TInt aBytes)
	{
	if (aPhysEp == EPhysBulkIn)
		{
		InBusy = EFalse;
		++InDone;
		}
	else if (aPhysEp == EPhysBulkOut)
		{
		const TUint8* p = A64Usb_Buf(EPhysBulkOut);
		if (RawLeft > 0)									// "blk put": raw bytes into the stage, no lines
			{
			TInt n = Min(aBytes, RawLeft);					// fits: the stage keeps KRawXfer of slack
			memcpy(RawStage + RawFill, p, n);
			RawFill += n;
			RawLeft -= n;
			RawFull = RawFill >= KRawStage || RawLeft == 0;	// the poll writes it, then re-arms the endpoint
			}
		else
			for (TInt i = 0; i < aBytes; ++i)
				{
				if (p[i] == '\r' || p[i] == '\n')
					LineReady = LineReady || Line.Length() > 0;
				else if (!LineReady && Line.Length() < Line.MaxLength())
					Line.Append(p[i]);
				}
		OutBusy = !RawFull && ArmOut() == KErrNone;
		}
	}

// the monitor's writer: blocks (polling the controller) until each piece is sent, gives up after ~2 s
static void UsbWrite(const TUint8* aData, TInt aLen)
	{
	const TUint32 generation = Generation;
	while (aLen > 0 && A64Usb_Configured() && Dtr && generation == Generation)
		{
		for (TInt i = 0; (InBusy || !A64Usb_CanStart(EPhysBulkIn)) && i < 2000 && generation == Generation; ++i)
			{
			A64Usb_ProcessEvents();
			const TUint32 t0 = NKern::FastCounter(), spin = NKern::FastCounterFrequency() / KSpinPerSecond;
			while ((InBusy || !A64Usb_CanStart(EPhysBulkIn)) && generation == Generation && NKern::FastCounter() - t0 < spin)
				A64Usb_ProcessEvents();
			if (InBusy || !A64Usb_CanStart(EPhysBulkIn))
				NKern::Sleep(1);
			}
		if (generation != Generation)
			return;											// a new session: its host never asked for this
		if (InBusy)
			{
			A64Usb_EndXfer(EPhysBulkIn);
			InBusy = EFalse;
			return;											// the reader went away: drop the rest
			}
		if (!A64Usb_CanStart(EPhysBulkIn))					// the controller still owns the buffer (a stop)
			return;
		TInt n = Min(aLen, KInMax);
		if (n % KBulkMax == 0)
			--n;											// end each transfer with a short packet
		memcpy(A64Usb_Buf(EPhysBulkIn), aData, n);
		InBusy = A64Usb_Start(EPhysBulkIn, n) == KErrNone;
		++InStarts;
		InFail += !InBusy;
		aData += n;
		aLen -= n;
		}
	}

// "blk put" (a64_dbgmon.cpp, telephony C5b): the host's next aBytes are raw data, not command lines. They gather in a
// 64 KB stage; while it is full the OUT endpoint stays unarmed, so USB holds the host back (no loss) until the
// monitor has written the stage (A64Mon_PutChunk, called from the poll below).

TInt A64Usb_RawIn(TInt aBytes)
	{
	if (!RawStage)
		{
		NKern::ThreadEnterCS();
		RawStage = (TUint8*)Kern::Alloc(KRawStage + KRawXfer);
		NKern::ThreadLeaveCS();
		if (!RawStage)
			return KErrNoMemory;
		}
	RawFill = 0;
	RawFull = EFalse;
	RawLeft = aBytes;
	return KErrNone;
	}

void A64UsbDump();

static void Poll()
	{
	if (AbortIn && A64Usb_Ep0Idle())
		{
		A64Usb_EndXfer(EPhysBulkIn);
		InBusy = AbortIn = EFalse;
		}
	if (RawFull)
		{
		TBool last = RawLeft == 0;
		TInt n = last ? RawFill : RawFill / KSector * KSector;	// whole sectors; the total is too
		const TUint32 generation = Generation;
		A64Mon_PutChunk(RawStage, n, last, UsbWrite);
		if (generation == Generation)						// else Reset() emptied the stage
			{
			memmove(RawStage, RawStage + n, RawFill - n);
			RawFill -= n;
			RawFull = EFalse;
			OutBusy = ArmOut() == KErrNone;
			}
		}
	else if (LineReady)
		{
		TBuf8<160> line(Line);
		Line.Zero();
		LineReady = EFalse;
		++OutLines;
		A64Mon_Line(line, UsbWrite);
		}
	else if (A64Usb_Configured() && Dtr && !InBusy && A64Usb_CanStart(EPhysBulkIn))	// stream new log text
		{
		TUint8* p = A64Usb_Buf(EPhysBulkIn);
		TInt n = A64Log_Read(LogPos, p, KInMax - 1);
		if (n > 0)
			{
			InBusy = A64Usb_Start(EPhysBulkIn, n) == KErrNone;
			++InStarts;
			InFail += !InBusy;
			}
		}
	static TInt ticks;
	if (++ticks % 3000 == 0 && !Dtr)						// until a host opens the port: status on the screen
		A64UsbDump();
	}

static void Append(TDes8& aBuf, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aBuf, aFmt, list);
	VA_END(list);
	}

void A64UsbStatus(TDes8& aBuf)							// the monitor's "usb" command
	{
	SA64UsbStats s;
	A64Usb_Stats(s);
	TUint32 size, ctrl, rsc;
	A64Usb_Trb(EPhysBulkIn, size, ctrl, rsc);
	Append(aBuf, "ev %d rst %d setups %d ep0 %d cfg %d dtr %d in %d/%d/%d busy %d abort %d lines %d logpos %d "
		"trb3 %08x %08x rsc3 %d dma %x/%x/%x restarts %d link %d/%d dbg %d", s.iEvents, s.iResets, s.iSetups, s.iEp0, s.iConfigured, Dtr,
		InStarts, InDone, InFail, InBusy, AbortIn, OutLines, LogPos, size, ctrl, rsc, s.iDmaOwned, s.iDmaPending,
		s.iDmaQuarantined, s.iRestarts, s.iSuspends, s.iResumes, s.iFunctions > 1);
	}

void A64UsbDump()
	{
	SA64UsbStats s;
	A64Usb_Stats(s);
	Kern::Printf("[usb] ev %d rst %d setups %d ep0 %d cfg %d dtr %d in %d/%d/%d busy %d abort %d lines %d dsts %08x",
		s.iEvents, s.iResets, s.iSetups, s.iEp0, s.iConfigured, Dtr, InStarts, InDone, InFail, InBusy, AbortIn, OutLines,
		s.iDsts);
	Kern::Printf("[usb] setups %08x %08x %08x %08x %08x %08x (type req value index; newest = #%d)", s.iSetupLog[0],
		s.iSetupLog[1], s.iSetupLog[2], s.iSetupLog[3], s.iSetupLog[4], s.iSetupLog[5], (s.iSetups + 5) % 6);
	}

const SA64UsbFunction KA64UsbFunction = {
	KDevice, sizeof(KDevice), KQualifier, sizeof(KQualifier), KFragment, sizeof(KFragment), 0, 2,
	KStrings, sizeof(KStrings) / sizeof(KStrings[0]), KEndpoints, sizeof(KEndpoints) / sizeof(KEndpoints[0]),
	ClassSetup, ClassOutData, Ready, Reset, XferComplete, Poll, A64UsbDump, NULL,
	};
