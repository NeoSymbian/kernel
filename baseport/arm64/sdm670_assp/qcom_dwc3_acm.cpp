// SPDX-License-Identifier: Apache-2.0
//
// qcom_dwc3_acm.cpp -- the Pixel's debug console: the SDM670 USB controller (DTB usb@a600000, Synopsys DWC3, device
// mode, high speed) as a USB CDC-ACM serial port, the Mac's /dev/cu.usbmodem*. Its bytes in are lines for the debug
// monitor (a64_dbgmon.cpp); its bytes out are the kernel log ring and the monitor's answers. tools/phone.py drives it.
// ABL's fastboot used this controller to receive our image, so its clocks, the QUSB2 PHY and the Qualcomm wrapper
// are set up: only the device controller is reset and programmed, after Linux drivers/usb/dwc3 (core.c, gadget.c,
// ep0.c). Every transfer is one TRB (LST) started by its own StartTransfer, so there are no TRB rings.
// IDs 0x1209:0x0001 (pid.codes test ID, private development use).
// shortcut: polled every 1 ms on its own DFC queue, no interrupt; HS only; one ACM port.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_dbgmon.h"

const TUint32 KDwc3 = 0x0a600000;
const TUint32 KQscratch = 0x0a6f8800, KQusb2 = 0x088e2000;	// the Qualcomm wrapper; the HS PHY (qusb2-v2)
enum { EHsPhyCtrl = 0x10, ESsPhyCtrl = 0x30 };				// QSCRATCH (Linux dwc3-qcom.c)
enum { EPhyPllStatus = 0x1a0, EPhyPowerDown = 0x210 };		// QUSB2 v2 (Linux phy-qcom-qusb2.c)
enum
	{
	EGctl = 0xc110, EGsnpsid = 0xc120, EGusb2PhyCfg = 0xc200, EGevntAdrLo = 0xc400, EGevntAdrHi = 0xc404,
	EGevntSiz = 0xc408, EGevntCount = 0xc40c, EDcfg = 0xc700, EDctl = 0xc704, EDevten = 0xc708, EDsts = 0xc70c,
	EDalepena = 0xc720, EDepBase = 0xc800,
	};
enum { EDepPar2 = 0x0, EDepPar1 = 0x4, EDepPar0 = 0x8, EDepCmd = 0xc };
enum { ECmdSetEpConfig = 1, ECmdSetXferRes = 2, ECmdClearStall = 5, ECmdSetStall = 4, ECmdStartXfer = 6, ECmdEndXfer = 8, ECmdStartCfg = 9 };
const TUint32 KCmdAct = 1u << 10, KCmdIoc = 1u << 8, KForceRm = 1u << 11;
enum { ETrbHwo = 1, ETrbLst = 2, ETrbIsp = 1 << 10, ETrbIoc = 1 << 11 };
enum { ETrbNormal = 1 << 4, ETrbSetup = 2 << 4, ETrbStatus2 = 3 << 4, ETrbStatus3 = 4 << 4, ETrbData = 5 << 4 };
enum { EEvXferComplete = 1, EEvXferNotReady = 3 };
enum { EDevDisconnect = 0, EDevReset = 1, EDevConnectDone = 2 };
enum { EPhysEp0Out = 0, EPhysEp0In = 1, EPhysBulkOut = 2, EPhysBulkIn = 3, EPhysIntrIn = 5 };
const TInt KEp0Max = 64, KBulkMax = 512;

// DMA area, physically contiguous and mapped non-cached: event buffer, TRBs, setup packet, EP0 data, bulk buffers
const TInt KAreaSize = 0x4000, KEvSize = 4096;
enum { EOffEv = 0x0000, EOffTrb = 0x1000, EOffSetup = 0x1100, EOffEp0Buf = 0x1200, EOffOut = 0x2000, EOffIn = 0x3000 };
const TInt KInMax = 4096;
static TPhysAddr AreaPhys;
static TLinAddr Area;
static TUint32 EvPos;

struct STrb { TUint32 iBpl, iBph, iSize, iCtrl; };
static volatile STrb* Trb(TInt aPhysEp) { return (volatile STrb*)(Area + EOffTrb + 16 * aPhysEp); }
static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KDwc3) + aOff); }
static volatile TUint32& Q(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KQscratch) + aOff); }
static volatile TUint32& P(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KQusb2) + aOff); }

// ---- state
enum TEp0State { ESetup, EDataIn, EDataOut, EWaitStatus2, EWaitStatus3In, EWaitStatus3Out, EStatus };
static TEp0State Ep0;
static TUint8 Setup[8];
static TBool Configured, Dtr, InBusy, OutBusy, AbortIn;
static TUint32 RscIdx[8];
static TUint8 LineCoding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0, 8 };	// 115200 8N1, as the host likes to see
static TUint32 LogPos;
static TBuf8<160> Line;
static TBool LineReady;
static TInt Events, Resets, Setups, InStarts, InDone, InFail, OutLines;
static TUint32 SetupLog[6];

// ---- endpoint commands
static TInt DepCmd(TInt aPhysEp, TUint32 aCmd, TUint32 aP0 = 0, TUint32 aP1 = 0, TUint32 aP2 = 0)
	{
	TUint32 base = EDepBase + 0x10 * aPhysEp;
	R(base + EDepPar2) = aP2;
	R(base + EDepPar1) = aP1;
	R(base + EDepPar0) = aP0;
	R(base + EDepCmd) = aCmd | KCmdAct;
	for (TInt i = 0; i < 1000000; ++i)
		{
		TUint32 c = R(base + EDepCmd);
		if (!(c & KCmdAct))
			{
			if (aPhysEp < 8 && (aCmd & 0xf) == ECmdStartXfer)
				RscIdx[aPhysEp] = (c >> 16) & 0x7f;
			return (c >> 12) & 0xf ? KErrGeneral : KErrNone;
			}
		}
	return KErrTimedOut;
	}

static TInt EpConfig(TInt aPhysEp, TUint32 aType, TInt aMaxPacket, TUint32 aAction, TUint32 aInterval = 0)
	{
	TUint32 p0 = (aType << 1) | (aMaxPacket << 3) | aAction;
	if (aPhysEp & 1)
		p0 |= (aPhysEp >> 1) << 17;						// IN: its TX FIFO
	TUint32 p1 = (1u << 8) | ((TUint32)aPhysEp << 25);	// XferComplete events, the endpoint number
	if (aPhysEp <= 1)
		p1 |= 1u << 10;									// EP0 also needs XferNotReady (the status phase)
	if (aInterval)
		p1 |= (aInterval - 1) << 16;
	return DepCmd(aPhysEp, ECmdSetEpConfig, p0, p1);
	}

// one TRB, one transfer
static TInt Start(TInt aPhysEp, TUint32 aType, TUint32 aBufOff, TInt aLen)
	{
	volatile STrb* t = Trb(aPhysEp);
	t->iBpl = AreaPhys + aBufOff;
	t->iBph = 0;
	t->iSize = aLen;
	t->iCtrl = aType | ETrbLst | ETrbIoc | ((aPhysEp & 1) ? 0 : ETrbIsp) | ETrbHwo;
	__e32_memory_barrier();
	return DepCmd(aPhysEp, ECmdStartXfer, 0, AreaPhys + EOffTrb + 16 * aPhysEp);
	}

static void EndXfer(TInt aPhysEp)
	{
	if (aPhysEp < 8 && (Trb(aPhysEp)->iCtrl & ETrbHwo))
		DepCmd(aPhysEp, ECmdEndXfer | KForceRm | (RscIdx[aPhysEp] << 16));
	}

static void StartSetup()
	{
	Ep0 = ESetup;
	Start(EPhysEp0Out, ETrbSetup, EOffSetup, 8);
	}

// ---- descriptors
static const TUint8 KDevice[] = {
	18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, KEp0Max, 0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 1, 2, 3, 1 };
static const TUint8 KQualifier[] = { 10, 6, 0x00, 0x02, 0x02, 0x00, 0x00, KEp0Max, 1, 0 };
static const TUint8 KConfig[] = {
	9, 2, 67, 0, 2, 1, 0, 0x80, 50,							// configuration: 2 interfaces, bus powered 100 mA
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

static TInt StringDesc(TInt aIndex, TUint8* aBuf)
	{
	if (aIndex == 0)
		{
		aBuf[0] = 4; aBuf[1] = 3; aBuf[2] = 0x09; aBuf[3] = 0x04;
		return 4;
		}
	if (aIndex >= (TInt)(sizeof(KStrings) / sizeof(KStrings[0])))
		return -1;
	TInt n = 0;
	for (const char* s = KStrings[aIndex]; *s; ++s, ++n)
		{
		aBuf[2 + 2 * n] = (TUint8)*s;
		aBuf[3 + 2 * n] = 0;
		}
	aBuf[0] = (TUint8)(2 + 2 * n);
	aBuf[1] = 3;
	return aBuf[0];
	}

static void Configure()
	{
	DepCmd(EPhysEp0Out, ECmdStartCfg | (2 << 16));		// transfer resources for the other endpoints
	const struct { TInt iEp; TUint32 iType; TInt iMax; TUint32 iInterval; } eps[] = {
		{ EPhysBulkOut, 2, KBulkMax, 0 }, { EPhysBulkIn, 2, KBulkMax, 0 }, { EPhysIntrIn, 3, 16, 9 } };
	for (TUint i = 0; i < sizeof(eps) / sizeof(eps[0]); ++i)
		{
		EpConfig(eps[i].iEp, eps[i].iType, eps[i].iMax, 0, eps[i].iInterval);
		DepCmd(eps[i].iEp, ECmdSetXferRes, 1);
		R(EDalepena) |= 1u << eps[i].iEp;
		}
	Configured = ETrue;
	InBusy = EFalse;
	OutBusy = Start(EPhysBulkOut, ETrbNormal, EOffOut, KBulkMax) == KErrNone;
	}

// a SETUP packet arrived: answer it (data stage started here), or stall
static void HandleSetup()
	{
	SetupLog[Setups % 6] = (Setup[0] << 24) | (Setup[1] << 16) | (Setup[2] << 8) | Setup[4];	// type req value index
	++Setups;
	TUint8 type = Setup[0], req = Setup[1];
	TUint16 value = Setup[2] | (Setup[3] << 8), length = Setup[6] | (Setup[7] << 8);
	TUint8* buf = (TUint8*)(Area + EOffEp0Buf);
	TInt in = -1;											// bytes to send, -1 = none
	TBool out = EFalse, ok = ETrue;
	if ((type & 0x60) == 0x00)								// standard
		switch (req)
			{
		case 6:												// GET_DESCRIPTOR
			switch (value >> 8)
				{
			case 1: memcpy(buf, KDevice, sizeof(KDevice)); in = sizeof(KDevice); break;
			case 2: memcpy(buf, KConfig, sizeof(KConfig)); in = sizeof(KConfig); break;
			case 3: in = StringDesc(value & 0xff, buf); ok = in >= 0; break;
			case 6: memcpy(buf, KQualifier, sizeof(KQualifier)); in = sizeof(KQualifier); break;
			default: ok = EFalse; break;
				}
			break;
		case 5:												// SET_ADDRESS: before the status stage
			R(EDcfg) = (R(EDcfg) & ~(0x7f << 3)) | ((value & 0x7f) << 3);
			break;
		case 9:												// SET_CONFIGURATION
			if (value == 1 && !Configured)
				Configure();
			else if (value == 1)							// again: the host restarts every data toggle at DATA0
				for (TInt ep = EPhysBulkOut; ep <= EPhysIntrIn; ++ep)
					DepCmd(ep, ECmdClearStall);
			break;
		case 8: buf[0] = Configured ? 1 : 0; in = 1; break;	// GET_CONFIGURATION
		case 0: buf[0] = buf[1] = 0; in = 2; break;			// GET_STATUS
		case 1:												// CLEAR_FEATURE: an endpoint's halt (macOS on each open)
			if ((type & 0x1f) == 2 && value == 0)			// -- resets its data toggle: so must the controller,
				{											// or both sides drop each other's packets as repeats
				TInt ep = Setup[4] & 0x0f, phys = 2 * ep + ((Setup[4] & 0x80) ? 1 : 0);
				if (phys >= 2 && phys < 8)
					DepCmd(phys, ECmdClearStall);
				}
			break;
		case 3: case 11: break;								// SET_FEATURE, SET_INTERFACE: accept
		default: ok = EFalse; break;
			}
	else if ((type & 0x60) == 0x20)						// class (CDC ACM)
		switch (req)
			{
		case 0x20: out = ETrue; break;						// SET_LINE_CODING: 7 bytes follow
		case 0x21: memcpy(buf, LineCoding, 7); in = 7; break;	// GET_LINE_CODING
		case 0x22:											// SET_CONTROL_LINE_STATE: bit 0 = DTR (port open)
			Dtr = value & 1;
			AbortIn = InBusy;								// a closed port never collects its last piece: abort it
															// (later: End Transfer stalls during a control transfer)
			if (Dtr)
				LogPos = 0;									// a new session sees the whole log
			break;
		default: ok = EFalse; break;
			}
	else
		ok = EFalse;
	if (!ok)
		{
		DepCmd(EPhysEp0Out, ECmdSetStall);
		StartSetup();
		return;
		}
	if (in >= 0)
		{
		Ep0 = EDataIn;
		Start(EPhysEp0In, ETrbData, EOffEp0Buf, Min(in, (TInt)length));
		}
	else if (out && length)
		{
		Ep0 = EDataOut;
		Start(EPhysEp0Out, ETrbData, EOffEp0Buf, KEp0Max);	// OUT lengths are whole packets
		}
	else
		Ep0 = EWaitStatus2;
	}

static void Ep0Event(TInt aPhysEp, TInt aType, TInt aStatus)
	{
	if (aType == EEvXferComplete)
		{
		if (Ep0 == ESetup && aPhysEp == EPhysEp0Out)
			{
			memcpy(Setup, (const TAny*)(Area + EOffSetup), 8);
			HandleSetup();
			}
		else if (Ep0 == EDataIn)
			Ep0 = EWaitStatus3Out;
		else if (Ep0 == EDataOut)
			{
			if (Setup[1] == 0x20)
				memcpy(LineCoding, (const TAny*)(Area + EOffEp0Buf), 7);
			Ep0 = EWaitStatus3In;
			}
		else if (Ep0 == EStatus)
			StartSetup();
		}
	else if (aType == EEvXferNotReady && (aStatus & 3) == 2)	// the host wants the status phase
		{
		if (Ep0 == EWaitStatus2 || Ep0 == EWaitStatus3In || Ep0 == EWaitStatus3Out)
			{
			TUint32 kind = Ep0 == EWaitStatus2 ? ETrbStatus2 : ETrbStatus3;
			Ep0 = EStatus;
			Start(aPhysEp, kind, EOffEp0Buf, 0);
			}
		}
	}

// ---- the event buffer
static void ProcessEvents()
	{
	TUint32 count = R(EGevntCount) & 0xffff;
	while (count >= 4)
		{
		TUint32 e = *(volatile TUint32*)(Area + EOffEv + EvPos);
		EvPos = (EvPos + 4) % KEvSize;
		R(EGevntCount) = 4;
		count -= 4;
		++Events;
		if (!(e & 1))										// an endpoint event
			{
			TInt ep = (e >> 1) & 0x1f, type = (e >> 6) & 0xf, status = (e >> 12) & 0xf;
			if (ep <= 1)
				Ep0Event(ep, type, status);
			else if (type == EEvXferComplete && ep == EPhysBulkIn)
				{
				InBusy = EFalse;
				++InDone;
				}
			else if (type == EEvXferComplete && ep == EPhysBulkOut)
				{
				TInt got = KBulkMax - (Trb(EPhysBulkOut)->iSize & 0xffffff);
				const TUint8* p = (const TUint8*)(Area + EOffOut);
				for (TInt i = 0; i < got; ++i)
					{
					if (p[i] == '\r' || p[i] == '\n')
						LineReady = LineReady || Line.Length() > 0;
					else if (!LineReady && Line.Length() < Line.MaxLength())
						Line.Append(p[i]);
					}
				OutBusy = Start(EPhysBulkOut, ETrbNormal, EOffOut, KBulkMax) == KErrNone;
				}
			}
		else if (((e >> 1) & 0x7f) == 0)					// a device event
			{
			TInt type = (e >> 8) & 0xf;
			if (type == EDevReset)
				{
				++Resets;
				EndXfer(EPhysBulkOut);
				EndXfer(EPhysBulkIn);
				R(EDcfg) &= ~(0x7f << 3);
				Configured = Dtr = InBusy = OutBusy = EFalse;
				}
			else if (type == EDevConnectDone)
				{
				EpConfig(EPhysEp0Out, 0, KEp0Max, 2u << 30);	// modify: EP0 max packet for this speed
				EpConfig(EPhysEp0In, 0, KEp0Max, 2u << 30);
				}
			else if (type == EDevDisconnect)
				Configured = Dtr = EFalse;
			}
		}
	}

// the monitor's writer: blocks (polling the controller) until each piece is sent, gives up after ~2 s
static void UsbWrite(const TUint8* aData, TInt aLen)
	{
	while (aLen > 0 && Configured && Dtr)
		{
		for (TInt i = 0; InBusy && i < 2000; ++i)
			{
			ProcessEvents();
			if (InBusy)
				NKern::Sleep(1);
			}
		if (InBusy)
			{
			EndXfer(EPhysBulkIn);
			InBusy = EFalse;
			return;											// the reader went away: drop the rest
			}
		TInt n = Min(aLen, KInMax);
		if (n % KBulkMax == 0)
			--n;											// end each transfer with a short packet
		memcpy((TAny*)(Area + EOffIn), aData, n);
		InBusy = Start(EPhysBulkIn, ETrbNormal, EOffIn, n) == KErrNone;
		++InStarts;
		InFail += !InBusy;
		aData += n;
		aLen -= n;
		}
	}

void A64UsbDump();
static void PollDfcFn(TAny*);
static TDfc PollDfc(PollDfcFn, NULL, 1);
static void PollTimerFn(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTimerFn, NULL);

static void PollDfcFn(TAny*)
	{
	ProcessEvents();
	if (AbortIn && Ep0 == ESetup)
		{
		EndXfer(EPhysBulkIn);
		InBusy = AbortIn = EFalse;
		}
	if (LineReady)
		{
		TBuf8<160> line(Line);
		Line.Zero();
		LineReady = EFalse;
		++OutLines;
		A64Mon_Line(line, UsbWrite);
		}
	else if (Configured && Dtr && !InBusy)					// stream new log text
		{
		TUint8* p = (TUint8*)(Area + EOffIn);
		TInt n = A64Log_Read(LogPos, p, KInMax - 1);
		if (n > 0)
			{
			InBusy = Start(EPhysBulkIn, ETrbNormal, EOffIn, n) == KErrNone;
			++InStarts;
			InFail += !InBusy;
			}
		}
	static TInt ticks;
	if (++ticks % 3000 == 0 && !Dtr)						// until a host opens the port: status on the screen
		A64UsbDump();
	PollTimer.OneShot(1);								// not Again(): it refuses (KErrArgument) once a slow poll put its next expiry in the past
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
	volatile STrb* t = Trb(EPhysBulkIn);
	Append(aBuf, "ev %d rst %d setups %d ep0 %d cfg %d dtr %d in %d/%d/%d busy %d abort %d lines %d logpos %d "
		"trb3 %08x %08x rsc3 %d", Events, Resets, Setups, Ep0, Configured, Dtr, InStarts, InDone, InFail, InBusy,
		AbortIn, OutLines, LogPos, t->iSize, t->iCtrl, RscIdx[EPhysBulkIn]);
	}

void A64UsbDump()
	{
	Kern::Printf("[usb] ev %d rst %d setups %d ep0 %d cfg %d dtr %d in %d/%d/%d busy %d abort %d lines %d dsts %08x",
		Events, Resets, Setups, Ep0, Configured, Dtr, InStarts, InDone, InFail, InBusy, AbortIn, OutLines, R(EDsts));
	Kern::Printf("[usb] setups %08x %08x %08x %08x %08x %08x (type req value index; newest = #%d)", SetupLog[0],
		SetupLog[1], SetupLog[2], SetupLog[3], SetupLog[4], SetupLog[5], (Setups + 5) % 6);
	}

static TDynamicDfcQue* UsbQ;

void A64StartUsbConsole()
	{
	// ABL powers the USB controller only for fastboot: after a normal boot (the phone installed, boot_a) its power
	// domain and clocks are off, and a DWC3 register access would stall the bus -- so look at them first (gcc-sdm845.c
	// usb30_prim_gdsc 0xf004 PWR_ON bit 31, gcc_usb30_prim_master_clk 0xf00c CLK_OFF bit 31).
	// shortcut: no console then; bringing the controller and its PHY up ourselves is a follow-up
	const TUint32 gdsc = *(volatile TUint32*)A64_Io(0x0010f004), master = *(volatile TUint32*)A64_Io(0x0010f00c);
	if (!(gdsc & (1u << 31)) || (master & (1u << 31)))
		{
		Kern::Printf("[usb] controller off (gdsc %08x, master clock %08x): a normal boot, no USB console", gdsc, master);
		return;
		}
	TUint32 id = R(EGsnpsid);
	Kern::Printf("[usb] DWC3 id %08x, ABL left dctl %08x dsts %08x dcfg %08x gctl %08x", id, R(EDctl), R(EDsts),
		R(EDcfg), R(EGctl));
	if ((id >> 16) != 0x5533)
		return;
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(KAreaSize, AreaPhys, 12);
	DPlatChunkHw* chunk = NULL;
	if (r == KErrNone)
		r = DPlatChunkHw::New(chunk, AreaPhys, KAreaSize, EMapAttrSupRw | EMapAttrBufferedNC);
	_LIT(KName, "UsbQ");
	if (r == KErrNone)
		r = Kern::DynamicDfcQCreate(UsbQ, 27, KName);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		{
		Kern::Printf("[usb] no DMA area / queue: %d", r);
		return;
		}
	Area = chunk->LinearAddress();
	memclr((TAny*)Area, KAreaSize);

	// stop what ABL left running, then reset the device controller (the PHY setup stays)
	R(EDctl) &= ~(1u << 31);								// RunStop off
	for (TInt i = 0; i < 1000000 && !(R(EDsts) & (1u << 22)); ++i)		// until halted
		{}
	TUint32 phy = R(EGusb2PhyCfg);
	R(EDctl) |= 1u << 30;									// CSFTRST
	for (TInt i = 0; i < 1000000 && (R(EDctl) & (1u << 30)); ++i)
		{}
	R(EGusb2PhyCfg) = phy & ~((1u << 6) | (1u << 8));		// DTB: dis_u2_susphy_quirk, dis_enblslpm_quirk
	R(EGctl) = (R(EGctl) & ~(3u << 12)) | (2u << 12);		// device mode
	R(EGevntAdrLo) = AreaPhys + EOffEv;
	R(EGevntAdrHi) = 0;
	R(EGevntSiz) = (1u << 31) | KEvSize;					// interrupt masked: polled
	R(EGevntCount) = R(EGevntCount) & 0xffff;				// drop stale events
	EvPos = 0;
	R(EDcfg) = (R(EDcfg) & ~((7u) | (0x7f << 3))) | 0;		// high speed, address 0
	R(EDevten) = (1u << 0) | (1u << 1) | (1u << 2);			// disconnect, reset, connect done
	// ABL drops the session when it leaves fastboot: no VBUS sensing in device mode, so override it as Linux
	// dwc3-qcom does (vbus_override_enable), and power the HS PHY back up if ABL powered it down
	Kern::Printf("[usb] ABL left hsphy %08x ssphy %08x qusb2 pd %08x pll %08x", Q(EHsPhyCtrl), Q(ESsPhyCtrl),
		P(EPhyPowerDown), P(EPhyPllStatus));
	Q(ESsPhyCtrl) |= 1u << 24;								// LANE0_PWR_PRESENT
	Q(EHsPhyCtrl) |= (1u << 20) | (1u << 28);				// UTMI_OTG_VBUS_VALID, SW_SESSVLD_SEL
	if (P(EPhyPowerDown) & 1)
		P(EPhyPowerDown) &= ~1u;
	DepCmd(EPhysEp0Out, ECmdStartCfg);
	EpConfig(EPhysEp0Out, 0, KEp0Max, 0);
	EpConfig(EPhysEp0In, 0, KEp0Max, 0);
	DepCmd(EPhysEp0Out, ECmdSetXferRes, 1);
	DepCmd(EPhysEp0In, ECmdSetXferRes, 1);
	R(EDalepena) = 3;
	StartSetup();
	R(EDctl) = (R(EDctl) & ~(1u << 30)) | (1u << 31);		// RunStop: connect
	A64UsbDump();

	PollDfc.SetDfcQ(UsbQ);
	PollTimer.OneShot(1);
	}
