// SPDX-License-Identifier: Apache-2.0
//
// qcom_dwc3.cpp -- the SDM670 USB device controller (DTB usb@a600000, Synopsys DWC3, device mode, high speed) and the
// core of our gadget: bring-up, the event ring, EP0 with the standard requests, and each endpoint's transfers and DMA
// buffer. A function (a64_usb_function.h) supplies descriptors, class requests and endpoint callbacks; today that is
// the debug monitor's ACM serial port (common/specific/a64_usb_acm_monitor.cpp), the Mac's /dev/cu.usbmodem*.
// ABL's fastboot used this controller to receive our image, so its clocks, the QUSB2 PHY and the Qualcomm wrapper
// are set up: only the device controller is reset and programmed, after Linux drivers/usb/dwc3 (core.c, gadget.c,
// ep0.c). Every transfer is one TRB (LST) started by its own StartTransfer, so there are no TRB rings.
// shortcut: polled every 1 ms on its own DFC queue, no interrupt; HS only; one function.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_usb_function.h"
#include "qcom_rpmh.h"

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
enum { EPhysEp0Out = 0, EPhysEp0In = 1 };
const TInt KEp0Max = 64, KPhysEps = 8;

// DMA area, physically contiguous and mapped non-cached: event buffer, TRBs, setup packet, EP0 data, then the
// function's endpoint buffers (from EOffEpBufs, in its endpoint order)
const TInt KAreaSize = 0x4000, KEvSize = 4096;
enum { EOffEv = 0x0000, EOffTrb = 0x1000, EOffSetup = 0x1100, EOffEp0Buf = 0x1200, EOffEpBufs = 0x2000 };
static TPhysAddr AreaPhys;
static TLinAddr Area;
static TUint32 EvPos;

static const SA64UsbFunction& Fn = KA64UsbFunction;
static TInt BufOff[KPhysEps], Armed[KPhysEps];				// each endpoint's buffer; the length its transfer was armed with

struct STrb { TUint32 iBpl, iBph, iSize, iCtrl; };
static volatile STrb* Trb(TInt aPhysEp) { return (volatile STrb*)(Area + EOffTrb + 16 * aPhysEp); }
static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KDwc3) + aOff); }
static volatile TUint32& Q(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KQscratch) + aOff); }
static volatile TUint32& P(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KQusb2) + aOff); }

// ---- state
enum TEp0State { ESetup, EDataIn, EDataOut, EWaitStatus2, EWaitStatus3In, EWaitStatus3Out, EStatus };
static TEp0State Ep0;
static TUint8 Setup[8];
static TBool Configured;
static TUint32 RscIdx[KPhysEps];
static TInt Events, Resets, Setups;
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
			if (aPhysEp < KPhysEps && (aCmd & 0xf) == ECmdStartXfer)
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
	Armed[aPhysEp] = aLen;
	return DepCmd(aPhysEp, ECmdStartXfer, 0, AreaPhys + EOffTrb + 16 * aPhysEp);
	}

static void EndXfer(TInt aPhysEp)
	{
	if (aPhysEp < KPhysEps && (Trb(aPhysEp)->iCtrl & ETrbHwo))
		DepCmd(aPhysEp, ECmdEndXfer | KForceRm | (RscIdx[aPhysEp] << 16));
	}

static void StartSetup()
	{
	Ep0 = ESetup;
	Start(EPhysEp0Out, ETrbSetup, EOffSetup, 8);
	}

// ---- the function's side
TUint8* A64Usb_Buf(TInt aPhysEp) { return (TUint8*)(Area + BufOff[aPhysEp]); }
TInt A64Usb_Start(TInt aPhysEp, TInt aLen) { return Start(aPhysEp, ETrbNormal, BufOff[aPhysEp], aLen); }
void A64Usb_EndXfer(TInt aPhysEp) { EndXfer(aPhysEp); }
TBool A64Usb_Ep0Idle() { return Ep0 == ESetup; }
TBool A64Usb_Configured() { return Configured; }

void A64Usb_Stats(SA64UsbStats& aStats)
	{
	aStats.iEvents = Events;
	aStats.iResets = Resets;
	aStats.iSetups = Setups;
	aStats.iEp0 = Ep0;
	aStats.iConfigured = Configured;
	aStats.iDsts = R(EDsts);
	memcpy(aStats.iSetupLog, SetupLog, sizeof(SetupLog));
	}

void A64Usb_Trb(TInt aPhysEp, TUint32& aSize, TUint32& aCtrl, TUint32& aRscIdx)
	{
	volatile STrb* t = Trb(aPhysEp);
	aSize = t->iSize;
	aCtrl = t->iCtrl;
	aRscIdx = RscIdx[aPhysEp];
	}

// ---- descriptors
static TInt StringDesc(TInt aIndex, TUint8* aBuf)
	{
	if (aIndex == 0)
		{
		aBuf[0] = 4; aBuf[1] = 3; aBuf[2] = 0x09; aBuf[3] = 0x04;
		return 4;
		}
	if (aIndex >= Fn.iStringCount)
		return -1;
	TInt n = 0;
	for (const char* s = Fn.iStrings[aIndex]; *s; ++s, ++n)
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
	for (TInt i = 0; i < Fn.iEndpointCount; ++i)
		{
		const SA64UsbEndpoint& e = Fn.iEndpoints[i];
		EpConfig(e.iPhysEp, e.iType, e.iMaxPacket, 0, e.iInterval);
		DepCmd(e.iPhysEp, ECmdSetXferRes, 1);
		R(EDalepena) |= 1u << e.iPhysEp;
		}
	Configured = ETrue;
	Fn.iConfigured();
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
			case 1: memcpy(buf, Fn.iDevice, Fn.iDeviceLen); in = Fn.iDeviceLen; break;
			case 2: memcpy(buf, Fn.iConfig, Fn.iConfigLen); in = Fn.iConfigLen; break;
			case 3: in = StringDesc(value & 0xff, buf); ok = in >= 0; break;
			case 6: memcpy(buf, Fn.iQualifier, Fn.iQualifierLen); in = Fn.iQualifierLen; break;
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
				{
				TInt last = 0;
				for (TInt i = 0; i < Fn.iEndpointCount; ++i)
					last = Max(last, Fn.iEndpoints[i].iPhysEp);
				for (TInt ep = 2; ep <= last; ++ep)
					DepCmd(ep, ECmdClearStall);
				}
			break;
		case 8: buf[0] = Configured ? 1 : 0; in = 1; break;	// GET_CONFIGURATION
		case 0: buf[0] = buf[1] = 0; in = 2; break;			// GET_STATUS
		case 1:												// CLEAR_FEATURE: an endpoint's halt (macOS on each open)
			if ((type & 0x1f) == 2 && value == 0)			// -- resets its data toggle: so must the controller,
				{											// or both sides drop each other's packets as repeats
				TInt ep = Setup[4] & 0x0f, phys = 2 * ep + ((Setup[4] & 0x80) ? 1 : 0);
				if (phys >= 2 && phys < KPhysEps)
					DepCmd(phys, ECmdClearStall);
				}
			break;
		case 3: case 11: break;								// SET_FEATURE, SET_INTERFACE: accept
		default: ok = EFalse; break;
			}
	else if ((type & 0x60) == 0x20)						// class: the function's
		{
		TInt r = Fn.iClassSetup(Setup, buf);
		ok = r != KA64UsbStall;
		out = r == KA64UsbDataOut;
		if (r >= 0)
			in = r;
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
			Fn.iClassOutData(Setup, (const TUint8*)(Area + EOffEp0Buf));
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
void A64Usb_ProcessEvents()
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
			else if (type == EEvXferComplete && ep < KPhysEps)
				Fn.iXferComplete(ep, Armed[ep] - (TInt)(Trb(ep)->iSize & 0xffffff));
			}
		else if (((e >> 1) & 0x7f) == 0)					// a device event
			{
			TInt type = (e >> 8) & 0xf;
			if (type == EDevReset)
				{
				++Resets;
				for (TInt i = 0; i < Fn.iEndpointCount; ++i)
					if (Fn.iEndpoints[i].iBufSize)
						EndXfer(Fn.iEndpoints[i].iPhysEp);
				R(EDcfg) &= ~(0x7f << 3);
				Configured = EFalse;
				Fn.iBus(EA64UsbReset);
				}
			else if (type == EDevConnectDone)
				{
				EpConfig(EPhysEp0Out, 0, KEp0Max, 2u << 30);	// modify: EP0 max packet for this speed
				EpConfig(EPhysEp0In, 0, KEp0Max, 2u << 30);
				}
			else if (type == EDevDisconnect)
				{
				Configured = EFalse;
				Fn.iBus(EA64UsbDisconnect);
				}
			}
		}
	}

static void PollDfcFn(TAny*);
static TDfc PollDfc(PollDfcFn, NULL, 1);
static void PollTimerFn(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTimerFn, NULL);

static void PollDfcFn(TAny*)
	{
	A64Usb_ProcessEvents();
	Fn.iPoll();
	PollTimer.OneShot(1);								// not Again(): it refuses (KErrArgument) once a slow poll put its next expiry in the past
	}

static void Append(TDes8& aBuf, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aBuf, aFmt, list);
	VA_END(list);
	}

static TDynamicDfcQue* UsbQ;

TInt A64UsbPowerOn(TUint32& aPll);						// qcom_usbpower.cpp

// reset the device controller and start it as the debug console's gadget (the PHY setup stays)
static void Bringup()
	{
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
	}

void A64UsbPowerOff();									// qcom_usbpower.cpp
void A64UsbPowerDiag(TDes8& aOut);

// the monitor's "usb cold" (debug, a fastboot RAM boot): turn USB off the way a normal boot finds it, then bring it up
// ourselves (qcom_usbpower.cpp) and restart the console -- one RAM boot per try instead of a flash and a normal boot.
// Runs in the poll DFC (the monitor line came from it); the host sees the console vanish and come back.
extern TInt UsbVoteMask;
void A64UsbCold(TDes8& aOut, TInt aMask)
	{
	UsbVoteMask = aMask;
	R(EDctl) &= ~(1u << 31);
	for (TInt i = 0; i < 1000000 && !(R(EDsts) & (1u << 22)); ++i)
		{}
	A64UsbPowerOff();
	NKern::Sleep(NKern::TimerTicks(200));
	TUint32 pll = 0;
	TInt r = A64UsbPowerOn(pll);
	TBuf8<200> d;
	A64UsbPowerDiag(d);
	Kern::Printf("[usb] cold (votes %d): powered up -> %d, PLL %02x; %S", aMask, r, pll, &d);
	Append(aOut, "usb cold: powered up -> %d, PLL %02x", r, pll);
	if (r != KErrNone)
		return;
	Ep0 = ESetup;
	Configured = EFalse;
	Fn.iBus(EA64UsbRestart);
	memclr((TAny*)Area, KAreaSize);
	Bringup();
	Fn.iDump();
	}

void A64StartUsbConsole()
	{
	// ABL powers the USB controller only for fastboot: after a normal boot (the phone installed) its power domain and
	// clocks are off, and a DWC3 register access would stall the bus -- so look at them first (usb30_prim GDSC 0xf004
	// PWR_ON bit 31, master clock 0xf00c CLK_OFF bit 31). Off: only in dev mode (the kernel console on at boot,
	// CONSOLE=1) does the debug console power USB itself (qcom_usbpower.cpp) -- it gives full control of the phone.
	// A fastboot RAM boot keeps it as before: whoever can RAM-boot holds the phone already.
	const TUint32 gdsc = *(volatile TUint32*)A64_Io(0x0010f004), master = *(volatile TUint32*)A64_Io(0x0010f00c);
	if (!(gdsc & (1u << 31)) || (master & (1u << 31)))
		{
		if (!A64_ConsoleShown())
			{
			Kern::Printf("[usb] controller off (a normal boot) and not in dev mode: no USB debug console");
			return;
			}
		TUint32 pll = 0;
		TInt r = A64UsbPowerOn(pll);
		Kern::Printf("[usb] controller off (gdsc %08x, master clock %08x): powered up -> %d, QUSB2 PLL status %02x (ABL's: 17)",
			gdsc, master, r, pll);
		if (r != KErrNone)
			return;
		}
	TUint32 id = R(EGsnpsid);
	Kern::Printf("[usb] DWC3 id %08x, ABL left dctl %08x dsts %08x dcfg %08x gctl %08x", id, R(EDctl), R(EDsts),
		R(EDcfg), R(EGctl));
	if ((id >> 16) != 0x5533)
		return;
	TInt off = EOffEpBufs;									// the function's endpoint buffers
	for (TInt i = 0; i < Fn.iEndpointCount; ++i)
		{
		const SA64UsbEndpoint& e = Fn.iEndpoints[i];
		if (e.iPhysEp < 2 || e.iPhysEp >= KPhysEps || off + e.iBufSize > KAreaSize)
			{
			Kern::Printf("[usb] function endpoint %d (phys %d, buffer %d) doesn't fit", i, e.iPhysEp, e.iBufSize);
			return;
			}
		BufOff[e.iPhysEp] = off;
		off += e.iBufSize;
		}
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

	Bringup();
	Fn.iDump();

	PollDfc.SetDfcQ(UsbQ);
	PollTimer.OneShot(1);
	}
