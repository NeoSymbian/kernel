// SPDX-License-Identifier: Apache-2.0
//
// qcom_dwc3.cpp -- the SDM670 USB device controller (DTB usb@a600000, Synopsys DWC3, device mode, high speed) and the
// core of our gadget: bring-up, the event ring, EP0 with the standard requests, and each endpoint's transfers and DMA
// buffer. A function (a64_usb_function.h) supplies descriptors, class requests and endpoint callbacks; today that is
// the debug monitor's ACM serial port (common/specific/a64_usb_acm_monitor.cpp), the Mac's /dev/cu.usbmodem*.
// ABL's fastboot used this controller to receive our image, so its clocks, the QUSB2 PHY and the Qualcomm wrapper
// are set up: only the device controller is reset and programmed, after Linux v6.12 drivers/usb/dwc3 (core.c,
// gadget.c, ep0.c, dwc3-qcom.c). Every transfer is one TRB (LST) started by its own StartTransfer: no TRB rings.
// Recovery (a64_usb_dma.h): an endpoint's TRB and buffer stay the controller's until its transfer or its ENDTRANSFER
// completes; a SET_CONFIGURATION waits for its stops before its status stage; an active control transfer at a bus
// reset, or a failed start or stop, restarts the controller (halt, core reset, bring-up), the one thing that frees
// a quarantined endpoint. After startup only the poll DFC (UsbQ) touches the controller and the DMA area.
// shortcut: polled every 1 ms on its own DFC queue, no interrupt; HS only; one function.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_usb_function.h"
#include "a64_usb_dma.h"
#include "qcom_rpmh.h"

const TUint32 KDwc3 = 0x0a600000;
const TUint32 KQscratch = 0x0a6f8800, KQusb2 = 0x088e2000;	// the Qualcomm wrapper; the HS PHY (qusb2-v2)
enum { EGeneralCfg = 0x08, EHsPhyCtrl = 0x10, ESsPhyCtrl = 0x30 };	// QSCRATCH (Linux dwc3-qcom.c)
const TUint32 KUtmiClockSelect = 1u << 0, KPipePhyStatus = 1u << 3, KUtmiClockDisable = 1u << 8;	// its GENERAL_CFG
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
enum { EEvXferComplete = 1, EEvXferNotReady = 3, EEvCommandComplete = 7 };
enum { EDevDisconnect = 0, EDevReset = 1, EDevConnectDone = 2 };
enum { EPhysEp0Out = 0, EPhysEp0In = 1 };
const TInt KEp0Max = 64, KPhysEps = 8;
const TInt KMaxEvents = 64;									// per pass; the rest stay in the ring for the next
const TUint32 KStopTimeout = 2000;							// ticks (ms) for an ENDTRANSFER to complete

// DMA area, physically contiguous and mapped non-cached: event buffer, TRBs, setup packet, EP0 data, then the
// function's endpoint buffers (from EOffEpBufs, in its endpoint order)
const TInt KAreaSize = 0x4000, KEvSize = 4096;
enum { EOffEv = 0x0000, EOffTrb = 0x1000, EOffSetup = 0x1100, EOffEp0Buf = 0x1200, EOffEpBufs = 0x2000 };
static TPhysAddr AreaPhys;
static TLinAddr Area;
static TUint32 EvPos;

static const SA64UsbFunction& Fn = KA64UsbFunction;
static TInt BufOff[KPhysEps], Armed[KPhysEps];				// each endpoint's buffer; the length its transfer was armed with
static TUint32 FnMask;										// the function's endpoints

struct STrb { TUint32 iBpl, iBph, iSize, iCtrl; };
static volatile STrb* Trb(TInt aPhysEp) { return (volatile STrb*)(Area + EOffTrb + 16 * aPhysEp); }
static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KDwc3) + aOff); }
static volatile TUint32& Q(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KQscratch) + aOff); }
static volatile TUint32& P(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KQusb2) + aOff); }

// ---- state
static TBool Running, Restart;								// the controller runs; it must be restarted (from the poll)
static TEp0State Ep0;
static TBool Ep0RearmNeeded;								// a disconnect left EP0 idle: re-arm SETUP on the next connect
static TUint8 Setup[8];
static TBool Configured;
static TInt PendingConfig = -1;								// a deferred SET_CONFIGURATION's value
static TInt DeferredStatusEp = -1;							// its status stage, once the host asked for it
static TUint32 RscIdx[KPhysEps], StopTick[KPhysEps];
static TUsbDmaOwnership Dma;
static TInt Events, Resets, Setups, Restarts;
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
	Kern::Printf("[usb] endpoint timeout: ep %d cmd %08x final %08x par %08x/%08x/%08x", aPhysEp, aCmd,
		R(base + EDepCmd), aP0, aP1, aP2);
	Kern::Printf("[usb] timeout clocks: general %08x gctl %08x usb2 %08x dctl %08x dsts %08x", Q(EGeneralCfg),
		R(EGctl), R(EGusb2PhyCfg), R(EDctl), R(EDsts));
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
	if (aPhysEp >= 2 && !Dma.CanStart(aPhysEp))
		return KErrNotReady;
	volatile STrb* t = Trb(aPhysEp);
	t->iBpl = AreaPhys + aBufOff;
	t->iBph = 0;
	t->iSize = aLen;
	t->iCtrl = aType | ETrbLst | ETrbIoc | ((aPhysEp & 1) ? 0 : ETrbIsp) | ETrbHwo;
	__e32_memory_barrier();
	Armed[aPhysEp] = aLen;
	TInt r = DepCmd(aPhysEp, ECmdStartXfer, 0, AreaPhys + EOffTrb + 16 * aPhysEp);
	if (aPhysEp >= 2)
		{
		if (r == KErrNone)
			Dma.Started(aPhysEp);
		else
			Dma.StartFailed(aPhysEp);						// accepted or not is unknown: only a core reset frees it
		}
	return r;
	}

// ask for a stop: KErrNone = nothing owned, KErrNotReady = stopping (its command-complete event frees it)
static TInt EndXfer(TInt aPhysEp)
	{
	const TUint32 bit = 1u << aPhysEp;
	if (Dma.iQuarantined & bit)
		return KErrGeneral;
	if (Dma.iPending & bit)
		return KErrNotReady;
	if (!(Dma.iOwned & bit))
		return KErrNone;
	Dma.StopRequested(aPhysEp);
	StopTick[aPhysEp] = NKern::TickCount();
	TInt r = DepCmd(aPhysEp, ECmdEndXfer | KCmdIoc | KForceRm | (RscIdx[aPhysEp] << 16));
	if (r != KErrNone)
		Dma.StopFailed(aPhysEp);
	return r == KErrNone ? KErrNotReady : r;
	}

static void StopFunction()
	{
	for (TInt ep = 2; ep < KPhysEps; ++ep)
		if (FnMask & (1u << ep))
			EndXfer(ep);
	}

static void StartSetup()
	{
	PendingConfig = DeferredStatusEp = -1;
	Ep0RearmNeeded = EFalse;
	Ep0 = ESetup;
	Start(EPhysEp0Out, ETrbSetup, EOffSetup, 8);
	}

// ---- the function's side
TUint8* A64Usb_Buf(TInt aPhysEp) { return (TUint8*)(Area + BufOff[aPhysEp]); }
TBool A64Usb_CanStart(TInt aPhysEp) { return Running && !Restart && Dma.CanStart(aPhysEp); }
TInt A64Usb_Start(TInt aPhysEp, TInt aLen)
	{ return Running && !Restart ? Start(aPhysEp, ETrbNormal, BufOff[aPhysEp], aLen) : KErrNotReady; }
void A64Usb_EndXfer(TInt aPhysEp) { if (Running && !Restart) EndXfer(aPhysEp); }
TBool A64Usb_Ep0Idle() { return Ep0 == ESetup; }
TBool A64Usb_Configured() { return Configured; }

void A64Usb_Stats(SA64UsbStats& aStats)
	{
	aStats.iEvents = Events;
	aStats.iResets = Resets;
	aStats.iSetups = Setups;
	aStats.iEp0 = Ep0;
	aStats.iRestarts = Restarts;
	aStats.iConfigured = Configured;
	aStats.iDsts = R(EDsts);
	memcpy(aStats.iSetupLog, SetupLog, sizeof(SetupLog));
	aStats.iDmaOwned = Dma.iOwned;
	aStats.iDmaPending = Dma.iPending;
	aStats.iDmaQuarantined = Dma.iQuarantined;
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

static TInt Configure()
	{
	TInt r = DepCmd(EPhysEp0Out, ECmdStartCfg | (2 << 16));	// transfer resources for the other endpoints
	for (TInt i = 0; r == KErrNone && i < Fn.iEndpointCount; ++i)
		{
		const SA64UsbEndpoint& e = Fn.iEndpoints[i];
		r = EpConfig(e.iPhysEp, e.iType, e.iMaxPacket, 0, e.iInterval);
		if (r == KErrNone)
			r = DepCmd(e.iPhysEp, ECmdSetXferRes, 1);
		if (r == KErrNone)
			R(EDalepena) |= 1u << e.iPhysEp;
		}
	if (r == KErrNone)
		r = Fn.iConfigured();
	Configured = r == KErrNone;
	return r;
	}

// a deferred SET_CONFIGURATION: once its endpoints' stops completed, configure and start its status stage
static void ProgressControl()
	{
	if (PendingConfig < 0 || Restart)
		return;
	TUsbStopProgress stop = UsbStopProgress(Dma.iPending, Dma.iQuarantined, FnMask);
	if (stop == EUsbStopRestart)							// a later SETUP can block ENDTRANSFER (Linux gadget.c)
		{
		Restart = ETrue;
		return;
		}
	if (stop == EUsbStopWait)
		return;
	TInt value = PendingConfig;
	PendingConfig = -1;
	R(EDalepena) = 3;
	TInt r = value == 1 ? Configure() : KErrNone;
	if (r != KErrNone)
		{
		DepCmd(EPhysEp0Out, ECmdSetStall);
		StartSetup();
		return;
		}
	Ep0 = EWaitStatus2;
	if (DeferredStatusEp >= 0)
		{
		Ep0 = EStatus;
		Start(DeferredStatusEp, ETrbStatus2, EOffEp0Buf, 0);
		DeferredStatusEp = -1;
		}
	}

// the host's session ended (or a new configuration begins): the function forgets it, its transfers stop
static void EndSession()
	{
	PendingConfig = DeferredStatusEp = -1;
	Configured = EFalse;
	Fn.iReset();
	StopFunction();
	}

// a SETUP packet arrived: answer it (data stage started here), or stall
static void HandleSetup()
	{
	PendingConfig = DeferredStatusEp = -1;					// a new SETUP cancels a deferred one (not its stops)
	SetupLog[Setups % 6] = (Setup[0] << 24) | (Setup[1] << 16) | (Setup[2] << 8) | Setup[4];	// type req value index
	++Setups;
	TUint8 type = Setup[0], req = Setup[1];
	TUint16 value = Setup[2] | (Setup[3] << 8), index = Setup[4] | (Setup[5] << 8), length = Setup[6] | (Setup[7] << 8);
	TUint8* buf = (TUint8*)(Area + EOffEp0Buf);
	TInt in = -1;											// bytes to send, -1 = none
	TBool out = EFalse, ok = ETrue;
	const TUint16 interfaces = Fn.iConfig[4];				// bNumInterfaces
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
		case 9:												// SET_CONFIGURATION: stop the old one first, then
			if (value > 1 || type != 0 || index != 0 || length != 0)	// configure (fresh data toggles) and only
				{											// then the status stage (ProgressControl)
				ok = EFalse;
				break;
				}
			EndSession();
			PendingConfig = value;
			Ep0 = EDeferred;
			ProgressControl();
			return;
		case 8: buf[0] = Configured ? 1 : 0; in = 1; break;	// GET_CONFIGURATION
		case 0: buf[0] = buf[1] = 0; in = 2; break;			// GET_STATUS
		case 1:												// CLEAR_FEATURE: an endpoint's halt (macOS on each open)
			if ((type & 0x1f) == 2 && value == 0)			// -- resets its data toggle: so must the controller,
				{											// or both sides drop each other's packets as repeats
				TInt ep = Setup[4] & 0x0f, phys = 2 * ep + ((Setup[4] & 0x80) ? 1 : 0);
				if (phys >= 2 && phys < KPhysEps && (FnMask & (1u << phys)))
					DepCmd(phys, ECmdClearStall);
				else if (phys >= 2)
					ok = EFalse;
				}
			break;
		case 10:											// GET_INTERFACE: alternate setting 0 only
			ok = Configured && type == 0x81 && value == 0 && length == 1 && index < interfaces;
			buf[0] = 0;
			in = 1;
			break;
		case 11:											// SET_INTERFACE
			ok = Configured && type == 0x01 && value == 0 && length == 0 && index < interfaces;
			break;
		case 3: break;										// SET_FEATURE: accept
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
		if (Ep0 == EDeferred)
			DeferredStatusEp = aPhysEp;						// ProgressControl starts it
		else if (Ep0 == EWaitStatus2 || Ep0 == EWaitStatus3In || Ep0 == EWaitStatus3Out)
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
	if (!Running || Restart)
		return;
	if (Dma.iQuarantined)
		{
		Restart = ETrue;
		return;
		}
	TUint32 count = R(EGevntCount) & 0xffff;
	__e32_memory_barrier();									// the count publishes the controller's ring writes
	for (TInt n = 0; count >= 4 && n < KMaxEvents && !Restart; ++n)
		{
		TUint32 e = *(volatile TUint32*)(Area + EOffEv + EvPos);
		__e32_memory_barrier();								// read the event before giving its slot back
		EvPos = (EvPos + 4) % KEvSize;
		R(EGevntCount) = 4;
		count -= 4;
		++Events;
		if (!(e & 1))										// an endpoint event
			{
			TInt ep = (e >> 1) & 0x1f, type = (e >> 6) & 0xf, status = (e >> 12) & 0xf;
			if (ep >= KPhysEps)
				continue;
			if (type == EEvCommandComplete)					// ENDTRANSFER's (command in event[27:24]) frees the DMA
				{
				if (ep >= 2 && ((e >> 24) & 0xf) == ECmdEndXfer && Dma.StopCompleted(ep, status == 0))
					{
					__e32_memory_barrier();
					Trb(ep)->iCtrl &= ~ETrbHwo;
					RscIdx[ep] = 0;
					}
				continue;
				}
			if (ep <= 1)
				Ep0Event(ep, type, status);
			else if (type == EEvXferComplete && Dma.Completed(ep))	// else: a stop is pending, or it's an old one
				Fn.iXferComplete(ep, Armed[ep] - (TInt)(Trb(ep)->iSize & 0xffffff));
			}
		else if (((e >> 1) & 0x7f) == 0)					// a device event
			{
			TInt type = (e >> 8) & 0xf;
			if (type == EDevReset)
				{
				++Resets;
				EndSession();
				R(EDcfg) &= ~(0x7f << 3);
				if (UsbResetNeedsRestart(Ep0, Dma.iQuarantined))
					Restart = ETrue;						// a control transfer may still be live: restart
				else if (UsbEp0Rearmable(Ep0) || Ep0RearmNeeded)
					StartSetup();
				}
			else if (type == EDevConnectDone)
				{
				EpConfig(EPhysEp0Out, 0, KEp0Max, 2u << 30);	// modify: EP0 max packet for this speed
				EpConfig(EPhysEp0In, 0, KEp0Max, 2u << 30);
				if (Ep0RearmNeeded)
					StartSetup();
				}
			else if (type == EDevDisconnect)
				{
				EndSession();
				if (UsbEp0Rearmable(Ep0))
					Ep0RearmNeeded = ETrue;				// re-armed at the next reset or connect
				if (UsbResetNeedsRestart(Ep0, Dma.iQuarantined))
					Restart = ETrue;
				}
			}
		}
	for (TInt ep = 2; ep < KPhysEps; ++ep)
		if ((Dma.iPending & ~Dma.iQuarantined & (1u << ep)) && NKern::TickCount() - StopTick[ep] >= KStopTimeout)
			{
			Dma.StopFailed(ep);
			Kern::Printf("[usb] ENDTRANSFER ep %d timed out; DMA quarantined until a controller restart", ep);
			}
	if (Dma.iQuarantined)
		Restart = ETrue;
	ProgressControl();
	}

static void Append(TDes8& aBuf, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aBuf, aFmt, list);
	VA_END(list);
	}

TInt A64UsbPowerOn(TUint32& aPll);						// qcom_usbpower.cpp

static TInt Halt()
	{
	R(EDctl) &= ~(1u << 31);								// RunStop off
	for (TInt i = 0; i < 1000000 && !(R(EDsts) & (1u << 22)); ++i)		// until halted
		{}
	return R(EDsts) & (1u << 22) ? KErrNone : KErrTimedOut;
	}

// halt and reset the device controller and start it as the function's gadget (the PHY setup stays). Only a proven
// halt + core reset clears the DMA area and quarantine; on failure the area stays untouched (the controller may own it).
static TInt Bringup()
	{
	Running = EFalse;
	Kern::Printf("[usb] bringup: general %08x gctl %08x usb2 %08x dctl %08x dsts %08x", Q(EGeneralCfg), R(EGctl),
		R(EGusb2PhyCfg), R(EDctl), R(EDsts));
	TInt r = Halt();
	if (r != KErrNone)
		return r;
	// bonito's DTB has qcom,select-utmi-as-pipe-clk (no SuperSpeed PHY gives a PIPE clock). A GCC reset loses the
	// wrapper's mux (0x0d -> 0x04 on a cold power-up), and the first endpoint command then times out: restore it while
	// halted, as Linux dwc3_qcom_select_utmi_clk() does (disable, select, enable).
	// shortcut: HS-only board policy fixed here; board data when a second SDM670 board wants USB3.
	const TUint32 clockMask = KUtmiClockSelect | KPipePhyStatus | KUtmiClockDisable;
	if ((Q(EGeneralCfg) & clockMask) != (KUtmiClockSelect | KPipePhyStatus))
		{
		Q(EGeneralCfg) |= KUtmiClockDisable;
		if (!(Q(EGeneralCfg) & KUtmiClockDisable))			// read back: the write has landed before the wait
			return KErrGeneral;
		Kern::NanoWait(100000);
		Q(EGeneralCfg) |= KUtmiClockSelect | KPipePhyStatus;
		if ((Q(EGeneralCfg) & clockMask) != clockMask)
			return KErrGeneral;
		Kern::NanoWait(100000);
		Q(EGeneralCfg) &= ~KUtmiClockDisable;
		__e32_memory_barrier();
		Kern::Printf("[usb] UTMI/PIPE clock restored: general %08x", Q(EGeneralCfg));
		if ((Q(EGeneralCfg) & clockMask) != (KUtmiClockSelect | KPipePhyStatus))
			return KErrGeneral;
		}
	TUint32 phy = R(EGusb2PhyCfg);
	R(EDctl) |= 1u << 30;									// CSFTRST
	for (TInt i = 0; i < 1000000 && (R(EDctl) & (1u << 30)); ++i)
		{}
	if (R(EDctl) & (1u << 30))
		return KErrTimedOut;
	__e32_memory_barrier();
	Dma.ControllerReset();
	memclr((TAny*)Area, KAreaSize);
	memclr(RscIdx, sizeof(RscIdx));
	Ep0RearmNeeded = EFalse;
	EndSession();											// nothing is owned now: it only resets the function
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
	r = DepCmd(EPhysEp0Out, ECmdStartCfg);
	if (r == KErrNone)
		r = EpConfig(EPhysEp0Out, 0, KEp0Max, 0);
	if (r == KErrNone)
		r = EpConfig(EPhysEp0In, 0, KEp0Max, 0);
	if (r == KErrNone)
		r = DepCmd(EPhysEp0Out, ECmdSetXferRes, 1);
	if (r == KErrNone)
		r = DepCmd(EPhysEp0In, ECmdSetXferRes, 1);
	if (r != KErrNone)
		return r;
	R(EDalepena) = 3;
	StartSetup();
	R(EDctl) = (R(EDctl) & ~(1u << 30)) | (1u << 31);		// RunStop: connect
	Running = ETrue;
	return KErrNone;
	}

// from the poll only: halt (the host sees a disconnect), then a full bring-up. On failure USB stays stopped and
// everything the controller might own stays quarantined.
static void RestartController()
	{
	Restart = EFalse;
	Running = EFalse;
	++Restarts;
	TInt r = Halt();
	if (r == KErrNone)
		{
		NKern::Sleep(NKern::TimerTicks(100));				// long enough for the host to see the device go
		r = Bringup();
		}
	if (r != KErrNone)
		Dma.iQuarantined |= Dma.iOwned | Dma.iPending;
	Kern::Printf("[usb] controller restart %d; DMA %s", r, r == KErrNone ? "reinitialised" : "kept quarantined");
	}

static void PollDfcFn(TAny*);
static TDfc PollDfc(PollDfcFn, NULL, 1);
static void PollTimerFn(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTimerFn, NULL);
static TDynamicDfcQue* UsbQ;

static void PollDfcFn(TAny*)
	{
	if (Restart)
		RestartController();
	if (Running)
		{
		A64Usb_ProcessEvents();
		if (Restart)
			RestartController();
		if (Running && !Restart)
			Fn.iPoll();
		if (Restart)										// the function's writer polled events, too
			RestartController();
		}
	PollTimer.OneShot(1);								// not Again(): it refuses (KErrArgument) once a slow poll put its next expiry in the past
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
	Running = EFalse;
	TInt r = Halt();
	if (r != KErrNone)
		{
		Dma.iQuarantined |= Dma.iOwned | Dma.iPending;
		Append(aOut, "usb cold: halt failed %d; DMA quarantined", r);
		return;
		}
	A64UsbPowerOff();
	NKern::Sleep(NKern::TimerTicks(200));
	TUint32 pll = 0;
	r = A64UsbPowerOn(pll);
	TBuf8<200> d;
	A64UsbPowerDiag(d);
	Kern::Printf("[usb] cold (votes %d): powered up -> %d, PLL %02x; %S", aMask, r, pll, &d);
	Append(aOut, "usb cold: powered up -> %d, PLL %02x", r, pll);
	if (r != KErrNone)
		return;
	r = Bringup();
	if (r != KErrNone)
		Kern::Printf("[usb] cold: bring-up failed %d; DMA kept", r);
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
		FnMask |= 1u << e.iPhysEp;
		}
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(KAreaSize, AreaPhys, 12);
	const TBool allocated = r == KErrNone;
	DPlatChunkHw* chunk = NULL;
	if (r == KErrNone)
		r = DPlatChunkHw::New(chunk, AreaPhys, KAreaSize, EMapAttrSupRw | EMapAttrBufferedNC);
	_LIT(KName, "UsbQ");
	if (r == KErrNone)
		r = Kern::DynamicDfcQCreate(UsbQ, 27, KName);
	if (r != KErrNone)
		{
		if (chunk)
			chunk->Close(NULL);
		if (allocated)
			Epoc::FreePhysicalRam(AreaPhys, KAreaSize);
		}
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		{
		Kern::Printf("[usb] no DMA area / queue: %d", r);
		return;
		}
	Area = chunk->LinearAddress();
	r = Bringup();
	if (r != KErrNone)
		{
		Kern::Printf("[usb] controller startup failed %d; DMA area kept", r);
		return;
		}
	Fn.iDump();

	PollDfc.SetDfcQ(UsbQ);
	PollTimer.OneShot(1);
	}
