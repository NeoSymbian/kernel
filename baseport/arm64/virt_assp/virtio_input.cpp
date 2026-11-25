// SPDX-License-Identifier: Apache-2.0
//
// virtio_input.cpp -- QEMU virt: touch and keys from virtio-input devices (run-qemu.sh -device virtio-tablet-device
// -device virtio-keyboard-device), so the UI can be driven over VNC and by QMP input-send-event (tools/ui-qemu.py).
// The legacy virtio-mmio v1 transport as virtio_blk.cpp; each device's event queue holds 8-byte Linux input events
// (type, code, value) the device writes into buffers posted here. The transport's interrupt (SPI 16 + slot) queues a
// DFC that turns them into what the Pixel's touch and key drivers post (Kern::AddEvent):
// - a tablet's absolute X/Y (0..0x7fff, Syborg's pointer range) scaled to screen pixels as v * size / 0x8000, its
//   left button or BTN_TOUCH as EButton1Down/EPointerMove/EButton1Up, one event per SYN_REPORT;
// - a keyboard's Linux key codes as EKeyDown/EKeyUp with EPOC standard scan codes (EKDATA translates them, as
//   a64_serialkbd.cpp's). Auto-repeat (value 2) is left to the window server.
// shortcut: one pointer (no multi-touch slots) and a US keyboard map; add slots when a test needs two fingers.
#include <kernel/kern_priv.h>
#include <e32keys.h>
#include "board.h"
#include "a64_platform.h"
#include "a64_device.h"

const TUint32 KMmioBase = 0x0a000000, KMmioStride = 0x200, KMmioSlots = 32;
const TUint32 KMmioIntId0 = 48;							// hw/arm/virt.c: transport i raises SPI 16 + i
enum
	{
	EMagic = 0x000, EVersion = 0x004, EDeviceId = 0x008, EGuestFeatures = 0x020, EGuestPageSize = 0x028,
	EQueueSel = 0x030, EQueueNumMax = 0x034, EQueueNum = 0x038, EQueueAlign = 0x03c, EQueuePfn = 0x040,
	EQueueNotify = 0x050, EIntStatus = 0x060, EIntAck = 0x064, EStatus = 0x070,
	};
enum { EStAck = 1, EStDriver = 2, EStDriverOk = 4, EStFailed = 128 };
const TUint32 KVirtioInput = 18;
const TUint16 KDescWrite = 2;
const TInt KQ = 64;										// QEMU's virtio-input queue size

enum { EEvSyn = 0, EEvKey = 1, EEvAbs = 3 };
enum { EAbsX = 0, EAbsY = 1, EBtnLeft = 0x110, EBtnTouch = 0x14a };
const TInt KAbsRange = 0x8000;							// QEMU's INPUT_EVENT_ABS_MAX + 1

struct SDesc { TUint64 iAddr; TUint32 iLen; TUint16 iFlags, iNext; };
struct SAvail { TUint16 iFlags, iIdx, iRing[KQ], iUsedEvent; };
struct SUsedElem { TUint32 iId, iLen; };
struct SUsed { TUint16 iFlags, iIdx; SUsedElem iRing[KQ]; TUint16 iAvailEvent; };
struct SEvent { TUint16 iType, iCode; TUint32 iValue; };

// one device: page 0 = descriptors + avail ring, page 1 = used ring (legacy layout), page 2 = the event buffers
struct SInput
	{
	volatile TUint32* iDev;
	TUint32 iIntId;
	TLinAddr iArea;
	TUint16 iLastUsed;
	TInt iX, iY, iDown, iWasDown, iLastX, iLastY;
	};
const TInt KMaxDevices = 4;
static SInput Devices[KMaxDevices];
static TInt DeviceCount;
static TInt StartResult = KErrNotReady;

static volatile TUint32& R(SInput& aIn, TUint32 aOff) { return aIn.iDev[aOff / 4]; }
static SDesc* Descs(SInput& aIn) { return (SDesc*)aIn.iArea; }
static volatile SAvail* Avail(SInput& aIn) { return (volatile SAvail*)(aIn.iArea + KQ * sizeof(SDesc)); }
static volatile SUsed* Used(SInput& aIn) { return (volatile SUsed*)(aIn.iArea + 4096); }
static volatile SEvent* Events(SInput& aIn) { return (volatile SEvent*)(aIn.iArea + 2 * 4096); }

// Linux key code (input-event-codes.h) -> EPOC standard scan code; 0 = ignored
static TInt StdKey(TUint aCode)
	{
	static const char KRow1[] = "1234567890", KRowQ[] = "QWERTYUIOP", KRowA[] = "ASDFGHJKL", KRowZ[] = "ZXCVBNM";
	if (aCode >= 2 && aCode <= 11) return KRow1[aCode - 2];
	if (aCode >= 16 && aCode <= 25) return KRowQ[aCode - 16];
	if (aCode >= 30 && aCode <= 38) return KRowA[aCode - 30];
	if (aCode >= 44 && aCode <= 50) return KRowZ[aCode - 44];
	if (aCode >= 59 && aCode <= 68) return EStdKeyF1 + (TInt)(aCode - 59);
	switch (aCode)
		{
	case 1:		return EStdKeyEscape;
	case 12:	return EStdKeyMinus;
	case 13:	return EStdKeyEquals;
	case 14:	return EStdKeyBackspace;
	case 15:	return EStdKeyTab;
	case 26:	return EStdKeySquareBracketLeft;
	case 27:	return EStdKeySquareBracketRight;
	case 28:	return EStdKeyEnter;
	case 29:	return EStdKeyLeftCtrl;
	case 39:	return EStdKeySemiColon;
	case 40:	return EStdKeySingleQuote;
	case 41:	return EStdKeyXXX;						// grave, left of 1
	case 42:	return EStdKeyLeftShift;
	case 43:	return EStdKeyBackSlash;
	case 51:	return EStdKeyComma;
	case 52:	return EStdKeyFullStop;
	case 53:	return EStdKeyForwardSlash;
	case 54:	return EStdKeyRightShift;
	case 55:	return EStdKeyNkpAsterisk;
	case 56:	return EStdKeyLeftAlt;
	case 57:	return EStdKeySpace;
	case 58:	return EStdKeyCapsLock;
	case 87:	return EStdKeyF11;
	case 88:	return EStdKeyF12;
	case 96:	return EStdKeyNkpEnter;
	case 97:	return EStdKeyRightCtrl;
	case 100:	return EStdKeyRightAlt;
	case 102:	return EStdKeyHome;
	case 103:	return EStdKeyUpArrow;
	case 104:	return EStdKeyPageUp;
	case 105:	return EStdKeyLeftArrow;
	case 106:	return EStdKeyRightArrow;
	case 107:	return EStdKeyEnd;
	case 108:	return EStdKeyDownArrow;
	case 109:	return EStdKeyPageDown;
	case 110:	return EStdKeyInsert;
	case 111:	return EStdKeyDelete;
	default:	return 0;
		}
	}

// as the monitor's PostKey: never drop one (a lost button-up would leave a finger down); the window server drains the
// queue
static void Post(const TRawEvent& aEvent)
	{
	while (Kern::AddEvent(aEvent) == KErrOverflow)
		NKern::Sleep(1);
	}

static void PostPointer(TRawEvent::TType aType, TInt aX, TInt aY)
	{
	TRawEvent e;
	e.Set(aType, aX, aY);
	Post(e);
	}

static void Handle(SInput& aIn, const SEvent& aEv)
	{
	switch (aEv.iType)
		{
	case EEvAbs:
		if (aEv.iCode == EAbsX)
			aIn.iX = (TInt)((TUint64)aEv.iValue * A64_FbWidth / KAbsRange);
		else if (aEv.iCode == EAbsY)
			aIn.iY = (TInt)((TUint64)aEv.iValue * A64_FbHeight / KAbsRange);
		break;
	case EEvKey:
		if (aEv.iCode == EBtnLeft || aEv.iCode == EBtnTouch)
			aIn.iDown = aEv.iValue != 0;
		else if (aEv.iValue <= 1)
			{
			TInt code = StdKey(aEv.iCode);
			if (code)
				{
				TRawEvent e;
				e.Set(aEv.iValue ? TRawEvent::EKeyDown : TRawEvent::EKeyUp, code);
				Post(e);
				}
			}
		break;
	case EEvSyn:
		if (aIn.iDown != aIn.iWasDown)
			PostPointer(aIn.iDown ? TRawEvent::EButton1Down : TRawEvent::EButton1Up, aIn.iX, aIn.iY);
		else if (aIn.iDown && (aIn.iX != aIn.iLastX || aIn.iY != aIn.iLastY))
			PostPointer(TRawEvent::EPointerMove, aIn.iX, aIn.iY);
		aIn.iWasDown = aIn.iDown;
		aIn.iLastX = aIn.iX;
		aIn.iLastY = aIn.iY;
		break;
		}
	}

static void InputDfcFn(TAny*);
static TDfc InputDfc(InputDfcFn, NULL, 1);

static void InputIsr(TAny* aPtr)
	{
	SInput& in = *(SInput*)aPtr;
	R(in, EIntAck) = R(in, EIntStatus);					// the line is level: quiet until the next event
	InputDfc.Add();
	}

// every used buffer: handle its event, then give it back to the device
static void InputDfcFn(TAny*)
	{
	NKern::ThreadEnterCS();
	for (TInt d = 0; d < DeviceCount; ++d)
		{
		SInput& in = Devices[d];
		TBool posted = EFalse;
		while (Used(in)->iIdx != in.iLastUsed)
			{
			__e32_memory_barrier();
			TUint32 id = Used(in)->iRing[in.iLastUsed % KQ].iId % KQ;
			SEvent ev;
			ev.iType = Events(in)[id].iType;
			ev.iCode = Events(in)[id].iCode;
			ev.iValue = Events(in)[id].iValue;
			++in.iLastUsed;
			Handle(in, ev);
			volatile SAvail* a = Avail(in);
			a->iRing[a->iIdx % KQ] = (TUint16)id;
			__e32_memory_barrier();
			a->iIdx = a->iIdx + 1;
			posted = ETrue;
			}
		if (posted)
			{
			__e32_memory_barrier();
			R(in, EQueueNotify) = 0;
			}
		}
	NKern::ThreadLeaveCS();
	}

static TInt Setup(SInput& aIn)
	{
	TPhysAddr phys;
	DPlatChunkHw* chunk = NULL;
	TInt r = Epoc::AllocPhysicalRam(3 * 4096, phys, 12);
	if (r != KErrNone)
		return r;
	r = DPlatChunkHw::New(chunk, phys, 3 * 4096, EMapAttrSupRw | EMapAttrBufferedNC);
	if (r != KErrNone)
		{
		Epoc::FreePhysicalRam(phys, 3 * 4096);
		return r;
		}
	aIn.iArea = chunk->LinearAddress();
	memclr((TAny*)aIn.iArea, 3 * 4096);
	R(aIn, EStatus) = 0;								// reset
	R(aIn, EStatus) = EStAck;
	R(aIn, EStatus) = EStAck | EStDriver;
	R(aIn, EGuestFeatures) = 0;
	R(aIn, EGuestPageSize) = 4096;
	R(aIn, EQueueSel) = 0;								// the event queue (1, the status queue for LEDs, stays off)
	if (R(aIn, EQueueNumMax) < (TUint32)KQ)
		{
		R(aIn, EStatus) = EStFailed;
		return KErrNotSupported;
		}
	R(aIn, EQueueNum) = KQ;
	R(aIn, EQueueAlign) = 4096;
	R(aIn, EQueuePfn) = phys >> 12;
	SDesc* desc = Descs(aIn);
	volatile SAvail* a = Avail(aIn);
	for (TInt i = 0; i < KQ; ++i)						// every buffer posted, one event each
		{
		desc[i].iAddr = phys + 2 * 4096 + i * sizeof(SEvent);
		desc[i].iLen = sizeof(SEvent);
		desc[i].iFlags = KDescWrite;
		desc[i].iNext = 0;
		a->iRing[i] = (TUint16)i;
		}
	__e32_memory_barrier();
	a->iIdx = KQ;
	__e32_memory_barrier();								// the ring is complete before the device starts
	R(aIn, EStatus) = EStAck | EStDriver | EStDriverOk;
	return KErrNone;
	}

void A64StartTouch()
	{
	InputDfc.SetDfcQ(Kern::DfcQue0());					// a static TDfc was constructed before the queue existed
	NKern::ThreadEnterCS();
	StartResult = KErrNone;
	for (TUint32 i = 0; i < KMmioSlots && DeviceCount < KMaxDevices; ++i)
		{
		volatile TUint32* d = (volatile TUint32*)A64_Io(KMmioBase + i * KMmioStride);
		if (d[EMagic / 4] != 0x74726976 || d[EVersion / 4] != 1 || d[EDeviceId / 4] != KVirtioInput)
			continue;
		SInput& in = Devices[DeviceCount];
		in.iDev = d;
		in.iIntId = KMmioIntId0 + i;
		TInt r = Setup(in);
		if (r == KErrNone)
			{
			++DeviceCount;								// counted before its interrupt can run the DFC
			r = A64_BindSpi(in.iIntId, InputIsr, &in);
			R(in, EQueueNotify) = 0;					// events the device held while the queue was empty
			}
		Kern::Printf("[input] virtio-input in slot %d (INTID %d) -> %d", i, in.iIntId, r);
		if (r != KErrNone)
			StartResult = r;
		}
	NKern::ThreadLeaveCS();
	}

static TA64DevState InputStatus(TDes8& aDetail)
	{
	if (StartResult == KErrNotReady)
		return EA64DevStarting;
	if (!DeviceCount)
		return StartResult < 0 ? EA64DevError : EA64DevAbsent;
	A64Device_Format(aDetail, "%d virtio-input device(s): touch %dx%d, keys", DeviceCount, A64_FbWidth, A64_FbHeight);
	return EA64DevOk;
	}
A64_DEVICE("input", "Touch and keys (virtio-input)", "virtio_input", InputStatus);
