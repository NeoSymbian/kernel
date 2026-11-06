// SPDX-License-Identifier: Apache-2.0
//
// a64_serialkbd.cpp -- QEMU virt: characters typed on the PL011 serial console become key presses for the window
// server (Kern::AddEvent EKeyDown/EKeyUp with EPOC standard scan codes, translated by EKDATA like a real keyboard's),
// so eshell can be driven over the serial line. The shape of Phorepoc's xk_keyboard.cpp: the ISR drains the FIFO
// into a ring, a DFC posts the events (Kern::AddEvent needs a thread).
// Ctrl-T (0x14) dumps every thread's state instead (bring-up aid for hangs).
// shortcut: ASCII letters, digits, space, Enter, Backspace, Escape and the punctuation a shell needs; no cursor
// keys (VT100 escape sequences) and no EHalGroupKeyboard info -- add them when an app needs them.
#include <kernel/kern_priv.h>
#include <e32keys.h>
#include "board.h"
#include "../platform/a64_platform.h"

const TUint32 KUartPhys = 0x09000000;					// PL011 (qemu_virt.cpp's console)
const TUint32 KUartIntId = 33;							// SPI 1
enum { EUartDR = 0x00, EUartFR = 0x18, EUartIMSC = 0x38, EUartICR = 0x44 };
const TUint32 KFrRxEmpty = 1u << 4, KIntRx = 1u << 4, KIntRxTimeout = 1u << 6;

static volatile TUint32* Uart(TUint32 aReg) { return (volatile TUint32*)(A64_Io(KUartPhys) + aReg); }

static TUint8 Ring[256];
static volatile TUint32 RingHead, RingTail;				// ISR writes head, the DFC reads tail
static void RxDfcFn(TAny*);
static TDfc RxDfc(RxDfcFn, NULL, 1);

static void UartIsr(TAny*)
	{
	*Uart(EUartICR) = KIntRx | KIntRxTimeout;			// clear first: a byte arriving after the drain interrupts again
	while (!(*Uart(EUartFR) & KFrRxEmpty))
		{
		TUint8 c = (TUint8)*Uart(EUartDR);
		if (RingHead - RingTail < sizeof(Ring))
			Ring[RingHead++ % sizeof(Ring)] = c;
		}
	RxDfc.Add();
	}

// ASCII -> (standard scan code, shift); 0 = no key
static TInt ScanCode(TUint8 aChar, TBool& aShift)
	{
	aShift = EFalse;
	if (aChar >= 'a' && aChar <= 'z')
		return aChar - 'a' + 'A';
	if (aChar >= 'A' && aChar <= 'Z')
		{
		aShift = ETrue;
		return aChar;
		}
	if (aChar >= '0' && aChar <= '9')
		return aChar;
	switch (aChar)
		{
	case '\r': case '\n':	return EStdKeyEnter;
	case 0x08: case 0x7f:	return EStdKeyBackspace;
	case 0x1b:				return EStdKeyEscape;
	case ' ':				return EStdKeySpace;
	case '.':				return EStdKeyFullStop;
	case ',':				return EStdKeyComma;
	case '/':				return EStdKeyForwardSlash;
	case '\\':				return EStdKeyBackSlash;
	case ';':				return EStdKeySemiColon;
	case '-':				return EStdKeyMinus;
	case '=':				return EStdKeyEquals;
	case ':':	aShift = ETrue;	return EStdKeySemiColon;
	case '_':	aShift = ETrue;	return EStdKeyMinus;
	case '?':	aShift = ETrue;	return EStdKeyForwardSlash;
	case '*':	aShift = ETrue;	return '8';
	default:				return 0;
		}
	}

static void Post(TRawEvent::TType aType, TInt aScanCode)
	{
	TRawEvent e;
	e.Set(aType, aScanCode);
	while (Kern::AddEvent(e) == KErrOverflow)			// the window server will drain the queue
		NKern::Sleep(1);
	}

// Ctrl-T: every thread's state (after Phorepoc's xk_threaddump.cpp) -- for a hang, who waits on what; a user thread's
// AArch32 pc and lr from its exception frame at the top of its supervisor stack
static void DumpThreads()
	{
	DObjectCon& c = *K::Containers[EThread];
	c.Wait();
	Kern::Printf("[dump] ===== %d threads at tick %d =====", c.Count(), NKern::TickCount());
	Kern::Printf("[dump] event queue: %d queued, request %08x status %08x ready %d, event thread %O",
		(TInt)((K::EventHeadPtr - K::EventTailPtr + (K::EventBufferEnd - K::EventBufferStart)) % (K::EventBufferEnd - K::EventBufferStart)),
		(TUint32)(TLinAddr)K::EventRequest, (TUint32)K::EventRequest->iStatus, K::EventRequest->IsReady(), K::EventThread);
	for (TInt i = 0; i < c.Count(); ++i)
		{
		DThread* t = (DThread*)c[i];
		NThreadBase& n = t->iNThread;
		const NThreadWaitState::S& w = n.iWaitState.iWtC;
		Kern::Printf("[dump] %O wait %02x type %d obj %08x waitobj %08x pri %d cs %d umcb %08x", t, w.iWtStFlags,
			w.iWtObjType, (TUint32)(TLinAddr)w.iWtObj, (TUint32)(TLinAddr)t->iWaitObj, n.iPriority, n.iCsCount,
			(TUint32)(TLinAddr)n.iUserModeCallbacks);
		if (t->iThreadType == EThreadUser && n.iStackBase)
			{
			const SThreadExcStack* f = (const SThreadExcStack*)((TLinAddr)n.iStackBase + n.iStackSize) - 1;
			if ((f->iSpsr & 0x1F) == 0x10)
				{
				TInt word = 0x0BADBAD0;					// the word r0 points at (a TRequestStatus, for WaitForRequest)
				Kern::ThreadRawRead(t, (const TAny*)(TLinAddr)(TUint32)f->iX[0], &word, sizeof(word));
				Kern::Printf("[dump]   user pc %08x lr %08x sp %08x r0 %08x [r0] %08x reqsem %d", (TUint32)f->iElr,
					(TUint32)f->iX[14], (TUint32)f->iX[13], (TUint32)f->iX[0], word, n.iRequestSemaphore.iCount);
				TUint32 st[96], hit[16];
				TInt found = 0;
				if (Kern::ThreadRawRead(t, (const TAny*)(TLinAddr)((TUint32)f->iX[13] & ~3u), st, sizeof(st)) == KErrNone)
					for (TInt k = 0; k < 96 && found < 16; ++k)
						if ((st[k] & ~1u) >= 0x80200000u && st[k] < 0x80800000u)	// return addresses into the ROM section
							hit[found++] = st[k];
				for (TInt k = 0; k < found; k += 8)
					Kern::Printf("[dump]   stack code: %08x %08x %08x %08x %08x %08x %08x %08x", hit[k], hit[k+1],
						hit[k+2], hit[k+3], hit[k+4], hit[k+5], hit[k+6], hit[k+7]);
				}
			}
		}
	c.Signal();
	Kern::Printf("[dump] ===== end =====");
	}

static void RxDfcFn(TAny*)
	{
	NKern::ThreadEnterCS();
	while (RingTail != RingHead)
		{
		TBool shift;
		TUint8 ch = Ring[RingTail++ % sizeof(Ring)];
		if (ch == 0x14)
			{
			DumpThreads();
			continue;
			}
		TInt code = ScanCode(ch, shift);
		if (!code)
			continue;
		if (shift)
			Post(TRawEvent::EKeyDown, EStdKeyLeftShift);
		Post(TRawEvent::EKeyDown, code);
		NKern::Sleep(10);								// a key is held a little, and keys come apart, as typed:
		Post(TRawEvent::EKeyUp, code);					// a burst of a whole pasted line overruns the window
		if (shift)										// server's queue for the console (eshell then stalls)
			Post(TRawEvent::EKeyUp, EStdKeyLeftShift);
		NKern::Sleep(20);
		}
	NKern::ThreadLeaveCS();
	}

void A64StartSerialKeyboard()
	{
	RxDfc.SetDfcQ(Kern::DfcQue0());						// a static TDfc was constructed before the queue existed
	*Uart(EUartIMSC) |= KIntRx | KIntRxTimeout;
	TInt r = A64_BindSpi(KUartIntId, UartIsr, NULL);
	Kern::Printf("[kbd] serial keyboard on the PL011 (INTID %d) -> %d", KUartIntId, r);
	}
