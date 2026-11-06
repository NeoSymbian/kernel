// SPDX-License-Identifier: Apache-2.0
//
// a64_serialkbd.cpp -- QEMU virt: characters typed on the PL011 serial console become key presses for the window
// server (Kern::AddEvent EKeyDown/EKeyUp with EPOC standard scan codes, translated by EKDATA like a real keyboard's),
// so eshell can be driven over the serial line. The shape of Phorepoc's xk_keyboard.cpp: the ISR drains the FIFO
// into a ring, a DFC posts the events (Kern::AddEvent needs a thread).
// Ctrl-T (0x14) dumps every thread's state instead (bring-up aid for hangs); Ctrl-B (0x02) sends the rest of the
// line (up to Enter) to the debug monitor (a64_dbgmon.cpp), which answers on the serial port.
// shortcut: ASCII letters, digits, space, Enter, Backspace, Escape and the punctuation a shell needs; no cursor
// keys (VT100 escape sequences) and no EHalGroupKeyboard info -- add them when an app needs them.
#include <kernel/kern_priv.h>
#include <e32keys.h>
#include "board.h"
#include "a64_platform.h"
#include "a64_dbgmon.h"

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

static TBool MonMode;
static TBuf8<128> MonLine;
static void SerialWrite(const TUint8* aData, TInt aLen)
	{
	for (TInt i = 0; i < aLen; ++i)
		A64_DebugPutc((char)aData[i]);
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
			A64DumpThreads();
			continue;
			}
		if (ch == 0x02)
			{
			MonMode = ETrue;
			MonLine.Zero();
			continue;
			}
		if (MonMode)
			{
			if (ch == '\r' || ch == '\n')
				{
				MonMode = EFalse;
				A64Mon_Line(MonLine, SerialWrite);
				}
			else if (MonLine.Length() < MonLine.MaxLength())
				MonLine.Append(ch);
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
