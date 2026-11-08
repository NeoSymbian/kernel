// SPDX-License-Identifier: Apache-2.0
//
// qcom_qmp.cpp -- QMP, text messages to the SDM670's always-on processor (DTB power-management@c300000,
// "qcom,aoss-qmp", doorbell bit 0). A descriptor at the start of the message RAM holds both sides' link and channel
// states and the mailbox; a message is a length word + 64 bytes, and the AOP clears the length when it has taken
// it. The remote processors need one before they boot: "{class: image, res: load_state, name: modem, val: on}".
// The message RAM only does 32-bit accesses. Polled.
// Written from the protocol in Linux drivers/soc/qcom/qcom_aoss.c. Telephony stage C2.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_ipc.h"
#include "qcom_qmp.h"

const TUint32 KMsgRam = 0x0c300000, KMapped = 0x1000;
enum { EMagic = 0x0, EVersion = 0x4, EUcoreLink = 0xc, EUcoreLinkAck = 0x10, EUcoreCh = 0x14, EUcoreChAck = 0x18,
	EMcoreLink = 0x24, EMcoreLinkAck = 0x28, EMcoreCh = 0x2c, EMcoreChAck = 0x30, EMcoreSize = 0x34, EMcoreOffset = 0x38 };
const TUint32 KMagic = 0x4d41494c, KUp = 0xffff, KDown = 0xffff0000;
const TInt KMsgLen = 64;

static TUint32 MboxOff, MboxSize;
static TBool Open;

static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KMsgRam) + aOff); }

static TBool Wait(TUint32 aOff, TUint32 aValue)				// up to ~1 s, as Linux
	{
	for (TInt i = 0; i < 1000; ++i)
		{
		if (R(aOff) == aValue)
			return ETrue;
		NKern::Sleep(NKern::TimerTicks(1));
		}
	return EFalse;
	}

TInt Qmp_Open()
	{
	if (Open)
		return KErrNone;
	if (!A64_Io(KMsgRam))
		return KErrNotSupported;
	if (R(EMagic) != KMagic || R(EVersion) != 1)
		return KErrCorrupt;
	MboxOff = R(EMcoreOffset);
	MboxSize = R(EMcoreSize);
	if (!MboxSize || MboxOff + 4 + KMsgLen > KMapped)
		{
		Kern::Printf("[qmp] mailbox at +0x%x size 0x%x: outside the mapped page", MboxOff, MboxSize);
		return KErrNotSupported;
		}
	if (R(EMcoreLink) == KUp && R(EMcoreLinkAck) == KUp && R(EMcoreCh) == KUp && R(EMcoreChAck) == KUp)
		{
		Open = ETrue;										// still open from an earlier owner (ABL)
		return KErrNone;
		}
	R(EUcoreLinkAck) = R(EUcoreLink);						// ack the AOP's link state, then ours up
	R(EMcoreLink) = KUp;
	Ipc_Kick(KIpcAop);
	TInt r = KErrTimedOut;
	if (Wait(EMcoreLinkAck, KUp))
		{
		R(EMcoreCh) = KUp;
		Ipc_Kick(KIpcAop);
		if (Wait(EUcoreCh, KUp))
			{
			R(EUcoreChAck) = KUp;
			Ipc_Kick(KIpcAop);
			if (Wait(EMcoreChAck, KUp))
				r = KErrNone;
			}
		if (r != KErrNone)
			R(EMcoreCh) = KDown;
		}
	if (r != KErrNone)
		{
		R(EMcoreLink) = KDown;
		Ipc_Kick(KIpcAop);
		return r;
		}
	Open = ETrue;
	return KErrNone;
	}

TInt Qmp_Send(const char* aMsg)
	{
	TInt r = Qmp_Open();
	if (r != KErrNone)
		return r;
	TUint32 w[KMsgLen / 4];
	memclr(w, sizeof(w));
	TInt n = 0;
	while (aMsg[n])
		++n;
	if (n >= KMsgLen)
		return KErrTooBig;
	memcpy(w, aMsg, n);
	for (TInt i = 0; i < KMsgLen / 4; ++i)
		R(MboxOff + 4 + 4 * i) = w[i];
	R(MboxOff) = KMsgLen;
	(void)R(MboxOff);										// read back: the data is in the RAM before the kick
	Ipc_Kick(KIpcAop);
	if (Wait(MboxOff, 0))
		return KErrNone;
	R(MboxOff) = 0;											// the AOP never took it
	return KErrTimedOut;
	}
