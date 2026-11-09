// SPDX-License-Identifier: Apache-2.0
//
// qcom_glink.cpp -- GLINK over SMEM to the remote processors: the modem (DTB glink-edge "modem", remote pid 1,
//   doorbell bit 12) and the ADSP ("lpass", remote pid 2, doorbell bit 24); each edge has its own FIFOs and channels:
//   transport  SMEM item 478 = 4 words {our TX tail, our TX head, our RX tail, our RX head}; item 479 = our TX
//              FIFO (16 KB, ours to allocate), 480 = our RX FIFO (the remote's). Every command is 8-byte aligned.
//   protocol   commands {u16 cmd, u16 param1, u32 param2, data}: VERSION / VERSION_ACK; a channel is open when
//              each side sent OPEN (its id + name) and got OPEN_ACK; a sender needs one of the receiver's INTENTs
//              (an id + a buffer size) per message: TX_DATA {+ chunk size, bytes left}, continued by
//              TX_DATA_CONT; RX_DONE gives the intent back. RX_INTENT_REQ asks for a bigger buffer.
// Polled from its own thread every millisecond (docs/telephony.md C2: GIC lines over 255 come with C12).
// Written from the protocol as described by Linux drivers/rpmsg/qcom_glink_smem.c and qcom_glink_native.c.
// Telephony C3: the QRTR channel "IPCRTR" to the modem. C6: the ADSP's "IPCRTR" and "apr_audio_svc".
// A channel id is edge << 8 | lcid, so the modem's channels keep ids 1..8.
// shortcut: no intent reuse (features 0).
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_ipc.h"
#include "qcom_smem.h"
#include "qcom_glink.h"

enum { EVersion = 0, EVersionAck = 1, EOpen = 2, EClose = 3, EOpenAck = 4, EIntent = 5, ERxDone = 6, EIntentReq = 7,
	EIntentReqAck = 8, ETxData = 9, ECloseAck = 11, ETxDataCont = 12, EReadNotif = 13, ERxDoneReuse = 14,
	ESignals = 15 };
const TUint32 KFifoSize = 0x4000, KReserve = 16;			// FIFO_FULL_RESERVE + TX_BLOCKED_CMD_RESERVE
const TInt KChannels = 8, KIntents = 4, KRemoteIntents = 16, KIntentSize = 2048, KName = 32;

struct SMsg { TUint16 iCmd, iParam1; TUint32 iParam2; };

struct SIntent { TUint32 iId, iSize; TUint8* iBuf; TUint32 iFill; };
enum { EFree, EAvailable, EInUse };
struct SRemoteIntent { TUint32 iId, iSize, iState; };
struct SChannel
	{
	TUint8 iName[KName];			// not a TBuf8: NewChannel memclr()s the struct, which would zero its max length
	TInt iNameLen;
	TUint16 iLcid, iRcid;			// ours (1-based index + 1), the remote's (0: not opened by it yet)
	TBool iOpenSent, iOpenAcked;
	TGlinkRx iRx;
	TAny* iPtr;
	SIntent iIn[KIntents];			// our receive buffers, advertised to the remote
	SRemoteIntent iRemote[KRemoteIntents];	// the remote's receive buffers we may send into
	TInt iRxCount, iTxCount;
	};

struct SEdge
	{
	TInt iRemote, iIpcBit;
	volatile TUint32* iDesc;		// item 478
	TUint8* iTx;					// item 479
	TUint8* iRx;					// item 480
	TUint32 iRxLen;
	TBool iUp, iVersionAcked;
	TUint32 iNextIntent;
	TInt iBadCmds;
	SChannel iCh[KChannels];
	TInt iNCh;
	};
static SEdge Edges[] = { { KSmemHostModem, KIpcModemGlink }, { KSmemHostAdsp, KIpcAdspGlink } };
const TInt KEdges = sizeof(Edges) / sizeof(Edges[0]);
static SEdge* EdgeOf(TInt aRemote)
	{
	for (TInt i = 0; i < KEdges; ++i)
		if (Edges[i].iRemote == aRemote)
			return &Edges[i];
	return NULL;
	}
static TInt Id(const SEdge& aE, const SChannel& aC) { return (TInt)(&aE - Edges) << 8 | aC.iLcid; }

// ---- the FIFOs
static TUint32 RxAvail(SEdge& E)
	{
	if (!E.iRx)
		{
		TUint32 len = 0;
		E.iRx = (TUint8*)Smem_Get(E.iRemote, 480, &len);
		E.iRxLen = len;
		if (!E.iRx)
			return 0;
		}
	TUint32 head = E.iDesc[3], tail = E.iDesc[2];
	return head >= tail ? head - tail : E.iRxLen - tail + head;
	}

static void RxPeek(SEdge& E, TAny* aData, TUint32 aOffset, TUint32 aCount)
	{
	TUint32 tail = E.iDesc[2] + aOffset;
	if (tail >= E.iRxLen)
		tail -= E.iRxLen;
	TUint32 n = Min(aCount, E.iRxLen - tail);
	memcpy(aData, E.iRx + tail, n);
	if (n < aCount)
		memcpy((TUint8*)aData + n, E.iRx, aCount - n);
	}

static void RxAdvance(SEdge& E, TUint32 aCount)
	{
	TUint32 tail = E.iDesc[2] + ((aCount + 7) & ~7u);
	if (tail >= E.iRxLen)
		tail -= E.iRxLen;
	E.iDesc[2] = tail;
	}

static TUint32 TxAvail(SEdge& E)
	{
	TUint32 head = E.iDesc[1], tail = E.iDesc[0];
	TUint32 avail = tail <= head ? KFifoSize - head + tail : tail - head;
	return avail > KReserve ? avail - KReserve : 0;
	}

static TUint32 TxWriteOne(SEdge& E, TUint32 aHead, const TAny* aData, TUint32 aCount)
	{
	TUint32 n = Min(aCount, KFifoSize - aHead);
	memcpy(E.iTx + aHead, aData, n);
	if (n < aCount)
		memcpy(E.iTx, (const TUint8*)aData + n, aCount - n);
	aHead += aCount;
	return aHead >= KFifoSize ? aHead - KFifoSize : aHead;
	}

static NFastMutex TxLock;										// the GLINK thread and user threads (QrtrSend) both send

static TInt Tx(SEdge& E, const TAny* aHdr, TUint32 aHlen, const TAny* aData = NULL, TUint32 aDlen = 0)
	{
	TUint32 need = (aHlen + aDlen + 7) & ~7u;
	for (TInt i = 0; ; ++i)									// the modem drains it; wait up to ~100 ms
		{
		NKern::FMWait(&TxLock);
		if (TxAvail(E) >= need)
			break;											// keep the lock for the write
		NKern::FMSignal(&TxLock);
		if (i >= 100)
			return KErrOverflow;
		NKern::Sleep(NKern::TimerTicks(1));
		}
	TUint32 head = TxWriteOne(E, E.iDesc[1], aHdr, aHlen);
	if (aDlen)
		head = TxWriteOne(E, head, aData, aDlen);
	head = (head + 7) & ~7u;
	if (head >= KFifoSize)
		head -= KFifoSize;
	__e32_memory_barrier();									// the data before the head that publishes it
	E.iDesc[1] = head;
	NKern::FMSignal(&TxLock);
	Ipc_Kick(E.iIpcBit);
	return KErrNone;
	}

static void Cmd(SEdge& E, TUint16 aCmd, TUint16 aP1, TUint32 aP2)
	{
	SMsg m = { aCmd, aP1, aP2 };
	Tx(E, &m, sizeof(m));
	}

// ---- channels
static SChannel* ByName(SEdge& E, const TDesC8& aName)
	{
	for (TInt i = 0; i < E.iNCh; ++i)
		if (TPtrC8(E.iCh[i].iName, E.iCh[i].iNameLen) == aName)
			return &E.iCh[i];
	return NULL;
	}
static SChannel* ByLcid(SEdge& E, TUint aLcid) { return aLcid >= 1 && aLcid <= (TUint)E.iNCh ? &E.iCh[aLcid - 1] : NULL; }
static SChannel* ByRcid(SEdge& E, TUint aRcid)
	{
	for (TInt i = 0; i < E.iNCh; ++i)
		if (E.iCh[i].iRcid == aRcid)
			return &E.iCh[i];
	return NULL;
	}

static SChannel* NewChannel(SEdge& E, const TDesC8& aName)
	{
	if (E.iNCh >= KChannels)
		return NULL;
	SChannel* c = &E.iCh[E.iNCh];
	memclr(c, sizeof(*c));
	c->iNameLen = Min(aName.Length(), KName - 1);
	memcpy(c->iName, aName.Ptr(), c->iNameLen);
	c->iLcid = (TUint16)++E.iNCh;
	return c;
	}

static void SendOpen(SEdge& E, SChannel& aC)
	{
	struct { SMsg iM; TUint8 iName[KName]; } m;
	memclr(&m, sizeof(m));
	m.iM.iCmd = EOpen;
	m.iM.iParam1 = aC.iLcid;
	m.iM.iParam2 = aC.iNameLen + 1;
	memcpy(m.iName, aC.iName, aC.iNameLen);
	Tx(E, &m, (sizeof(SMsg) + aC.iNameLen + 1 + 7) & ~7u);
	aC.iOpenSent = ETrue;
	}

static void Advertise(SEdge& E, SChannel& aC, SIntent& aI)
	{
	aI.iId = ++E.iNextIntent;
	aI.iFill = 0;
	struct { SMsg iM; TUint32 iSize, iId; } m = { { EIntent, aC.iLcid, 1 }, aI.iSize, aI.iId };
	Tx(E, &m, sizeof(m));
	}

static void AdvertiseAll(SEdge& E, SChannel& aC)						// once both sides opened: our receive buffers
	{
	for (TInt i = 0; i < KIntents; ++i)
		{
		SIntent& in = aC.iIn[i];
		if (!in.iBuf)
			{
			NKern::ThreadEnterCS();
			in.iBuf = (TUint8*)Kern::Alloc(KIntentSize);
			NKern::ThreadLeaveCS();
			in.iSize = in.iBuf ? KIntentSize : 0;
			}
		if (in.iBuf)
			Advertise(E, aC, in);
		}
	}

// ---- receive
static TBool RxOne(SEdge& E)										// one command; EFalse if it is not all there yet
	{
	TUint32 avail = RxAvail(E);
	if (avail < sizeof(SMsg))
		return EFalse;
	SMsg m;
	RxPeek(E, &m, 0, sizeof(m));
	switch (m.iCmd)
		{
	case EVersion:
		RxAdvance(E, sizeof(m));
		Cmd(E, EVersionAck, 1, 0);								// version 1, no features
		return ETrue;
	case EVersionAck:
		RxAdvance(E, sizeof(m));
		E.iVersionAcked = ETrue;
		return ETrue;
	case EOpen:
		{
		TUint32 len = m.iParam2 & 0xffff;
		if (avail < sizeof(m) + len)
			return EFalse;
		TBuf8<KName> name;
		TUint8 buf[KName];
		RxPeek(E, buf, sizeof(m), Min(len, (TUint32)KName));
		for (TUint32 i = 0; i < len && i < (TUint32)KName && buf[i]; ++i)
			name.Append(buf[i]);
		RxAdvance(E, sizeof(m) + len);
		SChannel* c = ByName(E, name);
		if (!c)
			c = NewChannel(E, name);
		if (!c)
			return ETrue;
		c->iRcid = m.iParam1;
		Kern::Printf("[glink %d] remote opened \"%S\" (rcid %d, id %x)", E.iRemote, &name, c->iRcid, Id(E, *c));
		Cmd(E, EOpenAck, c->iRcid, 0);
		// our side opens only for a channel someone here uses (Glink_Open), as Linux (its rpmsg device opens when a
		// driver binds): a fully opened "apr_voice_svc" told the modem the AP takes its voice APR, which went
		// nowhere, and the ADSP's START_VOICE waited forever (C6, 2026-11-09)
		if (c->iRx && !c->iOpenSent)
			SendOpen(E, *c);
		if (c->iOpenAcked)
			AdvertiseAll(E, *c);
		return ETrue;
		}
	case EOpenAck:
		{
		RxAdvance(E, sizeof(m));
		SChannel* c = ByLcid(E, m.iParam1);
		if (c)
			{
			c->iOpenAcked = ETrue;
			if (c->iRcid)
				AdvertiseAll(E, *c);
			}
		return ETrue;
		}
	case EIntent:
		{
		TUint32 len = sizeof(m) + 8 * m.iParam2;
		if (avail < len)
			return EFalse;
		SChannel* c = ByRcid(E, m.iParam1);
		NKern::FMWait(&TxLock);								// the intent table is shared with Glink_Send
		for (TUint32 i = 0; c && i < m.iParam2; ++i)
			{
			TUint32 pair[2];								// {size, id}
			RxPeek(E, pair, sizeof(m) + 8 * i, 8);
			// a free slot; with none, the smallest idle one gives way to a bigger intent (the one an INTENT_REQ asked
			// for: a full table of small intents dropped it, and big sends never went: C6, 2026-11-09)
			TInt slot = -1;
			for (TInt k = 0; k < KRemoteIntents && slot < 0; ++k)
				if (c->iRemote[k].iState == EFree)
					slot = k;
			if (slot < 0)
				for (TInt k = 0; k < KRemoteIntents; ++k)
					if (c->iRemote[k].iState == EAvailable && c->iRemote[k].iSize < pair[0]
						&& (slot < 0 || c->iRemote[k].iSize < c->iRemote[slot].iSize))
						slot = k;
			if (slot >= 0)
				{
				c->iRemote[slot].iId = pair[1];
				c->iRemote[slot].iSize = pair[0];
				c->iRemote[slot].iState = EAvailable;
				}
			}
		NKern::FMSignal(&TxLock);
		RxAdvance(E, len);
		return ETrue;
		}
	case ETxData:
	case ETxDataCont:
		{
		struct { SMsg iM; TUint32 iChunk, iLeft; } h;
		if (avail < sizeof(h))
			return EFalse;
		RxPeek(E, &h, 0, sizeof(h));
		if (avail < sizeof(h) + h.iChunk)
			return EFalse;
		SChannel* c = ByRcid(E, h.iM.iParam1);
		SIntent* in = NULL;
		for (TInt i = 0; c && i < KIntents; ++i)
			if (c->iIn[i].iBuf && c->iIn[i].iId == h.iM.iParam2)
				in = &c->iIn[i];
		if (in && in->iFill + h.iChunk <= in->iSize)
			{
			RxPeek(E, in->iBuf + in->iFill, sizeof(h), h.iChunk);
			in->iFill += h.iChunk;
			}
		RxAdvance(E, sizeof(h) + h.iChunk);
		if (in && !h.iLeft)
			{
			++c->iRxCount;
			if (c->iRx)
				c->iRx(c->iPtr, in->iBuf, in->iFill);
			Cmd(E, ERxDone, c->iLcid, in->iId);			// the buffer is free again: give it back under a new id
			Advertise(E, *c, *in);
			}
		return ETrue;
		}
	case EIntentReq:										// the remote wants a buffer of param2 bytes
		{
		RxAdvance(E, sizeof(m));
		SChannel* c = ByRcid(E, m.iParam1);
		TBool granted = EFalse;
		for (TInt i = 0; c && i < KIntents && !granted; ++i)
			if (!c->iIn[i].iFill)							// an idle one: regrow it
				{
				SIntent& in = c->iIn[i];
				NKern::ThreadEnterCS();
				TUint8* b = (TUint8*)Kern::Alloc(m.iParam2);
				if (b)
					{
					Kern::Free(in.iBuf);
					in.iBuf = b;
					in.iSize = m.iParam2;
					}
				NKern::ThreadLeaveCS();
				if (b)
					{
					Advertise(E, *c, in);
					granted = ETrue;
					}
				}
		if (c)
			Cmd(E, EIntentReqAck, c->iLcid, granted);
		return ETrue;
		}
	case EClose:
		{
		RxAdvance(E, sizeof(m));
		SChannel* c = ByRcid(E, m.iParam1);
		if (c)
			{
			TPtrC8 n(c->iName, c->iNameLen);
			Kern::Printf("[glink %d] remote closed \"%S\"", E.iRemote, &n);
			c->iRcid = 0;
			Cmd(E, ECloseAck, m.iParam1, 0);
			}
		return ETrue;
		}
	case ERxDone:											// the remote consumed our message in intent param2:
	case ERxDoneReuse:										// with reuse it stays ours to send into, else it is gone
		{
		RxAdvance(E, sizeof(m));
		SChannel* c = ByRcid(E, m.iParam1);
		NKern::FMWait(&TxLock);
		for (TInt k = 0; c && k < KRemoteIntents; ++k)
			if (c->iRemote[k].iState == EInUse && c->iRemote[k].iId == m.iParam2)
				c->iRemote[k].iState = m.iCmd == ERxDoneReuse ? EAvailable : EFree;
		NKern::FMSignal(&TxLock);
		return ETrue;
		}
	case ECloseAck:
	case EReadNotif:
	case ESignals:
	case EIntentReqAck:
		RxAdvance(E, sizeof(m));
		return ETrue;
	default:
		++E.iBadCmds;
		Kern::Printf("[glink] unknown command %d: dropping the FIFO's contents", m.iCmd);
		RxAdvance(E, avail);
		return EFalse;
		}
	}

// ---- the polling thread
static TDynamicDfcQue* Queue;
static void PollDfcFn(TAny*);
static TDfc PollDfc(PollDfcFn, NULL, 1);
static void PollTimerFn(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTimerFn, NULL);
static void PollDfcFn(TAny*)
	{
	for (TInt e = 0; e < KEdges; ++e)
		for (TInt n = 0; Edges[e].iUp && n < 64 && RxOne(Edges[e]); ++n)
			;
	PollTimer.OneShot(1);
	}

TInt Glink_Start(TInt aRemote)
	{
	SEdge* e = EdgeOf(aRemote);
	if (!e)
		return KErrNotSupported;
	SEdge& E = *e;
	if (E.iUp)
		return KErrNone;
	TInt r = Smem_Alloc(aRemote, 478, 32);
	if (r != KErrNone && r != KErrAlreadyExists)
		return r;
	TUint32 len = 0;
	E.iDesc = (volatile TUint32*)Smem_Get(aRemote, 478, &len);
	if (!E.iDesc || len != 32)
		return KErrCorrupt;
	r = Smem_Alloc(aRemote, 479, KFifoSize);
	if (r != KErrNone && r != KErrAlreadyExists)
		return r;
	E.iTx = (TUint8*)Smem_Get(aRemote, 479, &len);
	if (!E.iTx || len < KFifoSize)
		return KErrCorrupt;
	E.iDesc[2] = 0;											// our RX tail and TX head (Linux qcom_glink_smem_register)
	E.iDesc[1] = 0;
	if (!Queue)
		{
		if ((r = Kern::DynamicDfcQCreate(Queue, 25, _L8("Glink"))) != KErrNone)
			return r;
		PollDfc.SetDfcQ(Queue);
		}
	TBool polling = EFalse;
	for (TInt i = 0; i < KEdges; ++i)
		polling |= Edges[i].iUp;
	E.iUp = ETrue;
	Cmd(E, EVersion, 1, 0);
	if (!polling)
		PollTimer.OneShot(1);
	return KErrNone;
	}

TInt Glink_Open(TInt aRemote, const char* aName, TGlinkRx aRx, TAny* aPtr)
	{
	SEdge* e = EdgeOf(aRemote);
	if (!e || !e->iUp)
		return KErrNotReady;
	SEdge& E = *e;
	TPtrC8 name((const TUint8*)aName);
	SChannel* c = ByName(E, name);
	if (!c && !(c = NewChannel(E, name)))
		return KErrOverflow;
	c->iRx = aRx;
	c->iPtr = aPtr;
	if (!c->iOpenSent)
		SendOpen(E, *c);
	return Id(E, *c);
	}

TInt Glink_Send(TInt aChannel, const TAny* aData, TInt aLen)
	{
	if ((aChannel >> 8) >= KEdges)
		return KErrNotReady;
	SEdge& E = Edges[aChannel >> 8];
	SChannel* c = ByLcid(E, aChannel & 0xff);
	if (!c || !c->iRcid || !c->iOpenAcked)
		return KErrNotReady;
	TUint32 id = 0;
	TBool found = EFalse;
	for (TInt tries = 0; !found && tries < 200; ++tries)		// none free: ask for one, the GLINK thread files it
		{
		NKern::FMWait(&TxLock);
		for (TInt k = 0; k < KRemoteIntents && !found; ++k)
			if (c->iRemote[k].iState == EAvailable && c->iRemote[k].iSize >= (TUint32)aLen)
				{
				id = c->iRemote[k].iId;
				c->iRemote[k].iState = EInUse;			// until RX_DONE gives it back (or not)
				found = ETrue;
				}
		NKern::FMSignal(&TxLock);
		if (found)
			break;
		if (tries % 50 == 0)
			Cmd(E, EIntentReq, c->iLcid, aLen);				// RX_INTENT_REQ: our id, the size we need
		NKern::Sleep(NKern::TimerTicks(1));
		}
	if (!found)
		return KErrNotReady;
	struct { SMsg iM; TUint32 iChunk, iLeft; } h = { { ETxData, c->iLcid, id }, (TUint32)aLen, 0 };
	++c->iTxCount;
	return Tx(E, &h, sizeof(h), aData, aLen);
	}

void Glink_Dump(void (*aSay)(const char*, ...))
	{
	static const char* const KNames[] = { "modem", "lpass" };
	for (TInt e = 0; e < KEdges; ++e)
		{
		SEdge& E = Edges[e];
		if (!E.iUp)
			{
			aSay("glink %s: not started", KNames[e]);
			continue;
			}
		aSay("glink %s: version acked %d; tx tail %x head %x, rx tail %x head %x (fifo %x); bad cmds %d", KNames[e],
			E.iVersionAcked, E.iDesc[0], E.iDesc[1], E.iDesc[2], E.iDesc[3], E.iRxLen, E.iBadCmds);
		for (TInt i = 0; i < E.iNCh; ++i)
			{
			const SChannel& c = E.iCh[i];
			TInt remote = 0;
			for (TInt k = 0; k < KRemoteIntents; ++k)
				remote += c.iRemote[k].iState == EAvailable;
			TPtrC8 n(c.iName, c.iNameLen);
			aSay("  \"%S\" id %x rcid %d open sent %d acked %d; rx %d tx %d; remote intents %d", &n, Id(E, c),
				c.iRcid, c.iOpenSent, c.iOpenAcked, c.iRxCount, c.iTxCount, remote);
			}
		}
	}
