// SPDX-License-Identifier: Apache-2.0
//
// qcom_smp2p.cpp -- SMP2P: each side owns one SMEM item holding up to 16 named 32-bit entries, and kicks the other
// through the APCS doorbell after a change. The modem pair (DTB smp2p-mpss): inbound item 435, outbound 428,
// doorbell bit 14; out entries "master-kernel" (bit 0 = stop) and "ipa"; in "slave-kernel" (fatal, ready, handover,
// stop-ack, shutdown-ack) and "ipa". The ADSP pair: in 443, out 429, bit 26.
// Polled: callers read the inbound words (the GIC lines come later, docs/telephony.md C12).
// Written from the layout and handshake in Linux drivers/soc/qcom/smp2p.c. Telephony stage C2.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_ipc.h"
#include "qcom_smem.h"
#include "qcom_smp2p.h"

const TUint32 KMagic = 0x504d5324;							// "$SMP"
const TInt KEntries = 16, KName = 16;
const TUint32 KFeatureSsrAck = 1, KFlagRestartDone = 1, KFlagRestartAck = 2;

struct SItem
	{
	TUint32 iMagic;
	TUint32 iVersionFeatures;								// version in bits 0..7, features 8..31
	TUint16 iLocalPid, iRemotePid, iTotal, iValid;
	TUint32 iFlags;
	struct { TUint8 iName[KName]; TUint32 iValue; } iEntry[KEntries];
	};

struct SEdge
	{
	TInt iRemote, iInItem, iOutItem, iBit;
	const char* iOutNames[2];
	SItem* iOut;
	TBool iNegotiated, iSsrAck, iAckState;
	TUint32 iGeneration;
	};
static NFastMutex StateLock; // outbound publication, generation, negotiation and reset (no sleeping calls)
static SEdge Edges[] = {
	{ KSmemHostModem, 435, 428, KIpcModemSmp2p, { "master-kernel", "ipa" }, NULL, EFalse, EFalse, EFalse },
	{ KSmemHostAdsp, 443, 429, KIpcAdspSmp2p, { "master-kernel", "sleepstate" }, NULL, EFalse, EFalse, EFalse },
	};

static SEdge* Edge(TInt aRemote)
	{
	for (TUint i = 0; i < sizeof(Edges) / sizeof(Edges[0]); ++i)
		if (Edges[i].iRemote == aRemote)
			return &Edges[i];
	return NULL;
	}

static SItem* In(const SEdge& aE)
	{
	SItem* in = (SItem*)Smem_Get(aE.iRemote, aE.iInItem);
	return in && in->iMagic == KMagic && (in->iVersionFeatures & 0xff) ? in : NULL;
	}

static void SetName(TUint8* aDst, const char* aSrc)
	{
	TInt i = 0;
	for (; i < KName && aSrc[i]; ++i)
		aDst[i] = (TUint8)aSrc[i];
	for (; i < KName; ++i)
		aDst[i] = 0;
	}

static TBool NameIs(const TUint8* aName, const char* aS)
	{
	for (TInt i = 0; i < KName; ++i)
		{
		if (aName[i] != (TUint8)aS[i])
			return EFalse;
		if (!aS[i])
			return ETrue;
		}
	return ETrue;
	}

TInt Smp2p_Start(TInt aRemote)
	{
	SEdge* e = Edge(aRemote);
	if (!e)
		return KErrArgument;
	NKern::FMWait(&StateLock);
	TBool up = e->iOut != NULL;
	TUint32 generation = e->iGeneration;
	NKern::FMSignal(&StateLock);
	if (up)
		return KErrNone;
	TInt r = Smem_Alloc(aRemote, e->iOutItem, sizeof(SItem));
	if (r != KErrNone && r != KErrAlreadyExists)
		return r;
	SItem* out = (SItem*)Smem_Get(aRemote, e->iOutItem);
	if (!out)
		return KErrNotFound;
	SItem* in = In(*e);
	TUint32 inVersion = in ? in->iVersionFeatures & 0xff : 0;
	NKern::FMWait(&StateLock);
	if (generation != e->iGeneration)
		{
		NKern::FMSignal(&StateLock);
		return KErrDied; // reset won while SMEM was being allocated
		}
	if (e->iOut) // a concurrent initializer may have published while SMEM was allocated
		{
		NKern::FMSignal(&StateLock);
		return KErrNone;
		}
	memclr(out, sizeof(SItem));
	out->iMagic = KMagic;
	out->iLocalPid = 0;
	out->iRemotePid = (TUint16)aRemote;
	out->iTotal = KEntries;
	for (TInt i = 0; i < 2; ++i)							// the entries the DTB names, all bits 0
		SetName(out->iEntry[out->iValid++].iName, e->iOutNames[i]);
	__e32_memory_barrier();									// the header before the version that validates it
	out->iVersionFeatures = (KFeatureSsrAck << 8) | (inVersion && inVersion <= 2 ? inVersion : 2);
	e->iOut = out;
	Ipc_Kick(e->iBit);
	NKern::FMSignal(&StateLock);
	return KErrNone;
	}

TInt Smp2p_In(TInt aRemote, const char* aEntry, TUint32& aValue)
	{
	SEdge* e = Edge(aRemote);
	SItem* in = e ? In(*e) : NULL;
	if (!in)
		return KErrNotReady;
	for (TInt i = 0; i < in->iValid && i < KEntries; ++i)
		if (NameIs(in->iEntry[i].iName, aEntry))
			{
			aValue = in->iEntry[i].iValue;
			return KErrNone;
			}
	return KErrNotReady;
	}

TUint32 Smp2p_Generation(TInt aRemote)
	{
	SEdge* e = Edge(aRemote);
	NKern::FMWait(&StateLock);
	TUint32 generation = e ? e->iGeneration : ~0u;
	NKern::FMSignal(&StateLock);
	return generation;
	}

TInt Smp2p_Set(TInt aRemote, const char* aEntry, TUint32 aMask, TUint32 aValue, TUint32 aGeneration)
	{
	SEdge* e = Edge(aRemote);
	if (!e)
		return KErrArgument;
	NKern::FMWait(&StateLock);
	SItem* out = e->iOut;
	TInt r = KErrNotReady;
	if (out && (aGeneration == ~0u || aGeneration == e->iGeneration))
		{
		r = KErrNotFound;
		for (TInt i = 0; i < out->iValid && i < KEntries; ++i)
			if (NameIs(out->iEntry[i].iName, aEntry))
				{
				volatile TUint32& v = out->iEntry[i].iValue;
				TUint32 old = v, now = (old & ~aMask) | (aValue & aMask);
				v = now;
				if (now != old)
					Ipc_Kick(e->iBit);
				r = KErrNone;
				break;
				}
		}
	NKern::FMSignal(&StateLock);
	return r;
	}

void Smp2p_Poll(TInt aRemote)
	{
	SEdge* e = Edge(aRemote);
	SItem* in = e ? In(*e) : NULL;
	if (!in)
		return;
	NKern::FMWait(&StateLock);
	if (!e->iOut)
		{
		NKern::FMSignal(&StateLock);
		return;
		}
	TUint32 inV = in->iVersionFeatures & 0xff, outV = e->iOut->iVersionFeatures & 0xff;
	if (!e->iNegotiated)
		{
		if (inV == outV)
			{
			TUint32 features = (e->iOut->iVersionFeatures >> 8) & (in->iVersionFeatures >> 8);
			e->iOut->iVersionFeatures = outV | features << 8;
			e->iSsrAck = (features & KFeatureSsrAck) != 0;
			e->iNegotiated = ETrue;
			}
		else if (inV && inV < outV)
			{
			e->iOut->iVersionFeatures = (e->iOut->iVersionFeatures & ~0xffu) | inV;
			Ipc_Kick(e->iBit);
			}
		}
	if (e->iSsrAck && ((in->iFlags & KFlagRestartDone) != 0) != e->iAckState)	// the remote restarted: ack it
		{
		e->iAckState = !e->iAckState;
		e->iOut->iFlags = (e->iOut->iFlags & ~KFlagRestartAck) | (e->iAckState ? KFlagRestartAck : 0);
		Ipc_Kick(e->iBit);
		}
	NKern::FMSignal(&StateLock);
	}

void Smp2p_Dump(TInt aRemote, void (*aSay)(const char*, ...))
	{
	SEdge* e = Edge(aRemote);
	if (!e)
		return;
	SItem snapshot;
	NKern::FMWait(&StateLock);
	TBool haveOut = e->iOut != NULL;
	if (haveOut)
		memcpy(&snapshot, e->iOut, sizeof(snapshot));
	TBool negotiated = e->iNegotiated;
	NKern::FMSignal(&StateLock);
	const SItem* items[2] = { haveOut ? &snapshot : NULL, In(*e) };
	for (TInt k = 0; k < 2; ++k)
		{
		const SItem* it = items[k];
		if (!it)
			{
			aSay("smp2p %d %s: none", aRemote, k ? "in" : "out");
			continue;
			}
		aSay("smp2p %d %s: version %d features %x pids %d->%d entries %d flags %x%s", aRemote, k ? "in" : "out",
			it->iVersionFeatures & 0xff, it->iVersionFeatures >> 8, it->iLocalPid, it->iRemotePid, it->iValid,
			it->iFlags, !k && negotiated ? " negotiated" : "");
		for (TInt i = 0; i < it->iValid && i < KEntries; ++i)
			{
			TBuf8<KName + 1> n;
			for (TInt c = 0; c < KName && it->iEntry[i].iName[c]; ++c)
				n.Append(it->iEntry[i].iName[c]);
			aSay("  %S = %08x", &n, it->iEntry[i].iValue);
			}
		}
	}

// Remote must be stopped first. Only clear our entries; the remote owns its inbound item.
TInt Smp2p_Reset(TInt aRemote)
	{
	SEdge* e = Edge(aRemote);
	if (!e)
		return KErrArgument;
	NKern::FMWait(&StateLock);
	if (e->iOut)
		{
		for (TInt i = 0; i < e->iOut->iValid && i < KEntries; ++i)
			e->iOut->iEntry[i].iValue = 0;
		__e32_memory_barrier();
		Ipc_Kick(e->iBit);
		}
	e->iNegotiated = EFalse;
	e->iSsrAck = EFalse;
	e->iAckState = EFalse;
	e->iOut = NULL;
	++e->iGeneration;
	NKern::FMSignal(&StateLock);
	return KErrNone;
	}
