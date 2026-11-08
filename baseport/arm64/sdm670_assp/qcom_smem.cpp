// SPDX-License-Identifier: Apache-2.0
//
// qcom_smem.cpp -- SMEM, the memory the SDM670 shares with its remote processors (DTB smem@86000000, 2 MB, no-map,
// hwlock = TCSR mutex 3). The boot chain lays it out:
//   +0          a header: version[32] (SBL version in [7]: 11 = global heap, 12 = global partition) and a
//               table of contents for the global heap's items
//   end - 4 KB  "$TOC": the partition table. Each entry is a private partition between two hosts (apps = 0 and a
//               remote) or the global one (0xfffe both). A partition "$PRT" grows uncached items up from its header
//               and cached items down from its end.
// Items are found by number (SMP2P 435 / 428, GLINK 478..480); allocating one takes the TCSR hardware mutex,
// because the remote processors allocate too.
// Written from the layout in Linux drivers/soc/qcom/smem.c. Telephony stage C2 (docs/telephony.md).
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_smem.h"

const TUint32 KSmemPhys = 0x86000000, KSmemSize = 0x200000;
const TUint32 KMutexPhys = 0x1f43000;						// TCSR mutex block 0x1f40000, stride 0x1000, lock 3
const TUint32 KMutexApps = 1;								// the apps processor's lock value
const TInt KItems = 512, KHostApps = 0, KHostGlobal = 0xfffe;

struct SGlobalEntry { TUint32 iAllocated, iOffset, iSize, iAuxBase; };
struct SHeader { TUint32 iProcComm[16]; TUint32 iVersion[32]; TUint32 iInitialized, iFreeOffset, iAvailable,
	iReserved; SGlobalEntry iToc[KItems]; };
struct SPtEntry { TUint32 iOffset, iSize, iFlags; TUint16 iHost0, iHost1; TUint32 iCacheline, iReserved[7]; };
struct SPtable { TUint8 iMagic[4]; TUint32 iVersion, iNum, iReserved[5]; SPtEntry iEntry[1]; };
struct SPartHdr { TUint8 iMagic[4]; TUint16 iHost0, iHost1; TUint32 iSize, iFreeUncached, iFreeCached, iReserved[3]; };
struct SPrivEntry { TUint16 iCanary, iItem; TUint32 iSize; TUint16 iPadData, iPadHdr; TUint32 iReserved; };
const TUint16 KCanary = 0xa5a5;

struct SPart { SPartHdr* iHdr; TUint32 iSize, iCacheline; TInt iRemote; };
const TInt KMaxParts = 8;
static SPart Parts[KMaxParts];
static TInt NParts;
static SPart Global;										// iHdr NULL: a version-11 global heap (the header's toc)
static TUint8* Base;
static TUint32 Version;

static TUint32 Align(TUint32 aV, TUint32 aA) { return (aV + aA - 1) & ~(aA - 1); }

static void Fmt(TDes8& aBuf, const char* aFmt, ...)		// append, Kern::Printf's format
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aBuf, aFmt, list);
	VA_END(list);
	}

static NFastMutex InitLock;									// the monitor and the modem's boot thread race to init

static TInt DoInit();
TInt Smem_Init()
	{
	if (Base)
		return KErrNone;
	NKern::FMWait(&InitLock);
	TInt r = Base ? KErrNone : DoInit();
	NKern::FMSignal(&InitLock);
	return r;
	}

static TInt DoInit()
	{
	TUint8* base = (TUint8*)A64_Io(KSmemPhys);
	if (!base)
		return KErrNotSupported;
	const SHeader* h = (const SHeader*)base;
	Version = h->iVersion[7] >> 16;
	if (!h->iInitialized || (Version != 11 && Version != 12))
		return KErrNotReady;
	const SPtable* pt = (const SPtable*)(base + KSmemSize - 0x1000);
	if (memcompare(pt->iMagic, 4, (const TUint8*)"$TOC", 4) != 0 || pt->iVersion != 1)
		return KErrCorrupt;
	NParts = 0;
	for (TUint32 i = 0; i < pt->iNum && i < 64; ++i)
		{
		const SPtEntry& e = pt->iEntry[i];
		if (!e.iOffset || !e.iSize || e.iOffset + e.iSize > KSmemSize)
			continue;
		TBool global = e.iHost0 == KHostGlobal && e.iHost1 == KHostGlobal;
		if (!global && e.iHost0 != KHostApps && e.iHost1 != KHostApps)
			continue;										// another pair's partition: never touch it (the bus hangs)
		SPartHdr* ph = (SPartHdr*)(base + e.iOffset);
		if (memcompare(ph->iMagic, 4, (const TUint8*)"$PRT", 4) != 0)
			continue;
		if (global)
			{
			Global.iHdr = ph;
			Global.iSize = e.iSize;
			Global.iCacheline = e.iCacheline;
			Global.iRemote = KSmemHostGlobal;
			}
		else if (NParts < KMaxParts)
			{
			TInt remote = e.iHost0 == KHostApps ? e.iHost1 : e.iHost0, k = 0;
			while (k < NParts && Parts[k].iRemote != remote)
				++k;
			if (k < NParts)
				continue;									// the table can list a pair twice
			SPart& p = Parts[NParts++];
			p.iHdr = ph;
			p.iSize = e.iSize;
			p.iCacheline = e.iCacheline;
			p.iRemote = e.iHost0 == KHostApps ? e.iHost1 : e.iHost0;
			}
		}
	Base = base;
	return KErrNone;
	}

static SPart* Part(TInt aHost)
	{
	if (aHost == KSmemHostGlobal)
		return Global.iHdr ? &Global : NULL;
	for (TInt i = 0; i < NParts; ++i)
		if (Parts[i].iRemote == aHost)
			return &Parts[i];
	return Global.iHdr ? &Global : NULL;					// a host without its own partition uses the global one
	}

// an item in a partition: uncached entries upwards from the header, then cached ones downwards from the end
static TAny* GetPrivate(const SPart& aP, TUint aItem, TUint32* aSize)
	{
	TUint8* p = (TUint8*)aP.iHdr;
	TUint8* end = p + aP.iHdr->iFreeUncached;
	for (TUint8* e = p + sizeof(SPartHdr); e < end; )
		{
		const SPrivEntry* pe = (const SPrivEntry*)e;
		if (pe->iCanary != KCanary)
			return NULL;
		if (pe->iItem == aItem)
			{
			if (aSize)
				*aSize = pe->iSize - pe->iPadData;
			return e + sizeof(SPrivEntry) + pe->iPadHdr;
			}
		e += sizeof(SPrivEntry) + pe->iPadHdr + pe->iSize;
		}
	TUint32 cl = aP.iCacheline ? aP.iCacheline : 64;
	TUint8* last = p + aP.iHdr->iFreeCached;
	for (TUint8* e = p + aP.iHdr->iSize - Align(sizeof(SPrivEntry), cl); e > last; )
		{
		const SPrivEntry* pe = (const SPrivEntry*)e;
		if (pe->iCanary != KCanary)
			return NULL;
		if (pe->iItem == aItem)
			{
			if (aSize)
				*aSize = pe->iSize - pe->iPadData;
			return e - pe->iSize;
			}
		e -= pe->iSize + Align(sizeof(SPrivEntry), cl);
		}
	return NULL;
	}

TAny* Smem_Get(TInt aHost, TUint aItem, TUint32* aSize)
	{
	if (Smem_Init() != KErrNone || aItem >= (TUint)KItems)
		return NULL;
	if (const SPart* p = Part(aHost))
		return GetPrivate(*p, aItem, aSize);
	const SGlobalEntry& g = ((const SHeader*)Base)->iToc[aItem];	// version 11: the global heap
	if (!g.iAllocated || g.iOffset + g.iSize > KSmemSize)
		return NULL;
	if (aSize)
		*aSize = g.iSize;
	return Base + g.iOffset;
	}

// the TCSR hardware mutex: write our id, it holds if it reads back
static TBool Lock()
	{
	volatile TUint32* m = (volatile TUint32*)A64_Io(KMutexPhys);
	if (!m)
		return EFalse;
	for (TInt i = 0; i < 100000; ++i)
		{
		*m = KMutexApps;
		if (*m == KMutexApps)
			return ETrue;
		Kern::NanoWait(1000);
		}
	return EFalse;
	}
static void Unlock() { *(volatile TUint32*)A64_Io(KMutexPhys) = 0; }

TInt Smem_Alloc(TInt aHost, TUint aItem, TUint32 aSize)
	{
	TInt r = Smem_Init();
	if (r != KErrNone)
		return r;
	if (aItem >= (TUint)KItems)
		return KErrArgument;
	SPart* p = Part(aHost);
	if (!p)
		return KErrNotSupported;							// shortcut: no version-11 global heap allocation (sdm670 is 12)
	if (!Lock())
		return KErrTimedOut;
	r = KErrNone;
	if (GetPrivate(*p, aItem, NULL))
		r = KErrAlreadyExists;
	else
		{
		SPartHdr* ph = p->iHdr;
		TUint32 need = sizeof(SPrivEntry) + Align(aSize, 8);
		if (ph->iFreeUncached + need > ph->iFreeCached)
			r = KErrNoMemory;
		else
			{
			SPrivEntry* e = (SPrivEntry*)((TUint8*)ph + ph->iFreeUncached);
			e->iCanary = KCanary;
			e->iItem = (TUint16)aItem;
			e->iSize = Align(aSize, 8);
			e->iPadData = (TUint16)(e->iSize - aSize);
			e->iPadHdr = 0;
			e->iReserved = 0;
			memclr((TUint8*)e + sizeof(SPrivEntry), e->iSize);
			__e32_memory_barrier();
			ph->iFreeUncached += need;
			}
		}
	Unlock();
	return r;
	}

TUint32 Smem_Phys(const TAny* aPtr) { return KSmemPhys + (TUint32)((const TUint8*)aPtr - Base); }

void Smem_Dump(void (*aSay)(const char*, ...))
	{
	TInt r = Smem_Init();
	if (r != KErrNone)
		{
		aSay("smem: not ready (%d)", r);
		return;
		}
	aSay("smem: version %d, %d private partitions, global %s", Version, NParts, Global.iHdr ? "partition" : "heap");
	for (TInt i = -1; i < NParts; ++i)
		{
		const SPart& p = i < 0 ? Global : Parts[i];
		if (!p.iHdr)
			continue;
		TBuf8<200> b;
		Fmt(b, "%s host %d: +0x%x size 0x%x free 0x%x..0x%x items", i < 0 ? "global" : "part",
			p.iRemote, (TUint32)((TUint8*)p.iHdr - Base), p.iSize, p.iHdr->iFreeUncached, p.iHdr->iFreeCached);
		TInt n = 0;
		TUint8* end = (TUint8*)p.iHdr + p.iHdr->iFreeUncached;
		for (TUint8* e = (TUint8*)p.iHdr + sizeof(SPartHdr); e < end && n < 24; ++n)
			{
			const SPrivEntry* pe = (const SPrivEntry*)e;
			if (pe->iCanary != KCanary)
				break;
			Fmt(b, " %d", pe->iItem);
			e += sizeof(SPrivEntry) + pe->iPadHdr + pe->iSize;
			}
		aSay("%S", &b);
		}
	}
