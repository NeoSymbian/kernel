// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// a64s_rwspin.cpp -- TRWSpinLock and NKern::KernelLocked for the AArch64 backend: C++ ports of
// nkernsmp/arm/ncutilf.cia (TRWSpinLock) and nkernsmp/arm/ncsched.cia (KernelLocked), same algorithm.
// The lock word (low 32 bits of iLock) holds four byte tickets: in.w (0), in.r (1), out.w (2), out.r (3).
//   read lock:   take in.r++ (no carry), wait until out.w == my in.w        read unlock:  out.r++
//   write lock:  take in.w++ (no carry), wait until out(w,r) == my in(w,r)  write unlock: out.w++
#include <e32def.h>
#include <nkern.h>
#include <nk_priv.h>
#include "a64_sysreg.h"

TSubScheduler& SubScheduler();

static inline volatile TUint32* Word(volatile TUint64& aLock) { return (volatile TUint32*)&aLock; }

static inline TUint32 TakeTicket(volatile TUint32* aW, TUint32 aByteShift)
	{	// increment one byte of the word without carrying into the next; returns the ORIGINAL word
	TUint32 o = __atomic_load_n(aW, __ATOMIC_RELAXED), n;
	do	{
		TUint32 b = ((o >> aByteShift) + 1) & 0xFF;
		n = (o & ~(0xFFu << aByteShift)) | (b << aByteShift);
		} while (!__atomic_compare_exchange_n(aW, &o, n, true, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED));
	return o;
	}

static inline void RLock(volatile TUint64& aLock)
	{
	volatile TUint32* w = Word(aLock);
	TUint32 inw = TakeTicket(w, 8) & 0xFF;
	while (((__atomic_load_n(w, __ATOMIC_ACQUIRE) >> 16) & 0xFF) != inw)
		A64_PAUSE();
	}
static inline void RUnlock(volatile TUint64& aLock)	{ __atomic_fetch_add(Word(aLock), 0x01000000u, __ATOMIC_RELEASE); }

static inline void WLock(volatile TUint64& aLock)
	{
	volatile TUint32* w = Word(aLock);
	TUint32 in = TakeTicket(w, 0) & 0xFFFF;
	while ((__atomic_load_n(w, __ATOMIC_ACQUIRE) >> 16) != in)
		A64_PAUSE();
	}
static inline void WUnlock(volatile TUint64& aLock)
	{	// only the writer touches out.w, but readers update out.r in the same word: one atomic byte store
	volatile TUint8* outw = (volatile TUint8*)&aLock + 2;
	__atomic_store_n(outw, (TUint8)(*outw + 1), __ATOMIC_RELEASE);
	}
static inline TBool WriterWaiting(volatile TUint64& aLock)	// called by a READER holding the lock
	{	// out.w was equal to our in.w ticket when we got in; any later writer has moved in.w on
	TUint32 v = __atomic_load_n(Word(aLock), __ATOMIC_RELAXED);
	return (v & 0xFF) != ((v >> 16) & 0xFF);
	}
static inline TBool AnyoneWaiting(volatile TUint64& aLock)	// called by the WRITER holding the lock
	{	// our ticket made in.w = out.w + 1; another writer moves in.w further, a reader moves in.r past out.r
	TUint32 v = __atomic_load_n(Word(aLock), __ATOMIC_RELAXED);
	return (TUint8)(v & 0xFF) != (TUint8)(((v >> 16) & 0xFF) + 1) || ((v >> 8) & 0xFF) != (v >> 24);
	}

EXPORT_C TRWSpinLock::TRWSpinLock(TUint /*aOrder*/)	{ iLock = 0; }

EXPORT_C void TRWSpinLock::LockOnlyR()			{ RLock(iLock); }
EXPORT_C void TRWSpinLock::UnlockOnlyR()		{ RUnlock(iLock); }
EXPORT_C void TRWSpinLock::LockOnlyW()			{ WLock(iLock); }
EXPORT_C void TRWSpinLock::UnlockOnlyW()		{ WUnlock(iLock); }
EXPORT_C void TRWSpinLock::LockIrqR()			{ A64_CLI(); RLock(iLock); }
EXPORT_C void TRWSpinLock::UnlockIrqR()			{ RUnlock(iLock); A64_STI(); }
EXPORT_C void TRWSpinLock::LockIrqW()			{ A64_CLI(); WLock(iLock); }
EXPORT_C void TRWSpinLock::UnlockIrqW()			{ WUnlock(iLock); A64_STI(); }
EXPORT_C TInt TRWSpinLock::LockIrqSaveR()		{ TInt r = NKern::DisableAllInterrupts(); RLock(iLock); return r; }
EXPORT_C void TRWSpinLock::UnlockIrqRestoreR(TInt a)	{ RUnlock(iLock); NKern::RestoreInterrupts(a); }
EXPORT_C TInt TRWSpinLock::LockIrqSaveW()		{ TInt r = NKern::DisableAllInterrupts(); WLock(iLock); return r; }
EXPORT_C void TRWSpinLock::UnlockIrqRestoreW(TInt a)	{ WUnlock(iLock); NKern::RestoreInterrupts(a); }

EXPORT_C TBool TRWSpinLock::FlashOnlyR()		{ if (!WriterWaiting(iLock)) return FALSE; UnlockOnlyR(); LockOnlyR(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashOnlyW()		{ if (!AnyoneWaiting(iLock)) return FALSE; UnlockOnlyW(); LockOnlyW(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashIrqR()			{ if (!WriterWaiting(iLock)) return FALSE; UnlockIrqR(); LockIrqR(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashIrqW()			{ if (!AnyoneWaiting(iLock)) return FALSE; UnlockIrqW(); LockIrqW(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashIrqRestoreR(TInt a)	{ if (!WriterWaiting(iLock)) return FALSE; UnlockIrqRestoreR(a); LockIrqR(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashIrqRestoreW(TInt a)	{ if (!AnyoneWaiting(iLock)) return FALSE; UnlockIrqRestoreW(a); LockIrqW(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashPreemptR()		{ if (!WriterWaiting(iLock)) return FALSE; UnlockOnlyR(); NKern::PreemptionPoint(); LockOnlyR(); return TRUE; }
EXPORT_C TBool TRWSpinLock::FlashPreemptW()		{ if (!AnyoneWaiting(iLock)) return FALSE; UnlockOnlyW(); NKern::PreemptionPoint(); LockOnlyW(); return TRUE; }

/* ncsched.cia: TRUE iff the kernel is locked (aCount == 0) or locked exactly aCount times; returns the count */
EXPORT_C TBool NKern::KernelLocked(TInt aCount)
	{
	TInt irq = NKern::DisableAllInterrupts();
	TInt n = SubScheduler().iKernLockCount;
	NKern::RestoreInterrupts(irq);
	if (aCount == 0) aCount = n;
	return aCount == n ? n : 0;
	}
