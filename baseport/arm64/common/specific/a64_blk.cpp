// SPDX-License-Identifier: Apache-2.0
//
// a64_blk.cpp -- the window every block access goes through (a64_blk.h): a request outside it is refused before
// the backend sees it. On the Pixel the window is one eMMC partition, so a bug above can't touch the bootloaders.
#include <kernel/kern_priv.h>
#include "a64_blk.h"

__attribute__((weak)) TInt A64BlkDev_Init(TUint64&) { return KErrNotSupported; }
__attribute__((weak)) TInt A64BlkDev_Read(TUint64, TInt, TUint8*) { return KErrNotSupported; }
__attribute__((weak)) TInt A64BlkDev_Write(TUint64, TInt, const TUint8*) { return KErrNotSupported; }
__attribute__((weak)) TUint32 A64BlkDev_EraseBlock() { return 0; }
__attribute__((weak)) TInt A64BlkDev_Discard(TUint64, TInt) { return KErrNone; }
__attribute__((weak)) TInt A64BlkDev_Partition(const char*, TUint64&, TUint64&) { return KErrNotFound; }
__attribute__((weak)) TInt A64BlkBoard_Window(TUint64 aSectors, TUint64& aFirst, TUint64& aCount)
	{
	aFirst = 0;
	aCount = aSectors;
	return KErrNone;
	}

static TUint64 First, Count;

TInt A64Blk_Start()
	{
	TUint64 sectors = 0;
	TInt r = A64BlkDev_Init(sectors);
	if (r == KErrNone)
		r = A64BlkBoard_Window(sectors, First, Count);
	if (r == KErrNone && (First + Count > sectors || Count == 0))
		r = KErrCorrupt;
	if (r != KErrNone)
		First = Count = 0;
	Kern::Printf("[blk] device %d sectors, window %d + %d -> %d", (TInt)sectors, (TInt)First, (TInt)Count, r);
	return r;
	}

TUint64 A64Blk_Sectors() { return Count; }
TUint64 A64Blk_First() { return First; }

static TBool Inside(TUint64 aLba, TInt aCount)
	{
	return aCount > 0 && aLba < Count && (TUint64)aCount <= Count - aLba;
	}

TInt A64Blk_Read(TUint64 aLba, TInt aCount, TUint8* aBuf)
	{
	return Inside(aLba, aCount) ? A64BlkDev_Read(First + aLba, aCount, aBuf) : KErrAccessDenied;
	}

TInt A64Blk_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf)
	{
	return Inside(aLba, aCount) ? A64BlkDev_Write(First + aLba, aCount, aBuf) : KErrAccessDenied;
	}

TInt A64Blk_Discard(TUint64 aLba, TInt aCount)
	{
	return Inside(aLba, aCount) ? A64BlkDev_Discard(First + aLba, aCount) : KErrAccessDenied;
	}
