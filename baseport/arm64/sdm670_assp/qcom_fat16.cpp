// SPDX-License-Identifier: Apache-2.0
//
// qcom_fat16.cpp -- read-only files from a FAT16 partition of the eMMC, found by GPT name. The modem's firmware
// partition modem_a is FAT16 with 4096-byte sectors and 8.3 names (IMAGE/MBA.MBN, MODEM.MDT, MODEM.B00..B30), so the
// kernel loads the modem itself, without the file server (docs/telephony.md C3). Device reads only
// (A64BlkDev_Read): nothing here can write the eMMC.
// shortcut: one cluster per device read and 8.3 names only; enough for 110 MB of firmware at boot.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_blk.h"
#include "qcom_fat16.h"

const TInt KSector = KA64BlkSector;

static TUint8 Sec[KSector];

TInt Gpt_Find(const char* aName, TUint64& aFirst, TUint64& aCount)
	{
	TInt r = A64BlkDev_Read(1, 1, Sec);
	if (r != KErrNone)
		return r;
	if (memcompare(Sec, 8, (const TUint8*)"EFI PART", 8) != 0)
		return KErrCorrupt;
	TUint64 lba = *(TUint64*)(Sec + 72);
	TUint32 n = *(TUint32*)(Sec + 80), size = *(TUint32*)(Sec + 84);
	if (size < 128 || size > (TUint32)KSector || KSector % size)
		return KErrCorrupt;
	for (TUint32 i = 0; i < n; ++i)
		{
		TUint32 off = i * size;
		if (off % KSector == 0 && (r = A64BlkDev_Read(lba + off / KSector, 1, Sec)) != KErrNone)
			return r;
		const TUint8* e = Sec + off % KSector;
		TInt c = 0;
		for (; c < 36 && aName[c]; ++c)
			if (e[56 + 2 * c] != (TUint8)aName[c] || e[57 + 2 * c])
				break;
		if (aName[c] || c >= 36 || e[56 + 2 * c] || e[57 + 2 * c])
			continue;
		aFirst = *(const TUint64*)(e + 32);
		aCount = *(const TUint64*)(e + 40) - aFirst + 1;
		return KErrNone;
		}
	return KErrNotFound;
	}

// ---- the mounted volume
static struct
	{
	TUint64 iLba, iCount;			// the partition, in device sectors
	TUint32 iBps, iSpc, iFat, iRoot, iRootEntries, iData, iClusters;	// iFat/iRoot/iData: byte offsets
	TUint8* iClus;					// one cluster
	} V;
static TUint32 FatCacheOff = 0xffffffff;
static TUint8 FatCache[KSector];

static TInt ReadBytes(TUint32 aOff, TAny* aBuf, TUint32 aLen)	// sector-aligned offset and length
	{
	if (aOff % KSector || aLen % KSector || (TUint64)(aOff + aLen) / KSector > V.iCount)
		return KErrArgument;
	return A64BlkDev_Read(V.iLba + aOff / KSector, aLen / KSector, (TUint8*)aBuf);
	}

TInt Fat_Mount(const char* aPartition)
	{
	TInt r = Gpt_Find(aPartition, V.iLba, V.iCount);
	if (r != KErrNone)
		return r;
	if ((r = A64BlkDev_Read(V.iLba, 1, Sec)) != KErrNone)
		return r;
	TUint32 bps = Sec[11] | Sec[12] << 8, spc = Sec[13], rsv = Sec[14] | Sec[15] << 8, nfat = Sec[16];
	TUint32 nroot = Sec[17] | Sec[18] << 8, total = Sec[19] | Sec[20] << 8, spf = Sec[22] | Sec[23] << 8;
	if (!total)
		total = *(TUint32*)(Sec + 32);
	if (Sec[510] != 0x55 || Sec[511] != 0xaa || bps < 512 || bps % KSector || !spc || !nfat || !spf || !nroot)
		return KErrCorrupt;										// FAT32 (no root entries) is not handled
	V.iBps = bps;
	V.iSpc = spc;
	V.iFat = rsv * bps;
	V.iRoot = (rsv + nfat * spf) * bps;
	V.iRootEntries = nroot;
	V.iData = V.iRoot + ((nroot * 32 + bps - 1) / bps) * bps;
	V.iClusters = (total * bps - V.iData) / (spc * bps);
	if (V.iClusters < 4085)
		return KErrNotSupported;								// FAT12
	if (V.iClus)
		Kern::Free(V.iClus);
	NKern::ThreadEnterCS();
	V.iClus = (TUint8*)Kern::Alloc(spc * bps);
	NKern::ThreadLeaveCS();
	FatCacheOff = 0xffffffff;
	return V.iClus ? KErrNone : KErrNoMemory;
	}

static TUint32 Next(TUint32 aCluster)						// the FAT entry: the chain's next cluster
	{
	TUint32 off = aCluster * 2, sec = V.iFat + (off & ~(KSector - 1));
	if (sec != FatCacheOff)
		{
		if (ReadBytes(sec, FatCache, KSector) != KErrNone)
			return 0xffff;
		FatCacheOff = sec;
		}
	return FatCache[off % KSector] | FatCache[off % KSector + 1] << 8;
	}

static TBool End(TUint32 aC) { return aC < 2 || aC >= 0xfff8; }

static TInt ReadCluster(TUint32 aC)
	{
	if (aC < 2 || aC - 2 >= V.iClusters)
		return KErrCorrupt;
	return ReadBytes(V.iData + (aC - 2) * V.iSpc * V.iBps, V.iClus, V.iSpc * V.iBps);
	}

static void Name83(const TUint8* aEntry, TDes8& aName)		// "MODEM   B05" -> "MODEM.B05"
	{
	aName.Zero();
	for (TInt i = 0; i < 8 && aEntry[i] != ' '; ++i)
		aName.Append(aEntry[i]);
	if (aEntry[8] != ' ')
		aName.Append('.');
	for (TInt i = 8; i < 11 && aEntry[i] != ' '; ++i)
		aName.Append(aEntry[i]);
	}

static TBool SameName(const TDesC8& aA, const char* aB, TInt aLen)	// case-insensitive
	{
	if (aA.Length() != aLen)
		return EFalse;
	for (TInt i = 0; i < aLen; ++i)
		{
		TUint8 a = aA[i], b = (TUint8)aB[i];
		if ((a >= 'a' && a <= 'z' ? a - 32 : a) != (b >= 'a' && b <= 'z' ? b - 32 : b))
			return EFalse;
		}
	return ETrue;
	}

// walk a directory (aDir.iCluster 0 = the root): calls aFn on each file/dir entry until it returns ETrue
template <class F> static TInt Walk(const SFatFile& aDir, F& aFn)
	{
	TBuf8<13> name;
	if (!aDir.iCluster)
		{
		for (TUint32 i = 0; i < V.iRootEntries; i += KSector / 32)
			{
			if (ReadBytes(V.iRoot + i * 32, Sec, KSector) != KErrNone)
				return KErrCorrupt;
			for (TInt j = 0; j < KSector; j += 32)
				{
				const TUint8* e = Sec + j;
				if (!e[0])
					return KErrNotFound;
				if (e[0] == 0xe5 || e[0] == '.' || e[11] == 0x0f || e[11] & 8)
					continue;
				Name83(e, name);
				if (aFn(name, e))
					return KErrNone;
				}
			}
		return KErrNotFound;
		}
	for (TUint32 c = aDir.iCluster; !End(c); c = Next(c))
		{
		TInt r = ReadCluster(c);
		if (r != KErrNone)
			return r;
		for (TUint32 j = 0; j < V.iSpc * V.iBps; j += 32)
			{
			const TUint8* e = V.iClus + j;
			if (!e[0])
				return KErrNotFound;
			if (e[0] == 0xe5 || e[0] == '.' || e[11] == 0x0f || e[11] & 8)
				continue;
			Name83(e, name);
			if (aFn(name, e))
				return KErrNone;
			}
		}
	return KErrNotFound;
	}

struct SFindFn
	{
	const char* iName; TInt iLen; SFatFile iFound; TBool iDir;
	TBool operator()(const TDesC8& aName, const TUint8* aE)
		{
		if (!SameName(aName, iName, iLen))
			return EFalse;
		iFound.iCluster = aE[26] | aE[27] << 8;
		iFound.iSize = *(const TUint32*)(aE + 28);
		iDir = (aE[11] & 0x10) != 0;
		return ETrue;
		}
	};

static TInt Lookup(const char* aPath, SFatFile& aFile, TBool& aDir)
	{
	if (!V.iClus)
		return KErrNotReady;
	SFatFile cur = { 0, 0 };
	aDir = ETrue;
	for (const char* p = aPath; *p; )
		{
		const char* q = p;
		while (*q && *q != '/')
			++q;
		if (q > p)
			{
			if (!aDir)
				return KErrPathNotFound;
			SFindFn fn = { p, (TInt)(q - p), { 0, 0 }, EFalse };
			TInt r = Walk(cur, fn);
			if (r != KErrNone)
				return r;
			cur = fn.iFound;
			aDir = fn.iDir;
			}
		p = *q ? q + 1 : q;
		}
	aFile = cur;
	return KErrNone;
	}

TInt Fat_Find(const char* aPath, SFatFile& aFile)
	{
	TBool dir;
	TInt r = Lookup(aPath, aFile, dir);
	return r == KErrNone && dir ? KErrNotFound : r;
	}

TInt Fat_Read(const SFatFile& aFile, TUint32 aPos, TAny* aBuf, TUint32 aLen)
	{
	if (!V.iClus)
		return KErrNotReady;
	if (aPos > aFile.iSize || aLen > aFile.iSize - aPos)
		return KErrEof;
	const TUint32 cs = V.iSpc * V.iBps;
	TUint32 c = aFile.iCluster;
	for (TUint32 skip = aPos / cs; skip && !End(c); --skip)
		c = Next(c);
	TUint8* out = (TUint8*)aBuf;
	TUint32 off = aPos % cs;
	while (aLen)
		{
		if (End(c))
			return KErrCorrupt;
		TInt r = ReadCluster(c);
		if (r != KErrNone)
			return r;
		TUint32 n = Min(aLen, cs - off);
		memcpy(out, V.iClus + off, n);
		out += n;
		aLen -= n;
		off = 0;
		c = Next(c);
		}
	return KErrNone;
	}

struct SNthFn
	{
	TInt iWant, iAt; TDes8* iName; TUint32 iSize;
	TBool operator()(const TDesC8& aName, const TUint8* aE)
		{
		if (iAt++ != iWant)
			return EFalse;
		iName->Copy(aName);
		iSize = *(const TUint32*)(aE + 28);
		return ETrue;
		}
	};

TInt Fat_DirEntry(const char* aDir, TInt aIndex, TDes8& aName, TUint32& aSize)
	{
	SFatFile dir;
	TBool isDir;
	TInt r = Lookup(aDir, dir, isDir);
	if (r != KErrNone)
		return r;
	if (!isDir)
		return KErrPathNotFound;
	SNthFn fn = { aIndex, 0, &aName, 0 };
	r = Walk(dir, fn);
	if (r == KErrNotFound)
		return KErrEof;
	aSize = fn.iSize;
	return r;
	}
