// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// a64_medblk.cpp -- the persistent drives C: and E:: a fixed-media driver for ELOCD (shaped like the 2010 internal
// RAM drive, kernel/eka/drivers/medint/iram.cpp) on the block window of a64_blk.h -- QEMU's virtio-blk disk, the
// Pixel's eMMC userdata partition. An MBR in the window's sector 0 splits it (docs/filesystem.md FS3): C: (8 GB) and
// E: (the rest), local drives 0 and 1; ELOCD keeps each request inside its partition (CheckAndAdjustForPartition).
// Byte ranges are turned into whole sectors through a 64 KB bounce buffer; partial sectors are read-modify-written.
// shortcut: synchronous in the caller's thread, one request at a time, no request merging or read-ahead.
#include "locmedia.h"
#include "platform.h"
#include "a64_blk.h"

_LIT(KPddName, "Media.Blk");
_LIT(KDriveName, "Blk");
const TMediaDevice KMediaDevice = EFixedMedia1;
const TInt KBounce = 64 * 1024;
const TUint64 KCMax = (8ull << 30) / KA64BlkSector;		// C:'s size, at most
const TUint64 KAlign = (4 << 20) / KA64BlkSector;		// partitions start on 4 MiB boundaries of the device
const TUint64 KGap = (1 << 20) / KA64BlkSector;			// at least 1 MiB before C:: the MBR, then test sectors
const TUint64 KEMin = (64 << 20) / KA64BlkSector;		// a smaller rest gets no E:
const TInt KMaxParts = 2;

class DPhysicalDeviceMediaBlk : public DPhysicalDevice
	{
public:
	DPhysicalDeviceMediaBlk()
		{
		iUnitsMask = 0x1;
		iVersion = TVersion(KMediaDriverInterfaceMajorVersion, KMediaDriverInterfaceMinorVersion,
			KMediaDriverInterfaceBuildVersion);
		}
	virtual TInt Install() { return SetName(&KPddName); }
	virtual void GetCaps(TDes8&) const {}
	virtual TInt Create(DBase*& aChannel, TInt aMediaId, const TDesC8*, const TVersion& aVer);
	virtual TInt Validate(TInt aDeviceType, const TDesC8*, const TVersion& aVer)
		{
		if (!Kern::QueryVersionSupported(iVersion, aVer))
			return KErrNotSupported;
		return aDeviceType == KMediaDevice ? KErrNone : KErrNotSupported;
		}
	virtual TInt Info(TInt aFunction, TAny*) { return aFunction == EPriority ? KMediaDriverPriorityNormal : KErrNotSupported; }
	};

class DMediaDriverBlk : public DMediaDriver
	{
public:
	DMediaDriverBlk(TInt aMediaId) : DMediaDriver(aMediaId) {}
	TInt DoCreate();
	virtual TInt Request(TLocDrvRequest& aRequest);
	virtual void Disconnect(DLocalDrive*, TThreadMessage*) {}
	virtual TInt PartitionInfo(TPartitionInfo& aInfo);
	virtual void NotifyPowerDown() {}
	virtual void NotifyEmergencyPowerDown() {}
private:
	TInt Caps(TLocDrvRequest& m);
	TInt Transfer(TLocDrvRequest& m, TInt aKind);
	TInt Layout();
	TUint8* iBuf;
	};

static DMutex* Lock;									// one request at a time, through the one bounce buffer
static TUint64 Bytes() { return A64Blk_Sectors() * KA64BlkSector; }
struct TPart { TUint64 iFirst, iCount; TUint8 iType; };	// window sectors
static TPart Part[KMaxParts];
static TInt NParts = -1;								// -1: the MBR not read yet
static TUint32 Le32(const TUint8* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((TUint32)p[3] << 24); }
static void Put32(TUint8* p, TUint32 v) { p[0] = (TUint8)v; p[1] = (TUint8)(v >> 8); p[2] = (TUint8)(v >> 16); p[3] = (TUint8)(v >> 24); }

TInt DPhysicalDeviceMediaBlk::Create(DBase*& aChannel, TInt aMediaId, const TDesC8*, const TVersion& aVer)
	{
	if (!Kern::QueryVersionSupported(iVersion, aVer))
		return KErrNotSupported;
	DMediaDriverBlk* d = new DMediaDriverBlk(aMediaId);
	aChannel = d;
	return d ? d->DoCreate() : KErrNoMemory;
	}

TInt DMediaDriverBlk::DoCreate()
	{
	iBuf = (TUint8*)Kern::Alloc(KBounce);
	if (!iBuf)
		return KErrNoMemory;
	_LIT(KLockName, "MedBlk");
	TInt r = Lock ? KErrNone : Kern::MutexCreate(Lock, KLockName, KMutexOrdGeneral0);
	if (r != KErrNone)
		return r;
	SetTotalSizeInBytes((Int64)Bytes());
	return NParts < 0 ? Layout() : KErrNone;
	}

// our MBR in window sector 0 (the way Symbian's MMC driver reads one, drivers/medmmc/emmcptn.cpp), else the default
// layout written once: C: on the first 4 MiB boundary at least 1 MiB in (userdata + 7168 on the Pixel, so H2.3/FS1's
// C: keeps its place), at most 8 GB; E: on the next boundary, to the end. A FAT boot sector (the old superfloppy C:)
// is not an MBR: it is replaced
TInt DMediaDriverBlk::Layout()
	{
	const TUint64 n = A64Blk_Sectors(), first = A64Blk_First();
	TUint8* s = iBuf;
	TInt r = A64Blk_Read(0, 1, s);
	if (r != KErrNone)
		return r;
	NParts = 0;
	TBool ok = s[510] == 0x55 && s[511] == 0xaa && s[0] != 0xeb && s[0] != 0xe9;
	TUint64 end = 0;
	for (TInt i = 0; i < 4 && ok; ++i)
		{
		const TUint8* e = s + 446 + 16 * i;
		const TUint64 base = Le32(e + 8), len = Le32(e + 12);
		if (e[4] == 0 && len == 0)
			continue;
		ok = (e[4] == 0x0b || e[4] == 0x0c) && NParts < KMaxParts && base >= 1 && base >= end && len && base + len <= n;
		if (ok)
			{
			Part[NParts].iFirst = base;
			Part[NParts].iCount = len;
			Part[NParts++].iType = e[4];
			end = base + len;
			}
		}
	if (ok && NParts)
		{
		Kern::Printf("[media] MBR: %d partitions, C: %d + %d%s", NParts, (TInt)Part[0].iFirst, (TInt)Part[0].iCount,
			NParts > 1 ? ", E: after it" : "");
		return KErrNone;
		}
	NParts = 0;
	const TUint64 c = (first + KGap + KAlign - 1) / KAlign * KAlign - first;
	if (c + KEMin > n)
		return KErrNone;									// too small for a C: of its own: no drives
	Part[0].iFirst = c;
	Part[0].iCount = Min(KCMax, n - c);
	const TUint64 e = (first + c + Part[0].iCount + KAlign - 1) / KAlign * KAlign - first;
	TInt parts = e + KEMin <= n ? 2 : 1;
	if (parts == 2)
		{
		Part[1].iFirst = e;
		Part[1].iCount = n - e;
		}
	memclr(s, KA64BlkSector);
	for (TInt i = 0; i < parts; ++i)
		{
		TUint8* p = s + 446 + 16 * i;
		Part[i].iType = 0x0c;								// FAT32, LBA
		p[1] = p[5] = 0xfe; p[2] = p[6] = 0xff; p[3] = p[7] = 0xff;	// CHS: "beyond 8 GB, use the LBA fields"
		p[4] = 0x0c;
		Put32(p + 8, (TUint32)Part[i].iFirst);
		Put32(p + 12, (TUint32)Part[i].iCount);
		}
	s[510] = 0x55;
	s[511] = 0xaa;
	r = A64Blk_Write(0, 1, s);
	Kern::Printf("[media] wrote MBR: C: %d + %d, E: %d + %d -> %d", (TInt)Part[0].iFirst, (TInt)Part[0].iCount,
		parts > 1 ? (TInt)Part[1].iFirst : 0, parts > 1 ? (TInt)Part[1].iCount : 0, r);
	if (r == KErrNone)
		NParts = parts;
	return KErrNone;										// a read-only window: no drives, not a failure
	}

TInt DMediaDriverBlk::PartitionInfo(TPartitionInfo& aInfo)
	{
	aInfo.iMediaSizeInBytes = TotalSizeInBytes();
	aInfo.iPartitionCount = Max(NParts, 0);
	for (TInt i = 0; i < aInfo.iPartitionCount; ++i)
		{
		aInfo.iEntry[i].iPartitionBaseAddr = (Int64)Part[i].iFirst * KA64BlkSector;
		aInfo.iEntry[i].iPartitionLen = (Int64)Part[i].iCount * KA64BlkSector;
		aInfo.iEntry[i].iPartitionType = KPartitionTypeWin95FAT32;
		}
	return KErrNone;
	}

TInt DMediaDriverBlk::Caps(TLocDrvRequest& m)
	{
	TLocalDriveCapsV6& caps = *(TLocalDriveCapsV6*)m.RemoteDes();
	caps.iType = EMediaHardDisk;
	caps.iConnectionBusType = EConnectionBusInternal;
	caps.iDriveAtt = KDriveAttLocal | KDriveAttInternal;
	caps.iMediaAtt = KMediaAttFormattable | KMediaAttDeleteNotify;	// sfat32 then reports freed clusters (TRIM)
	caps.iFileSystemId = KDriveFileSysFAT;
	caps.iPartitionType = m.Drive()->iPartitionType;
	caps.iSize = m.Drive()->iPartitionLen;
	caps.iHiddenSectors = (TUint32)(A64Blk_First() + m.Drive()->iPartitionBaseAddr / KA64BlkSector);	// on the device
	caps.iEraseBlockSize = A64BlkDev_EraseBlock();	// sfat32's format aligns the first data sector to it
	caps.iBlockSize = KA64BlkSector;
	caps.iMaxBytesPerFormat = 1 << 30;		// EFormat is a no-op: without this a full format takes 16 KB steps (8 GB =
											// 512K file-server round trips, which looks like a hang)
	SetTotalSizeInBytes(caps);
	return KErrNone;
	}

// ERead / EWrite over [pos, pos + len), in bounce-buffer pieces
TInt DMediaDriverBlk::Transfer(TLocDrvRequest& m, TInt aKind)
	{
	Int64 pos = m.Pos(), len = m.Length();
	if (pos < 0 || len < 0 || (TUint64)(pos + len) > Bytes())
		return KErrArgument;
	TInt done = 0;
	while (done < len)
		{
		TUint64 at = pos + done;
		TUint64 lba = at / KA64BlkSector;
		TInt skip = (TInt)(at % KA64BlkSector);
		Int64 left = len - done;
		TInt n = left < KBounce - skip ? (TInt)left : KBounce - skip;	// bytes this piece
		TInt sectors = (skip + n + KA64BlkSector - 1) / KA64BlkSector;
		TInt r = KErrNone;
		if (aKind == DLocalDrive::ERead)
			{
			r = A64Blk_Read(lba, sectors, iBuf);
			TPtrC8 des(iBuf + skip, n);
			if (r == KErrNone)
				r = m.WriteRemote(&des, done);
			}
		else
			{
			TBool partial = skip || (n % KA64BlkSector);
			if (partial)									// read-modify-write the edge sectors (simply: all)
				r = A64Blk_Read(lba, sectors, iBuf);
			if (r == KErrNone)
				{
				TPtr8 des(iBuf + skip, n, n);
				r = m.ReadRemote(&des, done);
				}
			if (r == KErrNone)
				r = A64Blk_Write(lba, sectors, iBuf);
			}
		if (r != KErrNone)
			return r;
		done += n;
		}
	return KErrNone;
	}

TInt DMediaDriverBlk::Request(TLocDrvRequest& m)
	{
	TInt id = m.Id(), r;
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	switch (id)
		{
	case DLocalDrive::ECaps:
		r = Caps(m);
		break;
	case DLocalDrive::ERead:
	case DLocalDrive::EWrite:
		r = Transfer(m, id);
		break;
	case DLocalDrive::EDeleteNotify:	// whole sectors inside the range; advisory, so never an error (see a64_blk.h)
		{
		const TUint64 a = (m.Pos() + KA64BlkSector - 1) / KA64BlkSector, b = (m.Pos() + m.Length()) / KA64BlkSector;
		if (m.Pos() >= 0 && m.Length() > 0 && b > a && b * KA64BlkSector <= Bytes())
			for (TUint64 lba = a; lba < b; lba += 0x40000)		// 128 MB at a time
				A64Blk_Discard(lba, (TInt)Min(b - lba, (TUint64)0x40000));
		r = KErrNone;
		break;
		}
	case DLocalDrive::EFormat:		// FAT writes its own boot sector, FATs and root afterwards: nothing to erase
		r = KErrNone;				// shortcut: no erase (zeroing 8 GB at 2 MB/s takes an hour); eMMC ERASE/TRIM later
		break;
	default:
		r = KErrNotSupported;
		break;
		}
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	return r;
	}

DECLARE_EXTENSION_PDD()
	{
	return new DPhysicalDeviceMediaBlk;
	}

static const TInt DriveNumbers[KMaxParts] = { 0, 1 };	// local drives 0 and 1: estart.txt mounts them as C: and E:

DECLARE_STANDARD_EXTENSION()
	{
	DPrimaryMediaBase* pm = new DPrimaryMediaBase;
	if (!pm)
		return KErrNoMemory;
	return LocDrv::RegisterMediaDevice(KMediaDevice, KMaxParts, &DriveNumbers[0], pm, 1, KDriveName);
	}
