// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// a64_medblk.cpp -- the persistent C: drive: a fixed-media driver for ELOCD (shaped like the 2010 internal RAM
// drive, kernel/eka/drivers/medint/iram.cpp) on the block window of a64_blk.h -- QEMU's virtio-blk disk, the
// Pixel's eMMC userdata partition. Byte ranges from ELOCD are turned into whole sectors through a 64 KB bounce
// buffer; partial sectors are read-modify-written.
// shortcut: synchronous in the caller's thread, one request at a time, no request merging or read-ahead.
#include "locmedia.h"
#include "platform.h"
#include "a64_blk.h"

_LIT(KPddName, "Media.Blk");
_LIT(KDriveName, "Blk");
const TMediaDevice KMediaDevice = EFixedMedia1;
const TInt KBounce = 64 * 1024;
const TUint64 KFat32Above = 2ull << 30;				// drives above 2 GB get FAT32

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
	TUint8* iBuf;
	};

static DMutex* Lock;									// one request at a time, through the one bounce buffer
static TUint64 Bytes() { return A64Blk_Sectors() * KA64BlkSector; }
static TBool Fat32() { return Bytes() > KFat32Above; }

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
	return KErrNone;
	}

TInt DMediaDriverBlk::PartitionInfo(TPartitionInfo& aInfo)
	{
	aInfo.iPartitionCount = 1;
	aInfo.iEntry[0].iPartitionBaseAddr = 0;
	aInfo.iEntry[0].iPartitionLen = aInfo.iMediaSizeInBytes = TotalSizeInBytes();
	aInfo.iEntry[0].iPartitionType = Fat32() ? KPartitionTypeWin95FAT32 : KPartitionTypeFAT16;
	return KErrNone;
	}

TInt DMediaDriverBlk::Caps(TLocDrvRequest& m)
	{
	TLocalDriveCapsV6& caps = *(TLocalDriveCapsV6*)m.RemoteDes();
	caps.iType = EMediaHardDisk;
	caps.iConnectionBusType = EConnectionBusInternal;
	caps.iDriveAtt = KDriveAttLocal | KDriveAttInternal;
	caps.iMediaAtt = KMediaAttFormattable;
	caps.iFileSystemId = KDriveFileSysFAT;
	caps.iPartitionType = Fat32() ? KPartitionTypeWin95FAT32 : KPartitionTypeFAT16;
	caps.iSize = m.Drive()->iPartitionLen;
	caps.iHiddenSectors = 0;
	caps.iEraseBlockSize = 0;
	caps.iBlockSize = KA64BlkSector;
	SetTotalSizeInBytes(caps);
	return KErrNone;
	}

// ERead / EWrite / EFormat (zeroes) over [pos, pos + len), in bounce-buffer pieces
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
			if (r == KErrNone && aKind == DLocalDrive::EWrite)
				{
				TPtr8 des(iBuf + skip, n, n);
				r = m.ReadRemote(&des, done);
				}
			else if (r == KErrNone)
				memclr(iBuf + skip, n);
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
	case DLocalDrive::EFormat:
		r = Transfer(m, id);
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

static const TInt DriveNumbers[1] = { 0 };				// local drive 0: estart.txt mounts it as C:

DECLARE_STANDARD_EXTENSION()
	{
	DPrimaryMediaBase* pm = new DPrimaryMediaBase;
	if (!pm)
		return KErrNoMemory;
	return LocDrv::RegisterMediaDevice(KMediaDevice, 1, &DriveNumbers[0], pm, 1, KDriveName);
	}
