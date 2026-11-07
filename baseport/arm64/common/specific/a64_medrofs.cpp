// SPDX-License-Identifier: Apache-2.0
//
// a64_medrofs.cpp -- local drive 2: the ROFS image carried in the kernel image after the user-side section
// (ekern.lds .urofs; tools/build-userrom.sh ROFS=1), read-only. srofs reads its image only through a local drive (it
// never maps it), and estart.txt's FS_COMPOSITE line adds it to Z: with the core ROM (docs/filesystem.md FS4a).
// Shaped like a64_medblk.cpp: a fixed-media driver for ELOCD, synchronous in the caller's thread.
// shortcut: the image sits in RAM (part of boot.img); FS4b reads it from the system_<slot> partition instead.
#include "locmedia.h"
#include "platform.h"

extern "C" TUint8 __urofs_base[], __urofs_end[];		// ekern.lds

_LIT(KPddName, "Media.Rofs");
_LIT(KDriveName, "Rofs");
const TMediaDevice KMediaDevice = EFixedMedia2;
static TInt Size() { return __urofs_end - __urofs_base; }

class DPhysicalDeviceMediaRofs : public DPhysicalDevice
	{
public:
	DPhysicalDeviceMediaRofs()
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

class DMediaDriverRofs : public DMediaDriver
	{
public:
	DMediaDriverRofs(TInt aMediaId) : DMediaDriver(aMediaId) {}
	virtual TInt Request(TLocDrvRequest& aRequest);
	virtual void Disconnect(DLocalDrive*, TThreadMessage*) {}
	virtual TInt PartitionInfo(TPartitionInfo& aInfo);
	virtual void NotifyPowerDown() {}
	virtual void NotifyEmergencyPowerDown() {}
	};

TInt DPhysicalDeviceMediaRofs::Create(DBase*& aChannel, TInt aMediaId, const TDesC8*, const TVersion& aVer)
	{
	if (!Kern::QueryVersionSupported(iVersion, aVer))
		return KErrNotSupported;
	DMediaDriverRofs* d = new DMediaDriverRofs(aMediaId);
	aChannel = d;
	if (!d)
		return KErrNoMemory;
	d->SetTotalSizeInBytes(Size());
	return KErrNone;
	}

TInt DMediaDriverRofs::PartitionInfo(TPartitionInfo& aInfo)
	{
	aInfo.iPartitionCount = 1;
	aInfo.iEntry[0].iPartitionBaseAddr = 0;
	aInfo.iEntry[0].iPartitionLen = aInfo.iMediaSizeInBytes = Size();
	aInfo.iEntry[0].iPartitionType = KPartitionTypeRofs;
	return KErrNone;
	}

TInt DMediaDriverRofs::Request(TLocDrvRequest& m)
	{
	switch (m.Id())
		{
	case DLocalDrive::ECaps:
		{
		TLocalDriveCapsV6& caps = *(TLocalDriveCapsV6*)m.RemoteDes();
		caps.iType = EMediaRom;
		caps.iConnectionBusType = EConnectionBusInternal;
		caps.iDriveAtt = KDriveAttLocal | KDriveAttInternal | KDriveAttRom;
		caps.iMediaAtt = KMediaAttWriteProtected;
		caps.iFileSystemId = KDriveFileSysROFS;
		caps.iPartitionType = KPartitionTypeRofs;
		caps.iSize = Size();
		caps.iBlockSize = 512;
		SetTotalSizeInBytes(caps);
		return KErrNone;
		}
	case DLocalDrive::ERead:
		{
		const Int64 pos = m.Pos(), len = m.Length();
		if (pos < 0 || len < 0 || pos + len > Size())
			return KErrArgument;
		TPtrC8 des(__urofs_base + (TInt)pos, (TInt)len);
		return m.WriteRemote(&des, 0);
		}
	case DLocalDrive::EWrite:
	case DLocalDrive::EFormat:
		return KErrAccessDenied;
	default:
		return KErrNotSupported;
		}
	}

DECLARE_EXTENSION_PDD()
	{
	return new DPhysicalDeviceMediaRofs;
	}

static const TInt DriveNumbers[1] = { 2 };				// local drive 2: estart.txt's FS_COMPOSITE line

DECLARE_STANDARD_EXTENSION()
	{
	if (!Size())
		return KErrNone;									// no ROFS in this image: Z: is the core ROM alone
	DPrimaryMediaBase* pm = new DPrimaryMediaBase;
	if (!pm)
		return KErrNoMemory;
	return LocDrv::RegisterMediaDevice(KMediaDevice, 1, &DriveNumbers[0], pm, 1, KDriveName);
	}
