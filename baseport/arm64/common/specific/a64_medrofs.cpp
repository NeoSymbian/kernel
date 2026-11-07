// SPDX-License-Identifier: Apache-2.0
//
// a64_medrofs.cpp -- local drive 2: the ROFS part of Z: (docs/filesystem.md FS4), read-only. srofs reads its image
// only through a local drive (it never maps it), and estart.txt's FS_COMPOSITE line adds it to Z: with the core ROM.
// What the kernel image carries after the user-side section (ekern.lds .urofs; tools/build-ekern.sh ROFS=) decides
// where the image is read from:
//   - the whole image (ROFS=1): from RAM. Built with -DA64_ROFS_INSTALL (ROFS_INSTALL=1) it is also copied to the
//     GPT partition system_a, chunk by chunk where it differs, each written chunk read back and compared (FS4b);
//   - its first 4 KB only (ROFS=flash): from system_a, and only if system_a starts with exactly that header -- the
//     ROFS this core was built with, never another build's.
// Shaped like a64_medblk.cpp: a fixed-media driver for ELOCD, synchronous in the caller's thread.
#include "locmedia.h"
#include "platform.h"
#include "a64_blk.h"

extern "C" TUint8 __urofs_base[], __urofs_end[];		// ekern.lds

_LIT(KPddName, "Media.Rofs");
_LIT(KDriveName, "Rofs");
const TMediaDevice KMediaDevice = EFixedMedia2;
const TInt KHeader = 4096;								// ROFS=flash carries this much of the image
const TInt KChunk = 64 * 1024;

static TInt Carried() { return __urofs_end - __urofs_base; }
static TInt ImageSize()									// TRofsHeader::iImageSize, or 0 if no ROFS header
	{
	const TUint8* h = __urofs_base;
	if (Carried() < 48 || h[0] != 'R' || h[1] != 'O' || h[2] != 'F' || h[3] != 'S')
		return 0;
	return h[36] | (h[37] << 8) | (h[38] << 16) | (h[39] << 24);
	}
static TBool InRam() { return ImageSize() && Carried() >= ImageSize(); }
static TUint64 FlashFirst;								// system_a's first sector, when read from flash
static TUint8* Bounce;									// KChunk bytes, for flash reads and the install
static DMutex* Lock;

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
	d->SetTotalSizeInBytes(ImageSize());
	return KErrNone;
	}

TInt DMediaDriverRofs::PartitionInfo(TPartitionInfo& aInfo)
	{
	aInfo.iPartitionCount = 1;
	aInfo.iEntry[0].iPartitionBaseAddr = 0;
	aInfo.iEntry[0].iPartitionLen = aInfo.iMediaSizeInBytes = ImageSize();
	aInfo.iEntry[0].iPartitionType = KPartitionTypeRofs;
	return KErrNone;
	}

// [aPos, aPos + aLen) of the image from system_a, in whole sectors through the bounce buffer
static TInt ReadFlash(TLocDrvRequest& m, Int64 aPos, Int64 aLen)
	{
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	TInt r = KErrNone;
	for (Int64 done = 0; done < aLen && r == KErrNone; )
		{
		const Int64 at = aPos + done;
		const TInt skip = (TInt)(at % KA64BlkSector);
		const TInt n = (TInt)Min(aLen - done, (Int64)(KChunk - skip));
		r = A64BlkDev_Read(FlashFirst + at / KA64BlkSector, (skip + n + KA64BlkSector - 1) / KA64BlkSector, Bounce);
		if (r == KErrNone)
			{
			TPtrC8 des(Bounce + skip, n);
			r = m.WriteRemote(&des, (TInt)done);
			}
		done += n;
		}
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	return r;
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
		caps.iSize = ImageSize();
		caps.iBlockSize = KA64BlkSector;
		SetTotalSizeInBytes(caps);
		return KErrNone;
		}
	case DLocalDrive::ERead:
		{
		const Int64 pos = m.Pos(), len = m.Length();
		if (pos < 0 || len < 0 || pos + len > ImageSize())
			return KErrArgument;
		if (!InRam())
			return ReadFlash(m, pos, len);
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

#ifdef A64_ROFS_INSTALL
// the image in RAM -> system_a: only the 64 KB chunks that differ are written, and each is read back and compared
static TInt Install(TUint64 aFirst, TUint64 aCount, TInt& aWritten)
	{
	const TInt size = ImageSize();
	if ((TUint64)(size + KA64BlkSector - 1) / KA64BlkSector > aCount)
		return KErrTooBig;
	aWritten = 0;
	for (TInt off = 0; off < size; off += KChunk)
		{
		const TInt n = Min(KChunk, size - off), sectors = (n + KA64BlkSector - 1) / KA64BlkSector;
		const TUint64 lba = aFirst + off / KA64BlkSector;
		TInt r = A64BlkDev_Read(lba, sectors, Bounce);
		if (r != KErrNone)
			return r;
		if (memcompare(Bounce, n, __urofs_base + off, n) == 0)
			continue;
		memclr(Bounce, sectors * KA64BlkSector);			// the last chunk's tail: zero past the image
		memcpy(Bounce, __urofs_base + off, n);
		r = A64BlkDev_Write(lba, sectors, Bounce);
		if (r == KErrNone)
			r = A64BlkDev_Read(lba, sectors, Bounce);
		if (r == KErrNone && memcompare(Bounce, n, __urofs_base + off, n) != 0)
			r = KErrCorrupt;
		if (r != KErrNone)
			return r;
		aWritten += n;
		}
	return KErrNone;
	}
#endif

DECLARE_EXTENSION_PDD()
	{
	return new DPhysicalDeviceMediaRofs;
	}

static const TInt DriveNumbers[1] = { 2 };				// local drive 2: estart.txt's FS_COMPOSITE line

DECLARE_STANDARD_EXTENSION()
	{
	if (!ImageSize())
		return KErrNone;									// no ROFS in this image: Z: is the core ROM alone
	_LIT(KLockName, "MedRofs");
	NKern::ThreadEnterCS();
	TInt r = Kern::MutexCreate(Lock, KLockName, KMutexOrdGeneral0);
	Bounce = r == KErrNone ? (TUint8*)Kern::Alloc(KChunk) : NULL;
	NKern::ThreadLeaveCS();
	if (!Bounce)
		return KErrNoMemory;
	TUint64 first = 0, count = 0;
	TInt part = A64BlkDev_Partition("system_a", first, count);
	if (InRam())
		{
#ifdef A64_ROFS_INSTALL
		TInt written = 0;
		r = part == KErrNone ? Install(first, count, written) : part;
		Kern::Printf("[media] ROFS -> system_a: %d KB written, verified -> %d", written >> 10, r);
#endif
		}
	else
		{
		// header only: system_a must hold this build's image
		r = part == KErrNone ? A64BlkDev_Read(first, KHeader / KA64BlkSector, Bounce) : part;
		if (r == KErrNone && memcompare(Bounce, KHeader, __urofs_base, KHeader) != 0)
			r = KErrCorrupt;
		Kern::Printf("[media] ROFS from system_a (%d KB): %s -> %d", ImageSize() >> 10,
			r == KErrNone ? "this build's" : r == KErrCorrupt ? "another build's, not used" : "not readable", r);
		if (r != KErrNone)
			return KErrNone;								// Z: is the core ROM alone; estart's ROFS line finds no drive
		FlashFirst = first;
		}
	DPrimaryMediaBase* pm = new DPrimaryMediaBase;
	if (!pm)
		return KErrNoMemory;
	return LocDrv::RegisterMediaDevice(KMediaDevice, 1, &DriveNumbers[0], pm, 1, KDriveName);
	}
