// Copyright (c) 1995-2009 Nokia Corporation and/or its subsidiary(-ies).
// All rights reserved.
// This component and the accompanying materials are made available
// under the terms of the License "Eclipse Public License v1.0"
// which accompanies this distribution, and is available
// at the URL "http://www.eclipse.org/legal/epl-v10.html".
// SPDX-License-Identifier: EPL-2.0 (derived from e32test/mediaext/d_nfe.cpp by the NeoSymbian Authors)
//
// Initial Contributors:
// Nokia Corporation - initial contribution.
//
// Contributors:
// The NeoSymbian Authors (2026): this file, derived from d_nfe.cpp's media driver extension.
//
// Description:
// a64_medverity.cpp -- docs/filesystem.md FS5a: a media driver extension over local drive 2 (the ROFS part of Z:,
// a64_medrofs.cpp) that checks every read against a SHA-256 per 4 KB block. The table rides in boot.img
// (ekern.lds .urofshash, built by build-userrom.sh), so a changed system_a block fails its read with KErrCorrupt
// instead of reaching the loader. The pattern is NFE's: own primary media and DFC queue, Read() from the attached
// drive into a buffer, WriteRemote() to the client, KErrCompletion.
// shortcut: the plain-C SHA256 runs ~80 MB/s on the phone (4 MB in 50 ms), so ROFS reads from flash run at about half
// the eMMC's 70 MB/s (at most ~0.1 s for all of Z:); the ARMv8 SHA-256 instructions need EL1 SIMD enabled first.
// shortcut: boot_a is the root of trust and ABL doesn't verify it while unlocked; signing waits for a locked bootloader.

#include <drivers/locmedia.h>
#include <platform.h>
#include <sha256.h>

extern "C" TUint8 __urofs_hash[], __urofs_hash_end[];		// ekern.lds

_LIT(KPddName, "Media.Verity");
_LIT(KDriveName, "Verity");
const TMediaDevice KMediaDevice = EMediaExtension1;			// EMediaExtension0 is NFE's
const TInt KBlock = 4096, KHash = 32, KBuf = 64 * 1024;
static const TInt DriveNumbers[1] = { 2 };					// the ROFS drive (a64_medrofs.cpp)

static TInt TableBlocks() { return (__urofs_hash_end - __urofs_hash) / KHash; }

class DPhysicalDeviceMediaVerity : public DPhysicalDevice
	{
public:
	DPhysicalDeviceMediaVerity()
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
	virtual TInt Info(TInt aFunction, TAny*)
		{
		if (aFunction == EPriority)
			return KMediaDriverPriorityNormal;
		return aFunction == EMediaDriverPersistent ? KErrNone : KErrNotSupported;
		}
	};

class DMediaDriverVerity : public DMediaDriverExtension
	{
public:
	DMediaDriverVerity(TInt aMediaId) : DMediaDriverExtension(aMediaId) {}
	virtual TInt Request(TLocDrvRequest& aReq);
	virtual TInt PartitionInfo(TPartitionInfo& aInfo);
	TInt DoCreate();
private:
	TInt HandleRead(TLocDrvRequest& aReq);
	TBool Check(TInt aBlock, const TUint8* aData);
	TUint8* iBuf;										// KBuf bytes: each pass reads whole blocks into it
	Int64 iSize;										// the ROFS image, from the attached drive
	SHA256 iSha;
	TInt iBad;											// blocks that failed, for the log
	};

TInt DPhysicalDeviceMediaVerity::Create(DBase*& aChannel, TInt aMediaId, const TDesC8*, const TVersion& aVer)
	{
	if (!Kern::QueryVersionSupported(iVersion, aVer))
		return KErrNotSupported;
	DMediaDriverVerity* d = new DMediaDriverVerity(aMediaId);
	aChannel = d;
	TInt r = d ? d->DoCreate() : KErrNoMemory;
	if (r == KErrNone)
		d->OpenMediaDriverComplete(KErrNone);
	return r;
	}

TInt DMediaDriverVerity::DoCreate()
	{
	iBuf = (TUint8*)Kern::Alloc(KBuf);
	return iBuf ? KErrNone : KErrNoMemory;
	}

TInt DMediaDriverVerity::PartitionInfo(TPartitionInfo& aInfo)
	{
	TInt r = DoDrivePartitionInfo(aInfo);
	if (r != KErrNone)
		return r;
	iSize = aInfo.iEntry[0].iPartitionLen;
	const TInt need = (TInt)((iSize + KBlock - 1) / KBlock);
	if (TableBlocks() < need)
		r = KErrCorrupt;								// a table for another image: refuse the drive
	Kern::Printf("[verity] ROFS %d blocks, table %d -> %d", need, TableBlocks(), r);
	SetTotalSizeInBytes(iSize);
	return r == KErrNone ? KErrCompletion : r;
	}

TBool DMediaDriverVerity::Check(TInt aBlock, const TUint8* aData)
	{
	iSha.Update(aData, KBlock);
	const TDesC8& h = iSha.Final();						// Final() also resets it for the next block
	if (memcompare(h.Ptr(), KHash, __urofs_hash + aBlock * KHash, KHash) == 0)
		return ETrue;
	if (iBad++ < 10)
		Kern::Printf("[verity] block %d bad (ROFS offset %08x)", aBlock, aBlock * KBlock);
	return EFalse;
	}

// [pos, pos + len) in whole blocks through iBuf: each block hashed before any byte of it reaches the client
TInt DMediaDriverVerity::HandleRead(TLocDrvRequest& aReq)
	{
	Int64 pos = aReq.Pos(), len = aReq.Length();
	if (pos < 0 || len < 0 || pos + len > iSize)
		return KErrArgument;
	const TInt drive = aReq.Drive()->iDriveNumber;
	for (TInt desPos = 0; len > 0; )
		{
		const Int64 start = pos - pos % KBlock;
		const TInt skip = (TInt)(pos - start), n = (TInt)Min(len, (Int64)(KBuf - skip));
		const TInt span = (skip + n + KBlock - 1) & ~(KBlock - 1);
		const TInt got = (TInt)Min((Int64)span, iSize - start);	// the image's last block is short on the media
		TInt r = Read(drive, start, (TLinAddr)iBuf, got);
		if (r != KErrNone)
			return r;
		memclr(iBuf + got, span - got);					// build-userrom.sh hashed it zero-padded
		for (TInt b = 0; b < span / KBlock; ++b)
			if (!Check((TInt)(start / KBlock) + b, iBuf + b * KBlock))
				return KErrCorrupt;
		TPtrC8 des(iBuf + skip, n);
		r = aReq.WriteRemote(&des, desPos);
		if (r != KErrNone)
			return r;
		pos += n;
		len -= n;
		desPos += n;
		}
	return KErrCompletion;
	}

TInt DMediaDriverVerity::Request(TLocDrvRequest& aReq)
	{
	if (aReq.Id() == DLocalDrive::ERead)
		return HandleRead(aReq);
	return ForwardRequest(aReq);						// caps, and writes/formats that the ROFS driver refuses
	}

DECLARE_EXTENSION_PDD()
	{
	return new DPhysicalDeviceMediaVerity;
	}

// after medrofs (a64_media.cpp): the drive it wraps must be registered first
static TDfcQue DfcQ;									// the media's thread ("VerityQ")

DECLARE_STANDARD_EXTENSION()
	{
	if (!TableBlocks())
		return KErrNone;									// no ROFS in this image
	DPrimaryMediaBase* pm = new DPrimaryMediaBase;
	if (!pm)
		return KErrNoMemory;
	_LIT(KThreadName, "VerityQ");
	TInt r = Kern::DfcQInit(&DfcQ, 24, &KThreadName);
	if (r != KErrNone)
		return r;
	pm->iDfcQ = &DfcQ;
	pm->iMsgQ.Receive();
	r = LocDrv::RegisterMediaDevice(KMediaDevice, 1, &DriveNumbers[0], pm, 1, KDriveName);
	if (r == KErrNotReady)
		{
		Kern::Printf("[verity] no ROFS drive to check");	// medrofs found no usable ROFS: Z: is the core alone
		return KErrNone;
		}
	return r;
	}
