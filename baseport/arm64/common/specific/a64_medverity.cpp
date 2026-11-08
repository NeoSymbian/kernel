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
// The hash: the CPU's SHA-256 instructions when it has them (CpuHash4K, checked against the C SHA256 at boot), else
// the kernel's plain-C SHA256 (~80 MB/s on the phone: flash reads of Z: at about half the eMMC's 70 MB/s).
// shortcut: boot_a is the root of trust and ABL doesn't verify it while unlocked; signing waits for a locked bootloader.

#include <drivers/locmedia.h>
#include <platform.h>
#include <sha256.h>

extern "C" TUint8 __urofs_hash[], __urofs_hash_end[], __urofs_base[];	// ekern.lds

_LIT(KPddName, "Media.Verity");
_LIT(KDriveName, "Verity");
const TMediaDevice KMediaDevice = EMediaExtension1;			// EMediaExtension0 is NFE's
const TInt KBlock = 4096, KHash = 32, KBuf = 64 * 1024;
static const TInt DriveNumbers[1] = { 2 };					// the ROFS drive (a64_medrofs.cpp)

static TInt TableBlocks() { return (__urofs_hash_end - __urofs_hash) / KHash; }

// SHA-256 of one 4 KB block with the ARMv8 instructions (a64_sha256.S), when the CPU has them: FP/SIMD is enabled
// (CPACR_EL1.FPEN) only around the call, with interrupts off on this CPU -- no thread here owns SIMD state, so nothing
// needs saving. ~4 us per block on the phone, against ~50 us for the C SHA256.
extern "C" void A64Sha256Blocks(TUint32 aState[8], const TUint8* aData, TInt aBlocks);
static TBool UseCpu;										// set by the boot self-test
static TBool CpuHasSha2()
	{
	TUint64 isar0;
	__asm__ __volatile__("mrs %0, id_aa64isar0_el1" : "=r"(isar0));
	return ((isar0 >> 12) & 0xf) != 0;
	}
static void CpuHash4K(const TUint8* aData, TUint8 aOut[KHash])
	{
	TUint32 st[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
	TUint8 pad[64];
	memclr(pad, sizeof(pad));
	pad[0] = 0x80;
	pad[62] = 0x80;											// the length, 4096 * 8 bits = 0x8000, big-endian
	const TInt irq = NKern::DisableAllInterrupts();
	TUint64 cpacr;
	__asm__ __volatile__("mrs %0, cpacr_el1" : "=r"(cpacr));
	__asm__ __volatile__("msr cpacr_el1, %0\n\tisb" :: "r"(cpacr | (3ull << 20)) : "memory");
	A64Sha256Blocks(st, aData, KBlock / 64);
	A64Sha256Blocks(st, pad, 1);
	__asm__ __volatile__("msr cpacr_el1, %0\n\tisb" :: "r"(cpacr) : "memory");
	NKern::RestoreInterrupts(irq);
	for (TInt i = 0; i < 8; ++i)
		{
		aOut[4 * i] = (TUint8)(st[i] >> 24);
		aOut[4 * i + 1] = (TUint8)(st[i] >> 16);
		aOut[4 * i + 2] = (TUint8)(st[i] >> 8);
		aOut[4 * i + 3] = (TUint8)st[i];
		}
	}

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
	TUint8 cpu[KHash];
	const TUint8* h;
	if (UseCpu)
		{
		CpuHash4K(aData, cpu);
		h = cpu;
		}
	else
		{
		iSha.Update(aData, KBlock);
		h = iSha.Final().Ptr();							// Final() also resets it for the next block
		}
	if (memcompare(h, KHash, __urofs_hash + aBlock * KHash, KHash) == 0)
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
	if (CpuHasSha2())										// the CPU's SHA-256 against the C one, on this image's data
		{
		const TUint8* sample = __urofs_base;					// the ROFS header: always >= 4 KB carried
		TUint8 cpu[KHash];
		CpuHash4K(sample, cpu);
		SHA256* c = new SHA256;
		if (c)
			{
			c->Update(sample, KBlock);
			UseCpu = memcompare(cpu, KHash, c->Final().Ptr(), KHash) == 0;
			delete c;
			}
		const TUint32 t0 = NKern::TickCount();
		for (TInt i = 0; UseCpu && i < 256; ++i)				// 1 MB
			CpuHash4K(sample, cpu);
		Kern::Printf("[verity] SHA-256 instructions: self-test %s, 1 MB in %d ms", UseCpu ? "match" : "MISMATCH (C kept)",
			(NKern::TickCount() - t0) * NKern::TickPeriod() / 1000);
		}
	else
		Kern::Printf("[verity] no SHA-256 instructions: the C SHA256");
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
