// SPDX-License-Identifier: Apache-2.0
//
// virtio_blk.cpp -- QEMU virt: the A64BlkDev backend (a64_blk.h) on a virtio-blk disk (run-qemu.sh DISK=), through
// the legacy virtio-mmio v1 transport QEMU gives by default: 32 transports at 0x0a000000, 0x200 apart. One
// 8-entry virtqueue and a bounce buffer in physically contiguous RAM mapped non-cached (no cache maintenance on the
// device's view), polled: a request is header + data + status descriptors, then a wait for the used ring.
// shortcut: one request in flight, polled; upgrade to the transport's interrupt if a file workload needs it.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_blk.h"
#include "a64_device.h"

const TUint32 KMmioBase = 0x0a000000, KMmioStride = 0x200, KMmioSlots = 32;
enum
	{
	EMagic = 0x000, EVersion = 0x004, EDeviceId = 0x008, EHostFeatures = 0x010, EGuestFeatures = 0x020,
	EGuestPageSize = 0x028, EQueueSel = 0x030, EQueueNumMax = 0x034, EQueueNum = 0x038, EQueueAlign = 0x03c,
	EQueuePfn = 0x040, EQueueNotify = 0x050, EIntStatus = 0x060, EIntAck = 0x064, EStatus = 0x070, EConfig = 0x100,
	};
enum { EStAck = 1, EStDriver = 2, EStDriverOk = 4, EStFailed = 128 };
enum { EDescNext = 1, EDescWrite = 2 };
enum { EReqIn = 0, EReqOut = 1 };
const TInt KQ = 8;
const TInt KMaxSectors = 128;							// 64 KB per request

struct SDesc { TUint64 iAddr; TUint32 iLen; TUint16 iFlags, iNext; };
struct SAvail { TUint16 iFlags, iIdx, iRing[KQ], iUsedEvent; };
struct SUsedElem { TUint32 iId, iLen; };
struct SUsed { TUint16 iFlags, iIdx; SUsedElem iRing[KQ]; TUint16 iAvailEvent; };
struct SReq { TUint32 iType, iReserved; TUint64 iSector; };

// the DMA area: page 0 = descriptors + avail ring, page 1 = used ring (legacy layout, 4 KB alignment), page 2 =
// request header + status, then the data bounce buffer
const TInt KAreaSize = 3 * 4096 + KMaxSectors * KA64BlkSector;
static TPhysAddr AreaPhys;
static TLinAddr Area;
static volatile TUint32* Dev;
static TUint16 LastUsed;
static TUint64 Capacity;								// sectors, once set up (the device registry)

static volatile TUint32& R(TUint32 aOff) { return Dev[aOff / 4]; }
static SDesc* Descs() { return (SDesc*)Area; }
static volatile SAvail* Avail() { return (volatile SAvail*)(Area + KQ * sizeof(SDesc)); }
static volatile SUsed* Used() { return (volatile SUsed*)(Area + 4096); }
static SReq* Header() { return (SReq*)(Area + 2 * 4096); }
static volatile TUint8* StatusByte() { return (volatile TUint8*)(Area + 2 * 4096 + sizeof(SReq)); }
static TUint8* Data() { return (TUint8*)(Area + 3 * 4096); }

TInt A64BlkDev_Init(TUint64& aSectors)
	{
	for (TUint32 i = 0; i < KMmioSlots && !Dev; ++i)
		{
		volatile TUint32* d = (volatile TUint32*)A64_Io(KMmioBase + i * KMmioStride);
		if (d[EMagic / 4] == 0x74726976 && d[EVersion / 4] == 1 && d[EDeviceId / 4] == 2)
			Dev = d;
		}
	if (!Dev)
		return KErrNotFound;
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(KAreaSize, AreaPhys, 12);
	DPlatChunkHw* chunk = NULL;
	if (r == KErrNone)
		r = DPlatChunkHw::New(chunk, AreaPhys, KAreaSize, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return r;
	Area = chunk->LinearAddress();
	memclr((TAny*)Area, KAreaSize);
	R(EStatus) = 0;										// reset
	R(EStatus) = EStAck;
	R(EStatus) = EStAck | EStDriver;
	R(EGuestFeatures) = 0;								// no optional features: plain read/write
	R(EGuestPageSize) = 4096;
	R(EQueueSel) = 0;
	if (R(EQueueNumMax) < (TUint32)KQ)
		{
		R(EStatus) = EStFailed;
		return KErrNotSupported;
		}
	R(EQueueNum) = KQ;
	R(EQueueAlign) = 4096;
	R(EQueuePfn) = AreaPhys >> 12;
	R(EStatus) = EStAck | EStDriver | EStDriverOk;
	aSectors = Capacity = (TUint64)R(EConfig) | ((TUint64)R(EConfig + 4) << 32);	// capacity in 512-byte sectors
	return KErrNone;
	}

// one request of at most KMaxSectors through the bounce buffer
static TInt Transfer(TUint32 aType, TUint64 aLba, TInt aCount)
	{
	SReq* h = Header();
	h->iType = aType;
	h->iReserved = 0;
	h->iSector = aLba;
	*StatusByte() = 0xff;
	SDesc* d = Descs();
	TPhysAddr p = AreaPhys + 2 * 4096;
	d[0].iAddr = p; d[0].iLen = sizeof(SReq); d[0].iFlags = EDescNext; d[0].iNext = 1;
	d[1].iAddr = AreaPhys + 3 * 4096; d[1].iLen = aCount * KA64BlkSector;
	d[1].iFlags = EDescNext | (aType == EReqIn ? EDescWrite : 0); d[1].iNext = 2;
	d[2].iAddr = p + sizeof(SReq); d[2].iLen = 1; d[2].iFlags = EDescWrite; d[2].iNext = 0;
	volatile SAvail* a = Avail();
	a->iRing[a->iIdx % KQ] = 0;
	__e32_memory_barrier();
	a->iIdx = a->iIdx + 1;
	__e32_memory_barrier();
	R(EQueueNotify) = 0;
	for (TInt i = 0; i < 50000000 && Used()->iIdx == LastUsed; ++i)
		{}
	if (Used()->iIdx == LastUsed)
		return KErrTimedOut;
	LastUsed = Used()->iIdx;
	R(EIntAck) = R(EIntStatus);
	__e32_memory_barrier();
	return *StatusByte() == 0 ? KErrNone : KErrGeneral;
	}

TInt A64BlkDev_Read(TUint64 aLba, TInt aCount, TUint8* aBuf)
	{
	for (TInt done = 0; done < aCount; )
		{
		TInt n = Min(aCount - done, KMaxSectors);
		TInt r = Transfer(EReqIn, aLba + done, n);
		if (r != KErrNone)
			return r;
		memcpy(aBuf + done * KA64BlkSector, Data(), n * KA64BlkSector);
		done += n;
		}
	return KErrNone;
	}

TInt A64BlkDev_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf)
	{
	for (TInt done = 0; done < aCount; )
		{
		TInt n = Min(aCount - done, KMaxSectors);
		memcpy(Data(), aBuf + done * KA64BlkSector, n * KA64BlkSector);
		TInt r = Transfer(EReqOut, aLba + done, n);
		if (r != KErrNone)
			return r;
		done += n;
		}
	return KErrNone;
	}

static TA64DevState VirtioBlkStatus(TDes8& aDetail)
	{
	if (!Dev)
		return EA64DevAbsent;
	if (!Capacity)
		return EA64DevError;
	A64Device_Format(aDetail, "virtio-mmio, %d MB", (TInt)(Capacity >> 11));
	return EA64DevOk;
	}
A64_DEVICE("blk", "Storage (virtio-blk)", "virtio_blk", VirtioBlkStatus);
