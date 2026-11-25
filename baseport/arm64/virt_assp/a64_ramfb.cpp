// SPDX-License-Identifier: Apache-2.0
//
// a64_ramfb.cpp -- QEMU virt: show the display driver's RAM framebuffer on QEMU's ramfb device (run-qemu.sh
// -device ramfb), so QMP screendump and VNC see the screen. ramfb scans out guest RAM: the guest writes one config
// file, "etc/ramfb" (address, DRM fourcc, size, stride; big-endian), through fw_cfg's DMA interface (QEMU
// docs/specs/fw_cfg.rst and hw/display/ramfb.c). The framebuffer's BGRX bytes are XRGB8888 (fourcc "XR24").
#include <kernel/kern_priv.h>
#include "board.h"

const TUint32 KFwCfgPhys = 0x09020000;					// virt's fw_cfg (hw/arm/virt.c VIRT_FW_CFG)
enum { EFwData = 0x00, EFwSelector = 0x08, EFwDma = 0x10 };
const TUint16 KFwCfgId = 0x0001, KFwCfgFileDir = 0x0019;
const TUint32 KFwIdDma = 1u << 1;
enum { EDmaError = 0x01, EDmaSelect = 0x08, EDmaWrite = 0x10 };
const TUint32 KFourccXrgb8888 = 0x34325258;			// 'X' 'R' '2' '4'

struct SDmaAccess { TUint32 iControl, iLength; TUint64 iAddress; };				// big-endian
struct SRamfbCfg { TUint64 iAddr; TUint32 iFourcc, iFlags, iWidth, iHeight, iStride; } __attribute__((packed));

static TLinAddr FwCfg() { return A64_Io(KFwCfgPhys); }

static void Select(TUint16 aKey)
	{
	*(volatile TUint16*)(FwCfg() + EFwSelector) = __builtin_bswap16(aKey);
	}

static void Read(TAny* aBuf, TInt aLen)
	{
	for (TInt i = 0; i < aLen; ++i)
		((TUint8*)aBuf)[i] = *(volatile TUint8*)(FwCfg() + EFwData);
	}

static TUint32 ReadBe32()
	{
	TUint32 v;
	Read(&v, 4);
	return __builtin_bswap32(v);
	}

// the fw_cfg item named aName, or 0
static TUint16 FindFile(const char* aName)
	{
	Select(KFwCfgFileDir);
	TUint32 n = ReadBe32();
	for (TUint32 i = 0; i < n; ++i)
		{
		struct { TUint32 iSize; TUint16 iSelect, iReserved; char iName[56]; } f;
		Read(&f, sizeof(f));
		TInt k = 0;
		while (aName[k] && f.iName[k] == aName[k])
			++k;
		if (!aName[k] && !f.iName[k])
			return __builtin_bswap16(f.iSelect);
		}
	return 0;
	}

static const char* Scanout(TPhysAddr aFb, TInt aWidth, TInt aHeight, TInt aStride)
	{
	Select(KFwCfgId);
	TUint32 id;
	Read(&id, 4);										// little-endian, unlike the file directory
	if (!(id & KFwIdDma))
		return "no fw_cfg DMA";
	TUint16 key = FindFile("etc/ramfb");
	if (!key)
		return "no etc/ramfb (run-qemu.sh adds -device ramfb)";
	// the access descriptor and the config, in one non-cached page the device reads by physical address
	TPhysAddr phys;
	DPlatChunkHw* chunk = NULL;
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(4096, phys);
	if (r == KErrNone)
		r = DPlatChunkHw::New(chunk, phys, 4096, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return "no DMA page";
	volatile SDmaAccess* dma = (volatile SDmaAccess*)chunk->LinearAddress();
	SRamfbCfg* cfg = (SRamfbCfg*)(chunk->LinearAddress() + 64);
	cfg->iAddr = __builtin_bswap64((TUint64)aFb);
	cfg->iFourcc = __builtin_bswap32(KFourccXrgb8888);
	cfg->iFlags = 0;
	cfg->iWidth = __builtin_bswap32((TUint32)aWidth);
	cfg->iHeight = __builtin_bswap32((TUint32)aHeight);
	cfg->iStride = __builtin_bswap32((TUint32)aStride);
	dma->iControl = __builtin_bswap32(((TUint32)key << 16) | EDmaSelect | EDmaWrite);
	dma->iLength = __builtin_bswap32(sizeof(SRamfbCfg));
	dma->iAddress = __builtin_bswap64((TUint64)(phys + 64));
	__e32_memory_barrier();
	*(volatile TUint64*)(FwCfg() + EFwDma) = __builtin_bswap64((TUint64)phys);	// starts the transfer
	for (TInt i = 0; i < 1000000 && (__builtin_bswap32(dma->iControl) & ~EDmaError); ++i)
		{}
	TUint32 control = __builtin_bswap32(dma->iControl);
	// the page stays: QEMU keeps no pointer to it, but a64_display's framebuffer is used for the system's life anyway
	return control ? "fw_cfg DMA failed" : NULL;
	}

void A64Display_Scanout(TPhysAddr aFb, TInt aWidth, TInt aHeight, TInt aStride)
	{
	const char* err = Scanout(aFb, aWidth, aHeight, aStride);
	if (err)
		Kern::Printf("[ramfb] %s", err);
	else
		Kern::Printf("[ramfb] %dx%d XR24 at %08x: QMP screendump and VNC show it", aWidth, aHeight, aFb);
	}
