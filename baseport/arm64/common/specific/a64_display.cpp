// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: EPL-2.0
//
// a64_display.cpp -- the display HAL (EHalGroupDisplay) of the AArch64 full kernel, for the 2010 non-NGA window
// server stack (wserv, scdv, bitgdi, econswserv): they find the screen through hal.dll's display attributes and draw
// straight into it. The contract is the Syborg board's (baseport/syborg/svpframebuffer/svpframebuffer.cpp
// DLcdPowerHandler::HalFunction): the framebuffer mapped user-RW with DPlatChunkHw at one global address; and the
// shape of Phorepoc's x86-64 xk_display.cpp mode 1.
//   - one mode (0): 32 bpp a8r8g8b8, which scdv runs as EColor16MA; no palette; pixel order RGB, landscape;
//   - the flipped (90/270 degree) surface is not available (KErrArgument, as Syborg): normal and 180 only.
// The board says where the framebuffer is (board.h A64_Fb*): the Pixel's ABL splash framebuffer (the user side gets
// its top rows, the kernel console keeps the rest), or on QEMU virt, which has no display, a RAM framebuffer.
#include <kernel/kern_priv.h>
#include <videodriver.h>
#include <mmboot.h>
#include "board.h"
#include "a64_device.h"

static DPlatChunkHw* TheFbChunk;
static TPhysAddr TheFbPhys;
static TInt TheDisplayState = 1;					// on

static TInt ModeInfo(TInt aMode, TVideoInfoV01& a)
	{
	if (aMode != 0 || !TheFbChunk)					// also mode | EHalFlippedFlag: no rotated surface
		return KErrArgument;
	memclr(&a, sizeof(a));
	a.iSizeInPixels.iWidth = A64_FbWidth;
	a.iSizeInPixels.iHeight = A64_FbHeight;
	// a phone panel's density, ~400 dpi: 1440 twips per inch / 400 = 3.6 twips per pixel (scdv asserts non-zero)
	a.iSizeInTwips.iWidth = A64_FbWidth * 18 / 5;
	a.iSizeInTwips.iHeight = A64_FbHeight * 18 / 5;
	a.iIsMono = EFalse;
	a.iIsPalettized = EFalse;
	a.iBitsPerPixel = 32;
	a.iVideoAddress = TheFbChunk->LinearAddress();
	a.iOffsetToFirstPixel = 0;
	a.iOffsetBetweenLines = A64_FbStride;
	a.iIsPixelOrderRGB = ETrue;
	a.iIsPixelOrderLandscape = ETrue;
	a.iDisplayMode = 0;
	return KErrNone;
	}

static TInt DisplayHal(TAny*, TInt aFunction, TAny* a1, TAny* a2)
	{
	switch (aFunction)
		{
	case EDisplayHalScreenInfo:
		{
		TPckgBuf<TScreenInfoV01> v;
		v().iWindowHandleValid = EFalse;
		v().iWindowHandle = NULL;
		v().iScreenAddressValid = TheFbChunk != NULL;
		v().iScreenAddress = TheFbChunk ? (TAny*)TheFbChunk->LinearAddress() : NULL;
		v().iScreenSize.iWidth = A64_FbWidth;
		v().iScreenSize.iHeight = A64_FbHeight;
		Kern::InfoCopy(*(TDes8*)a1, v);
		return KErrNone;
		}
	case EDisplayHalMode:
		{
		TInt m = 0;
		kumemput32(a1, &m, sizeof(m));
		return KErrNone;
		}
	case EDisplayHalModeCount:
		{
		TInt n = 1;
		kumemput32(a1, &n, sizeof(n));
		return KErrNone;
		}
	case EDisplayHalSetMode:
		return (TInt)a1 == 0 ? KErrNone : KErrArgument;
	case EDisplayHalColors:
		{
		TInt n = 1 << 24;
		kumemput32(a1, &n, sizeof(n));
		return KErrNone;
		}
	case EDisplayHalState:
		kumemput32(a1, &TheDisplayState, sizeof(TheDisplayState));
		return KErrNone;
	case EDisplayHalSetState:
		TheDisplayState = a1 ? 1 : 0;
		return KErrNone;
	case EDisplayHalCurrentModeInfo:
	case EDisplayHalSpecifiedModeInfo:
		{
		TInt mode = 0;
		if (aFunction == EDisplayHalSpecifiedModeInfo)
			{
			kumemget32(&mode, a1, sizeof(mode));
			a1 = a2;
			}
		TPckgBuf<TVideoInfoV01> v;
		TInt r = ModeInfo(mode, v());
		if (r == KErrNone)
			Kern::InfoCopy(*(TDes8*)a1, v);
		return r;
		}
	case EDisplayHalWsRegisterSwitchOnScreenHandling:
	case EDisplayHalWsSwitchOnScreen:
		return KErrNone;
	default:
		return KErrNotSupported;					// palette, backlight, contrast, brightness, secure screen
		}
	}

TLinAddr A64_FbLinear()								// a64_dbgmon.cpp "shot"
	{
	return TheFbChunk ? TheFbChunk->LinearAddress() : 0;
	}

void A64StartDisplay()
	{
	TInt size = A64_FbStride * A64_FbHeight;
	size = (size + KPageMask) & ~KPageMask;
	TInt r = KErrNone;
	TheFbPhys = A64_FbPhys;
	if (!TheFbPhys)
		r = Epoc::AllocPhysicalRam(size, TheFbPhys);
	if (r == KErrNone)
		r = DPlatChunkHw::New(TheFbChunk, TheFbPhys, size, EMapAttrUserRw | EMapAttrBufferedC);
	if (r != KErrNone)
		{
		Kern::Printf("[display] no framebuffer: %d", r);
		return;
		}
	if (!A64_FbPhys)
		memclr((TAny*)TheFbChunk->LinearAddress(), size);
	r = Kern::AddHalEntry(EHalGroupDisplay, DisplayHal, NULL);
	Kern::Printf("[display] %dx%d 32 bpp, framebuffer phys %08x size %x, user linear %08x, HAL -> %d",
		A64_FbWidth, A64_FbHeight, TheFbPhys, size, TheFbChunk->LinearAddress(), r);
	A64_ConsoleReserveTop(A64_FbHeight);
	}

static TA64DevState DisplayStatus(TDes8& aDetail)
	{
	if (!TheFbChunk)
		return EA64DevError;
	A64Device_Format(aDetail, "%dx%d 32 bpp, %s framebuffer", A64_FbWidth, A64_FbHeight, A64_FbPhys ? "the bootloader's" : "a RAM");
	if (!TheDisplayState)
		A64Device_Format(aDetail, ", off");
	return TheDisplayState ? EA64DevOk : EA64DevOff;
	}
A64_DEVICE("display", "Display", "a64_display", DisplayStatus);
