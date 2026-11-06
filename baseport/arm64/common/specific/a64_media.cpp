// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: Apache-2.0
//
// a64_media.cpp -- the local-media stack of the AArch64 kernel image (Phorepoc's xk_media.cpp): the unmodified ELOCD
// (drivers/locmedia) and local drive 0 -- the persistent block drive (a64_medblk.cpp: QEMU virtio-blk, Pixel eMMC
// userdata) when the board has one, else the internal RAM drive (drivers/medint/iram.cpp) -- are linked in as
// built-in extensions, their entry points renamed by build-ekern.sh. In a real ROM each is its own DLL; the user side's loader (the
// symbian-2010 efile.exe) answers KErrAlreadyExists when F32 asks for them.
#include <kernel/kern_priv.h>
#include "a64_blk.h"

TInt InitExtension_locmedia();
DLogicalDevice* CreateLogicalDevice_locmedia();
TInt InitExtension_medint();
DPhysicalDevice* CreatePhysicalDevice_medint();
TInt InitExtension_medblk();
DPhysicalDevice* CreatePhysicalDevice_medblk();

void A64StartMedia()
	{
	TInt r = InitExtension_locmedia();				// EHalGroupMedia: estart stops at ESTART_7 without it
	Kern::Printf("[media] locmedia (ELOCD) InitExtension -> %d", r);
	DLogicalDevice* ld = CreateLogicalDevice_locmedia();
	r = ld ? Kern::InstallLogicalDevice(ld) : KErrNoMemory;
	Kern::Printf("[media] InstallLogicalDevice(LocDrv) -> %d", r);
	TBool blk = A64Blk_Start() == KErrNone;				// a persistent disk: it is local drive 0, else the RAM drive
	DPhysicalDevice* pd = blk ? CreatePhysicalDevice_medblk() : CreatePhysicalDevice_medint();
	r = pd ? Kern::InstallPhysicalDevice(pd) : KErrNoMemory;
	Kern::Printf("[media] InstallPhysicalDevice(%s) -> %d", blk ? "Media.Blk" : "Media.IRam", r);
	r = blk ? InitExtension_medblk() : InitExtension_medint();
	Kern::Printf("[media] %s = local drive 0 -> %d", blk ? "block drive" : "IRAM", r);
	}
