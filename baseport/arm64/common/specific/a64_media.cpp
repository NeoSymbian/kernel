// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: Apache-2.0
//
// a64_media.cpp -- the local-media stack of the AArch64 kernel image (Phorepoc's xk_media.cpp): the unmodified ELOCD
// (drivers/locmedia) and internal RAM drive (drivers/medint/iram.cpp) are linked in as built-in extensions, their
// entry points renamed by build-ekern.sh. In a real ROM each is its own DLL; the user side's loader (the
// symbian-2010 efile.exe) answers KErrAlreadyExists when F32 asks for them.
#include <kernel/kern_priv.h>

TInt InitExtension_locmedia();
DLogicalDevice* CreateLogicalDevice_locmedia();
TInt InitExtension_medint();
DPhysicalDevice* CreatePhysicalDevice_medint();

void A64StartMedia()
	{
	TInt r = InitExtension_locmedia();				// EHalGroupMedia: estart stops at ESTART_7 without it
	Kern::Printf("[media] locmedia (ELOCD) InitExtension -> %d", r);
	DLogicalDevice* ld = CreateLogicalDevice_locmedia();
	r = ld ? Kern::InstallLogicalDevice(ld) : KErrNoMemory;
	Kern::Printf("[media] InstallLogicalDevice(LocDrv) -> %d", r);
	DPhysicalDevice* pd = CreatePhysicalDevice_medint();
	r = pd ? Kern::InstallPhysicalDevice(pd) : KErrNoMemory;
	Kern::Printf("[media] InstallPhysicalDevice(Media.IRam) -> %d", r);
	r = InitExtension_medint();
	Kern::Printf("[media] medint InitExtension (IRAM = local drive 0) -> %d", r);
	}
