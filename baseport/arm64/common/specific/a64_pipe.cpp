// SPDX-License-Identifier: Apache-2.0
// a64_pipe.cpp -- Symbian's pipe LDD (kernel/eka/drivers/pipe/dpipe.cpp, "SymbianPipe", unchanged), installed at
// boot: this kernel links its drivers in. RPipe (rpipe.dll) and so P.I.P.S.'s pipe()/popen() use it; NeoSymbian/build's
// loader answers User::LoadLogicalDevice("PIPELIB") with KErrAlreadyExists (ldr_builtin_devices.cpp alias). Rust's mio
// (Tokio) wakes its poll() with a pipe (browser WB2d).
#include <kernel/kern_priv.h>

DLogicalDevice* CreateLogicalDevice_pipe();					// dpipe.cpp, its entry points renamed (build-ekern.sh)

void A64StartPipe()
	{
	NKern::ThreadEnterCS();
	DLogicalDevice* ld = CreateLogicalDevice_pipe();
	TInt r = ld ? Kern::InstallLogicalDevice(ld) : KErrNoMemory;
	if (ld && r != KErrNone)
		ld->Close(NULL);									// as DoLogicalDeviceLoad: a failed install is the caller's
	NKern::ThreadLeaveCS();
	Kern::Printf("[pipe] SymbianPipe LDD -> %d", r);
	}
