// SPDX-License-Identifier: Apache-2.0
// a64_ecomm.cpp -- Symbian's serial LDD (ECOMM, kernel/eka/drivers/ecomm/d_comm.cpp, unchanged but for one cast) and the
// board's debug serial PDD (unit 1), installed at boot: this kernel links its drivers in and loads no driver files.
// RBusDevComm::Open(1) reaches them. Boards: QEMU virt's second PL011 (virt_assp/a64_pl011_comm.cpp).
// shortcut: User::LoadLogicalDevice("ecomm") / LoadPhysicalDevice("euart1") still answer KErrNotFound until the
// loader's built-in aliases know them (NeoSymbian/build ldr_builtin_devices.cpp, roadmap T7).
#include <kernel/kern_priv.h>
#include "a64_device.h"

DLogicalDevice* CreateLogicalDevice_ecomm();				// d_comm.cpp, its entry points renamed (variant.conf)
DPhysicalDevice* A64CreateDebugSerialPdd();				// the board's PDD
__attribute__((weak)) void A64DebugSerial_Status(TDes8&) {}	// its counters
static TInt InstallResult = KErrNotReady;

void A64StartDebugSerial()
	{
	NKern::ThreadEnterCS();
	DLogicalDevice* ld = CreateLogicalDevice_ecomm();
	TInt r = ld ? Kern::InstallLogicalDevice(ld) : KErrNoMemory;
	if (ld && r != KErrNone)
		ld->Close(NULL);									// as DoLogicalDeviceLoad: a failed install is the caller's
	if (r == KErrNone)
		{
		DPhysicalDevice* pd = A64CreateDebugSerialPdd();
		r = pd ? Kern::InstallPhysicalDevice(pd) : KErrNoMemory;
		if (pd && r != KErrNone)
			pd->Close(NULL);
		}
	InstallResult = r;
	NKern::ThreadLeaveCS();
	Kern::Printf("[serial] ECOMM + debug serial PDD (unit 1) -> %d", r);
	}

static TA64DevState DebugSerialStatus(TDes8& aDetail)
	{
	if (InstallResult == KErrNotReady)
		return EA64DevStarting;
	if (InstallResult != KErrNone)
		{
		A64Device_Format(aDetail, "ECOMM/PDD install: %d", InstallResult);
		return EA64DevError;
		}
	A64Device_Format(aDetail, "ECOMM unit 1");
	A64DebugSerial_Status(aDetail);
	return EA64DevOk;
	}
A64_DEVICE("serial1", "Debug serial port", "a64_ecomm", DebugSerialStatus);
