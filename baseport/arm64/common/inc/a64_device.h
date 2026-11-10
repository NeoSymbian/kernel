// SPDX-License-Identifier: Apache-2.0
// a64_device.h -- the device status registry: each board driver describes the part it drives with one A64_DEVICE
// entry, and a64_devices.cpp lists them for the user side (HAL ENeoHalDeviceList, neoui's Dev tools) and the debug
// monitor ("drivers"). Entries sit in the .a64dev section (ekern.lds), so a driver that is not linked has none.
#ifndef A64_DEVICE_H
#define A64_DEVICE_H

#include <kernel/kernel.h>

// The order is the wire order: a64_devices.cpp names them ("absent", "off", ...) in the HAL lines.
enum TA64DevState
	{
	EA64DevAbsent,			// the driver is here, the part was not found
	EA64DevOff,				// found, switched off or not started on purpose
	EA64DevStarting,		// bring-up still running
	EA64DevOk,
	EA64DevPartial,			// works with a known gap (e.g. polled, no DMA)
	EA64DevError			// bring-up or the last operation failed
	};

struct SA64Device
	{
	const char* iTag;		// the driver's log tag without brackets ("touch"), also the user side's catalogue key
	const char* iName;		// the part ("Synaptics RMI4 touchscreen")
	const char* iDriver;	// the source file's base name ("a64_touch")
	// the live state, and one short detail line appended to aDetail (it has room for at least 80 bytes); called in
	// thread context, must not block for long
	TA64DevState (*iStatus)(TDes8& aDetail);
	};

#define A64_DEVICE_CAT2(a, b) a##b
#define A64_DEVICE_CAT(a, b) A64_DEVICE_CAT2(a, b)
// One per driver file, at file scope: A64_DEVICE("touch", "Synaptics RMI4 touchscreen", "a64_touch", TouchStatus);
#define A64_DEVICE(aTag, aName, aDriver, aStatus)															\
	__attribute__((used, section(".a64dev"), aligned(sizeof(void*))))										\
	static const SA64Device A64_DEVICE_CAT(A64Device_, __LINE__) = { aTag, aName, aDriver, aStatus }

// Kern::AppendFormat for status functions: A64Device_Format(aDetail, "%dx%d", w, h); cut at aOut's MaxLength
void A64Device_Format(TDes8& aOut, const char* aFmt, ...);

TInt A64Device_Count();
// device aIndex as one line, "tag\tstate\tname\tdriver\tdetail" (no newline); KErrNotFound past the end
TInt A64Device_Line(TInt aIndex, TDes8& aLine);
// ENeoHalDeviceList: a1 = TUint32* cursor (in/out, 0 = the first device), a2 = TDes8* -> whole lines while they fit
TInt A64Device_Hal(TAny* a1, TAny* a2);

#endif
