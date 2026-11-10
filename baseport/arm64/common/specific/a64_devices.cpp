// SPDX-License-Identifier: Apache-2.0
// a64_devices.cpp -- the device status registry (a64_device.h): the A64_DEVICE entries the linker gathered in
// .a64dev, listed as text lines for the user side (HAL ENeoHalDeviceList = 0x4e19, ReadDeviceData; neoui's Dev tools
// parses them in neoui-proto devtools::device) and the debug monitor's "drivers" command.
#include <kernel/kernel.h>
#include "a64_device.h"

extern "C" const SA64Device __a64dev_start[], __a64dev_end[];	// ekern.lds

static const char* const KStateNames[] = { "absent", "off", "starting", "ok", "partial", "error" };
// a bigger detail is cut; one line is at most this long
const TInt KLineMax = 200;

static void AppendText(TDes8& aOut, const char* aText)
	{
	for (const char* p = aText; *p && aOut.Length() < aOut.MaxLength(); ++p)
		aOut.Append(*p == '\t' || *p == '\n' ? ' ' : *p);	// the separators never appear inside a field
	}

void A64Device_Format(TDes8& aOut, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<KLineMax> b;
	Kern::AppendFormat(b, aFmt, list);
	VA_END(list);
	aOut.Append(b.Left(aOut.MaxLength() - aOut.Length()));
	}

TInt A64Device_Count()
	{
	return __a64dev_end - __a64dev_start;
	}

TInt A64Device_Line(TInt aIndex, TDes8& aLine)
	{
	if (aIndex < 0 || aIndex >= A64Device_Count())
		return KErrNotFound;
	const SA64Device& d = __a64dev_start[aIndex];
	TBuf8<KLineMax> detail;
	TA64DevState s = d.iStatus ? d.iStatus(detail) : EA64DevOk;
	if ((TUint)s >= sizeof(KStateNames) / sizeof(KStateNames[0]))
		s = EA64DevError;
	aLine.Zero();
	AppendText(aLine, d.iTag);
	aLine.Append('\t');
	AppendText(aLine, KStateNames[s]);
	aLine.Append('\t');
	AppendText(aLine, d.iName);
	aLine.Append('\t');
	AppendText(aLine, d.iDriver);
	aLine.Append('\t');
	for (TInt i = 0; i < detail.Length() && aLine.Length() < aLine.MaxLength(); ++i)
		aLine.Append(detail[i] == '\t' || detail[i] == '\n' ? ' ' : detail[i]);
	return KErrNone;
	}

TInt A64Device_Hal(TAny* a1, TAny* a2)
	{
	if (!Kern::CurrentThreadHasCapability(ECapabilityReadDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian devices")))
		return KErrPermissionDenied;
	TUint32 cursor;
	kumemget32(&cursor, a1, sizeof(cursor));
	TBuf8<1024> out;						// as the log reads: the caller's buffer takes 1024 bytes (else KErrOverflow)
	TBuf8<KLineMax + 1> line;
	for (; (TInt)cursor < A64Device_Count(); ++cursor)
		{
		A64Device_Line(cursor, line);
		line.Append('\n');
		if (out.Length() + line.Length() > out.MaxLength())
			break;							// a line is at most KLineMax + 1 bytes, so each call returns at least one
		out.Append(line);
		}
	kumemput32(a1, &cursor, sizeof(cursor));
	return Kern::ThreadDesWrite(&Kern::CurrentThread(), a2, out, 0, KChunkShiftBy0, NULL);
	}
