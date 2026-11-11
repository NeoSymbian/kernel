// SPDX-License-Identifier: Apache-2.0
// PM660 observation only: no PMIC writes, SRAM/profile access or ADC triggers.
// Register facts: AOSP qpnp-fg-gen3.c (6289dd9905d0, 0f5f44a2e74c), fg-util.c
// (60e67638d95e), and Linux v6.16 drivers/power/supply/qcom_pmi8998_charger.c.
// Profile integrity and hardware sample age cannot be proven with these direct reads.
#include <kernel/kern_priv.h>
#include "qcom_spmi.h"
#include "qcom_battery.h"
#include "a64_device.h"
#include "a64_dbgmon.h"

static TInt Byte(TUint32 aAddress) { return Spmi_Read(0, aAddress); }
static TInt Pair(TUint32 aAddress)
	{
	TInt lo = Byte(aAddress);
	if (lo < 0) return lo;
	TInt hi = Byte(aAddress + 1);
	return hi < 0 ? hi : lo | (hi << 8);
	}

TInt A64BatteryRead(TUint32 a[8])
	{
	for (TInt i = 0; i < 8; ++i) a[i] = 0;
	a[0] = 1;
	if (Spmi_Version() < 0) return KErrNotReady;
	TInt type = Byte(0x0104), subtype = Byte(0x0105);
	if (type < 0) return type;
	if (subtype < 0) return subtype;
	if (type != 0x51 || subtype != 0x1b) return KErrNotSupported;

	TInt batif = Byte(0x1210);
	TBool gauge = Byte(0x4005) == 0x10 && Byte(0x4105) == 0x11;
	TInt missing = gauge ? Byte(0x4110) : KErrNotReady;
	if (batif >= 0 && missing >= 0)
		{
		a[1] |= 1;
		a[2] = !(batif & 0x30) && !(missing & 8);
		}
	// Readiness is a guard, not proof of the retained battery profile/calibration.
	if (gauge && (a[1] & 1) && a[2])
		{
		TInt enable = Byte(0x4046), restart = Byte(0x4048), reset = Byte(0x4109), ready = Byte(0x4010);
		TBool running = enable >= 0 && restart >= 0 && reset >= 0 &&
			(enable & 0x80) && !(restart & 1) && !(reset & 0x80);
		if (running && ready >= 0 && (ready & 2))
			for (TInt tries = 0; tries < 5; ++tries)
				{
				TInt soc = Byte(0x4009), shadow = Byte(0x400a);
				if (soc < 0 || shadow < 0) break;
				if (soc != shadow) continue;
				a[3] = soc == 0 ? 0 : soc == 255 ? 100 : ((soc - 1) * 98 + 126) / 253 + 1;
				a[1] |= 2;
				break;
				}
		if (running)
			for (TInt tries = 0; tries < 5; ++tries)
				{
				TInt value = Pair(0x41a0), shadow = Pair(0x41a6);
				if (value < 0 || shadow < 0) break;
				if (value != shadow) continue;
				a[4] = (TUint32)((TUint64)value * 122070 / 1000);
				a[1] |= 4;
				break;
				}
		}
	// Check relevant status bits twice to avoid combining an attach edge with old charger state.
	for (TInt tries = 0; tries < 5; ++tries)
		{
		TInt usb = Byte(0x1310), path = Byte(0x160b), phase = Byte(0x1006), faults = Byte(0x1007);
		if (usb >= 0 && path >= 0 && phase >= 0 && faults >= 0)
			{
			TInt u2 = Byte(0x1310), p2 = Byte(0x160b), s2 = Byte(0x1006), f2 = Byte(0x1007);
			if (u2 < 0 || p2 < 0 || s2 < 0 || f2 < 0) break;
			if (((usb ^ u2) & 0x10) || ((path ^ p2) & 0x11) ||
				((phase ^ s2) & 7) || ((faults ^ f2) & 0x20)) continue;
			a[1] |= 8 | 16;
			a[5] = !!(usb & 0x10);
			a[6] = (path & 0x11) == 0x11;
			// No charging/full claim when battery presence is unknown or missing.
			if (a[1] & 1)
				{
				a[1] |= 32;
				a[7] = !a[2] ? 3 : !a[6] ? 1 : (faults & 0x20) ? 3 :
					(phase & 7) <= 4 ? 2 : (phase & 7) <= 6 ? 4 : 3;
				}
			break;
			}
		// USB presence remains useful if another charger peripheral is unavailable.
		if (usb >= 0) { a[1] |= 8; a[5] = !!(usb & 0x10); }
		break;
		}
	return a[1] ? KErrNone : KErrNotReady;
	}

TInt A64Battery_Hal(TAny* aDescriptor)
	{
	TUint32 words[8];
	TInt r = A64BatteryRead(words);
	if (r != KErrNone) return r;
	TBuf8<32> out;
	for (TInt i = 0; i < 8; ++i)
		for (TInt b = 0; b < 4; ++b)
			out.Append((TUint8)(words[i] >> (8 * b)));
	return Kern::ThreadDesWrite(&Kern::CurrentThread(), aDescriptor, out, 0, KChunkShiftBy0, NULL);
	}

void A64BatteryDump()
	{
	TUint32 a[8];
	TInt r = A64BatteryRead(a);
	A64Mon_Say("battery read %d; valid 0x%x; present %u; reported %u%%; %u uV", r, a[1], a[2], a[3], a[4]);
	A64Mon_Say("battery USB present %u online %u; charge state %u (0 unknown,1 discharge,2 charge,3 idle,4 full)", a[5], a[6], a[7]);
	A64Mon_Say("battery: validity bits 0 present,1 percent,2 voltage,3 USB present,4 online,5 state; profile/age unverified; read only");
	if (Spmi_Version() >= 0)
		{
		A64Mon_Say("battery IDs: PMIC type=%d subtype=%d; FG SOC=%d info=%d", Byte(0x0104), Byte(0x0105), Byte(0x4005), Byte(0x4105));
		if (r != KErrNotSupported)
			{
			A64Mon_Say("battery gauge: ready=0x%x enable=0x%x restart=0x%x reset=0x%x", Byte(0x4010), Byte(0x4046), Byte(0x4048), Byte(0x4109));
			A64Mon_Say("battery detection: BATIF=0x%x FG missing=0x%x; charger phase=0x%x faults=0x%x", Byte(0x1210), Byte(0x4110), Byte(0x1006), Byte(0x1007));
			}
		}
	}

static TA64DevState BatteryStatus(TDes8& aDetail)
	{
	TUint32 a[8];
	TInt r = A64BatteryRead(a);
	if (r != KErrNone)
		{
		A64Device_Format(aDetail, "unavailable (%d); read only", r);
		return r == KErrNotReady ? EA64DevOff : EA64DevError;
		}
	A64Device_Format(aDetail, "valid=0x%x; ", a[1]);
	if (a[1] & 2) A64Device_Format(aDetail, "reported %u%%; ", a[3]);
	else A64Device_Format(aDetail, "percent unknown; ");
	if (a[1] & 4) A64Device_Format(aDetail, "%u uV; ", a[4]);
	else A64Device_Format(aDetail, "voltage unknown; ");
	A64Device_Format(aDetail, "profile/age unverified; read only");
	return EA64DevPartial;
	}
A64_DEVICE("battery", "Battery and charger (PM660)", "qcom_battery", BatteryStatus);
