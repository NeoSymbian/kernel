// SPDX-License-Identifier: Apache-2.0
// SDM670 TSENS v2 telemetry only. Register facts: Linux drivers/thermal/qcom/tsens-v2.c
// and tsens.c (https://github.com/torvalds/linux/tree/master/drivers/thermal/qcom).
// TM status +0xa0+4*n: VALID bit 21, signed 12-bit deci-Celsius. SROT: version +0,
// control +4 (enable bit 0, enabled sensors bits 3..18). No register writes, IRQs or policy.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_device.h"
#include "a64_dbgmon.h"
#include "qcom_tsens.h"

static const TUint32 KTm[] = { 0x0c263000, 0x0c265000 };
static const TUint32 KSrot[] = { 0x0c222000, 0x0c223000 };
static const TInt KSensorCount[] = { 13, 8 }; // supplied bonito DTB, controller-local IDs

static TInt Read(TUint32 aPhys, TUint32& aValue)
	{
	TLinAddr p = A64_Io(aPhys);
	if (!p)
		return KErrNotSupported;
	TInt r = KErrNone;
	XTRAP(r, XT_DEFAULT, aValue = *(volatile TUint32*)p);
	return r;
	}

TInt A64ThermalRead(TInt aController, TInt aSensor, TInt& aMilliC)
	{
	if (aController < 0 || aController >= 2 || aSensor < 0 || aSensor >= KSensorCount[aController])
		return KErrArgument;
	TUint32 version = 0, control = 0;
	TInt r = Read(KSrot[aController], version);
	if (r != KErrNone)
		return r;
	if ((version >> 28) != 2)
		return KErrNotSupported;
	r = Read(KSrot[aController] + 4, control);
	if (r != KErrNone)
		return r;
	if (!(control & 1) || !(control & (1u << (aSensor + 3))))
		return KErrNotReady;
	for (TInt attempt = 0; attempt < 3; ++attempt)
		{
		TUint32 status = 0;
		r = Read(KTm[aController] + 0xa0 + 4 * aSensor, status);
		if (r != KErrNone)
			return r;
		if (status & (1u << 21))
			{
			TInt value = status & 0xfff;
			if (value & 0x800)
				value -= 0x1000;
			aMilliC = value * 100;
			return KErrNone;
			}
		}
	return KErrNotReady;
	}

void A64ThermalDump()
	{
	for (TInt c = 0; c < 2; ++c)
		for (TInt s = 0; s < KSensorCount[c]; ++s)
			{
			TInt temperature = 0;
			TInt r = A64ThermalRead(c, s, temperature);
			if (r == KErrNone)
				A64Mon_Say("thermal %d:%d %d mC", c, s, temperature);
			else
				A64Mon_Say("thermal %d:%d unavailable (%d)", c, s, r);
			}
	}

static TA64DevState ThermalStatus(TDes8& aDetail)
	{
	TInt valid = 0, maximum = -204800, failures = 0;
	for (TInt c = 0; c < 2; ++c)
		for (TInt s = 0; s < KSensorCount[c]; ++s)
			{
			TInt temperature = 0;
			TInt r = A64ThermalRead(c, s, temperature);
			if (r == KErrNone)
				{
				++valid;
				if (temperature > maximum) maximum = temperature;
				}
			else if (r != KErrNotReady) ++failures;
			}
	if (valid)
		{
		A64Device_Format(aDetail, "%d/21 valid; max %d mC; read only; no thermal policy", valid, maximum);
		return EA64DevPartial;
		}
	A64Device_Format(aDetail, "no valid samples; %d read/version errors; read only", failures);
	return failures ? EA64DevError : EA64DevOff;
	}
A64_DEVICE("thermal", "SoC temperature (TSENS v2)", "qcom_tsens", ThermalStatus);
