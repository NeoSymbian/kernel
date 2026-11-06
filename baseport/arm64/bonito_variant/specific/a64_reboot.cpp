// SPDX-License-Identifier: Apache-2.0
//
// a64_reboot.cpp -- Pixel: reset the phone back into ABL's fastboot mode, so test images can be RAM-booted again with
// nobody holding Power + Volume Down. Like Linux drivers/power/reset/qcom-pon.c for the gen2 PON (DTB
// "qcom,pm8998-pon", mode-bootloader = 2): the reboot reason goes in PM660 PON_SOFT_RB_SPARE bits [7:1], ABL reads it
// after the PSCI SYSTEM_RESET.
// Triggers: Volume Up + Volume Down held 2 s (a64_keys.cpp), the debug monitor's "fastboot" command, and debug
// builds with AUTOFASTBOOT=<s> (-DA64_AUTOFASTBOOT): a timer, so a test image that hangs or loses its USB link
// can't strand the phone.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_spmi.h"
#include "../platform/a64_platform.h"

const TUint32 KPonSoftRbSpare = 0x088f;				// PM660 (SID 0) PON 0x800 + 0x8F
const TUint32 KReasonBootloader = 2, KReasonShift = 1;
const TUint64 KPsciSystemReset = 0x84000009;

void A64_RebootToFastboot()								// thread context (the SPMI write maps its channel)
	{
	TInt old = Spmi_Read(0, KPonSoftRbSpare);
	TInt r = Spmi_Write(0, KPonSoftRbSpare, ((old < 0 ? 0 : old) & 1) | (KReasonBootloader << KReasonShift));
	Kern::Printf("[reboot] to fastboot (reason write -> %d)", r);
	A64_Psci(KPsciSystemReset, 0, 0, 0);
	Kern::Printf("[reboot] SYSTEM_RESET returned");		// never: PSCI doesn't come back from a reset
	}

#ifdef A64_AUTOFASTBOOT
static void AutoDfcFn(TAny*)
	{
	Kern::Printf("[reboot] AUTOFASTBOOT: %d s are up", A64_AUTOFASTBOOT);
	A64_RebootToFastboot();
	}
static TDfc AutoDfc(AutoDfcFn, NULL, 7);
static void AutoTimerFn(TAny*) { AutoDfc.Add(); }
static NTimer AutoTimer(AutoTimerFn, NULL);
#endif

void A64StartReboot()
	{
#ifdef A64_AUTOFASTBOOT
	AutoDfc.SetDfcQ(Kern::DfcQue0());
	AutoTimer.OneShot(NKern::TimerTicks(A64_AUTOFASTBOOT * 1000));
	Kern::Printf("[reboot] AUTOFASTBOOT in %d s", A64_AUTOFASTBOOT);
#endif
	}
