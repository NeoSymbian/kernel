// SPDX-License-Identifier: Apache-2.0
//
// a64_reboot.cpp -- Pixel: reset the phone back into ABL's fastboot mode, so test images can be RAM-booted again with
// nobody holding Power + Volume Down. Like Linux drivers/power/reset/qcom-pon.c for the gen2 PON (DTB
// "qcom,pm8998-pon", mode-bootloader = 2): the reboot reason goes in PM660 PON_SOFT_RB_SPARE bits [7:1], ABL reads it
// after the PSCI SYSTEM_RESET.
// Triggers: Volume Up + Volume Down held 2 s (a64_keys.cpp), the debug monitor's "fastboot" command, and debug
// builds with AUTOFASTBOOT=<s> (-DA64_AUTOFASTBOOT): a timer, so a test image that hangs or loses its USB link
// can't strand the phone.
// Also the user's restart and power off (HAL ENeoHalPower from neoui's Power menu, monitor reboot/poweroff), and
// Kern::Restart (A64_Restart: the end of Kern::Fault).
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_spmi.h"
#include "a64_platform.h"

const TUint32 KPonSoftRbSpare = 0x088f;				// PM660 (SID 0) PON 0x800 + 0x8F
const TUint32 KReasonBootloader = 2, KReasonShift = 1;
const TUint64 KPsciSystemOff = 0x84000008, KPsciSystemReset = 0x84000009;
// the APSS watchdog (Linux qcom-wdt.c, KPSS layout, 32768 Hz; at 0x17980000 as on SDM845 -- the DTB has no node):
// a bite resets the SoC even when a CPU is wedged on the bus and PSCI SYSTEM_RESET can't complete
const TUint32 KWdt = 0x17980000;
enum { EWdtRst = 0x4, EWdtEn = 0x8, EWdtBark = 0x10, EWdtBite = 0x14 };

void A64_WatchdogBite(TInt aMs)							// arm a bite aMs from now (no bark handler)
	{
	volatile TUint32* w = (volatile TUint32*)A64_Io(KWdt);
	w[EWdtEn / 4] = 0;
	w[EWdtRst / 4] = 1;
	w[EWdtBark / 4] = 0x7fffff;
	w[EWdtBite / 4] = 32768 * aMs / 1000;
	w[EWdtEn / 4] = 1;
	}

void A64_RebootToFastboot()								// thread context (the SPMI write maps its channel)
	{
	TInt old = Spmi_Read(0, KPonSoftRbSpare);
	TInt r = Spmi_Write(0, KPonSoftRbSpare, ((old < 0 ? 0 : old) & 1) | (KReasonBootloader << KReasonShift));
	Kern::Printf("[reboot] to fastboot (reason write -> %d)", r);
	A64_WatchdogBite(2000);								// if PSCI can't reset (a CPU stuck on the bus)
	A64_Psci(KPsciSystemReset, 0, 0, 0);
	Kern::Printf("[reboot] SYSTEM_RESET returned");		// never: PSCI doesn't come back from a reset
	}

#ifdef A64_AUTOFASTBOOT
// debug builds: from boot, the reason says "bootloader" and the watchdog runs, petted every second -- if anything
// freezes the whole SoC (a register access the bus never answers stalls every CPU), the bite brings the phone back
// to fastboot with no software left running
static void PetDfcFn(TAny*);
static TDfc PetDfc(PetDfcFn, NULL, 7);
static void PetTimerFn(TAny*) { PetDfc.Add(); }
static NTimer PetTimer(PetTimerFn, NULL);
static void PetDfcFn(TAny*)
	{
	((volatile TUint32*)A64_Io(KWdt))[EWdtRst / 4] = 1;
	PetTimer.OneShot(NKern::TimerTicks(1000));
	}

static void AutoDfcFn(TAny*)
	{
	Kern::Printf("[reboot] AUTOFASTBOOT: %d s are up", A64_AUTOFASTBOOT);
	A64_RebootToFastboot();
	}
static TDfc AutoDfc(AutoDfcFn, NULL, 7);
static void AutoTimerFn(TAny*) { AutoDfc.Add(); }
static NTimer AutoTimer(AutoTimerFn, NULL);
#endif

// a normal restart: ABL boots the active slot (qcom_bootctrl.cpp switches slots this way). Thread context
void A64_RebootNormal()
	{
	TInt old = Spmi_Read(0, KPonSoftRbSpare);
	TInt r = Spmi_Write(0, KPonSoftRbSpare, (old < 0 ? 0 : old) & 1);
	Kern::Printf("[reboot] normal restart (reason write -> %d)", r);
	A64_WatchdogBite(2000);								// if PSCI can't reset
	A64_Psci(KPsciSystemReset, 0, 0, 0);
	}

// power off (the Power menu, the monitor's "poweroff"). Thread context. No watchdog backup: a bite would turn the
// phone back on. TrustZone drops PS_HOLD for SYSTEM_OFF; the PMIC then waits for the Power key (or a charger, when
// ABL boots into off-mode charging)
TInt A64_PowerOff()
	{
	TInt old = Spmi_Read(0, KPonSoftRbSpare);
	TInt r = Spmi_Write(0, KPonSoftRbSpare, (old < 0 ? 0 : old) & 1);
	Kern::Printf("[reboot] power off (reason write -> %d)", r);
	A64_Psci(KPsciSystemOff, 0, 0, 0);
	Kern::Printf("[reboot] SYSTEM_OFF returned");
	return KErrGeneral;
	}

// a kernel fault (NotifyCrash, interrupts off): register writes only. The bite restarts the phone 10 s later -- time
// to read the FAULT on screen -- and ABL then boots the active slot normally (an install image cleared the reason at
// boot), counting a failed try on a slot not yet marked successful: the FS5b rollback. Test images keep petting the
// watchdog from the other CPUs and stay on the screen until AUTOFASTBOOT.
static volatile TBool CrashBiteArmed;
extern "C" void A64_CrashReset()
	{
	CrashBiteArmed = ETrue;
	A64_WatchdogBite(10000);
	}

// Kern::Restart (Kern::Fault ends there): registers only. After a crash the bite above is already counting down, so
// the FAULT stays on screen; otherwise a reset now, the bite as backup. The reason is left as it is (test images
// keep "bootloader")
extern "C" void A64_Restart(int)
	{
	if (!CrashBiteArmed)
		{
		A64_WatchdogBite(2000);
		A64_Psci(KPsciSystemReset, 0, 0, 0);
		}
	for (;;)
		__asm__ __volatile__("wfi");
	}

void A64StartReboot()
	{
#ifndef A64_AUTOFASTBOOT
	// an install image: a restart (crash or reset) is a normal boot again, whatever a test image left in the reason
	TInt old = Spmi_Read(0, KPonSoftRbSpare);
	TInt r = old < 0 ? old : Spmi_Write(0, KPonSoftRbSpare, old & 1);
	Kern::Printf("[reboot] reason cleared (was %02x) -> %d", old < 0 ? 0 : old, r);
#endif
#ifdef A64_AUTOFASTBOOT
	AutoDfc.SetDfcQ(Kern::DfcQue0());
	AutoTimer.OneShot(NKern::TimerTicks(A64_AUTOFASTBOOT * 1000));
	TInt old = Spmi_Read(0, KPonSoftRbSpare);
	TInt r = Spmi_Write(0, KPonSoftRbSpare, ((old < 0 ? 0 : old) & 1) | (KReasonBootloader << KReasonShift));
	A64_WatchdogBite(10000);
	PetDfc.SetDfcQ(Kern::DfcQue0());
	PetTimer.OneShot(NKern::TimerTicks(1000));
	Kern::Printf("[reboot] AUTOFASTBOOT in %d s; reason write -> %d, watchdog armed (10 s, petted)", A64_AUTOFASTBOOT, r);
#endif
	}
