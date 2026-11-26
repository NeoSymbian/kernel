// SPDX-License-Identifier: Apache-2.0
//
// qcom_dload.cpp -- turn off Qualcomm's download (crash dump) mode at boot. OnePlus firmware leaves it on, so any
// reset that isn't a clean one -- a watchdog bite, a PSCI reset after a fault -- ends in "QUALCOMM CrashDump Mode"
// instead of the bootloader (docs fajita.md OP3, 2026-11-26). Like downstream msm-poweroff's set_dload_mode(0): the
// TrustZone boot call (QCOM_SCM_SVC_BOOT, SET_DLOAD_MODE) when TZ has it, else a secure I/O write of 0 to
// TCSR_BOOT_MISC_DETECT (the live DTB's restart@10ac000 "tcsr-boot-misc-detect" = 0x1fd3000).
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_scm.h"

const TUint KScmSvcBoot = 0x01, KScmBootSetDloadMode = 0x10;	// Linux qcom_scm.h
const TUint KScmSvcIo = 0x05, KScmIoWrite = 0x02;
const TUint32 KTcsrBootMiscDetect = 0x01fd3000;

void A64StartFirmware()
	{
	TUint64 res[3];
	TInt avail = Scm_IsAvailable(KScmSvcBoot, KScmBootSetDloadMode);
	TInt r;
	if (avail == 1)
		{
		const TUint64 args[2] = { 0, 0 };
		r = Scm_Call(KScmSvcBoot, KScmBootSetDloadMode, 2, args, 2, res);
		}
	else
		{
		const TUint64 args[2] = { KTcsrBootMiscDetect, 0 };
		r = Scm_Call(KScmSvcIo, KScmIoWrite, 2, args, 2, res);
		}
	Kern::Printf("[dload] download mode off: %s -> %d", avail == 1 ? "SCM boot call" : "TCSR_BOOT_MISC_DETECT = 0", r);
	}
