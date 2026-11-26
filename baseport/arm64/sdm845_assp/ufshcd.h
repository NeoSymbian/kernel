// SPDX-License-Identifier: Apache-2.0
//
// ufshcd.h -- the UFS host core (ufshcd.cpp: UFSHCI 2.1, UIC, UPIUs, the block backend) and the vendor hooks it
// calls (qcom_ufs.cpp: Qualcomm's host glue; qcom_ufs_phy.cpp: the QMP PHY). Linux's split: ufshcd.c, ufs-qcom.c,
// phy-qcom-qmp-ufs.c (design and sequences only; docs fajita/ufs-linux-reference.md). The core has no SoC facts.
#ifndef UFSHCD_H
#define UFSHCD_H

// ---- the core, for the vendor files
volatile TUint32& Ufs_Reg(TUint32 aOff);						// a host register (UFSHCI or the vendor's own)
TInt Ufs_DmeGet(TUint32 aAttr, TUint32 aSel, TUint32& aVal, TBool aPeer = EFalse);
TInt Ufs_DmeSet(TUint32 aAttr, TUint32 aSel, TUint32 aVal, TBool aPeer = EFalse);
void Ufs_Log(const char* aFmt, ...);							// "[ufs] ..." in the log and in "blk info"
void Ufs_Delay(TInt aUs);										// busy below 1 ms, else sleeps (thread context)

// ---- the vendor (strong definitions in qcom_ufs.cpp), in the order the core calls them
TBool UfsVop_Clocked();				// may the host's registers be touched (power domain and interface clock on)?
TInt UfsVop_PowerUp();				// supplies, power domain, clocks, bus votes
void UfsVop_DeviceReset();			// pulse the device's reset pin
TInt UfsVop_HcePre();				// before HCE = 1: core reset, PHY power-up and calibration, vendor selects
TInt UfsVop_HcePost();				// after HCE = 1: vendor clock gating, PHY in HIBERN8
TInt UfsVop_LinkPre();				// before each DME_LINKSTARTUP
TInt UfsVop_LinkPost();				// after a successful link start-up
TInt UfsVop_GearPre(TUint32 aVendor);	// before the power-mode change: reference clock, the device's quirks
void UfsVop_Dump();					// "blk info": the vendor's state, read only
#endif
