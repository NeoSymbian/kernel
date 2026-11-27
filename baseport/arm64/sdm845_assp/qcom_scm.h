// SPDX-License-Identifier: Apache-2.0
// qcom_scm.h -- calls into the SDM670's TrustZone (SCM over SMCCC), qcom_scm.cpp. Thread context.
#ifndef QCOM_SCM_H
#define QCOM_SCM_H

const TUint32 KScmVmidHlos = 3, KScmVmidMssMsa = 15;		// Linux include/linux/firmware/qcom/qcom_scm.h
const TUint32 KScmPermRw = 6, KScmPermRwx = 7;				// read 4 | write 2 | exec 1
struct SScmVmPerm { TUint32 iVmid, iPerm; };

// one SIP call: aArgInfo = the arg count | 2-bit types (Linux QCOM_SCM_ARGS); up to 10 args (more than 4 go in a
// buffer TZ reads). KErrNone and x1..x3 in aRes, or a negative SCM error mapped to an EPOC error.
TInt Scm_Call(TUint aSvc, TUint aCmd, TUint32 aArgInfo, const TUint64* aArgs, TInt aCount, TUint64 aRes[3]);
TInt Scm_IsAvailable(TUint aSvc, TUint aCmd);
extern TInt64 Scm_LastError;								// the raw TZ error of the last failed call				// 1 available, 0 not, < 0 the call failed
// hand [aPhys, aPhys + aSize) from the VMIDs in aSrc (a bit mask) to aDst; aSrc gets the new owners' mask
TInt Scm_AssignMem(TUint32 aPhys, TUint32 aSize, TUint64& aSrc, const SScmVmPerm* aDst, TInt aDstCount);
#endif
