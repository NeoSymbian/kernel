// SPDX-License-Identifier: Apache-2.0
// qcom_mss.h -- boots the modem (qcom_mss.cpp). Thread context.
#ifndef QCOM_MSS_H
#define QCOM_MSS_H
TInt Mss_StartBoot();										// queues the boot on its own thread; progress in the log
void Mss_State(TInt& aStep, TInt& aError, TUint32 aRmb[4]);	// step 8 = running; RMB pbl, status, mss, code length
#endif
