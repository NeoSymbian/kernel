// SPDX-License-Identifier: Apache-2.0
// qcom_ufs_phy.h -- SDM845's QMP UFS PHY (qcom_ufs_phy.cpp), for the host glue (qcom_ufs.cpp)
#ifndef QCOM_UFS_PHY_H
#define QCOM_UFS_PHY_H
void QmpUfs_PowerOff();				// the PHY's own power-down control (its clocks and supplies are the caller's)
void QmpUfs_PowerOn();
TInt QmpUfs_Calibrate();			// tables under the host's PHY soft reset, start, wait for ready (10 ms)
void QmpUfs_Dump();					// start / power-down / ready, to the log
#endif
