// SPDX-License-Identifier: Apache-2.0
// qcom_spmi.h -- the Qualcomm SPMI PMIC arbiter v5 (SDM670: PM660 at SID 0, PM660L at SID 2), polled.
#ifndef QCOM_SPMI_H
#define QCOM_SPMI_H
// Map the channel table; the arbiter's version (>= 0x50000000), or a negative error
int Spmi_Init();
// One PMIC register byte (aAddr = peripheral << 8 | register), or a negative error. Thread or DFC context.
int Spmi_Read(unsigned aSid, unsigned aAddr);
// The apid (arbiter channel) of a peripheral, -1 if the arbiter has none
int Spmi_Apid(unsigned aSid, unsigned aPeriph);
#endif
