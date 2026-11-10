// SPDX-License-Identifier: Apache-2.0
#ifndef QCOM_BATTERY_H
#define QCOM_BATTERY_H
// Thread context. Eight little-endian u32 words over HAL 0x4e1a:
// version, valid mask, present, reported percent, microvolts, USB present, USB online, charge state.
// Valid bits 0..5 correspond to the six data words. States: 0 unknown, 1 discharging,
// 2 charging, 3 not charging, 4 full. PMIC profile/sample age remain unverified.
TInt A64BatteryRead(TUint32 aWords[8]);
TInt A64Battery_Hal(TAny* aDescriptor);
void A64BatteryDump();
#endif
