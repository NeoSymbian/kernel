// SPDX-License-Identifier: Apache-2.0
// qcom_spmi.h -- the Qualcomm SPMI PMIC arbiter v5 (SDM670: PM660 at SID 0, PM660L at SID 2), polled.
#ifndef QCOM_SPMI_H
#define QCOM_SPMI_H
// Map the channel table once at boot, before peripheral workers start; repeated calls return the cached version.
// The arbiter's version (>= 0x50000000), or a negative error.
int Spmi_Init();
// Cached bus readiness without touching MMIO or initializing the bus. KErrNotReady before platform bus startup.
int Spmi_Version();
// One PMIC register byte (aAddr = peripheral << 8 | register), or a negative error. Thread or DFC context.
int Spmi_Read(unsigned aSid, unsigned aAddr);
// Write one PMIC register byte through the peripheral's own channel (needs ee 0 to own it): KErrNone or an error.
// Thread context (maps the channel's page on first use).
int Spmi_Write(unsigned aSid, unsigned aAddr, unsigned aValue);
// The apid (arbiter channel) of a peripheral, -1 if the arbiter has none
int Spmi_Apid(unsigned aSid, unsigned aPeriph);
#endif
