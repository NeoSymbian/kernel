// SPDX-License-Identifier: Apache-2.0
// qcom_ipa.h -- the SDM670's IPA, the packet engine between the modem and the AP (qcom_ipa.cpp, telephony C5b).
#ifndef QCOM_IPA_H
#define QCOM_IPA_H

// the GSI firmware (an ELF image, the .mdt's aMdtLen bytes first; MODEMD reads it from vendor_a), kept for Ipa_Start
TInt Ipa_SetFirmware(const TUint8* aImage, TUint32 aLen, TUint32 aMdtLen);
TInt Ipa_PowerOn();										// the RPMh IPA clock
TInt Ipa_Start();										// power, firmware (PAS 15), SMMU, config, GSI, tables, LAN_RX
void Ipa_Report(void (*aSay)(const char*, ...));			// the monitor's "ipa"
TInt Ipa_Reg(TUint32 aOffset, TUint32& aValue);			// an IPA register (ipa-reg + aOffset) once powered: bring-up
// for the monitor's "ipa": KErrNotReady until the firmware runs (before, a GSI register read hangs the bus)
TInt Ipa_Status(TUint32& aGsiStatus);
#endif
