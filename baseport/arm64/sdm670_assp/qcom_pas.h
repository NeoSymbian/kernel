// SPDX-License-Identifier: Apache-2.0
// qcom_pas.h -- TrustZone's peripheral authentication service (PAS): load a signed firmware image into its carve-out
// and let TZ authenticate and start it (qcom_pas.cpp). Thread context.
#ifndef QCOM_PAS_H
#define QCOM_PAS_H

const TUint32 KPasIpa = 15;								// Linux IPA_PAS_ID (the ADSP is 1, for C6)

// aImage: the firmware as one ELF (the split .mdt + .bNN files laid out at their p_offsets); its first aMdtLen bytes
// are the metadata TZ checks (headers + hash segment, the .mdt file). Segments go into [aPhys, aPhys + aSize).
// KErrNone once TZ has authenticated and reset the peripheral.
TInt Pas_Load(TUint32 aPas, const TUint8* aImage, TUint32 aLen, TUint32 aMdtLen, TUint32 aPhys, TUint32 aSize);
#endif
