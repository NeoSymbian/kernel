// SPDX-License-Identifier: Apache-2.0
// qcom_pas.h -- TrustZone's peripheral authentication service (PAS): load a signed firmware image into its carve-out
// and let TZ authenticate and start it (qcom_pas.cpp). Thread context.
#ifndef QCOM_PAS_H
#define QCOM_PAS_H

const TUint32 KPasIpa = 15, KPasAdsp = 1;					// Linux IPA_PAS_ID; the sdm670 ADSP's pas_id

// aImage: the firmware as one ELF (the split .mdt + .bNN files laid out at their p_offsets); its first aMdtLen bytes
// are the metadata TZ checks (headers + hash segment, the .mdt file). Segments go into [aPhys, aPhys + aSize).
// KErrNone once TZ has authenticated and reset the peripheral.
TInt Pas_Load(TUint32 aPas, const TUint8* aImage, TUint32 aLen, TUint32 aMdtLen, TUint32 aPhys, TUint32 aSize);

// The same in steps, for an image too big to hold at once. Begin: the .mdt (headers + hash) to TZ (init_image,
// mem_setup). Segment: aLen bytes of program header aIndex's file data at aOffset into the carve-out. End: the zero
// tails, then auth_and_reset. One load at a time (KErrInUse); Abort drops a load that failed half-way.
TInt Pas_Begin(TUint32 aPas, const TUint8* aMdt, TUint32 aMdtLen, TUint32 aPhys, TUint32 aSize);
TInt Pas_Segment(TUint32 aIndex, TUint32 aOffset, const TUint8* aData, TUint32 aLen);
TInt Pas_End();
void Pas_Abort();
#endif
