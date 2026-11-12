// SPDX-License-Identifier: Apache-2.0
// qcom_adsp.h -- boots the ADSP (the audio DSP) through TrustZone PAS and opens its GLINK channels (qcom_adsp.cpp).
// Thread context.
#ifndef QCOM_ADSP_H
#define QCOM_ADSP_H
// The firmware arrives from MODEMD in steps (HAL AdspLoad): aOp 0 = the .mdt in aData; 1 = aLen bytes of program
// header aSegment's file data at aOffset; 2 = done: auth_and_reset, then the ADSP's thread waits for ready, starts
// GLINK "lpass" and opens IPCRTR (ring 1) and apr_audio_svc (ring 2).
TInt Adsp_Load(TInt aOp, TInt aSegment, TUint32 aOffset, const TUint8* aData, TUint32 aLen);
TInt Adsp_Stop();
void Adsp_State(TInt& aStep, TInt& aError);					// step 4 = running (after handover)
void Adsp_Report(void (*aSay)(const char*, ...));
#endif
