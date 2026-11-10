// SPDX-License-Identifier: Apache-2.0
//
// qcom_adsp.cpp -- the ADSP, the audio DSP (DTB remoteproc@62400000 "qcom,sdm670-adsp-pas"). Unlike the modem it is
// started entirely by TrustZone: QMP "load_state adsp on", SMP2P to the ADSP, PAS 1 (init_image, mem_setup, the
// segments into the adsp carve-out 0x95800000 + 0x2200000, auth_and_reset); then it reports ready over SMP2P, and
// its GLINK edge "lpass" (remote pid 2) carries QRTR (IPCRTR: the audio PD asks our locator for avs/audio) and APR
// (apr_audio_svc: AFE ports and the voice sessions, MODEMD). Telephony C6.
// The firmware (17 MB) streams in from MODEMD (HAL AdspLoad) segment by segment; the waits run on our own thread.
// From Linux drivers/remoteproc/qcom_q6v5_pas.c (sdm845_adsp_resource_init, used for sdm670: pas_id 1, load_state
// "adsp", crash reason SMEM 423, no proxy power domains) and qcom_q6v5.c (the SMP2P ready/handover bits).
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_smem.h"
#include "qcom_smp2p.h"
#include "qcom_qmp.h"
#include "qcom_pas.h"
#include "qcom_glink.h"
#include "qcom_telhal.h"
#include "qcom_adsp.h"
#include "a64_device.h"

const TUint32 KAdspPhys = 0x95800000, KAdspSize = 0x02200000;
const TUint32 KCrashReason = 423;							// SMEM_SSR_REASON_LPASS0

static TInt Step, Error;
static volatile TBool UpBusy;									// 1 loading, 2 authenticated, 3 ready + GLINK, 4 handover

static TInt Fail(TInt aErr, const char* aWhat)
	{
	Error = aErr;
	Kern::Printf("[adsp] step %d: %s -> %d", Step, aWhat, aErr);
	return aErr;
	}

static void Up()
	{
	TUint32 v = 0;
	TInt i;
	for (i = 0; i < 5000; ++i)								// Linux waits 5 s for ready
		{
		Smp2p_Poll(KSmemHostAdsp);
		if (Smp2p_In(KSmemHostAdsp, "slave-kernel", v) == KErrNone && (v & (KMssReady | KMssFatal)))
			break;
		NKern::Sleep(NKern::TimerTicks(1));
		}
	if (!(v & KMssReady))
		{
		TUint32 len = 0;
		const char* why = (const char*)Smem_Get(KSmemHostGlobal, KCrashReason, &len);
		Kern::Printf("[adsp] slave-kernel %08x; crash reason: %s", v, why && len ? why : "(none)");
		Fail(v & KMssFatal ? KErrDied : KErrTimedOut, "ready");
		return;
		}
	Step = 3;
	TInt r = Glink_Start(KSmemHostAdsp);
	TInt qrtr = r == KErrNone ? Glink_Open(KSmemHostAdsp, "IPCRTR", Qrtr_Rx, Tel_Ring(1)) : r;
	TInt apr = r == KErrNone ? Glink_Open(KSmemHostAdsp, "apr_audio_svc", Qrtr_Rx, Tel_Ring(2)) : r;
	if (qrtr > 0)
		Tel_SetChannel(1, qrtr);
	if (apr > 0)
		Tel_SetChannel(2, apr);
	Kern::Printf("[adsp] running after %d ms (slave-kernel %08x); glink %d, IPCRTR %x, apr %x", i, v, r, qrtr, apr);
	for (i = 0; !(v & (KMssHandover | KMssFatal)) && i < 10000; ++i)
		{
		NKern::Sleep(NKern::TimerTicks(1));
		Smp2p_Poll(KSmemHostAdsp);
		Smp2p_In(KSmemHostAdsp, "slave-kernel", v);
		}
	if (v & KMssHandover)
		Step = 4;
	Kern::Printf("[adsp] after %d ms: slave-kernel %08x%s", i, v, v & KMssHandover ? " (handover)" : "");
	}

static TDynamicDfcQue* Queue;
static void UpDfcFn(TAny*) { Up(); __e32_memory_barrier(); UpBusy = EFalse; }
static TDfc UpDfc(UpDfcFn, NULL, 1);

TInt Adsp_Load(TInt aOp, TInt aSegment, TUint32 aOffset, const TUint8* aData, TUint32 aLen)
	{
	TInt r;
	switch (aOp)
		{
	case 0:
		if (Step || UpBusy)
			return KErrInUse;									// a failed or running generation must be stopped before another load
		Step = 1;
		Error = 0;
		if ((r = Qmp_Open()) != KErrNone)
			return Fail(r, "qmp open");
		r = Qmp_Send("{class: image, res: load_state, name: adsp, val: on}");
		Kern::Printf("[adsp] load_state on -> %d", r);
		if ((r = Smp2p_Start(KSmemHostAdsp)) != KErrNone)
			return Fail(r, "smp2p");
		if ((r = Pas_Begin(KPasAdsp, aData, aLen, KAdspPhys, KAdspSize)) != KErrNone)
			return Fail(r, "pas begin");
		return KErrNone;
	case 1:
		if (Step != 1 || Error)
			return KErrNotReady;
		r = Pas_Segment(aSegment, aOffset, aData, aLen);
		if (r != KErrNone)
			{
			Pas_Abort();
			return Fail(r, "pas segment");
			}
		return KErrNone;
	case 2:
		if (Step != 1 || Error)
			return KErrNotReady;
		if ((r = Pas_End()) != KErrNone)
			return Fail(r, "auth_and_reset");
		Step = 2;
		if (!Queue)
			{
			if ((r = Kern::DynamicDfcQCreate(Queue, 24, _L8("AdspBoot"))) != KErrNone)
				return Fail(r, "thread");
			UpDfc.SetDfcQ(Queue);
			}
		UpBusy = ETrue;
		UpDfc.Enque();
		return KErrNone;
		}
	return KErrArgument;
	}

void Adsp_State(TInt& aStep, TInt& aError)
	{
	aStep = Step;
	aError = Error;
	}

void Adsp_Report(void (*aSay)(const char*, ...))
	{
	TUint32 v = 0;
	if (Step)
		Smp2p_In(KSmemHostAdsp, "slave-kernel", v);
	aSay("adsp: step %d error %d slave-kernel %08x", Step, Error, v);
	}

static TA64DevState AdspStatus(TDes8& aDetail)
	{
	static const char* const KSteps[] = { "not started (MODEMD loads it)", "loading", "authenticated", "ready, GLINK up",
		"running" };
	A64Device_Format(aDetail, "%s", KSteps[Step >= 0 && Step <= 4 ? Step : 0]);
	if (Error)
		{
		A64Device_Format(aDetail, "; failed: %d", Error);
		return EA64DevError;
		}
	return Step == 0 ? EA64DevOff : Step == 4 ? EA64DevOk : EA64DevStarting;
	}
A64_DEVICE("adsp", "Audio DSP (ADSP)", "qcom_adsp", AdspStatus);

TInt Adsp_Stop()
	{
	if (UpBusy)
		return KErrInUse;
	Glink_Stop(KSmemHostAdsp);
	TInt r = Step ? Pas_Shutdown(KPasAdsp) : KErrNone;
	if (r != KErrNone)
		return Error = r; // firmware and DMA memory remain quarantined
	Smp2p_Reset(KSmemHostAdsp);
	Qmp_Send("{class: image, res: load_state, name: adsp, val: off}");
	Step = 0;
	Error = 0;
	return KErrNone;
	}
