// SPDX-License-Identifier: Apache-2.0
//
// qcom_cpufreq.cpp -- SDM670 CPU and L3 clocks through the OSM blocks ABL leaves enabled (DTB cpufreq@17d43000
// "qcom,sdm670-cpufreq-hw" freq-domain0 silver / freq-domain1 gold, interconnect@17d41000 "qcom,sdm670-osm-l3").
// Each block holds a table of performance steps (LUT) and takes the step to run at in PERF_STATE; the OSM firmware
// sets the clock and voltage. ABL leaves silver at its top step (1708.8 MHz), gold at its bottom (300 MHz) and L3 at
// step 5 of 10 (smooth-ui S3b, monitor `cpus`).
// Touch boost (smooth-ui S3c): every input event (Kern::AddEvent -> A64CpuBoost) puts all three at their top step;
// 3 s after the last one they go back to the steps ABL left.
// Written from the register layout in Linux drivers/cpufreq/qcom-cpufreq-hw.c (sdm845: LUT at +0x110, 32-byte
// rows, PERF_STATE at +0x920) and drivers/interconnect/qcom/osm-l3.c, checked against the phone with `peek`.
// shortcut: no governor and no thermal driver (Qualcomm's LMh limits clocks in hardware at 95 C); a power policy
// comes with H3.
#include <kernel/kern_priv.h>
#include "board.h"

enum { EEnable = 0x0, ELut = 0x110, ERow = 32, EPerfState = 0x920, EMaxRows = 40 };
const TInt KBoostMs = 3000;

struct SDomain
	{
	const char* iName;
	TUint32 iBase;
	TInt iTop, iIdle;										// LUT rows; iTop < 0: the block is off, left alone
	TInt iTopMHz, iIdleMHz;
	};
static SDomain Domains[] = { { "silver", 0x17d43000, -1, 0, 0, 0 }, { "gold", 0x17d45800, -1, 0, 0, 0 },
	{ "L3", 0x17d41000, -1, 0, 0, 0 } };
const TInt KDomains = sizeof(Domains) / sizeof(Domains[0]);

static volatile TUint32& Reg(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }

// A LUT row's clock: LVAL x 19.2 MHz, or the 300 MHz alternate clock (GPLL0 / 2) when the source field is 0.
static TInt RowMHz(TUint32 aRow) { return (aRow >> 30) ? (TInt)(aRow & 0xff) * 192 / 10 : 300; }
static TInt StepMHz(const SDomain& aD, TInt aStep) { return RowMHz(Reg(aD.iBase + ELut + ERow * aStep)); }

static TBool Ready;
static volatile TUint32 Boosted;

static void SetSteps(TBool aTop)
	{
	for (TInt i = 0; i < KDomains; ++i)
		if (Domains[i].iTop >= 0)
			Reg(Domains[i].iBase + EPerfState) = aTop ? Domains[i].iTop : Domains[i].iIdle;
	}

static void DropFn(TAny*)
	{
	__e32_atomic_store_rel32(&Boosted, 0);
	SetSteps(EFalse);
	}
static NTimer DropTimer(DropFn, NULL);

// Called for every input event: top steps now, ABL's steps 3 s after the last event.
void A64CpuBoost()
	{
	if (!Ready)
		return;
	if (!__e32_atomic_swp_ord32(&Boosted, 1))
		SetSteps(ETrue);
	DropTimer.Cancel();
	DropTimer.OneShot(NKern::TimerTicks(KBoostMs));
	}

void A64CpufreqDump()
	{
	for (TInt i = 0; i < KDomains; ++i)
		{
		const SDomain& d = Domains[i];
		if (d.iTop < 0)
			{
			Kern::Printf("[cpufreq] %s: off", d.iName);
			continue;
			}
		TInt now = (TInt)Reg(d.iBase + EPerfState);
		Kern::Printf("[cpufreq] %s: step %d = %d MHz (top %d = %d MHz, idle %d = %d MHz)%s", d.iName, now,
			StepMHz(d, now), d.iTop, StepMHz(d, d.iTop), d.iIdle, StepMHz(d, d.iIdle), Boosted ? ", boosted" : "");
		}
	}

// The fastest core's top clock, for the HAL's ECPUSpeed; 0 before the LUTs are read.
TUint A64CpuMaxKHz()
	{
	return Ready && Domains[1].iTop >= 0 ? (TUint)StepMHz(Domains[1], Domains[1].iTop) * 1000 : 0;
	}

// The load balancer's weight for each CPU (nk_bal.cpp, smooth-ui S3d): DTB capacity-dmips-mhz (silver 610, gold 1024)
// x the clock it runs at now, 1024 = gold at its top step. Gold boosted 1024, silver 522, gold at ABL's 300 MHz 153: busy
// threads go to gold while the user touches the phone, and stay on silver otherwise.
unsigned int A64CpuCapacity(int aCpu)
	{
	const SDomain& d = Domains[aCpu < 6 ? 0 : 1];			// DTB: cpu@0..500 freq-domain 0, cpu@600..700 freq-domain 1
	const SDomain& g = Domains[1];
	if (!Ready || d.iTop < 0 || g.iTop < 0)
		return 1024;
	TInt mhz = Boosted ? d.iTopMHz : d.iIdleMHz;
	TInt cap = (aCpu < 6 ? 610 : 1024) * mhz / g.iTopMHz;
	return cap < 1 ? 1 : cap > 1024 ? 1024 : (unsigned int)cap;
	}

void A64StartCpufreq()
	{
	for (TInt i = 0; i < KDomains; ++i)
		{
		SDomain& d = Domains[i];
		if (!(Reg(d.iBase + EEnable) & 1))
			{
			Kern::Printf("[cpufreq] %s: OSM off, left alone", d.iName);
			continue;
			}
		TUint32 prev = 0;
		TInt top = 0;
		for (TInt r = 0; r < EMaxRows; ++r)					// the table ends where a row repeats
			{
			TUint32 v = Reg(d.iBase + ELut + ERow * r);
			if (r && v == prev)
				break;
			prev = v;
			top = r;
			}
		d.iIdle = (TInt)Reg(d.iBase + EPerfState);
		d.iTop = top;
		d.iTopMHz = StepMHz(d, top);
		d.iIdleMHz = StepMHz(d, d.iIdle);
		Kern::Printf("[cpufreq] %s: %d steps, %d-%d MHz, ABL left step %d (%d MHz)", d.iName, top + 1, StepMHz(d, 0),
			StepMHz(d, top), d.iIdle, StepMHz(d, d.iIdle));
		}
	Ready = ETrue;
	}
