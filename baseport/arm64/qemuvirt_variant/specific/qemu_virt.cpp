// SPDX-License-Identifier: Apache-2.0
// qemu_virt.cpp -- QEMU "virt" board glue for the AArch64 EKA2 port: PL011 debug output, boot banner and the
// exception dispatcher. Step 1: entry, stack, .bss, VBAR, exception frame round trip. Step 2: GICv3 + timer tick + SGI.
typedef unsigned int TUint32;
typedef unsigned long long TUint64;
#include "gicv3.h"

static const unsigned long KGicd = 0x08000000, KGicr = 0x080A0000;
static const unsigned int KTimerPpi = 27;	// EL1 virtual timer
static const unsigned int KTickHz = 100;

static volatile TUint32* const KUart = (volatile TUint32*)0x09000000;	// PL011 UARTDR, UARTFR at +0x18

extern "C" void A64_DebugPutc(char c)
	{
	if (c == '\n') A64_DebugPutc('\r');
	while (KUart[0x18 / 4] & (1u << 5)) {}	// TXFF
	KUart[0] = (TUint32)c;
	}
extern "C" void A64_DebugPuts(const char* s) { while (*s) A64_DebugPutc(*s++); }
extern "C" void A64_DebugHex(TUint64 v)
	{
	A64_DebugPuts("0x");
	for (int i = 60; i >= 0; i -= 4) A64_DebugPutc("0123456789abcdef"[(v >> i) & 0xF]);
	}

#define SYSREG(r) ({ TUint64 _v; __asm__ __volatile__("mrs %0, " #r : "=r"(_v)); _v; })

struct SFrame { TUint64 iX[31], iSpEl0, iElr, iSpsr, iEsr, iFar, iExcCode, iPad; };	// == nk_plat.h SThreadExcStack
static_assert(sizeof(SFrame) == 38 * 8, "frame layout must match vectors.S");

static volatile int gBrkSeen;
static volatile unsigned gTicks, gSgis;
static TUint64 gTimerReload;

static void TimerArm() { __asm__ __volatile__("msr cntv_tval_el0, %0\n\tmsr cntv_ctl_el0, %1\n\tisb" :: "r"(gTimerReload), "r"(1ull)); }

static void HandleIrq()
	{
	for (;;)
		{
		TUint32 id = Gicv3_Ack();
		if (id >= 1020) return;		// spurious: nothing (more) pending
		if (id == KTimerPpi) { ++gTicks; TimerArm(); }
		else if (id < 16) ++gSgis;
		Gicv3_Eoi(id);
		}
	}

extern "C" void A64_ExceptionDispatch(SFrame* aF, TUint64 aKind)
	{
	if ((aKind & 3) == 1) { HandleIrq(); return; }	// IRQ from any EL
	TUint32 ec = (TUint32)(aF->iEsr >> 26);
	if ((aKind & 3) == 0 && ec == 0x3C)		// sync, BRK from AArch64: the self-test
		{
		++gBrkSeen;
		aF->iElr += 4;				// step over the BRK
		return;
		}
	A64_DebugPuts("\n*** exception kind="); A64_DebugHex(aKind);
	A64_DebugPuts(" esr="); A64_DebugHex(aF->iEsr);
	A64_DebugPuts(" elr="); A64_DebugHex(aF->iElr);
	A64_DebugPuts(" far="); A64_DebugHex(aF->iFar);
	A64_DebugPuts("\n");
	for (;;) __asm__ __volatile__("wfi");
	}

extern "C" void A64_BootMain(TUint64 aDtb)
	{
	A64_DebugPuts("\nEKA2/arm64: boot ok\n");
	A64_DebugPuts("  EL=");      A64_DebugHex(SYSREG(CurrentEL) >> 2);
	A64_DebugPuts(" MPIDR=");    A64_DebugHex(SYSREG(mpidr_el1));
	A64_DebugPuts(" CNTFRQ=");   A64_DebugHex(SYSREG(cntfrq_el0));
	A64_DebugPuts(" DTB=");      A64_DebugHex(aDtb);
	A64_DebugPuts("\n  ptr size="); A64_DebugHex(sizeof(void*));
	register TUint64 x21 __asm__("x21") = 0x5151;	// a callee-saved value that must survive the exception
	__asm__ __volatile__("brk #0x1234" : "+r"(x21) :: "memory");
	A64_DebugPuts("\n  vectors: brk ");
	A64_DebugPuts(gBrkSeen == 1 && x21 == 0x5151 ? "PASS" : "FAIL");
	A64_DebugPuts("\nEKA2/arm64: step 1 done\n");

	// step 2: GICv3 + generic timer + SGI to self
	TUint64 m = SYSREG(mpidr_el1);
	unsigned int aff = (unsigned int)((m & 0xFFFFFF) | ((m >> 8) & 0xFF000000));
	Gicv3_InitDist(KGicd);
	int r = Gicv3_InitCpu(KGicr, aff);
	A64_DebugPuts("  gic: redistributor "); A64_DebugPuts(r == 0 ? "found\n" : "NOT FOUND\n");
	Gicv3_EnablePpiSgi(KTimerPpi, 0xA0);
	Gicv3_EnablePpiSgi(1, 0x80);
	gTimerReload = SYSREG(cntfrq_el0) / KTickHz;
	TUint64 t0 = SYSREG(cntvct_el0);
	TimerArm();
	__asm__ __volatile__("msr daifclr, #2" ::: "memory");	// IRQs on
	while (gTicks < 50) __asm__ __volatile__("wfi");
	TUint64 ms = (SYSREG(cntvct_el0) - t0) * 1000 / SYSREG(cntfrq_el0);
	A64_DebugPuts("  timer: 50 ticks in "); A64_DebugHex(ms); A64_DebugPuts(" ms ");
	A64_DebugPuts(ms >= 400 && ms <= 650 ? "PASS\n" : "FAIL\n");
	__asm__ __volatile__("dsb ishst\n\tmsr S3_0_C12_C11_5, %0\n\tisb" :: "r"((1ull << 24) | 1ull) : "memory");	// ICC_SGI1R: SGI 1 -> Aff0 0 (self)
	for (int i = 0; i < 1000000 && !gSgis; ++i) __asm__ __volatile__("yield");
	A64_DebugPuts("  sgi: self IPI "); A64_DebugPuts(gSgis == 1 ? "PASS\n" : "FAIL\n");
	A64_DebugPuts("EKA2/arm64: step 2 done\n");
	for (;;) __asm__ __volatile__("wfi");
	}
