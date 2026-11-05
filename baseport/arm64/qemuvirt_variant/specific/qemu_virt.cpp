// SPDX-License-Identifier: Apache-2.0
// qemu_virt.cpp -- QEMU "virt" board glue for the AArch64 EKA2 port: PL011 debug output, boot banner and the
// exception dispatcher. Step 1: prove entry, stack, .bss, VBAR and the exception frame round trip.
typedef unsigned int TUint32;
typedef unsigned long long TUint64;

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

extern "C" void A64_ExceptionDispatch(SFrame* aF, TUint64 aKind)
	{
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
	for (;;) __asm__ __volatile__("wfi");
	}
