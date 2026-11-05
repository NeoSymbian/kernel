// SPDX-License-Identifier: Apache-2.0
// boot_common.cpp -- the board-independent boot sequence: banner, vector self-test, MMU on, C++ constructors,
// then the nanokernel (A64_NkSaStart -> KernelMain). Console and MMU come from the board file.
#include "board.h"
typedef unsigned long long TUint64;

#define SYSREG(r) ({ TUint64 _v; __asm__ __volatile__("mrs %0, " #r : "=r"(_v)); _v; })

extern "C" void A64_DebugPuts(const char* s) { while (*s) A64_DebugPutc(*s++); }
extern "C" void A64_DebugHex(TUint64 v)
	{
	A64_DebugPuts("0x");
	for (int i = 60; i >= 0; i -= 4) A64_DebugPutc("0123456789abcdef"[(v >> i) & 0xF]);
	}

extern "C" volatile int A64_BrkSeen;		// a64_nksa.cpp exception dispatch counts the self-test BRK
extern "C" void A64_NkSaStart();			// a64_nksa.cpp: ROM/super page stand-ins, then KernelMain()

typedef void (*TCtor)();
extern TCtor __init_array_start[], __init_array_end[];
extern char _start[];

extern "C" void A64_BootMain(TUint64 aDtb)
	{
	A64_DebugPuts("\nEKA2/arm64 on "); A64_DebugPuts(A64_BoardName); A64_DebugPuts(": boot ok\n");
	A64_DebugPuts("  EL=");      A64_DebugHex(SYSREG(CurrentEL) >> 2);
	A64_DebugPuts(" MPIDR=");    A64_DebugHex(SYSREG(mpidr_el1));
	A64_DebugPuts("\n  CNTFRQ=");  A64_DebugHex(SYSREG(cntfrq_el0));
	A64_DebugPuts(" DTB=");      A64_DebugHex(aDtb);
	A64_DebugPuts(" image=");    A64_DebugHex((unsigned long)_start);
	register TUint64 x21 __asm__("x21") = 0x5151;	// a callee-saved value that must survive the exception
	__asm__ __volatile__("brk #0x1234" : "+r"(x21) :: "memory");
	A64_DebugPuts("\n  vectors: brk ");
	A64_DebugPuts(A64_BrkSeen == 1 && x21 == 0x5151 ? "PASS" : "FAIL");
	A64_MmuInit();
	A64_DebugPuts("\n  mmu: on\n");
	for (TCtor* c = __init_array_start; c < __init_array_end; ++c)
		(*c)();
	A64_NkSaStart();
	for (;;) __asm__ __volatile__("wfi");
	}
