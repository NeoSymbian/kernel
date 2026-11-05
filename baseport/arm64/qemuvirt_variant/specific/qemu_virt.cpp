// SPDX-License-Identifier: Apache-2.0
// qemu_virt.cpp -- QEMU "virt" board glue for the AArch64 EKA2 port: PL011 debug output, the boot sequence
// (banner, vector self-test, MMU identity map with caches, C++ constructors) and the hand-off to KernelMain.
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

#define SYSREG(r)		({ TUint64 _v; __asm__ __volatile__("mrs %0, " #r : "=r"(_v)); _v; })
#define SYSREG_W(r, v)	__asm__ __volatile__("msr " #r ", %0" :: "r"((TUint64)(v)) : "memory")

/* --- MMU: identity map of the 4 GB the ILP32 kernel can address, 1 GB blocks ------------------------------------
 * VA = PA. 0-1 GB: devices (GIC 0x08000000, PL011 0x09000000), Device-nGnRnE, never executable.
 * 1-4 GB: RAM from 0x40000000, Normal write-back cacheable, inner shareable (atomics need Normal memory).
 * shortcut: one flat map for the nanokernel tests; the EKA2 memory model (port-map 2C) replaces it. */
static TUint64 gL1[512] __attribute__((aligned(4096)));

extern "C" void A64_MmuInit()
	{
	const TUint64 KBlock = 1, KAf = 1ull << 10, KInnerSh = 3ull << 8, KXn = (1ull << 54) | (1ull << 53);
	gL1[0] = 0x00000000ull | KBlock | KAf | (0ull << 2) | KXn;				// AttrIndx 0: device
	for (TUint64 i = 1; i < 4; ++i)
		gL1[i] = (i << 30) | KBlock | KAf | KInnerSh | (1ull << 2);			// AttrIndx 1: normal WB
	SYSREG_W(mair_el1, 0xFF00ull);			// attr0 = Device-nGnRnE, attr1 = Normal WB RA/WA
	SYSREG_W(tcr_el1, 32ull					// T0SZ = 32: 4 GB from TTBR0, walk starts at level 1
					| (1ull << 8) | (1ull << 10) | (3ull << 12)	// IRGN0/ORGN0 WB, SH0 inner
					| (1ull << 23));		// EPD1: no TTBR1 walks
	SYSREG_W(ttbr0_el1, (TUint64)(unsigned long)gL1);
	__asm__ __volatile__("dsb ish\n\ttlbi vmalle1\n\tdsb ish\n\tisb" ::: "memory");
	SYSREG_W(sctlr_el1, SYSREG(sctlr_el1) | 1 | (1 << 2) | (1 << 12));	// M, C, I (A stays 0: unaligned allowed)
	__asm__ __volatile__("isb" ::: "memory");
	}

/* a secondary CPU: the same translation regime the boot CPU built (TTBR0 = gL1, already populated) */
extern "C" void A64_MmuInitAp()
	{
	SYSREG_W(mair_el1, 0xFF00ull);
	SYSREG_W(tcr_el1, 32ull | (1ull << 8) | (1ull << 10) | (3ull << 12) | (1ull << 23));
	SYSREG_W(ttbr0_el1, (TUint64)(unsigned long)gL1);
	__asm__ __volatile__("dsb ish\n\ttlbi vmalle1\n\tdsb ish\n\tisb" ::: "memory");
	SYSREG_W(sctlr_el1, SYSREG(sctlr_el1) | 1 | (1 << 2) | (1 << 12));
	__asm__ __volatile__("isb" ::: "memory");
	}

extern "C" volatile int A64_BrkSeen;		// a64_nksa.cpp exception dispatch counts the self-test BRK
extern "C" void A64_NkSaStart();			// a64_nksa.cpp: ROM/super page stand-ins, then KernelMain()

typedef void (*TCtor)();
extern TCtor __init_array_start[], __init_array_end[];

extern "C" void A64_BootMain(TUint64 aDtb)
	{
	A64_DebugPuts("\nEKA2/arm64: boot ok\n");
	A64_DebugPuts("  EL=");      A64_DebugHex(SYSREG(CurrentEL) >> 2);
	A64_DebugPuts(" MPIDR=");    A64_DebugHex(SYSREG(mpidr_el1));
	A64_DebugPuts(" CNTFRQ=");   A64_DebugHex(SYSREG(cntfrq_el0));
	A64_DebugPuts(" DTB=");      A64_DebugHex(aDtb);
	register TUint64 x21 __asm__("x21") = 0x5151;	// a callee-saved value that must survive the exception
	__asm__ __volatile__("brk #0x1234" : "+r"(x21) :: "memory");
	A64_DebugPuts("\n  vectors: brk ");
	A64_DebugPuts(A64_BrkSeen == 1 && x21 == 0x5151 ? "PASS" : "FAIL");
	A64_MmuInit();
	A64_DebugPuts("\n  mmu: on (identity, caches on)\n");
	for (TCtor* c = __init_array_start; c < __init_array_end; ++c)
		(*c)();
	A64_NkSaStart();
	for (;;) __asm__ __volatile__("wfi");
	}
