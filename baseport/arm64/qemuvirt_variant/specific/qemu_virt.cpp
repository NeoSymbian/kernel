// SPDX-License-Identifier: Apache-2.0
// qemu_virt.cpp -- QEMU "virt" board (GICv3, Cortex-A55, -m 512): PL011 console, 1 GB identity map, constants.
#include "board.h"
typedef unsigned int TUint32;
typedef unsigned long long TUint64;

#define SYSREG(r)		({ TUint64 _v; __asm__ __volatile__("mrs %0, " #r : "=r"(_v)); _v; })
#define SYSREG_W(r, v)	__asm__ __volatile__("msr " #r ", %0" :: "r"((TUint64)(v)) : "memory")

extern "C" {
const char* const A64_BoardName = "QEMU virt";
const unsigned long A64_GicdBase = 0x08000000, A64_GicrBase = 0x080A0000;
const unsigned long A64_HeapBase = 0x41000000, A64_HeapSize = 0x06000000;	// below the DTB QEMU puts at 0x48000000
const int A64_PsciSmc = 0;
const int A64_HoldResults = 0;		// serial log: power off (QEMU exits) at once
const unsigned int A64_TimerPpi = 27;
unsigned int A64_CpuMpidr(int aCpu) { return (unsigned int)aCpu; }			// Aff0 = CPU number
int A64_CpuCount() { return 1; }	// shortcut: one CPU until the full kernel's BootAP exists (M4.2 step 6), then from the DTB
}

static volatile TUint32* const KUart = (volatile TUint32*)0x09000000;	// PL011 UARTDR, UARTFR at +0x18

extern "C" void A64_DebugPutc(char c)
	{
	if (c == '\n') A64_DebugPutc('\r');
	while (KUart[0x18 / 4] & (1u << 5)) {}	// TXFF
	KUart[0] = (TUint32)c;
	}

/* VA = PA, 1 GB blocks. 0-1 GB devices (GIC, PL011): Device-nGnRnE, XN. 1-4 GB RAM: Normal WB, inner shareable.
 * shortcut: one flat map for the nanokernel tests; the EKA2 memory model (port-map 2C) replaces it. */
static TUint64 gL1[512] __attribute__((aligned(4096)));

static void MmuEnable()
	{
	SYSREG_W(mair_el1, 0xFF00ull);			// attr0 = Device-nGnRnE, attr1 = Normal WB RA/WA
	SYSREG_W(tcr_el1, 32ull | (1ull << 8) | (1ull << 10) | (3ull << 12) | (1ull << 23));	// T0SZ 32, WB, ISH, EPD1
	SYSREG_W(ttbr0_el1, (TUint64)(unsigned long)gL1);
	__asm__ __volatile__("dsb ish\n\ttlbi vmalle1\n\tdsb ish\n\tisb" ::: "memory");
	SYSREG_W(sctlr_el1, SYSREG(sctlr_el1) | 1 | (1 << 2) | (1 << 12));	// M, C, I (A stays 0)
	__asm__ __volatile__("isb" ::: "memory");
	}

extern "C" void A64_MmuInit()
	{
	const TUint64 KBlock = 1, KAf = 1ull << 10, KInnerSh = 3ull << 8, KXn = (1ull << 54) | (1ull << 53);
	gL1[0] = KBlock | KAf | (0ull << 2) | KXn;
	for (TUint64 i = 1; i < 4; ++i)
		gL1[i] = (i << 30) | KBlock | KAf | KInnerSh | (1ull << 2);
	MmuEnable();
	}

extern "C" void A64_MmuInitAp() { MmuEnable(); }
