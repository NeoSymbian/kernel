// SPDX-License-Identifier: Apache-2.0
// board.h -- what the kernel-side code needs from a board (QEMU virt, Pixel 3a XL bonito).
#ifndef A64_BOARD_H
#define A64_BOARD_H
extern "C" {
// console
void A64_DebugPutc(char c);
void A64_DebugPuts(const char* s);
void A64_DebugHex(unsigned long long v);
// one-line status (current test, output lines, failures, seconds); boards without a screen ignore it
void A64_ConsoleStatus(const char* aTest, unsigned aLines, unsigned aFails, unsigned aSecs);
// pin a line (a failure) on screen so it never scrolls away; aSlot 0..7
void A64_ConsolePin(int aSlot, const char* aText);
// MMU: the boot CPU builds and enables the map; secondary CPUs enable the same one
void A64_MmuInit();
void A64_MmuInitAp();
// board constants
extern const char* const A64_BoardName;
extern const unsigned long A64_GicdBase, A64_GicrBase;		// GICv3 distributor, first redistributor
extern const unsigned long A64_HeapBase, A64_HeapSize;		// RAM for the nanokernel test heap
extern const int A64_HoldResults;							// 1: keep the final screen until the user powers off
extern const int A64_PsciSmc;								// PSCI conduit: 1 = SMC, 0 = HVC
extern const unsigned int A64_TimerPpi;					// EL1 virtual timer INTID (DTB arm,armv8-timer, 3rd entry + 16)
unsigned int A64_CpuMpidr(int aCpu);						// PSCI target_cpu affinity of logical CPU aCpu
int A64_CpuCount();										// CPUs present (platform layer, PSCI AFFINITY_INFO)

// Device memory. Board code reaches its registers through A64_Io(physical address). The nanokernel test image runs
// identity-mapped (A64_IoWindow = 0: A64_Io returns its argument). The full kernel's bootstrap maps the regions below
// back to back from KPrimaryIOBase (sizes are page multiples), and _E32Startup sets A64_IoWindow to that base.
struct SA64IoRegion { unsigned long iPhys, iSize, iAttr; };	// iAttr: MAIR index (1 Device-nGnRE, 2 Normal NC)
extern const SA64IoRegion A64_IoRegions[];
extern const int A64_IoRegionCount;
extern unsigned long A64_IoWindow;
unsigned long A64_Io(unsigned long aPhys);
}
#endif
