// SPDX-License-Identifier: Apache-2.0
// board.h -- what the kernel-side code needs from a board (QEMU virt, Pixel 3a XL bonito).
#ifndef A64_BOARD_H
#define A64_BOARD_H
extern "C" {
// console
void A64_DebugPutc(char c);
void A64_DebugPuts(const char* s);
void A64_DebugHex(unsigned long long v);
// MMU: the boot CPU builds and enables the map; secondary CPUs enable the same one
void A64_MmuInit();
void A64_MmuInitAp();
// board constants
extern const char* const A64_BoardName;
extern const unsigned long A64_GicdBase, A64_GicrBase;		// GICv3 distributor, first redistributor
extern const unsigned long A64_HeapBase, A64_HeapSize;		// RAM for the nanokernel test heap
extern const int A64_PsciSmc;								// PSCI conduit: 1 = SMC, 0 = HVC
extern const unsigned int A64_TimerPpi;					// EL1 virtual timer INTID (DTB arm,armv8-timer, 3rd entry + 16)
unsigned int A64_CpuMpidr(int aCpu);						// PSCI target_cpu affinity of logical CPU aCpu
}
#endif
