/* SPDX-License-Identifier: EPL-1.0
 *
 * arm64_prelude.h -- force-included (after port_prelude.h) for every kernel-side TU of the AArch64 ILP32 port.
 *
 * EKA2's nk_cpu.h has an architecture section for __CPU_ARM and one for __CPU_X86, and nothing for a 64-bit ARM
 * kernel. Rather than edit the baseline, this prelude supplies the AArch64 equivalent of the x86 section (the
 * capability flags and the spin-wait hints). __CPU_ARM stays undefined: the ARM section is AArch32 asm.
 */
#ifndef ARM64_PRELUDE_H
#define ARM64_PRELUDE_H

#if !defined(__aarch64__) || !defined(__ILP32__)
#error "the arm64 kernel port is built with aarch64 GCC -mabi=ilp32"
#endif

#define __CPU_ARM64

#define __CPU_HAS_MMU
#define __CPU_HAS_CACHE
#define __CPU_SUPPORTS_FAST_PROCESS_SWITCH	/* ASIDs: no TLB flush on process switch */
#define __CPU_PAGE_TABLES_FULLY_CACHED		/* VMSAv8 walks are cache-coherent (TCR_EL1 IRGN/ORGN) */

/* spin-wait hints (nk_cpu.h x86 section: pause) */
#define __chill()	__asm__ __volatile__("yield" ::: "memory")
#define __snooze()	__chill()
#define __holler()

/* deliberate crash (cpudefs.h defines it per architecture): a BRK the exception vector reports as a fault */
#define __crash()	__asm__ __volatile__("brk #0xdeff" ::: "memory")

#endif /* ARM64_PRELUDE_H */
