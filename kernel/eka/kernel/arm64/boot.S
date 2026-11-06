/* SPDX-License-Identifier: Apache-2.0
 *
 * boot.S -- entry of the AArch64 EKA2 kernel image. An arm64 "Image" (the Linux boot protocol), so the same
 * file boots on QEMU virt (-kernel) and through the Pixel 3a XL's ABL (text_offset 0x80000 is what ABL wants).
 * In: x0 = DTB physical address, MMU off, caches off, at EL1 (ABL, QEMU virt) or EL2 (QEMU virtualization=on).
 * Step 1 runs at its link address (no MMU yet): QEMU virt loads it at 0x40080000.
 */
	.section .text.head, "ax"
	.global	_start
_start:
	b	entry				// code0
	.long	0				// code1
	.quad	0x80000				// text_offset
	.long	__image_size			// image_size incl. .bss, from the linker script (ILP32 objects have no 64-bit relocs)
	.long	0
	.quad	0xa				// flags: LE, 4K pages, load anywhere
	.quad	0, 0, 0
	.ascii	"ARM\x64"
	.long	0

entry:
	mov	x19, x0				// keep the DTB pointer
	mrs	x1, CurrentEL
	lsr	x1, x1, #2
	cmp	x1, #2
	b.ne	at_el1
	/* EL2: run EL1 in AArch64, no traps, EL1 gets the physical counter/timer, then drop to EL1h */
	mov	x1, #(1 << 31)			// HCR_EL2.RW: EL1 is AArch64
	msr	hcr_el2, x1
	mov	x1, #3
	msr	cnthctl_el2, x1
	msr	cntvoff_el2, xzr
	mov	x1, #0x33ff
	msr	cptr_el2, x1			// no FP/SIMD traps to EL2
	msr	hstr_el2, xzr
	mov	x1, #0x3c5			// EL1h, DAIF masked
	msr	spsr_el2, x1
	adr	x1, at_el1
	msr	elr_el2, x1
	eret
at_el1:
	msr	daifset, #0xf
	msr	tpidr_el1, xzr			// no TSubScheduler yet (UNKNOWN at reset on real CPUs)
	adr	x1, _start			// running where we were linked? (no relocation: absolute addresses in C++)
	ldr	w2, =_start
	cmp	x1, x2
	b.ne	load_mismatch
	adrp	x1, __boot_stack_top
	add	x1, x1, :lo12:__boot_stack_top
	mov	sp, x1
	adrp	x1, __bss_start			// zero .bss (PC-relative: no 64-bit literals under ILP32)
	add	x1, x1, :lo12:__bss_start
	adrp	x2, __bss_end
	add	x2, x2, :lo12:__bss_end
1:	cmp	x1, x2
	b.hs	2f
	str	xzr, [x1], #8
	b	1b
2:	adrp	x1, A64_Vectors
	add	x1, x1, :lo12:A64_Vectors
	msr	vbar_el1, x1
	isb
	mov	x0, x19
	bl	A64_BootMain			// never returns
3:	wfe
	b	3b

load_mismatch:
#ifdef BOARD_BONITO
	/* no console yet: paint the boot framebuffer magenta so a wrong load address is visible on the phone */
	movz	x1, #0x9c00, lsl #16
	movz	w2, #0xff00
	movk	w2, #0xffff, lsl #16		// 0xffff00ff = magenta (a8r8g8b8)
	movz	x3, #0x6200
	movk	x3, #0x8e, lsl #16		// 4320 * 2160 bytes
4:	str	w2, [x1], #4
	subs	x3, x3, #4
	b.ne	4b
	dsb	sy
#endif
5:	wfe
	b	5b
	.ltorg

#ifndef A64_EKERN
/* Secondary CPU entry (PSCI CPU_ON target). In: x0 = context = SAPBootInfo* (identity-mapped), MMU off, EL1.
 * Stack = iInitStackBase + iInitStackSize; same vectors and MMU map as the boot CPU; then A64_ApMain(info). */
	.text
	.global	A64_ApEntry
A64_ApEntry:
	msr	daifset, #0xf
	msr	tpidr_el1, xzr
	mov	x19, x0
	ldr	w1, [x19, #8]			// iInitStackBase
	ldr	w2, [x19, #4]			// iInitStackSize
	add	x1, x1, x2
	and	x1, x1, #~15
	mov	sp, x1
	adrp	x1, A64_Vectors
	add	x1, x1, :lo12:A64_Vectors
	msr	vbar_el1, x1
	isb
	bl	A64_MmuInitAp
	mov	x0, x19
	bl	A64_ApMain			// never returns
1:	wfe
	b	1b
#else
/* Full kernel: secondary CPU entry (PSCI CPU_ON target, ekern/a64e_nkern.cpp NKern::BootAP). In: x0 = physical
 * address of the SA64ApBoot block {MAIR, TCR, TTBR0, SCTLR, sp, entry, info} (7 x 8 bytes), MMU and caches off, EL1.
 * TTBR0 is a private boot root that also maps this stub 1:1, so the instruction after the MMU switch is mapped; the
 * C++ entry (A64_ApStart, linear) moves to the kernel's own root at once. */
	.text
	.global	A64_ApEntry
A64_ApEntry:
	msr	daifset, #0xf
	msr	tpidr_el1, xzr			// no TSubScheduler until NKern::Init
	ldp	x1, x2, [x0, #0]		// MAIR, TCR
	ldp	x3, x4, [x0, #16]		// TTBR0 (private root), SCTLR
	ldp	x5, x6, [x0, #32]		// sp (linear), entry (linear)
	ldr	x7, [x0, #48]			// SAPBootInfo* (linear)
	msr	mair_el1, x1
	msr	tcr_el1, x2
	msr	ttbr0_el1, x3
	isb
	tlbi	vmalle1
	dsb	nsh
	isb
	msr	pan, #0
	msr	sctlr_el1, x4
	isb
	mov	sp, x5
	mov	x0, x7
	mov	x29, xzr
	mov	x30, xzr
	br	x6
#endif

	.section .bss.stack, "aw", %nobits
	.balign	16
	.space	16384
	.global	__boot_stack_top
__boot_stack_top:
