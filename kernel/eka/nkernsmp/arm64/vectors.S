/* SPDX-License-Identifier: Apache-2.0
 *
 * vectors.S -- VBAR_EL1 table. Every entry saves an SThreadExcStack-shaped frame (x0-x30, sp_el0, elr, spsr,
 * esr, far, type) and calls A64_ExceptionDispatch(frame, kind) in C++; on return the frame is restored and the
 * exception returns. kind = (group << 2) | type: group 0 current EL SP0, 1 current EL SPx, 2 lower EL AArch64,
 * 3 lower EL AArch32; type 0 sync, 1 IRQ, 2 FIQ, 3 SError. The frame layout must match nk_plat.h SThreadExcStack.
 */
#define FRAME_SIZE	(38 * 8)

.macro	ENTRY kind
	.balign	128
	sub	sp, sp, #FRAME_SIZE
	stp	x0, x1, [sp, #0]
	mov	x1, #\kind
	b	exc_common
.endm

	.text
	.balign	2048
	.global	A64_Vectors
A64_Vectors:
	ENTRY 0
	ENTRY 1
	ENTRY 2
	ENTRY 3
	ENTRY 4
	ENTRY 5
	ENTRY 6
	ENTRY 7
	ENTRY 8
	ENTRY 9
	ENTRY 10
	ENTRY 11
	ENTRY 12
	ENTRY 13
	ENTRY 14
	ENTRY 15

exc_common:
	stp	x2, x3, [sp, #16]
	stp	x4, x5, [sp, #32]
	stp	x6, x7, [sp, #48]
	stp	x8, x9, [sp, #64]
	stp	x10, x11, [sp, #80]
	stp	x12, x13, [sp, #96]
	stp	x14, x15, [sp, #112]
	stp	x16, x17, [sp, #128]
	stp	x18, x19, [sp, #144]
	stp	x20, x21, [sp, #160]
	stp	x22, x23, [sp, #176]
	stp	x24, x25, [sp, #192]
	stp	x26, x27, [sp, #208]
	stp	x28, x29, [sp, #224]
	mrs	x2, sp_el0
	stp	x30, x2, [sp, #240]		// iX[30], iSpEl0
	mrs	x2, elr_el1
	mrs	x3, spsr_el1
	stp	x2, x3, [sp, #256]		// iElr, iSpsr
	mrs	x2, esr_el1
	mrs	x3, far_el1
	stp	x2, x3, [sp, #272]		// iEsr, iFar
	stp	x1, xzr, [sp, #288]		// iExcCode = kind, iPad
	mov	x0, sp
	bl	A64_ExceptionDispatch
	ldp	x2, x3, [sp, #256]
	msr	elr_el1, x2
	msr	spsr_el1, x3
	ldr	x2, [sp, #248]
	msr	sp_el0, x2
	ldp	x0, x1, [sp, #0]
	ldp	x2, x3, [sp, #16]
	ldp	x4, x5, [sp, #32]
	ldp	x6, x7, [sp, #48]
	ldp	x8, x9, [sp, #64]
	ldp	x10, x11, [sp, #80]
	ldp	x12, x13, [sp, #96]
	ldp	x14, x15, [sp, #112]
	ldp	x16, x17, [sp, #128]
	ldp	x18, x19, [sp, #144]
	ldp	x20, x21, [sp, #160]
	ldp	x22, x23, [sp, #176]
	ldp	x24, x25, [sp, #192]
	ldp	x26, x27, [sp, #208]
	ldp	x28, x29, [sp, #224]
	ldr	x30, [sp, #240]
	add	sp, sp, #FRAME_SIZE
	eret
