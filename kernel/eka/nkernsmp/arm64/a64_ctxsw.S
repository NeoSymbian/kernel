/* SPDX-License-Identifier: Apache-2.0
 * Derived from Phorepoc adaptation-x86-64 port/x64_ctxsw.S (Copyright The Phorepoc Authors, Apache-2.0).
 *
 * a64_ctxsw.S -- AArch64 context switch primitives for the SMP reschedule (same contract as the amd64 file).
 * A parked context is a 96-byte frame of the AAPCS64 callee-saved registers on the thread's own stack:
 *     [sp+ 0] x19 x20   [sp+16] x21 x22   [sp+32] x23 x24   [sp+48] x25 x26   [sp+64] x27 x28   [sp+80] x29 x30
 * and NThreadBase::iSavedSP (a 4-byte slot under ILP32) holds that sp. The kernel is built -mgeneral-regs-only,
 * so there is no kernel FP/SIMD state to save here (an AArch32 user's VFP state is saved by the exception code).
 */
	.text

.macro	PARK
	sub	sp, sp, #96
	stp	x19, x20, [sp, #0]
	stp	x21, x22, [sp, #16]
	stp	x23, x24, [sp, #32]
	stp	x25, x26, [sp, #48]
	stp	x27, x28, [sp, #64]
	stp	x29, x30, [sp, #80]
.endm

.macro	UNPARK
	ldp	x19, x20, [sp, #0]
	ldp	x21, x22, [sp, #16]
	ldp	x23, x24, [sp, #32]
	ldp	x25, x26, [sp, #48]
	ldp	x27, x28, [sp, #64]
	ldp	x29, x30, [sp, #80]
	add	sp, sp, #96
.endm

/* void __phore_switch(void** aOldSPSlot, void* aNewSP);  x0, x1 */
	.global	__phore_switch
	.type	__phore_switch, %function
__phore_switch:
	PARK
	mov	x2, sp
	str	w2, [x0]		/* *aOldSPSlot = sp (32-bit slot) */
	mov	w1, w1			/* zero-extend the incoming 32-bit sp */
	mov	sp, x1
	UNPARK
	ret
	.size	__phore_switch, . - __phore_switch

/* void __phore_resched_park(void** aOldSPSlot, void* const* aTempSPSlot, void (*aFn)(void*), void* aArg);
 * x0, x1, x2, x3. Park as __phore_switch, then move to the temporary stack *aTempSPSlot -- read AFTER the park,
 * since for a cpu's initial thread both slots are the same -- and call aFn(aArg), which never returns. */
	.global	__phore_resched_park
	.type	__phore_resched_park, %function
__phore_resched_park:
	PARK
	mov	x4, sp
	str	w4, [x0]
	ldr	w4, [x1]		/* zero-extends */
	and	x4, x4, #~15
	mov	sp, x4
	mov	x0, x3
	blr	x2
	brk	#0xdead
	.size	__phore_resched_park, . - __phore_resched_park

/* void __phore_resume(void* aNewSP);  x0. The second half of __phore_switch. */
	.global	__phore_resume
	.type	__phore_resume, %function
__phore_resume:
	mov	w0, w0
	mov	sp, x0
	UNPARK
	ret
	.size	__phore_resume, . - __phore_resume

/* First code a freshly primed thread runs: NThread::Create puts the parameter block in x19, the thread
 * function in x20 and this address in x30, so UNPARK's ret lands here. */
	.global	__phore_thread_start
	.type	__phore_thread_start, %function
__phore_thread_start:
	mov	x0, x19
	mov	x1, x20
	bl	A64_ThreadStart
1:	wfe
	b	1b
	.size	__phore_thread_start, . - __phore_thread_start

	.section .note.GNU-stack, "", %progbits
