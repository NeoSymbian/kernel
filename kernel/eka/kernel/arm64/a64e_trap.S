/* SPDX-License-Identifier: EPL-1.0
 *
 * a64e_trap.S -- the setjmp-style exception traps and the "magic" fault-tolerant accessors of the ARM kernel
 * (kernel/arm/ckernel.cia, csched.cia) for AArch64. The C++ members (TExcTrap::Trap, ...) are naked forwarders in
 * a64e_exc.cpp that branch here with their arguments untouched, so no mangled names are needed.
 *
 * Trap context (TExcTrap/TPagingExcTrap::iState, EXC_TRAP_CTX_SZ = 26 words, 4-byte aligned under ILP32; SCTLR.A=0
 * so 8-byte stores there are fine): x19-x30 at 0..95, sp at 96.
 */
	.text

	.macro	SAVE_CTX reg
	stp		x19, x20, [\reg, #0]
	stp		x21, x22, [\reg, #16]
	stp		x23, x24, [\reg, #32]
	stp		x25, x26, [\reg, #48]
	stp		x27, x28, [\reg, #64]
	stp		x29, x30, [\reg, #80]
	mov		x9, sp
	str		x9, [\reg, #96]
	.endm

	.macro	RESTORE_CTX reg
	ldp		x19, x20, [\reg, #0]
	ldp		x21, x22, [\reg, #16]
	ldp		x23, x24, [\reg, #32]
	ldp		x25, x26, [\reg, #48]
	ldp		x27, x28, [\reg, #64]
	ldp		x29, x30, [\reg, #80]
	ldr		x9, [\reg, #96]
	mov		sp, x9
	.endm

/* Trap(...): save the caller's callee-saved context, then tail-call the C++ bookkeeping, which returns 0 to the
 * caller of Trap (x30 is untouched). x0 = this, x1 = handler / client thread. */
	.global	A64_TrapSaveExc
A64_TrapSaveExc:
	mov		w0, w0			// ILP32: a pointer argument's upper half is not defined
	SAVE_CTX x0
	b		A64TrapSetExc			// TInt A64TrapSetExc(TExcTrap*, TExcTrapHandler)

	.global	A64_TrapSaveIpc
A64_TrapSaveIpc:
	mov		w0, w0			// ILP32: a pointer argument's upper half is not defined
	SAVE_CTX x0
	b		A64TrapSetIpc			// TInt A64TrapSetIpc(TIpcExcTrap*, DThread*)

	.global	A64_TrapSavePaging
A64_TrapSavePaging:
	mov		w0, w0			// ILP32: a pointer argument's upper half is not defined
	SAVE_CTX x0
	b		A64TrapSetPaging		// TInt A64TrapSetPaging(TPagingExcTrap*)

/* Exception(r): unhook the trap from its thread (C++), restore the context Trap saved and return r from Trap.
 * x0 = this, w1 = r. Called from inside Exc::Dispatch (the exception shim's stack frame is abandoned). */
	.global	A64_TrapRestoreExc
A64_TrapRestoreExc:
	mov		w19, w0
	mov		w20, w1
	bl		A64TrapClearExc			// void A64TrapClearExc(TExcTrap*)
	mov		x0, x19
	mov		w1, w20
	RESTORE_CTX x0
	mov		w0, w1
	ret

	.global	A64_TrapRestorePaging
A64_TrapRestorePaging:
	mov		w19, w0
	mov		w20, w1
	bl		A64TrapClearPaging		// void A64TrapClearPaging(TPagingExcTrap*)
	mov		x0, x19
	mov		w1, w20
	RESTORE_CTX x0
	mov		w0, w1
	ret

/* ---- magic accessors ---------------------------------------------------------------------------------------
 * Every load/store between A64_MagicStart and A64_MagicEnd may fault on purpose. Exc::IsMagic() recognises the
 * PC, and Exc::Dispatch (kernel/arm/ckernel.cpp) skips the instruction, sets Z and puts the fault address in r12
 * (x12), exactly ARM's contract. Z is cleared before each access (cmp sp, #0: sp is never 0), so b.eq after it
 * means "that access faulted". Words; the size is a non-zero multiple of 4 (ARM's contract).
 * All return 0 on success or the address that faulted.
 *   A64_SafeReadKern(src, dest, size)   privileged loads   (Kern::SafeRead)
 *   A64_SafeReadUser(src, dest, size)   unprivileged LDTR  (K::USafeRead)
 *   A64_SafeWriteUser(dest, src, size)  unprivileged STTR  (K::USafeWrite)
 *   A64_SafeWriteKern(dest, src, size)  privileged stores  (Kern::KUSafeWrite from kernel code)
 */
	.global	A64_MagicStart
A64_MagicStart:

	.global	A64_SafeReadKern
A64_SafeReadKern:
	mov		w0, w0
	mov		w1, w1
1:	cmp		sp, #0
	ldr		w3, [x0]
	b.eq	9f
	str		w3, [x1], #4
	add		x0, x0, #4
	subs	w2, w2, #4
	b.gt	1b
	mov		x0, xzr
	ret
9:	mov		x0, x12
	ret

	.global	A64_SafeReadUser
A64_SafeReadUser:
	mov		w0, w0
	mov		w1, w1
1:	cmp		sp, #0
	ldtr	w3, [x0]
	b.eq	9f
	str		w3, [x1], #4
	add		x0, x0, #4
	subs	w2, w2, #4
	b.gt	1b
	mov		x0, xzr
	ret
9:	mov		x0, x12
	ret

	.global	A64_SafeWriteUser
A64_SafeWriteUser:
	mov		w0, w0
	mov		w1, w1
1:	ldr		w3, [x1], #4
	cmp		sp, #0
	sttr	w3, [x0]
	b.eq	9f
	add		x0, x0, #4
	subs	w2, w2, #4
	b.gt	1b
	mov		x0, xzr
	ret
9:	mov		x0, x12
	ret

	.global	A64_SafeWriteKern
A64_SafeWriteKern:
	mov		w0, w0
	mov		w1, w1
1:	ldr		w3, [x1], #4
	cmp		sp, #0
	str		w3, [x0]
	b.eq	9f
	add		x0, x0, #4
	subs	w2, w2, #4
	b.gt	1b
	mov		x0, xzr
	ret
9:	mov		x0, x12
	ret

	.global	A64_MagicEnd
A64_MagicEnd:
