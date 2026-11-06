@ SPDX-License-Identifier: Apache-2.0
@
@ a32test.S -- the first USER-MODE program of the AArch64 kernel: 32-bit ARM code (ARMv5TE, the 2010 user side's
@ ISA) at AArch32 EL0, started by exstart as the ROM secondary (efile.exe's slot) until the real user side arrives.
@ No euser: the exec calls are hand-rolled SVCs (u32exec.h: "swi n" slow, "swi n|0x800000" fast; r0-r3 arguments,
@ r0 result). It runs XIP from the ROM, position-independent (the build flattens it with objcopy and places it in
@ the kernel image's .utext), on the user stack the kernel made for it. It must never exit: the secondary is
@ system-critical, so it ends in Exec::WaitForAnyRequest for ever.
@ Checks, each printed through Exec::DebugPrint:
@   fast exec (NTickCount), slow exec with a handle lookup (ThreadId of KCurrentThreadHandle), the tick interrupt
@   taken from EL0 (a 300-tick spin with no exec calls in its inner loop), and the AArch32 fix-ups: SWP (UNDEFINED on
@   ARMv8) and a misaligned LDRD (always an alignment fault).
	.syntax	unified
	.arm
	.include "exec_enum.inc"				@ generated from exec_enum.h by build-ekern.sh
	.equ	FAST, 0x800000

	.text
entry:
	tst		pc, #0							@ EKA2_ENTRY_POINT_VERSION_IDENTIFIER
	b		start
	.word	0								@ padding
	.word	0								@ offset 12: the code segment's unique ID (RAM-loaded code)

start:
	mov		r10, #0							@ failure bits
	adrl	r0, s_hello
	bl		print_str

	svc		#(FAST + EFastExecNTickCount)
	mov		r5, r0
	mov		r1, r0
	adrl	r0, s_tick
	bl		print_hex

	ldr		r0, =0xFFFF8001					@ KCurrentThreadHandle: the gate's preprocess turns it into the DThread
	svc		#EExecThreadId
	mov		r1, r0
	adrl	r0, s_tid
	bl		print_hex

	@ spin at EL0: only the tick interrupt, taken from user mode, can move the clock
	mov		r6, #0
1:	ldr		r2, =100000
2:	subs	r2, r2, #1
	bne		2b
	add		r6, r6, #1
	svc		#(FAST + EFastExecNTickCount)
	sub		r1, r0, r5
	cmp		r1, #300
	blo		1b
	adrl	r0, s_spun
	bl		print_hex

	sub		sp, sp, #64
	@ SWP: UNDEFINED on ARMv8, emulated by the kernel
	ldr		r1, =0x11111111
	str		r1, [sp]
	ldr		r2, =0x22222222
	swp		r3, r2, [sp]
	ldr		r4, [sp]
	cmp		r3, r1
	orrne	r10, r10, #1
	cmp		r4, r2
	orrne	r10, r10, #2
	mov		r1, r3
	adrl	r0, s_swp
	bl		print_hex

	@ LDRD from an address that is not word-aligned: an alignment fault on any ARMv8, emulated by the kernel
	ldr		r1, =0x03020100
	str		r1, [sp, #16]
	ldr		r1, =0x07060504
	str		r1, [sp, #20]
	ldr		r1, =0x0b0a0908
	str		r1, [sp, #24]
	add		r3, sp, #17
	ldrd	r0, r1, [r3]
	ldr		r4, =0x04030201
	ldr		r5, =0x08070605
	cmp		r0, r4
	orrne	r10, r10, #4
	cmp		r1, r5
	orrne	r10, r10, #8
	mov		r1, r0
	adrl	r0, s_ldrd
	bl		print_hex
	add		sp, sp, #64

	cmp		r10, #0
	bne		failed
	adrl	r0, s_pass
	bl		print_str
	b		park
failed:
	mov		r1, r10
	adrl	r0, s_fail
	bl		print_hex
park:
	svc		#(FAST + EFastExecWaitForAnyRequest)
	b		park

@ print(r0 = text, r1 = length): Exec::DebugPrint(TPtrC8*, 0); a TPtrC8 is {length | EPtrC << 28, pointer}
print:
	push	{r4, lr}
	sub		sp, sp, #8
	orr		r1, r1, #0x10000000
	str		r1, [sp]
	str		r0, [sp, #4]
	mov		r0, sp
	mov		r1, #0
	svc		#EExecDebugPrint
	add		sp, sp, #8
	pop		{r4, pc}

@ print_str(r0 = NUL-terminated text)
print_str:
	mov		r1, r0
1:	ldrb	r2, [r1], #1
	cmp		r2, #0
	bne		1b
	sub		r1, r1, r0
	sub		r1, r1, #1
	b		print

@ print_hex(r0 = NUL-terminated prefix, r1 = value): "<prefix><8 hex digits>"
print_hex:
	push	{r4, r5, lr}
	sub		sp, sp, #96
	mov		r4, sp
1:	ldrb	r2, [r0], #1
	cmp		r2, #0
	strbne	r2, [r4], #1
	bne		1b
	mov		r5, #28
2:	mov		r2, r1, lsr r5
	and		r2, r2, #15
	cmp		r2, #10
	addlo	r2, r2, #'0'
	addhs	r2, r2, #('a' - 10)
	strb	r2, [r4], #1
	subs	r5, r5, #4
	bge		2b
	mov		r0, sp
	sub		r1, r4, sp
	bl		print
	add		sp, sp, #96
	pop		{r4, r5, pc}

	.ltorg
s_hello:	.asciz	"[user] hello from AArch32 EL0"
s_tick:		.asciz	"[user] fast exec NTickCount = "
s_tid:		.asciz	"[user] slow exec ThreadId(current) = "
s_spun:		.asciz	"[user] spun at EL0, ticks = "
s_swp:		.asciz	"[user] SWP (emulated) old value = "
s_ldrd:		.asciz	"[user] misaligned LDRD (emulated) low word = "
s_pass:		.asciz	"[user] PASS: exec gate, EL0 preemption, SWP and LDRD fix-ups"
s_fail:		.asciz	"[user] FAIL, check bits = "
	.balign	4
