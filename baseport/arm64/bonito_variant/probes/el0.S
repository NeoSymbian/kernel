// Hybrid probe: AArch64 EL1 "kernel" drops to AArch32 EL0 (user mode) and the 32-bit code paints blue.
// Blue = the CPU runs 32-bit user code under a 64-bit kernel, so the 32-bit Symbian user side could run
// unchanged under an AArch64 EKA2. Red = an exception came back to EL1 (e.g. AArch32 EL0 unsupported).
#include "fb.h"
#define COLOR_FAIL 0xffff0000	// red

	.text
	.globl _start
_start:
	b	code			// code0
	.long	0			// code1
	.quad	0x80000			// text_offset: Pixel ABL rejects 0
	.quad	_end - _start		// image_size
	.quad	0xa			// flags: little endian, 4K pages, load anywhere
	.quad	0, 0, 0			// res2-res4
	.ascii	"ARM\x64"		// magic
	.long	0			// res5

code:
	adr	x0, vectors
	msr	vbar_el1, x0
	adr	x0, user32
	msr	elr_el1, x0
	mov	x0, #0x1d0		// SPSR: AArch32 (M[4]=1), USR mode, ARM state, A/I/F masked
	msr	spsr_el1, x0
	isb
	eret

fail:	// any exception taken to EL1: paint red and stop
	movz	x1, #(FB_BASE >> 16), lsl #16
	movz	w2, #(COLOR_FAIL & 0xffff)
	movk	w2, #(COLOR_FAIL >> 16), lsl #16
	movz	x3, #(FB_BYTES & 0xffff)
	movk	x3, #(FB_BYTES >> 16), lsl #16
1:	str	w2, [x1], #4
	subs	x3, x3, #4
	b.ne	1b
	dsb	sy
2:	wfe
	b	2b

	.balign	2048
vectors:
	.rept	16
	b	fail
	.balign	128
	.endr

	.balign	4
user32:
	.incbin	"out/el0_a32.bin"
_end:
