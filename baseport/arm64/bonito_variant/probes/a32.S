// AArch32 probe: ARM zImage that paints the bootloader's framebuffer blue and halts.
// If the screen turns blue, ABL switched the CPU to AArch32 for us, so Phorepoc's existing
// 32-bit ARM kernel port can be the base and no AArch64 EKA2 port is needed.
#include "fb.h"

	.arm
	.text
	.globl _start
_start:
	.rept	8
	mov	r0, r0
	.endr
	b	code			// 0x20
	.word	0x016f2818		// 0x24 zImage magic
	.word	0			// 0x28 start
	.word	_end - _start		// 0x2c end

code:
	mov	r1, #FB_BASE
	movw	r2, #(COLOR_A32 & 0xffff)
	movt	r2, #(COLOR_A32 >> 16)
	movw	r3, #(FB_BYTES & 0xffff)
	movt	r3, #(FB_BYTES >> 16)
1:	str	r2, [r1], #4
	subs	r3, r3, #4
	bne	1b
	dsb
2:	wfe
	b	2b
_end:
