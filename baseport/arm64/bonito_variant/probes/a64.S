// AArch64 probe: arm64 Image that paints the bootloader's framebuffer green and halts.
// Proves the boot.img packaging and the framebuffer address before any real kernel work.
#include "fb.h"

	.text
	.globl _start
_start:
	b	code			// code0
	.long	0			// code1
	.quad	0x80000			// text_offset: Pixel ABL rejects 0 ("Invalid Parameter")
	.quad	_end - _start		// image_size
	.quad	0xa			// flags: little endian, 4K pages, load anywhere
	.quad	0, 0, 0			// res2-res4
	.ascii	"ARM\x64"		// magic
	.long	0			// res5

code:
	movz	x1, #(FB_BASE >> 16), lsl #16
	movz	w2, #(COLOR_A64 & 0xffff)
	movk	w2, #(COLOR_A64 >> 16), lsl #16
	movz	x3, #(FB_BYTES & 0xffff)
	movk	x3, #(FB_BYTES >> 16), lsl #16
1:	str	w2, [x1], #4
	subs	x3, x3, #4
	b.ne	1b
	dsb	sy
2:	wfe
	b	2b
_end:
