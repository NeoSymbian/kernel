// AArch32 user-mode code for el0.S: paint the framebuffer blue, then spin (no WFE: it may trap at EL0).
#include "fb.h"

	.arm
	.text
	movw	r1, #(FB_BASE & 0xffff)		// any base (0x9d400000 is no ARM immediate)
	movt	r1, #(FB_BASE >> 16)
	movw	r2, #(COLOR_A32 & 0xffff)
	movt	r2, #(COLOR_A32 >> 16)
	movw	r3, #(FB_BYTES & 0xffff)
	movt	r3, #(FB_BYTES >> 16)
1:	str	r2, [r1], #4
	subs	r3, r3, #4
	bne	1b
	b	.
