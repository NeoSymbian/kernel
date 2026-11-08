// SPDX-License-Identifier: Apache-2.0
//
// a64_sha256.S -- SHA-256 blocks with the ARMv8 cryptographic extension (SHA256H/H2/SU0/SU1), for a64_medverity.cpp
// (docs/filesystem.md FS5a). The caller has checked ID_AA64ISAR0_EL1.SHA2, enabled FP/SIMD (CPACR_EL1.FPEN) and
// keeps interrupts off: no thread in this kernel ever owns SIMD state, so nothing is saved.
//
// void A64Sha256Blocks(TUint32 aState[8], const TUint8* aData, TInt aBlocks)	-- ILP32: 32-bit pointers in w0/w1
	.text
	.balign	16
	.global	A64Sha256Blocks
	.type	A64Sha256Blocks, %function

// four rounds: W (in \m0) + K, then abcd/efgh; with \upd, \m0 becomes the next W[t+16..t+19]
.macro ROUND4 m0, m1, m2, m3, upd
	ld1	{v16.4s}, [x3], #16
	add	v16.4s, v16.4s, \m0\().4s
	mov	v2.16b, v0.16b
	sha256h	q0, q1, v16.4s
	sha256h2 q1, q2, v16.4s
	.if \upd
	sha256su0 \m0\().4s, \m1\().4s
	sha256su1 \m0\().4s, \m2\().4s, \m3\().4s
	.endif
.endm

A64Sha256Blocks:
	mov	w0, w0						// zero-extend the ILP32 arguments
	mov	w1, w1
	ld1	{v0.4s, v1.4s}, [x0]			// abcd, efgh
1:	adr	x3, .LK
	ld1	{v4.16b, v5.16b, v6.16b, v7.16b}, [x1], #64
	rev32	v4.16b, v4.16b				// the message words are big-endian
	rev32	v5.16b, v5.16b
	rev32	v6.16b, v6.16b
	rev32	v7.16b, v7.16b
	mov	v20.16b, v0.16b
	mov	v21.16b, v1.16b
	ROUND4	v4, v5, v6, v7, 1
	ROUND4	v5, v6, v7, v4, 1
	ROUND4	v6, v7, v4, v5, 1
	ROUND4	v7, v4, v5, v6, 1
	ROUND4	v4, v5, v6, v7, 1
	ROUND4	v5, v6, v7, v4, 1
	ROUND4	v6, v7, v4, v5, 1
	ROUND4	v7, v4, v5, v6, 1
	ROUND4	v4, v5, v6, v7, 1
	ROUND4	v5, v6, v7, v4, 1
	ROUND4	v6, v7, v4, v5, 1
	ROUND4	v7, v4, v5, v6, 1
	ROUND4	v4, v5, v6, v7, 0
	ROUND4	v5, v6, v7, v4, 0
	ROUND4	v6, v7, v4, v5, 0
	ROUND4	v7, v4, v5, v6, 0
	add	v0.4s, v0.4s, v20.4s
	add	v1.4s, v1.4s, v21.4s
	subs	w2, w2, #1
	b.ne	1b
	st1	{v0.4s, v1.4s}, [x0]
	ret
	.size	A64Sha256Blocks, . - A64Sha256Blocks

	.balign	16							// the round constants, next to the code: adr reaches them
.LK:	.word	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5
	.word	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174
	.word	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da
	.word	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967
	.word	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85
	.word	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070
	.word	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3
	.word	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
