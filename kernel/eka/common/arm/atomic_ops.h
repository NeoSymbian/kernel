// Copyright (c) 2008-2009 Nokia Corporation and/or its subsidiary(-ies).
// All rights reserved.
// This component and the accompanying materials are made available
// under the terms of the License "Eclipse Public License v1.0"
// which accompanies this distribution, and is available
// at the URL "http://www.eclipse.org/legal/epl-v10.html".
// SPDX-License-Identifier: EPL-2.0 (changed by the NeoSymbian Authors)
// Initial Contributors:
// Nokia Corporation - initial contribution.
//
// Contributors:
// The NeoSymbian Authors (2026): acquire/release (LDA/STL) forms for the ARMv8 profile (__CPU_ARM_HAS_LDA_STL).
// Description:
// e32\common\arm\atomic_ops.h
// 
//

#ifdef	__OPERATION__

#undef	__OPERATION__
#undef	__OP_RMW1__
#undef	__OP_RMW2__
#undef	__OP_RMW3__
#undef	__OP_SIGNED__
#undef	__TYPE__
#undef	__SIZE_CODE__
#undef	__LDR_INST__
#undef	__LDRS_INST__
#undef	__STR_INST__
#undef	__LDREX_INST__
#undef	__STREX_INST__
#undef	__STLEX_INST__
#undef	__LDA_INST__
#undef	__STL_INST__
#undef	__LDA_STL_ATOMICS__
#undef	__NEO_STR__
#undef	__NEO_XSTR__
#undef	__SIGN_EXTEND__
#undef	__LOG2_DATA_SIZE__

#undef	__OP_LOAD__
#undef	__OP_STORE__
#undef	__OP_SWP__
#undef	__OP_CAS__
#undef	__OP_ADD__
#undef	__OP_AND__
#undef	__OP_IOR__
#undef	__OP_XOR__
#undef	__OP_AXO__
#undef	__OP_TAU__
#undef	__OP_TAS__

#undef ENSURE_8BYTE_ALIGNMENT

#else	// __OPERATION__

#if defined(__OP_LOAD__)
#define __OPERATION__	load
#elif defined(__OP_STORE__)
#define __OPERATION__	store
#elif defined(__OP_SWP__)
#define __OPERATION__	swp
#define	__OP_RMW1__
#elif defined(__OP_CAS__)
#define __OPERATION__	cas
#elif defined(__OP_ADD__)
#define __OPERATION__	add
#define	__OP_RMW1__
#elif defined(__OP_AND__)
#define __OPERATION__	and
#define	__OP_RMW1__
#elif defined(__OP_IOR__)
#define __OPERATION__	ior
#define	__OP_RMW1__
#elif defined(__OP_XOR__)
#define __OPERATION__	xor
#define	__OP_RMW1__
#elif defined(__OP_AXO__)
#define __OPERATION__	axo
#define	__OP_RMW2__
#elif defined(__OP_TAU__)
#define __OPERATION__	tau
#define	__OP_RMW3__
#elif defined(__OP_TAS__)
#define __OPERATION__	tas
#define	__OP_RMW3__
#define	__OP_SIGNED__
#else
#error Unknown atomic operation
#endif

#if __DATA_SIZE__==8
#define	__LOG2_DATA_SIZE__			3
#define __SIZE_CODE__				"b"
#ifdef __CPU_ARM_HAS_LDREX_STREX_V6K
#define	__LDREX_INST__(Rd,Rn)		LDREXB(Rd,Rn)
#define	__STREX_INST__(Rd,Rm,Rn)	STREXB(Rd,Rm,Rn)
#endif
#define	__SIGN_EXTEND__(reg)		asm("mov "#reg ", "#reg ", lsl #24 "); asm("mov "#reg ", "#reg ", asr #24 ");
#define	__LDR_INST__(cc,args)		asm("ldr"#cc "b " args)
#define	__LDRS_INST__(cc,args)		asm("ldr"#cc "sb " args)
#define	__STR_INST__(cc,args)		asm("str"#cc "b " args)
#ifdef	__OP_SIGNED__
#define	__TYPE__					TInt8
#else
#define	__TYPE__					TUint8
#endif
#elif __DATA_SIZE__==16
#define	__LOG2_DATA_SIZE__			4
#define __SIZE_CODE__				"h"
#ifdef __CPU_ARM_HAS_LDREX_STREX_V6K
#define	__LDREX_INST__(Rd,Rn)		LDREXH(Rd,Rn)
#define	__STREX_INST__(Rd,Rm,Rn)	STREXH(Rd,Rm,Rn)
#endif
#define	__SIGN_EXTEND__(reg)		asm("mov "#reg ", "#reg ", lsl #16 "); asm("mov "#reg ", "#reg ", asr #16 ");
#define	__LDR_INST__(cc,args)		asm("ldr"#cc "h " args)
#define	__LDRS_INST__(cc,args)		asm("ldr"#cc "sh " args)
#define	__STR_INST__(cc,args)		asm("str"#cc "h " args)
#ifdef	__OP_SIGNED__
#define	__TYPE__					TInt16
#else
#define	__TYPE__					TUint16
#endif
#elif __DATA_SIZE__==32
#define	__LOG2_DATA_SIZE__			5
#define __SIZE_CODE__				""
#ifdef __CPU_ARM_HAS_LDREX_STREX
#define	__LDREX_INST__(Rd,Rn)		LDREX(Rd,Rn)
#define	__STREX_INST__(Rd,Rm,Rn)	STREX(Rd,Rm,Rn)
#endif
#define	__SIGN_EXTEND__(reg)
#define	__LDR_INST__(cc,args)		asm("ldr"#cc " " args)
#define	__LDRS_INST__(cc,args)		asm("ldr"#cc " " args)
#define	__STR_INST__(cc,args)		asm("str"#cc " " args)
#ifdef	__OP_SIGNED__
#define	__TYPE__					TInt32
#else
#define	__TYPE__					TUint32
#endif
#elif __DATA_SIZE__==64
#define	__LOG2_DATA_SIZE__			6
#define __SIZE_CODE__				"d"
#ifdef __CPU_ARM_HAS_LDREX_STREX_V6K
#define	__LDREX_INST__(Rd,Rn)		LDREXD(Rd,Rn)
#define	__STREX_INST__(Rd,Rm,Rn)	STREXD(Rd,Rm,Rn)
#endif
#ifdef	__OP_SIGNED__
#define	__TYPE__					TInt64
#else
#define	__TYPE__					TUint64
#endif
#else
#error Invalid data size
#endif

// NeoSymbian: on ARMv8 (cpudefs.h __CPU_ARM_HAS_LDA_STL), 8, 16 and 32 bits (atomic_32_v6.h): load acq = LDA, store rel
// = STL, and the release RMWs that always store (swp, add, and, ior, xor, axo, tau, tas) = LDREX..STLEX, with no DMB.
// The rest keep their DMBs: Symbian's acq orders the whole operation, write included, before later accesses (LDAEX
// orders only the read); a release CAS whose compare fails stores nothing (no STLEX); ord is a full barrier; 64 bits.
#if defined(__CPU_ARM_HAS_LDA_STL) && defined(__BARRIERS_NEEDED__) && __DATA_SIZE__!=64
#define	__LDA_STL_ATOMICS__
#define	__NEO_STR__(x)				#x
#define	__NEO_XSTR__(x)				__NEO_STR__(x)		// expands the argument first (__SOURCE_REG__)
#define	__STLEX_INST__(Rd,Rm,Rn)	asm("stlex" __SIZE_CODE__ " r" __NEO_XSTR__(Rd) ", r" __NEO_XSTR__(Rm) ", [r" __NEO_XSTR__(Rn) "] ")
#define	__LDA_INST__(args)			asm("lda" __SIZE_CODE__ " " args)
#define	__STL_INST__(args)			asm("stl" __SIZE_CODE__ " " args)
#endif

#if (defined(__GNUC__) && (__GNUC__ >= 3)) || defined(__EABI__)
// Check 8 byte aligned and cause alignment fault if not.
// Doesn't work if alignment checking is disabled but gives consistent behaviour
// between processors with ldrexd etc and these hand coded versions.
#define ENSURE_8BYTE_ALIGNMENT(rAddr) 		\
	asm("tst r"#rAddr", #0x7 "); 			\
	asm("orrne r"#rAddr", r"#rAddr", #1 "); \
	asm("ldmne r"#rAddr", {r"#rAddr"} ")
#else
// Don't assert on old gcc (arm4) as it is not eabi compliant and this stops 
// kernel booting.
#define ENSURE_8BYTE_ALIGNMENT(rAddr)
#endif
#endif	// __OPERATION__
