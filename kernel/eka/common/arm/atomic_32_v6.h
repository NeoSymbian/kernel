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
// e32\common\arm\atomic_32_v6.h
// 32 bit atomic operations on V6 and V6K processors
// Also 8 and 16 bit atomic operations on V6K processors
// Also 8, 16 and 32 bit load/store on all processors
// 
//

#include "atomic_ops.h"

#if defined(__OP_LOAD__)
extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,acq,__DATA_SIZE__)(const volatile TAny* /*a*/)
	{
	// R0=a
	// return value in R0

#ifdef __LDA_STL_ATOMICS__
	__LDA_INST__("r0, [r0] ");
#else
	__LDR_INST__( ," r0, [r0] ");
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r1);
#endif
	__JUMP(,lr);
	}



#elif defined(__OP_STORE__)
extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rel,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*v*/)
	{
#ifdef __BARRIERS_NEEDED__
	// R0=a, R1=v
	// return value in R0 equal to v
#ifdef __LDA_STL_ATOMICS__
	__STL_INST__("r1, [r0] ");
#else
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
	__STR_INST__( ," r1, [r0] ");
#endif
	asm("mov r0, r1 ");
	__JUMP(,lr);
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,ord,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=v
	// return value in R0 equal to v
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
	__STR_INST__( ," r1, [r0] ");
	__LOCAL_DATA_MEMORY_BARRIER__(r12);
	asm("mov r0, r1 ");
	__JUMP(,lr);
	}


#elif defined(__OP_RMW1__)

#ifdef __OP_SWP__
#define	__SOURCE_REG__		1
#define	__DO_PROCESSING__
#else
#define	__SOURCE_REG__		2
#if defined(__OP_ADD__)
#define	__DO_PROCESSING__	asm("add r2, r0, r1 ");
#elif defined(__OP_AND__)
#define	__DO_PROCESSING__	asm("and r2, r0, r1 ");
#elif defined(__OP_IOR__)
#define	__DO_PROCESSING__	asm("orr r2, r0, r1 ");
#elif defined(__OP_XOR__)
#define	__DO_PROCESSING__	asm("eor r2, r0, r1 ");
#endif
#endif

#define __DO_RMW1_OPX__(LDX,STX)				\
	asm("mov r12, r0 ");			\
	asm("1: ");						\
	LDX(0,12);			\
	__DO_PROCESSING__				\
	STX(3,__SOURCE_REG__,12);	\
	asm("cmp r3, #0 ");				\
	asm("bne 1b ");
#define __DO_RMW1_OP__	__DO_RMW1_OPX__(__LDREX_INST__,__STREX_INST__)


extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rel,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
#ifdef __LDA_STL_ATOMICS__
	__DO_RMW1_OPX__(__LDREX_INST__,__STLEX_INST__)
	__JUMP(,lr);
#else
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rlx,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__DO_RMW1_OP__
	__JUMP(,lr);
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,ord,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#ifdef __LDA_STL_ATOMICS__
	__DO_RMW1_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r3);
	__JUMP(,lr);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,acq,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=v
	// return value in R0
	__DO_RMW1_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r3);
	__JUMP(,lr);
	}

#undef __DO_RMW1_OP__
#undef __DO_RMW1_OPX__
#undef __SOURCE_REG__
#undef __DO_PROCESSING__


#elif defined(__OP_CAS__)

#define __DO_CAS_OPX__(LDX,STX)				\
	__LDR_INST__( ," r12, [r1] ");	\
	asm("1: ");						\
	LDX(3,0);			\
	asm("cmp r3, r12 ");			\
	asm("bne 2f ");					\
	STX(3,2,0);			\
	asm("cmp r3, #0 ");				\
	asm("bne 1b ");					\
	asm("2: ");						\
	__STR_INST__(ne, "r3, [r1] ");	\
	asm("movne r0, #0 ");			\
	asm("moveq r0, #1 ");
#define __DO_CAS_OP__	__DO_CAS_OPX__(__LDREX_INST__,__STREX_INST__)


extern "C" EXPORT_C __NAKED__ TBool __fname__(__OPERATION__,rel,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ * /*q*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=q, R2=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);	// NeoSymbian: kept on ARMv8 too: a failed compare skips the STLEX
#endif
	}

extern "C" EXPORT_C __NAKED__ TBool __fname__(__OPERATION__,rlx,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ * /*q*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=q, R2=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__DO_CAS_OP__
	__JUMP(,lr);
#endif
	}

extern "C" EXPORT_C __NAKED__ TBool __fname__(__OPERATION__,ord,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ * /*q*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=q, R2=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#ifdef __LDA_STL_ATOMICS__
	__DO_CAS_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r3);
	__JUMP(,lr);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ TBool __fname__(__OPERATION__,acq,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ * /*q*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=q, R2=v
	// return value in R0
	__DO_CAS_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r3);
	__JUMP(,lr);
	}

#undef __DO_CAS_OP__
#undef __DO_CAS_OPX__



#elif defined(__OP_AXO__)

#define	__SAVE_REGS__		asm("str r4, [sp, #-4]! ");
#define	__RESTORE_REGS__	asm("ldr r4, [sp], #4 ");

#define __DO_AXO_OPX__(LDX,STX)				\
	asm("mov r12, r0 ");			\
	asm("1: ");						\
	LDX(0,12);			\
	asm("and r4, r0, r1 ");			\
	asm("eor r4, r4, r2 ");			\
	STX(3,4,12);			\
	asm("cmp r3, #0 ");				\
	asm("bne 1b ");
#define __DO_AXO_OP__	__DO_AXO_OPX__(__LDREX_INST__,__STREX_INST__)


extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rel,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=u, R2=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
#ifdef __LDA_STL_ATOMICS__
	__SAVE_REGS__
	__DO_AXO_OPX__(__LDREX_INST__,__STLEX_INST__)
	__RESTORE_REGS__
	__JUMP(,lr);
#else
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rlx,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=u, R2=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__SAVE_REGS__
	__DO_AXO_OP__
	__RESTORE_REGS__
	__JUMP(,lr);
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,ord,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=u, R2=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#ifdef __LDA_STL_ATOMICS__
	__SAVE_REGS__
	__DO_AXO_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r3);
	__RESTORE_REGS__
	__JUMP(,lr);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,acq,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=u, R2=v
	// return value in R0
	__SAVE_REGS__
	__DO_AXO_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r3);
	__RESTORE_REGS__
	__JUMP(,lr);
	}

#undef __SAVE_REGS__
#undef __RESTORE_REGS__
#undef __DO_AXO_OP__
#undef __DO_AXO_OPX__


#elif defined(__OP_RMW3__)

#define	__SAVE_REGS__		asm("stmfd sp!, {r4-r5} ");
#define	__RESTORE_REGS__	asm("ldmfd sp!, {r4-r5} ");

#if	defined(__OP_TAU__)
#define	__COND_GE__		"cs"
#define	__COND_LT__		"cc"
#define	__DO_SIGN_EXTEND__
#elif	defined(__OP_TAS__)
#define	__COND_GE__		"ge"
#define	__COND_LT__		"lt"
#define	__DO_SIGN_EXTEND__	__SIGN_EXTEND__(r0)
#endif

#define __DO_RMW3_OPX__(LDX,STX)				\
	asm("mov r12, r0 ");			\
	asm("1: ");						\
	LDX(0,12);			\
	__DO_SIGN_EXTEND__				\
	asm("cmp r0, r1 ");				\
	asm("add" __COND_GE__ " r4, r0, r2 ");	\
	asm("add" __COND_LT__ " r4, r0, r3 ");	\
	STX(5,4,12);			\
	asm("cmp r5, #0 ");				\
	asm("bne 1b ");
#define __DO_RMW3_OP__	__DO_RMW3_OPX__(__LDREX_INST__,__STREX_INST__)


extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rel,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*t*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=t, R2=u, R3=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
#ifdef __LDA_STL_ATOMICS__
	__SAVE_REGS__
	__DO_RMW3_OPX__(__LDREX_INST__,__STLEX_INST__)
	__RESTORE_REGS__
	__JUMP(,lr);
#else
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,rlx,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*t*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=t, R2=u, R3=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__SAVE_REGS__
	__DO_RMW3_OP__
	__RESTORE_REGS__
	__JUMP(,lr);
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,ord,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*t*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=t, R2=u, R3=v
	// return value in R0
#ifdef __BARRIERS_NEEDED__				// If no barriers, all ordering variants collapse to same function
	__LOCAL_DATA_MEMORY_BARRIER_Z__(r12);
#ifdef __LDA_STL_ATOMICS__
	__SAVE_REGS__
	__DO_RMW3_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r5);
	__RESTORE_REGS__
	__JUMP(,lr);
#endif
#endif
	}

extern "C" EXPORT_C __NAKED__ __TYPE__ __fname__(__OPERATION__,acq,__DATA_SIZE__)(volatile TAny* /*a*/, __TYPE__ /*t*/, __TYPE__ /*u*/, __TYPE__ /*v*/)
	{
	// R0=a, R1=t, R2=u, R3=v
	// return value in R0
	__SAVE_REGS__
	__DO_RMW3_OP__
	__LOCAL_DATA_MEMORY_BARRIER__(r5);
	__RESTORE_REGS__
	__JUMP(,lr);
	}

#undef __SAVE_REGS__
#undef __RESTORE_REGS__
#undef __DO_RMW3_OP__
#undef __DO_RMW3_OPX__
#undef __COND_GE__
#undef __COND_LT__
#undef __DO_SIGN_EXTEND__


#endif

// Second inclusion undefines temporaries
#include "atomic_ops.h"
