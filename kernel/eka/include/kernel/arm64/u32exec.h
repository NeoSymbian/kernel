// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// u32exec.h -- shadow of kernel/eka/include/u32exec.h for the AArch64 ILP32 kernel.
// The baseline has an executive-call section per CPU (WINS, X86, ARM) and #errors on anything else. That section
// only defines the USER-side exec stubs (SWI/INT sequences). Our user side is AArch32 and is built with the baseline
// header under __CPU_ARM; the AArch64 kernel never issues execs itself (no kernel TU uses the K*EXEC macros).
// So the arm64 branch keeps the ABI constants and makes the stub macros a compile error if anything expands them.
// Everything else is the baseline's CPU-neutral text, unchanged.

#ifndef __U32EXEC_H__
#define __U32EXEC_H__
#include <u32std.h>
#include <cpudefs.h>

#ifdef __KERNEL_MODE__
#define	EXEC_INT64		Int64
#define	EXEC_TIME		TTimeK
#else
#define	EXEC_INT64		TInt64
#define	EXEC_TIME		TInt64
#endif

#define __EXECDECL__	__NAKED__

#if defined(__CPU_ARM64)

// The AArch32 user side's SWI immediate: bit 23 selects the fast exec table (vectors.cia, our exec gate)
#define EXECUTIVE_FAST	0x00800000
#define EXECUTIVE_SLOW	0x00000000

#define __A64_NO_EXEC_STUB(n)	_Pragma("GCC error \"exec stubs are AArch32 user code: not built into the arm64 kernel\"")
#define	FAST_EXEC0(n)		__A64_NO_EXEC_STUB(n)
#define	FAST_EXEC1(n)		__A64_NO_EXEC_STUB(n)
#define	SLOW_EXEC0(n)		__A64_NO_EXEC_STUB(n)
#define	SLOW_EXEC1(n)		__A64_NO_EXEC_STUB(n)
#define	SLOW_EXEC2(n)		__A64_NO_EXEC_STUB(n)
#define SLOW_EXEC3(n)		__A64_NO_EXEC_STUB(n)
#define SLOW_EXEC4(n)		__A64_NO_EXEC_STUB(n)
#define	KFAST_EXEC0(n)		__A64_NO_EXEC_STUB(n)
#define	KFAST_EXEC1(n)		__A64_NO_EXEC_STUB(n)
#define	KSLOW_EXEC0(n)		__A64_NO_EXEC_STUB(n)
#define	KSLOW_EXEC1(n)		__A64_NO_EXEC_STUB(n)
#define	KSLOW_EXEC2(n)		__A64_NO_EXEC_STUB(n)
#define KSLOW_EXEC3(n)		__A64_NO_EXEC_STUB(n)
#define KSLOW_EXEC4(n)		__A64_NO_EXEC_STUB(n)

#else
#error "port/arm64/ekern/u32exec.h is the arm64 kernel's shadow: build with the arm64 prelude"
#endif

#ifdef __LEAVE_EQUALS_THROW__
// Hide TTrap to catch unwary uses of the old cleanup
// mechanism at compile time
class TTrap;
#endif //__LEAVE_EQUALS_THROW__

#include <exec_enum.h>
#include <e32btrace.h>
#include <exec_user.h>

#endif
