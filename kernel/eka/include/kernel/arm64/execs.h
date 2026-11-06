// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// execs.h -- shadow of kernel/eka/include/kernel/execs.h for the AArch64 ILP32 kernel (every user includes it as
// "execs.h", so port/arm64/ekern first on -I wins). The baseline builds the fast/slow executive tables per CPU and
// #errors on anything else; the ARM branch emits them with AArch32 inline asm (.word lists in a naked function).
// The arm64 branch is the X86 branch's portable C++ arrays: EpocFastExecTable = {count, fn...}, EpocSlowExecTable
// = {count, invalid handler, preprocess handler, {flags, fn}...}, laid out exactly as genexec.pl's exec_kernel.h
// expects. Everything else is the baseline's CPU-neutral text, unchanged.

#ifndef __K32EXEC_H__
#define __K32EXEC_H__
#include <kernel/kern_priv.h>

class DObject;
class DTimer;
class DThread;
class DMutex;
class DSemaphore;
class DProcess;
class DChunk;
class DServer;
class DSession;
class DLibrary;
class DLogicalDevice;
class DPhysicalDevice;
class DLogicalChannelBase;
class DChangeNotifier;
class DUndertaker;
class DMsgQueue;
class DPropertyRef;
class DShPool;
class DShBuf;

#include <u32exec.h>

#define EF_C	KExecFlagClaim
#define	EF_R	KExecFlagRelease
#define	EF_P	KExecFlagPreprocess
#define EF_A2	KExecFlagExtraArgs2
#define EF_A3	KExecFlagExtraArgs3
#define EF_A4	KExecFlagExtraArgs4
#define EF_A5	KExecFlagExtraArgs5
#define EF_A6	KExecFlagExtraArgs6
#define EF_A7	KExecFlagExtraArgs7
#define EF_A8	KExecFlagExtraArgs8

#if defined(__CPU_ARM64)

// Executive call tables for the AArch64 kernel (the X86 branch's form)

#define	FAST_EXEC_BEGIN	\
	GLDEF_D const TUint32 EpocFastExecTable[] =		\
		{

#define FAST_EXEC_END	\
		0				\
		};

#define DECLARE_WORD(n)				(n),
#define	DECLARE_FUNC(f)				TLinAddr(&f),
#define	DECLARE_FAST_EXEC_INVALID	DECLARE_FUNC(InvalidFastExec)

#define	SLOW_EXEC_BEGIN	\
	GLDEF_D const TUint32 EpocSlowExecTable[] =		\
		{

#define SLOW_EXEC_END	\
		0				\
		};

#define DECLARE_INVALID_EXEC_HANDLER		DECLARE_FUNC(InvalidExecHandler)
#define	DECLARE_EXEC_PREPROCESS_HANDLER		DECLARE_FUNC(PreprocessHandler)

#define	DECLARE_FLAGS_FUNC(flags, func)					(flags), DECLARE_FUNC(func)
#define	DECLARE_FLAGS_ASMFUNC(flags, asmfunc, cppfunc)	(flags), DECLARE_FUNC(cppfunc)
#define	DECLARE_SLOW_EXEC_INVALID			0, DECLARE_FUNC(InvalidExecHandler)

#else
#error "port/arm64/ekern/execs.h is the arm64 kernel's shadow: build with the arm64 prelude"
#endif

#include <kernel/exec_kernel.h>

#endif
