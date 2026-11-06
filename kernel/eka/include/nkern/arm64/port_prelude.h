/* Copied unchanged from Phorepoc core os/toolchain/x32/port_prelude.h (Apache-2.0, The Phorepoc Authors):
 * the toolchain gaps it fills (export decoration, _FOFF, VA_LIST, TEMPLATE_SPECIALIZATION) are
 * ABI-independent, so the AArch64 ILP32 port uses it as is. */
/* Copyright The Phorepoc Authors
 * SPDX-License-Identifier: Apache-2.0 */

/* port_prelude.h -- Phorepoc amd64 kernel-port toolchain prelude.
 *
 * Force-included (-include) ahead of the real EKA2 headers when building the
 * arch-NEUTRAL nkern core for a freestanding x86_64 target with modern GCC.
 *
 * These are things the original Symbian build ENVIRONMENT supplied (via the VC
 * x86 / RVCT ARM toolchains + build macros), not the source. We are compiling a
 * static, freestanding kernel with GCC, so we provide the equivalents here --
 * the analogue of the ARM build's modern-GCC prefix shim.
 * NOTHING here modifies baseline source; it only fills toolchain-provided gaps.
 */
#ifndef PHOREPOC_PORT_PRELUDE_H
#define PHOREPOC_PORT_PRELUDE_H

/* Export decoration: a static freestanding kernel has no DLL export/import.
 * The x86 EKA2 port was VC-built (IMPORT_C = __declspec(dllexport)); for our
 * static link these are empty (as they are for the __GCC32__ && __MARM__ path). */
#ifndef IMPORT_C
#define IMPORT_C
#endif
#ifndef EXPORT_C
#define EXPORT_C
#endif

/* Exception spec: build with -fno-exceptions; drop the nothrow decoration. */
#ifndef __NO_THROW
#define __NO_THROW
#endif
#ifndef __THROW
#define __THROW
#endif

/* Explicit template specialization syntax. e32def.h only defines this for
 * recognized compiler identities (MSVC, __GCC32__&&__MARM__, __EABI__, CW);
 * "modern GCC + x86" matches none, so it is left undefined and e32cmn.h's
 * RPointerArray<void> specializations fail to parse. Standard C++ (which our
 * GCC speaks) wants the `template<>` form -- the same value the __EABI__ path
 * picks. Supply it here so the full e32cmn.h parses (needed from nkerns.cpp on). */
/* e32def.h only defines these per known compiler identity (MSVC / GCC32+MARM / EABI / CW);
 * GCC-on-x86 matches none. Same value every identity picks. First needed by u32std.h (ekern). */
#ifndef NONSHARABLE_CLASS
#define NONSHARABLE_CLASS(x) class x
#endif
#ifndef NONSHARABLE_STRUCT
#define NONSHARABLE_STRUCT(x) struct x
#endif

#ifndef EXPORT_D
#define EXPORT_D
#endif
#ifndef IMPORT_D
#define IMPORT_D
#endif

/* The IA-32 port was built with MSVC-style calling-convention keywords. amd64 has ONE calling
 * convention (SysV), so they are noise here; the asm that relied on them is re-authored anyway. */
#ifndef __fastcall
#define __fastcall
#endif
#ifndef __cdecl
#define __cdecl
#endif
#ifndef __stdcall
#define __stdcall
#endif

/* e32def.h's GCC>=4 _FOFF is a bare __builtin_offsetof = size_t (unsigned). EKA2 means a TInt:
 * RArray(8, _FOFF(..), 512) is ambiguous otherwise (sprocess.cpp). Still a constant expression,
 * so the compile-time layout asserts keep working. e32def.h guards with #ifndef _FOFF. */
#ifdef PORT_FOFF_DYNAMIC	/* per-TU: des8/des16.cpp use a RUN-TIME index, _FOFF(HBufC8, iBuf[aMaxLength]) */
#define _FOFF(c,f) (((TInt)&(((c *)0x1000)->f))-0x1000)
#else
#define _FOFF(c,f) ((TInt)__builtin_offsetof(c,f))
#endif

/* Variable arguments. e32def.h's fallback VA_LIST is a hand-rolled walk over a CONTIGUOUS STACK
 * argument area (IA-32 cdecl). The SysV amd64 ABI -- x32 included -- passes the first six varargs
 * in REGISTERS, so that walk reads garbage (first seen: #PF at 0xFFFFFFFF inside
 * Kern::AppendFormat, the very first Kern::Printf). Use the compiler's va_list, exactly as the
 * __EABI__ branch does; it is also an array type, so EKA2's "pass the VA_LIST on and keep using
 * it" idiom behaves the same. e32def.h guards all of these with #ifndef. */
#define __VA_LIST_defined
typedef __builtin_va_list VA_LIST;
#define VA_START(ap,pn)	__builtin_va_start(ap,pn)
#define VA_ARG(ap,type)	__builtin_va_arg(ap,type)
#define VA_END(ap)		__builtin_va_end(ap)

#ifndef TEMPLATE_SPECIALIZATION
#define TEMPLATE_SPECIALIZATION template<>
#endif

/* NB: EKA2 headers (e.g. nklib.h) declare their OWN placement new/delete; we
 * must NOT redefine them here or they collide. */

#endif /* PHOREPOC_PORT_PRELUDE_H */
