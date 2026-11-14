// SPDX-License-Identifier: Apache-2.0
// DRTAEABI.DLL -- the C++ run-time library of RVCT-built Symbian 9 binaries (old .sis apps), for NeoSymbian.
//
// The 2010 release built it from RVCT's own libraries, which are not open. Ours exports the same ordinals
// (../eabi/drtaeabiu.def) from GCC's runtime, which implements the same ABI (the C++ ABI for
// the ARM architecture, the EHABI): libsupc++ (type_info and its vtables, __cxa_* exceptions, __dynamic_cast,
// __aeabi_vec_*) and libgcc (the EHABI unwinder with __aeabi_unwind_cpp_pr0..2, which read RVCT's unwind tables,
// and the __aeabi_* integer helpers). NeoSymbian/build emitter/components.py `export_archives` exports them from the archives.
//
// This file adds what those archives expect from a C library, how the unwinder finds an image's tables
// (UserSvr::ExceptionDescriptor), per-thread exception state, the RVCT-specific helpers of the .def, and
// XLeaveException: an old binary's TRAP is `try ... catch (XLeaveException&)`, and euser's User::Leave throws one
// (with __cxa_allocate_exception and __cxa_throw, which it looks up here by ordinal) when that TRAP is the innermost.

#include <e32std.h>
#include <e32svr.h>
#include <e32def_private.h>

extern "C" {

// --- the C library the archives use ---
void* malloc(unsigned int aSize) { return User::Alloc(aSize); }
void free(void* aPtr) { User::Free(aPtr); }
void* realloc(void* aPtr, unsigned int aSize) { return User::ReAlloc(aPtr, aSize); }
void* calloc(unsigned int aCount, unsigned int aSize) { return User::AllocZ(aCount * aSize); }

void abort()
	{
	User::Panic(_L("C++ abort"), 0);
	for (;;) {}
	}

void* memcpy(void* aDest, const void* aSrc, unsigned int aSize)
	{
	Mem::Copy(aDest, aSrc, aSize);
	return aDest;
	}

void* memmove(void* aDest, const void* aSrc, unsigned int aSize)
	{
	Mem::Move(aDest, aSrc, aSize);
	return aDest;
	}

void* memset(void* aDest, int aByte, unsigned int aSize)
	{
	Mem::Fill(aDest, aSize, TChar(aByte & 0xff));
	return aDest;
	}

int memcmp(const void* aLeft, const void* aRight, unsigned int aSize)
	{
	const unsigned char* l = static_cast<const unsigned char*>(aLeft);
	const unsigned char* r = static_cast<const unsigned char*>(aRight);
	for (unsigned int i = 0; i < aSize; ++i)
		if (l[i] != r[i])
			return l[i] - r[i];
	return 0;
	}

// no environment on Symbian (libsupc++'s emergency pool reads GLIBCXX_TUNABLES when drtaeabi loads)
char* getenv(const char* /*aName*/) { return 0; }

unsigned int strlen(const char* aStr)
	{
	unsigned int n = 0;
	while (aStr[n])
		++n;
	return n;
	}

// --- the RVCT memory helpers (RTABI section 4.3.4); note __aeabi_memset's (dest, n, c) order ---
void __aeabi_memcpy(void* aDest, const void* aSrc, unsigned int aSize) { Mem::Copy(aDest, aSrc, aSize); }
void __aeabi_memcpy4(void* aDest, const void* aSrc, unsigned int aSize) { Mem::Copy(aDest, aSrc, aSize); }
void __aeabi_memcpy8(void* aDest, const void* aSrc, unsigned int aSize) { Mem::Copy(aDest, aSrc, aSize); }
void __aeabi_memmove(void* aDest, const void* aSrc, unsigned int aSize) { Mem::Move(aDest, aSrc, aSize); }
void __aeabi_memmove4(void* aDest, const void* aSrc, unsigned int aSize) { Mem::Move(aDest, aSrc, aSize); }
void __aeabi_memmove8(void* aDest, const void* aSrc, unsigned int aSize) { Mem::Move(aDest, aSrc, aSize); }
void __aeabi_memset(void* aDest, unsigned int aSize, int aByte) { memset(aDest, aByte, aSize); }
void __aeabi_memset4(void* aDest, unsigned int aSize, int aByte) { memset(aDest, aByte, aSize); }
void __aeabi_memset8(void* aDest, unsigned int aSize, int aByte) { memset(aDest, aByte, aSize); }
void __aeabi_memclr(void* aDest, unsigned int aSize) { Mem::FillZ(aDest, aSize); }
void __aeabi_memclr4(void* aDest, unsigned int aSize) { Mem::FillZ(aDest, aSize); }
void __aeabi_memclr8(void* aDest, unsigned int aSize) { Mem::FillZ(aDest, aSize); }

// unaligned loads and stores (RTABI section 4.3.5)
int __aeabi_uread4(void* aPtr)
	{
	const unsigned char* p = static_cast<const unsigned char*>(aPtr);
	return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
	}

int __aeabi_uwrite4(int aValue, void* aPtr)
	{
	unsigned char* p = static_cast<unsigned char*>(aPtr);
	for (int i = 0; i < 4; ++i)
		p[i] = static_cast<unsigned char>(aValue >> (8 * i));
	return aValue;
	}

long long __aeabi_uread8(void* aPtr)
	{
	const unsigned char* p = static_cast<const unsigned char*>(aPtr);
	return static_cast<long long>(static_cast<unsigned int>(__aeabi_uread4(p)))
		| static_cast<long long>(__aeabi_uread4(const_cast<unsigned char*>(p) + 4)) << 32;
	}

long long __aeabi_uwrite8(long long aValue, void* aPtr)
	{
	__aeabi_uwrite4(static_cast<int>(aValue), aPtr);
	__aeabi_uwrite4(static_cast<int>(aValue >> 32), static_cast<unsigned char*>(aPtr) + 4);
	return aValue;
	}

// --- the unwinder's table lookup: the image holding a code address names its .ARM.exidx in the descriptor its
// E32 header points at (RVCT images and ours alike) ---
unsigned int __gnu_Unwind_Find_exidx(unsigned int aPc, int* aCount)
	{
	const TExceptionDescriptor* d =
		reinterpret_cast<const TExceptionDescriptor*>(UserSvr::ExceptionDescriptor(aPc) & ~1u);
	if (!d)
		return 0;
	*aCount = (d->iExIdxLimit - d->iExIdxBase) / 8;
	return d->iExIdxBase;
	}

// --- per-thread exception state (libsupc++'s eh_globals is one static: wrong once two threads throw) ---
struct TEhGlobals
	{
	void* iCaught;
	unsigned int iUncaught;
	void* iPropagating;		// ARM EABI: exceptions under cleanup
	};

TEhGlobals* __cxa_get_globals()
	{
	TEhGlobals* g = static_cast<TEhGlobals*>(Dll::Tls());
	if (!g)
		{
		g = static_cast<TEhGlobals*>(User::AllocZ(sizeof(TEhGlobals)));
		if (!g || Dll::SetTls(g) != KErrNone)
			abort();
		}
	return g;
	}

TEhGlobals* __cxa_get_globals_fast() { return __cxa_get_globals(); }

} // extern "C"

// --- the RVCT-specific parts of the .def ---

// what euser's User::Leave throws into an old binary's TRAP: {vtable, reason}, the RVCT 2.2+ layout (e32cmn.h under
// __SUPPORT_CPP_EXCEPTIONS__, with ForceKeyFunction as the key function that puts the vtable and typeinfo here)
class XLeaveException
	{
public:
	explicit XLeaveException(TInt aReason) : iR(aReason) {}
	virtual void ForceKeyFunction();
	TInt iR;
	};

void XLeaveException::ForceKeyFunction() {}

// RVCT's run-time globals object: nothing to hold here (the state is per thread, in TEhGlobals)
class TCppRTExceptionsGlobals
	{
public:
	TCppRTExceptionsGlobals();
	void Init2ndEmergencyBuffer();
	void Kill2ndEmergencyBuffer();
	};

TCppRTExceptionsGlobals::TCppRTExceptionsGlobals() {}
void TCppRTExceptionsGlobals::Init2ndEmergencyBuffer() {}
void TCppRTExceptionsGlobals::Kill2ndEmergencyBuffer() {}
