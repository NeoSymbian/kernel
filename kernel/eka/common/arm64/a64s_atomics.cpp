// Copied unchanged from Phorepoc core os/toolchain/x32/x64smp/x64s_atomics.cpp: the __e32_atomic_* family on GCC
// __atomic builtins is ABI-independent, so the AArch64 ILP32 port uses it as is (it compiles to LDAXR/STLXR or LSE).
// Copyright The Phorepoc Authors
// SPDX-License-Identifier: Apache-2.0

/* x64s_atomics.cpp -- Phorepoc amd64 SMP arch backend: the full e32atomics
 * primitive set the arch-neutral nkernSMP core references.
 *
 * The SMP nanokernel leans on the atomics far harder than the uniprocessor one
 * (spin locks, per-CPU ready-list interlocks, IPI accept masks, tied-event
 * interlocks), so this is the complete family -- load/store/swp/cas/add/and/ior/
 * xor/axo/tau/tas in 8/16/32/64 bit, each in rlx/acq/rel/ord ordering -- plus the
 * bit scans and barriers. It is the amd64 analogue of common/x86/atomics.cia.
 * xor + tas (32 entry points) were added for the euser EXPORT table (X4.9b): the
 * nanokernel itself never calls them, but euseru.def exports every one, so a
 * from-source euser.dll has to carry them. They replace the one-off
 * __e32_atomic_tas_ord32 that xe_nkern.cpp and xu_utl.cpp each defined privately.
 *
 * IMPLEMENTATION NOTE. x86-64 is a TSO (total-store-order) machine: every
 * LOCK-prefixed RMW is a full fence, plain aligned loads have acquire order and
 * plain aligned stores have release order already. So all four ordering suffixes
 * collapse onto the same instruction sequence -- we implement each op once with
 * GCC's __atomic builtins at __ATOMIC_SEQ_CST (the strongest; never weaker than
 * any requested order, hence always correct) and alias the four suffix names to
 * it. That is exactly what the hand-written x86 .cia does under the hood.
 */
#include <e32def.h>

/* ptr helpers */
#define P8(a)   ((volatile TUint8*)(a))
#define P16(a)  ((volatile TUint16*)(a))
#define P32(a)  ((volatile TUint32*)(a))
#define P64(a)  ((volatile TUint64*)(a))

extern "C" {

/* ---- generate the four ordering aliases of one body ------------------------ */
#define ALIAS4(ret, name, args, call) \
    EXPORT_C ret name##_rlx##args { call } \
    EXPORT_C ret name##_acq##args { call } \
    EXPORT_C ret name##_rel##args { call } \
    EXPORT_C ret name##_ord##args { call }

/* load: acquire read.  store: write, return v.  (only the documented suffixes
 * exist for load/store, so these are written out individually.) */
#define LOADSTORE(bits, T, PT) \
    EXPORT_C T __e32_atomic_load_acq##bits(const volatile TAny* a) \
        { return __atomic_load_n((const volatile T*)a, __ATOMIC_SEQ_CST); } \
    EXPORT_C T __e32_atomic_store_rel##bits(volatile TAny* a, T v) \
        { __atomic_store_n(PT(a), v, __ATOMIC_SEQ_CST); return v; } \
    EXPORT_C T __e32_atomic_store_ord##bits(volatile TAny* a, T v) \
        { __atomic_store_n(PT(a), v, __ATOMIC_SEQ_CST); return v; }

/* swp: write v, return original */
#define SWP(bits, T, PT) \
    ALIAS4(T, __e32_atomic_swp, bits(volatile TAny* a, T v), \
        return __atomic_exchange_n(PT(a), v, __ATOMIC_SEQ_CST);)

/* cas: if (*a==*q) {*a=v; return TRUE;} else {*q=*a; return FALSE;} */
#define CAS(bits, T, PT) \
    ALIAS4(TBool, __e32_atomic_cas, bits(volatile TAny* a, T* q, T v), \
        return __atomic_compare_exchange_n(PT(a), q, v, 0, \
                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST) ? TRUE : FALSE;)

/* add / and / ior: RMW, return original */
#define RMW(op, builtin, bits, T, PT) \
    ALIAS4(T, __e32_atomic_##op, bits(volatile TAny* a, T v), \
        return builtin(PT(a), v, __ATOMIC_SEQ_CST);)

/* axo: *a = (*a & u) ^ v; return original */
#define AXO(bits, T, PT) \
    ALIAS4(T, __e32_atomic_axo, bits(volatile TAny* a, T u, T v), \
        volatile T* p = PT(a); T o = __atomic_load_n(p, __ATOMIC_SEQ_CST); \
        while (!__atomic_compare_exchange_n(p, &o, (T)((o & u) ^ v), 0, \
                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {} \
        return o;)

/* tau: if (*a>=t) *a+=u else *a+=v; return original  (unsigned compare) */
#define TAU(bits, T, PT) \
    ALIAS4(T, __e32_atomic_tau, bits(volatile TAny* a, T t, T u, T v), \
        volatile T* p = PT(a); T o = __atomic_load_n(p, __ATOMIC_SEQ_CST); \
        while (!__atomic_compare_exchange_n(p, &o, (T)(o + (o >= t ? u : v)), 0, \
                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {} \
        return o;)

/* tas: the SIGNED twin of tau -- same operation, signed compare, signed argument
 * and return types (e32atomics.h: TInt8/16/32/64). The RMW itself is done on the
 * unsigned pointer type so the compare_exchange is width-exact either way. */
#define TAS(bits, T, S, PT) \
    ALIAS4(S, __e32_atomic_tas, bits(volatile TAny* a, S t, S u, S v), \
        volatile T* p = PT(a); T o = __atomic_load_n(p, __ATOMIC_SEQ_CST); \
        while (!__atomic_compare_exchange_n(p, &o, \
                    (T)((S)o + ((S)o >= t ? u : v)), 0, \
                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {} \
        return (S)o;)

#define ALLOPS(bits, T, S, PT) \
    LOADSTORE(bits, T, PT) \
    SWP(bits, T, PT) \
    CAS(bits, T, PT) \
    RMW(add, __atomic_fetch_add, bits, T, PT) \
    RMW(and, __atomic_fetch_and, bits, T, PT) \
    RMW(ior, __atomic_fetch_or,  bits, T, PT) \
    RMW(xor, __atomic_fetch_xor, bits, T, PT) \
    AXO(bits, T, PT) \
    TAU(bits, T, PT) \
    TAS(bits, T, S, PT)

ALLOPS(8,  TUint8,  TInt8,  P8)
ALLOPS(16, TUint16, TInt16, P16)
ALLOPS(32, TUint32, TInt32, P32)
ALLOPS(64, TUint64, TInt64, P64)

/* ---- bit scans ------------------------------------------------------------- */
EXPORT_C TInt __e32_find_ms1_32(TUint32 v)
    { return v ? 31 - __builtin_clz(v) : -1; }
EXPORT_C TInt __e32_find_ls1_32(TUint32 v)
    { return v ? __builtin_ctz(v) : -1; }
EXPORT_C TInt __e32_find_ms1_64(TUint64 v)
    { return v ? 63 - __builtin_clzll(v) : -1; }
EXPORT_C TInt __e32_find_ls1_64(TUint64 v)
    { return v ? __builtin_ctzll(v) : -1; }
/* SWAR popcount -- avoids libgcc's __popcount{si,di}2 in this freestanding link. */
EXPORT_C TInt __e32_bit_count_32(TUint32 v)
    {
    v = v - ((v >> 1) & 0x55555555u);
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    v = (v + (v >> 4)) & 0x0f0f0f0fu;
    return (TInt)((v * 0x01010101u) >> 24);
    }
EXPORT_C TInt __e32_bit_count_64(TUint64 v)
    {
    return __e32_bit_count_32((TUint32)v) + __e32_bit_count_32((TUint32)(v >> 32));
    }

/* ---- barriers -------------------------------------------------------------- */
EXPORT_C void __e32_memory_barrier(void)         { __sync_synchronize(); }
EXPORT_C void __e32_io_completion_barrier(void)  { __sync_synchronize(); }

} // extern "C"
