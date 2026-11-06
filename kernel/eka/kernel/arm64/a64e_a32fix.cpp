// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: EPL-2.0
//
// a64e_a32fix.cpp -- AArch32 EL0 instructions ARMv8 no longer executes the way the 2010 (ARMv5TE/ARMv6) user side
// expects, emulated on the trap. A port of Phorepoc's AArch32 fix-ups (adaptation-a64 ekern_seam.cpp
// EmulateSwp/EmulateAlign) to an AArch64 EL1 kernel: decoded from ESR_EL1 and the exception frame (r0-r14 = x0-x14,
// pc = ELR, CPSR = SPSR) instead of the AArch32 TArmExcInfo.
//   - SWP/SWPB: UNDEFINED in ARMv8 AArch32 (ESR EC 0x00); euser's locks use it. Emulated with an exclusive loop.
//   - misaligned LDRD/STRD/LDM/STM: these always need alignment, even with SCTLR_EL1.A=0 (data abort, DFSC 0x21
//     alignment). Done through the unprivileged user accessors (LDTR/STTR allow unaligned addresses at A=0).
// Called with interrupts off from A64_ExceptionDispatch; ETrue = handled (pc stepped), EFalse = let the exception
// take its normal course (a panic for user code).
#include <kernel/kern_priv.h>
#include <nk_priv.h>

extern "C" {
TAny* A64_SafeReadUser(const TAny* aSrc, TAny* aDest, TInt aSize);		// a64e_trap.S: 0 or the faulting address
TAny* A64_SafeWriteUser(TAny* aDest, const TAny* aSrc, TInt aSize);
}

static TBool CondPasses(TUint32 aCond, TUint32 aCpsr)
	{
	TBool n = aCpsr >> 31 & 1, z = aCpsr >> 30 & 1, c = aCpsr >> 29 & 1, v = aCpsr >> 28 & 1;
	switch (aCond)
		{
	case 0: return z;			case 1: return !z;
	case 2: return c;			case 3: return !c;
	case 4: return n;			case 5: return !n;
	case 6: return v;			case 7: return !v;
	case 8: return c && !z;		case 9: return !c || z;
	case 10: return n == v;		case 11: return n != v;
	case 12: return !z && n == v;	case 13: return z || n != v;
	default: return ETrue;		// AL (15 is unconditional space, never SWP/LDRD/LDM)
		}
	}

// EL0 may write (aWrite) or read aAddr in the current address space: a stage-1 EL0 translation (AT S1E0W/R)
static TBool UserCanAccess(TUint32 aAddr, TBool aWrite)
	{
	TUint64 par;
	if (aWrite)
		__asm__ __volatile__("at s1e0w, %1\n\tisb\n\tmrs %0, par_el1" : "=r"(par) : "r"((TUint64)aAddr) : "memory");
	else
		__asm__ __volatile__("at s1e0r, %1\n\tisb\n\tmrs %0, par_el1" : "=r"(par) : "r"((TUint64)aAddr) : "memory");
	return !(par & 1);
	}

static TBool EmulateSwp(SThreadExcStack* aF, TUint32 aInsn)
	{
	if ((aInsn & 0x0FB00FF0) != 0x01000090)
		return EFalse;
	TUint32 rn = aInsn >> 16 & 15, rt = aInsn >> 12 & 15, rt2 = aInsn & 15;
	TBool byte = aInsn >> 22 & 1;
	if (rn == 15 || rt == 15 || rt2 == 15)
		return EFalse;
	if (CondPasses(aInsn >> 28, (TUint32)aF->iSpsr))
		{
		TUint32 addr = (TUint32)aF->iX[rn], val = (TUint32)aF->iX[rt2], old, fail;
		if ((!byte && (addr & 3)) || !UserCanAccess(addr, ETrue))
			return EFalse;								// misaligned word SWP faults, as on ARMv7
		if (byte)
			__asm__ __volatile__("1: ldxrb %w0, [%2]\n\tstxrb %w1, %w3, [%2]\n\tcbnz %w1, 1b\n\tdmb ish"
				: "=&r"(old), "=&r"(fail) : "r"((TUint64)addr), "r"(val) : "memory");
		else
			__asm__ __volatile__("1: ldxr %w0, [%2]\n\tstxr %w1, %w3, [%2]\n\tcbnz %w1, 1b\n\tdmb ish"
				: "=&r"(old), "=&r"(fail) : "r"((TUint64)addr), "r"(val) : "memory");
		aF->iX[rt] = old;
		}
	aF->iElr += 4;
	return ETrue;
	}

static TUint32 Reg(const SThreadExcStack* aF, TUint32 aN)
	{
	return aN == 15 ? (TUint32)aF->iElr + 8 : (TUint32)aF->iX[aN];		// ARM state: a read of pc is pc+8
	}

// LDRD/STRD and LDM/STM that hit an alignment fault: the same transfer, byte-safe, through LDTR/STTR
static TBool EmulateAlign(SThreadExcStack* aF, TUint32 aInsn)
	{
	TUint32 words[16];
	TUint32 rn = aInsn >> 16 & 15;
	if (rn == 15)
		return EFalse;
	TBool p = aInsn >> 24 & 1, u = aInsn >> 23 & 1, w = aInsn >> 21 & 1;
	TUint32 base = (TUint32)aF->iX[rn];
	if ((aInsn & 0x0E1000D0) == 0x000000D0)				// LDRD (bits 7-4 1101) / STRD (1111)
		{
		TBool load = (aInsn & 0x20) == 0;
		TUint32 rt = aInsn >> 12 & 15;
		if ((rt & 1) || rt == 14)
			return EFalse;
		TUint32 off = (aInsn >> 22 & 1) ? ((aInsn >> 4 & 0xF0) | (aInsn & 0xF)) : Reg(aF, aInsn & 15);
		TUint32 moved = u ? base + off : base - off;
		TUint32 addr = p ? moved : base;
		if (load)
			{
			if (A64_SafeReadUser((const TAny*)addr, words, 8))
				return EFalse;
			aF->iX[rt] = words[0];
			aF->iX[rt + 1] = words[1];
			}
		else
			{
			words[0] = Reg(aF, rt);
			words[1] = Reg(aF, rt + 1);
			if (A64_SafeWriteUser((TAny*)addr, words, 8))
				return EFalse;
			}
		if (!p || w)
			aF->iX[rn] = moved;
		aF->iElr += 4;
		return ETrue;
		}
	if ((aInsn & 0x0E000000) == 0x08000000 && !(aInsn >> 22 & 1))	// LDM/STM, not the ^ (user bank / SPSR) forms
		{
		TBool load = aInsn >> 20 & 1;
		TUint32 list = aInsn & 0xFFFF;
		TInt count = 0;
		for (TUint32 m = list; m; m &= m - 1)			// no popcount: -mgeneral-regs-only has no CNT, and no libgcc
			++count;
		if (!count)
			return EFalse;
		TUint32 addr = u ? (p ? base + 4 : base) : (p ? base - 4 * count : base - 4 * count + 4);
		TUint32 wb = u ? base + 4 * count : base - 4 * count;
		TInt i, k;
		if (load)
			{
			if (A64_SafeReadUser((const TAny*)addr, words, 4 * count))
				return EFalse;
			if (w)
				aF->iX[rn] = wb;
			for (i = 0, k = 0; i < 16; ++i)
				{
				if (!(list & (1u << i)))
					continue;
				TUint32 v = words[k++];
				if (i == 15)							// ARMv5T: a loaded pc interworks
					{
					if (v & 1)
						aF->iSpsr |= 0x20;
					aF->iElr = v & ~1u;
					}
				else
					aF->iX[i] = v;
				}
			if (!(list & 0x8000))
				aF->iElr += 4;
			return ETrue;
			}
		for (i = 0, k = 0; i < 16; ++i)
			if (list & (1u << i))
				words[k++] = Reg(aF, i);
		if (A64_SafeWriteUser((TAny*)addr, words, 4 * count))
			return EFalse;
		if (w)
			aF->iX[rn] = wb;
		aF->iElr += 4;
		return ETrue;
		}
	return EFalse;
	}

extern "C" TBool A64_A32Fixup(SThreadExcStack* aF)
	{
	if (aF->iSpsr & 0x20)								// Thumb: no fix-ups (the 2010 user side's locks are ARM code)
		return EFalse;
	TUint32 ec = (TUint32)(aF->iEsr >> 26), insn;
	TBool undef = ec == 0x00;
	TBool align = ec == 0x24 && (aF->iEsr & 0x3F) == 0x21;
	if (!undef && !align)
		return EFalse;
	if (A64_SafeReadUser((const TAny*)(TLinAddr)aF->iElr, &insn, 4))
		return EFalse;
	return undef ? EmulateSwp(aF, insn) : EmulateAlign(aF, insn);
	}
