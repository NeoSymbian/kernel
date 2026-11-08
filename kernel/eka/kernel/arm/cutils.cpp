// Copyright (c) 1994-2009 Nokia Corporation and/or its subsidiary(-ies).
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
// The NeoSymbian Authors (2026): Kern::Restart calls the arm64 board (A64_Restart), the ROM header has no code.
// Description:
// e32\kernel\arm\cutils.cpp
// 
//

//#define __EARLY_DEBUG__

#include "arm_mem.h"
#include <hal.h>
#include <kernel/cache.h>

#ifdef __SMP__
TSpinLock DbgSpinLock(TSpinLock::EOrderBTrace);
#endif

TInt A::VariantHal(TInt aFunction, TAny* a1, TAny* a2)
	{
	return Arm::TheAsic->VariantHal(aFunction,a1,a2);
	}

#ifndef __EARLY_DEBUG__
void DebugOutputChar(TUint aChar)
//
// Transmit one character from the UART (for debugging only)
//
	{
	if ( (TheSuperPage().iDebugPort==Arm::EDebugPortJTAG) && (Arm::DebugOutJTAG(aChar)!=KErrNotSupported) )
		return;
	if(TheSuperPage().iDebugPort==KNullDebugPort)
		return;
	if (Arm::TheAsic)
		Arm::TheAsic->DebugOutput(aChar);
	}
#else
extern void DebugOutputChar(TUint);
#endif

void A::DebugPrint(const TText* aPtr, TInt aLen, TBool aNewLine)
//
// Send the string to kernel trace port (UART)
//
	{
	const TText* pE=aPtr+aLen;
	if (aPtr!=pE)
		{
#ifdef __SMP__
		extern TUint __arm_cpu_number();
		TInt irq = DbgSpinLock.LockIrqSave();
		if (aNewLine)
			{
			DebugOutputChar(__arm_cpu_number()+0x30);
			DebugOutputChar(0x3a);
			}
#endif
		do
			DebugOutputChar(*aPtr++);
		while(aPtr<pE);

		if (aNewLine)
			{
			DebugOutputChar(0x0d);
			DebugOutputChar(0x0a);
			}
#ifdef __SMP__
		DbgSpinLock.UnlockIrqRestore(irq);
#endif
		}
	}
	
/**
Gets a pointer to the Asic object for the ARM platform.

@return  A pointer to the Asic object.
*/
EXPORT_C Asic* Arch::TheAsic()
	{
	return Arm::TheAsic;
	}

TPtr8 A::MachineConfiguration()
	{
	return Arm::TheAsic->MachineConfiguration();
	}

TInt A::SystemTimeInSecondsFrom2000(TInt& aTime)
	{
	return Arm::TheAsic->SystemTimeInSecondsFrom2000(aTime);
	}

TInt A::SetSystemTimeInSecondsFrom2000(TInt aTime)
	{
	return Arm::TheAsic->SetSystemTimeInSecondsFrom2000(aTime);
	}

TInt K::FloatingPointTypes(TUint32& aTypes)
	{
#ifdef __CPU_HAS_VFP
#ifdef __VFP_V3
	if (Arm::NeonPresent())
		aTypes = EFpTypeVFPv3;
	else
		aTypes = EFpTypeVFPv3D16;
	return KErrNone;
#else 
	aTypes = EFpTypeVFPv2;
	return KErrNone;
#endif 
#else
	aTypes = EFpTypeNone;
	return KErrNotSupported;
#endif
	}


/** 
Restart the system. 
On hardware targets this calls the Restart Vector in the ROM Header.
Note, aMode is set to zero when this function is used by Kern::Fault()

@param aMode Argument passed to the restart routine. The meaning of this value
depends on the bootstrap implementation. 		
*/
#ifdef __CPU_ARM64
extern "C" void A64_Restart(int aMode);	// NeoSymbian, board.h: a64boot.cpp builds the ROM header as data, with no restart code in it
#endif
EXPORT_C void Kern::Restart(TInt aMode)
	{
#ifdef __CPU_ARM64
	A64_Restart(aMode);
#else
	TLinAddr f = (TLinAddr)&TheRomHeader().iRestartVector;
	(*(void (*)(TInt))f)(aMode);
#endif
	}


#ifdef __REQUEST_COMPLETE_MACHINE_CODED__
#if defined(_DEBUG)
extern "C" void __DebugMsgKernRequestComplete(TInt a0, TInt a1)
	{
	DThread* pT=(DThread*)a0;
	__KTRACE_OPT(KDATAPAGEWARN,Kern::Printf("Data paging: Use of deprecated Kern::RequestComplete API by %O at %08x", pT, a1));
	}
#endif
#endif

extern "C" void NKCrashHandler(TInt aPhase, const TAny* a0, TInt a1)
	{
	if (aPhase==0)
		{
		Cache::CpuRetires();
		return;
		}
	Cache::KernelRetires();
#ifdef __SMP__
	SFullArmRegSet* regs = SubScheduler().iSSX.iRegs;
#else
	SFullArmRegSet* regs = (SFullArmRegSet*)TheScheduler.i_Regs;
#endif
	TLinAddr ret = 0xffffffff;
	switch (regs->iN.iFlags & 0x1f)
		{
		case 0x10:
		case 0x1f:
					ret = regs->iN.iR14;		break;
		case 0x11:	ret = regs->iN.iR14Fiq;		break;
		case 0x12:	ret = regs->iN.iR14Irq;		break;
		case 0x13:	ret = regs->iN.iR14Svc;		break;
		case 0x17:	ret = regs->iN.iR14Abt;		break;
		case 0x1b:	ret = regs->iN.iR14Und;		break;
		}
	if (a1 != K::ESystemException)
		TheSuperPage().iKernelExcInfo.iCodeAddress = (TAny*)ret;
	K::DoFault(a0,a1);
	}


