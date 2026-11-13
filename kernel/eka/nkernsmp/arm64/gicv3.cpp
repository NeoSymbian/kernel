// SPDX-License-Identifier: Apache-2.0
// gicv3.cpp -- see gicv3.h. Group 1 non-secure only; affinity routing (ARE) on; SGIs/PPIs per redistributor.
#include "gicv3.h"
typedef unsigned long long TGicU64;

#define REG32(a)	(*(volatile TGicU32*)(unsigned long)(a))
#define REG64(a)	(*(volatile TGicU64*)(unsigned long)(a))
#define MSR(r, v)	__asm__ __volatile__("msr " #r ", %0" :: "r"((TGicU64)(v)) : "memory")
#define MRS(r)		({ TGicU64 _v; __asm__ __volatile__("mrs %0, " #r : "=r"(_v) :: "memory"); _v; })
#define ISB()		__asm__ __volatile__("isb" ::: "memory")

enum { GICD_CTLR = 0x0, GICD_TYPER = 0x4, GICD_IGROUPR = 0x80, GICD_ISENABLER = 0x100, GICD_ICENABLER = 0x180,
	   GICD_IPRIORITYR = 0x400, GICD_IROUTER = 0x6000,
	   GICR_TYPER = 0x8, GICR_WAKER = 0x14, GICR_SGI = 0x10000, GICR_STRIDE = 0x20000,
	   GICR_IGROUPR0 = 0x80, GICR_ISENABLER0 = 0x100, GICR_ICENABLER0 = 0x180, GICR_IPRIORITYR = 0x400 };


static void WaitRwp(unsigned long aCtlr) { while (REG32(aCtlr) & (1u << 31)) {} }

void Gicv3_InitDist(unsigned long aGicd)
	{
	REG32(aGicd + GICD_CTLR) = 0;
	WaitRwp(aGicd + GICD_CTLR);
	TGicU32 lines = ((REG32(aGicd + GICD_TYPER) & 0x1F) + 1) * 32;
	for (TGicU32 i = 32; i < lines; i += 32)
		{
		REG32(aGicd + GICD_ICENABLER + i / 8) = ~0u;	// SPIs off until a driver binds one
		REG32(aGicd + GICD_IGROUPR + i / 8) = ~0u;		// all group 1
		}
	REG32(aGicd + GICD_CTLR) = (1u << 4) | (1u << 1);	// ARE, EnableGrp1 (security-disabled / NS view)
	WaitRwp(aGicd + GICD_CTLR);
	}

unsigned long Gicv3_InitCpu(unsigned long aGicrBase, unsigned int aMpidrPacked)
	{
	for (unsigned long rd = aGicrBase; ; rd += GICR_STRIDE)
		{
		TGicU64 typer = REG64(rd + GICR_TYPER);
		if ((TGicU32)(typer >> 32) == aMpidrPacked)
			{
			REG32(rd + GICR_WAKER) &= ~(1u << 1);			// ProcessorSleep = 0
			while (REG32(rd + GICR_WAKER) & (1u << 2)) {}	// wait ChildrenAsleep = 0
			unsigned long sgi = rd + GICR_SGI;
			REG32(sgi + GICR_ICENABLER0) = ~0u;
			REG32(sgi + GICR_IGROUPR0) = ~0u;
			MSR(S3_0_C12_C12_5, MRS(S3_0_C12_C12_5) | 1);	// ICC_SRE_EL1.SRE
			ISB();
			MSR(S3_0_C4_C6_0, 0xFF);					// ICC_PMR_EL1: unmask all priorities
			MSR(S3_0_C12_C12_3, 0);						// ICC_BPR1_EL1
			MSR(S3_0_C12_C12_7, 1);						// ICC_IGRPEN1_EL1
			ISB();
			return sgi;
			}
		if (typer & (1u << 4))							// GICR_TYPER.Last
			return 0;
		}
	}

void Gicv3_EnablePpiSgi(unsigned long aRdSgi, unsigned int aIntId, unsigned char aPriority)
	{
	*(volatile unsigned char*)(aRdSgi + GICR_IPRIORITYR + aIntId) = aPriority;
	REG32(aRdSgi + GICR_ISENABLER0) = 1u << aIntId;
	}

void Gicv3_EnableSpi(unsigned long aGicd, unsigned int aIntId, unsigned char aPriority, TGicU64 aAffinity)
	{
	*(volatile unsigned char*)(aGicd + GICD_IPRIORITYR + aIntId) = aPriority;
	REG64(aGicd + GICD_IROUTER + 8 * aIntId) = aAffinity;	// Interrupt_Routing_Mode 0: that one CPU
	REG32(aGicd + GICD_ISENABLER + (aIntId / 32) * 4) = 1u << (aIntId % 32);
	}

void Gicv3_MaskSpi(unsigned long aGicd, unsigned int aIntId, int aMask)
	{
	REG32(aGicd + (aMask ? GICD_ICENABLER : GICD_ISENABLER) + (aIntId / 32) * 4) = 1u << (aIntId % 32);
	}

TGicU32 Gicv3_Ack()					{ return (TGicU32)MRS(S3_0_C12_C12_0) & 0xFFFFFF; }	// ICC_IAR1_EL1
void Gicv3_Eoi(TGicU32 aIntId)		{ MSR(S3_0_C12_C12_1, aIntId); }					// ICC_EOIR1_EL1
