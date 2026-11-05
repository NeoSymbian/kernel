// SPDX-License-Identifier: Apache-2.0
// gicv3.h -- minimal GICv3 driver (distributor, this CPU's redistributor, system-register CPU interface).
// Board-agnostic: the board passes the GICD / first-GICR physical addresses (QEMU virt, SDM670 from the DTB).
#ifndef GICV3_H
#define GICV3_H
typedef unsigned int TGicU32;

void Gicv3_InitDist(unsigned long aGicd);
int  Gicv3_InitCpu(unsigned long aGicrBase, unsigned int aMpidrPacked);	// 0 ok, -1 no redistributor matched
void Gicv3_EnablePpiSgi(unsigned int aIntId, unsigned char aPriority);	// INTID 0-31 on this CPU
TGicU32 Gicv3_Ack();												// ICC_IAR1_EL1 (1023 = spurious)
void Gicv3_Eoi(TGicU32 aIntId);
#endif
