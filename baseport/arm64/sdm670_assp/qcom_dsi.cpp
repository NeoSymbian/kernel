// SPDX-License-Identifier: Apache-2.0
// qcom_dsi.cpp -- DCS commands to the panel on DSI0 (SDM670, DSI 6G v2.2), through the command DMA.
// ABL leaves DSI0 in command mode with the DPU pushing the splash framebuffer (D2 display B0: CTRL 0x1f5, command
// DMA from memory, low power, software trigger). We keep all of that and only queue our own packets between frames.
// Register facts: Linux drivers/gpu/drm/msm/registers/display/dsi.xml and dsi/dsi_host.c (read, not copied): the
// 6G controller registers start 4 bytes past HW_VERSION; a packet in memory is the MIPI header bytes 1, 2, 0, then a
// flags byte (bit 7 last, bit 6 long), the payload, padded with 0xff to a multiple of 4.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_dsi.h"

static const TUint32 KDsi = 0x0ae94004;					// DSI0 controller registers (after 6G_HW_VERSION)
enum { ECtrl = 0x00, EDmaBase = 0x44, EDmaLen = 0x48, ETrigDma = 0x8c, EIntrCtrl = 0x10c };
static const TUint32 KCmdMode = 1u << 2, KDmaDone = 1u << 0;
static const TUint32 KIntrMasks = 0x02222202;				// INTR_CTRL's mask bits: kept when a status is acked
// the packet page: ABL's own (DMA_BASE 0x9a027000 at handoff, RAM outside our bank), which the DSI DMA can read
static const TUint32 KPacketPhys = 0x9a027000;

static inline volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KDsi + aOff); }

TInt Dsi_DcsWrite(const TUint8* aData, TInt aLen)
	{
	if (aLen < 1 || aLen > 15)
		return KErrArgument;
	if (!(R(ECtrl) & KCmdMode))
		return KErrNotReady;
	const TBool isLong = aLen > 2;
	// MIPI header: data type (0x05 no parameter, 0x15 one, 0x39 long write) on virtual channel 0, then two bytes
	const TUint8 type = isLong ? 0x39 : aLen == 2 ? 0x15 : 0x05;
	TUint8 p[20];
	p[0] = isLong ? (TUint8)aLen : aData[0];			// header byte 1: WC low, or the command
	p[1] = isLong ? 0 : aLen == 2 ? aData[1] : 0;		// header byte 2: WC high, or the parameter
	p[2] = type;										// header byte 0
	p[3] = (TUint8)(0x80 | (isLong ? 0x40 : 0));		// last packet; long
	TInt len = 4;
	if (isLong)
		{
		memcpy(p + 4, aData, aLen);
		len += aLen;
		}
	while (len & 3)
		p[len++] = 0xff;
	volatile TUint32* page = (volatile TUint32*)A64_Io(KPacketPhys);
	for (TInt i = 0; i < len / 4; ++i)
		page[i] = p[4 * i] | (p[4 * i + 1] << 8) | (p[4 * i + 2] << 16) | ((TUint32)p[4 * i + 3] << 24);
	__asm__ __volatile__("dsb sy" ::: "memory");		// the packet is in memory before the DMA reads it
	R(EIntrCtrl) = (R(EIntrCtrl) & KIntrMasks) | KDmaDone;	// ack a stale DMA done only
	R(EDmaBase) = KPacketPhys;
	R(EDmaLen) = len;
	R(ETrigDma) = 1;
	for (TInt ms = 0; ms < 200; ++ms)
		{
		if (R(EIntrCtrl) & KDmaDone)
			{
			R(EIntrCtrl) = (R(EIntrCtrl) & KIntrMasks) | KDmaDone;
			return KErrNone;
			}
		NKern::Sleep(NKern::TimerTicks(1));
		}
	Kern::Printf("[dsi] DCS %02x: no DMA done in 200 ms (intr %08x)", aData[0], R(EIntrCtrl));
	return KErrTimedOut;
	}
