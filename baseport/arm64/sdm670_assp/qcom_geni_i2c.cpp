// SPDX-License-Identifier: Apache-2.0
//
// qcom_geni_i2c.cpp -- an I2C master on a Qualcomm GENI serial engine (QUPv3), after Linux
// drivers/i2c/busses/i2c-qcom-geni.c and drivers/soc/qcom/qcom-geni-se.c in FIFO mode: geni_se_init +
// geni_se_config_packing(8 bits, 4 per word, msb-to-lsb) + qcom_geni_i2c_conf, then per message an M command
// (I2C_WRITE / I2C_READ with the address in the params, STOP_STRETCH for a repeated start) and the FIFOs.
// shortcut: polled with a bounded spin, one engine; upgrade to the SE interrupt (DTB SPI 354) if touch polling costs.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_geni_i2c.h"

enum
	{
	EForceDefault = 0x20, EOutputCtrl = 0x24, ECgcCtrl = 0x28, EStatus = 0x40, EMClkCfg = 0x48, EFwRev = 0x68,
	EClkSel = 0x7c, EByteGran = 0x254, EDmaModeEn = 0x258, ETxPack0 = 0x260, ETxPack1 = 0x264, EI2cTxLen = 0x26c,
	EI2cRxLen = 0x270, EI2cSclCounters = 0x278, ERxPack0 = 0x284, ERxPack1 = 0x288, EMCmd0 = 0x600,
	EMCmdCtrl = 0x604, EMIrqStatus = 0x610, EMIrqEn = 0x614, EMIrqClear = 0x618, ESIrqClear = 0x648,
	ETxFifo = 0x700, ERxFifo = 0x780, ETxFifoStatus = 0x800, ERxFifoStatus = 0x804, ERxWatermark = 0x810,
	ERxRfrWatermark = 0x814, EDmaTxIrqClr = 0xc44, EDmaRxIrqClr = 0xd44, EGsiEventEn = 0xe18, EIrqEn = 0xe1c,
	EHwParam0 = 0xe24, EDmaGeneralCfg = 0xe30,
	};
enum { EI2cWrite = 1, EI2cRead = 2 };
const TUint32 KStopStretch = 1u << 2, KSlvAddrShift = 9;
const TUint32 KMDone = 1u << 0, KMErr = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5), KMNack = 1u << 10,
	KMBusErr = (1u << 12) | (1u << 13);
const TUint32 KMIrqFifo = KMDone | KMErr | (0x3fu << 9) | (1u << 26) | (1u << 27) | (1u << 30);
const TInt KSpin = 2000000;								// ~a few ms at most; a byte at 100 kHz is ~90 us

static TLinAddr Se;
static TUint32 TxDepth;

static volatile TUint32& R(TUint32 aOff) { return *(volatile TUint32*)(Se + aOff); }

int I2c_Init(unsigned long aSe)
	{
	Se = A64_Io(aSe);
	TUint32 proto = (R(EFwRev) >> 8) & 0xff;
	if (proto != 3)
		return (int)proto;									// not I2C firmware: leave the engine alone
	TxDepth = (R(EHwParam0) >> 16) & 0x3f;
	// geni_se_irq_clear + geni_se_io_init + geni_se_io_set_mode
	R(EGsiEventEn) = 0;
	R(EMIrqClear) = 0xffffffff;
	R(ESIrqClear) = 0xffffffff;
	R(EDmaTxIrqClr) = 0xffffffff;
	R(EDmaRxIrqClr) = 0xffffffff;
	R(ECgcCtrl) |= 0x7f;									// DEFAULT_CGC_EN
	R(EDmaGeneralCfg) |= 0xf;								// AHB_SEC_SLV, DMA_AHB_SLV_CFG, DMA TX/RX clock CGC on
	R(EOutputCtrl) = 0x7f;
	R(EForceDefault) = 1;
	R(EIrqEn) |= 0xf;										// M, S, DMA TX/RX: the status bits we poll
	R(EDmaModeEn) &= ~1u;									// FIFO mode
	R(EGsiEventEn) = 0;
	R(ERxWatermark) = TxDepth - 1;
	R(ERxRfrWatermark) = TxDepth;
	R(EMIrqEn) |= KMIrqFifo;
	// 8-bit words, 4 per FIFO entry, first byte in the low bits (geni_se_config_packing(8, 4, true, ...))
	R(ETxPack0) = 0x0007f8fe;
	R(ETxPack1) = 0x000ffefe;
	R(ERxPack0) = 0x0007f8fe;
	R(ERxPack1) = 0x000ffefe;
	R(EByteGran) = 0;
	// 100 kHz from 19.2 MHz: divider 7, SCL high 10, low 11, cycle 26 (geni_i2c_clk_map)
	R(EClkSel) = 0;
	R(EMClkCfg) = (7u << 4) | 1;
	R(EI2cSclCounters) = (10u << 20) | (11u << 10) | 26;
	return (int)proto;
	}

// wait for the M command to finish, moving RX data meanwhile: KErrNone, KErrNotFound (NACK) or an error
static TInt Finish(TUint8* aR, TInt aRn)
	{
	TInt got = 0;
	for (TInt i = 0; i < KSpin; ++i)
		{
		TUint32 st = R(EMIrqStatus);
		for (TUint32 n = R(ERxFifoStatus) & 0x1ffffff; n && aR; --n)
			{
			TUint32 w = R(ERxFifo);
			for (TInt b = 0; b < 4 && got < aRn; ++b, w >>= 8)
				aR[got++] = (TUint8)w;
			}
		if (st & (KMDone | KMErr))
			{
			R(EMIrqClear) = st;
			if (st & KMNack)
				return KErrNotFound;
			if (st & (KMErr | KMBusErr))
				return KErrGeneral;
			return (aR && got < aRn) ? KErrUnderflow : KErrNone;
			}
		}
	R(EMCmdCtrl) = 1u << 1;									// M_GENI_CMD_ABORT
	for (TInt i = 0; i < KSpin && !(R(EMIrqStatus) & (1u << 5)); ++i)
		{}
	R(EMIrqClear) = 0xffffffff;
	return KErrTimedOut;
	}

int I2c_WriteRead(unsigned aAddr, const unsigned char* aW, int aWn, unsigned char* aR, int aRn)
	{
	if (!Se || aWn > 4 * (TInt)TxDepth)
		return KErrArgument;
	TUint32 addr = (aAddr & 0x7f) << KSlvAddrShift;
	R(EMIrqClear) = 0xffffffff;
	if (aWn)
		{
		R(EI2cTxLen) = aWn;
		R(EMCmd0) = (EI2cWrite << 27) | addr | (aRn ? KStopStretch : 0);
		for (TInt i = 0; i < aWn; i += 4)
			{
			TUint32 w = 0;
			for (TInt b = 0; b < 4 && i + b < aWn; ++b)
				w |= (TUint32)aW[i + b] << (8 * b);
			R(ETxFifo) = w;
			}
		TInt r = Finish(NULL, 0);
		if (r != KErrNone || !aRn)
			return r;
		}
	R(EI2cRxLen) = aRn;
	R(EMCmd0) = (EI2cRead << 27) | addr;
	return Finish(aR, aRn);
	}
