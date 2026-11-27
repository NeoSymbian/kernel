// SPDX-License-Identifier: Apache-2.0
//
// qcom_ufs_phy.cpp -- SDM845's QMP UFS PHY v3 (DTB ufsphy_mem@1d87000, two lanes): power, the calibration tables, start.
// The register values and their order are SDM845's in mainline Linux phy-qcom-qmp-ufs.c (sdm845_ufsphy_*), recorded
// as data in docs fajita/ufs-linux-reference.md; the sequence is Linux's qmp_ufs_power_on (no PCS software reset on
// this PHY: the host's UFS_CFG1 bit 1 holds it in reset while the tables go in).
#include <kernel/kern_priv.h>
#include "board.h"
#include "ufshcd.h"
#include "qcom_ufs_phy.h"

const TUint32 KPhy = 0x01d87000;
enum { ESerdes = 0x000, ETx0 = 0x400, ERx0 = 0x600, ETx1 = 0x800, ERx1 = 0xa00, EPcs = 0xc00 };
enum { EPcsStart = EPcs + 0x000, EPcsPwrDn = EPcs + 0x004, EPcsReady = EPcs + 0x160 };
enum { EUfsCfg1 = 0xdc, ECfg1PhySoftReset = 1 << 1 };

static volatile TUint32& Phy(TUint32 aOff) { return *(volatile TUint32*)(A64_Io(KPhy) + aOff); }

struct TRegVal { TUint16 iOff; TUint8 iVal; };
static const TRegVal KSerdes[] = {
	{ 0x03c, 0x02 }, { 0x034, 0x04 }, { 0x00c, 0x0a }, { 0x048, 0x07 }, { 0x15c, 0x06 }, { 0x080, 0xd5 },
	{ 0x088, 0x20 }, { 0x138, 0x30 }, { 0x13c, 0x00 }, { 0x090, 0x01 }, { 0x0ec, 0x00 }, { 0x154, 0x00 },
	{ 0x0f0, 0x04 }, { 0x164, 0x05 }, { 0x104, 0xff }, { 0x108, 0x00 }, { 0x0b0, 0x82 }, { 0x060, 0x06 },
	{ 0x068, 0x16 }, { 0x070, 0x36 }, { 0x0d8, 0x3f }, { 0x0dc, 0x00 }, { 0x0f4, 0xda }, { 0x0f8, 0x01 },
	{ 0x098, 0xff }, { 0x09c, 0x0c }, { 0x0b4, 0x98 }, { 0x064, 0x06 }, { 0x06c, 0x16 }, { 0x074, 0x36 },
	{ 0x0e0, 0x3f }, { 0x0e4, 0x00 }, { 0x0fc, 0xc1 }, { 0x100, 0x00 }, { 0x0a4, 0x32 }, { 0x0a8, 0x0f } };
static const TRegVal KTx[] = { { 0x08c, 0x06 }, { 0x044, 0x04 }, { 0x048, 0x07 } };
static const TRegVal KRx[] = {
	{ 0x108, 0x24 }, { 0x104, 0x0f }, { 0x10c, 0x1e }, { 0x11c, 0x40 }, { 0x030, 0x0b }, { 0x07c, 0x5b },
	{ 0x0d4, 0x06 }, { 0x0d8, 0x04 }, { 0x0dc, 0x1b }, { 0x024, 0x04 }, { 0x028, 0x04 }, { 0x02c, 0x04 },
	{ 0x034, 0x4b }, { 0x044, 0x81 }, { 0x03c, 0x80 }, { 0x164, 0x59 } };
static const TRegVal KPcs[] = {
	{ 0x140, 0x6e }, { 0x02c, 0x0a }, { 0x034, 0x02 }, { 0x134, 0x03 }, { 0x1bc, 0x43 }, { 0x13c, 0x0f },
	{ 0x138, 0x9a }, { 0x1c4, 0x02 } };
static const TRegVal KSerdesRateB[] = { { 0x0f0, 0x44 } };	// VCO_TUNE_MAP for HS Rate B, written last

#define WRITE_TABLE(aBase, aTable) Write(aBase, aTable, sizeof(aTable) / sizeof(aTable[0]))
static void Write(TUint32 aBase, const TRegVal* aTable, TInt aCount)
	{
	for (TInt i = 0; i < aCount; ++i)
		Phy(aBase + aTable[i].iOff) = aTable[i].iVal;
	}

// the clocks and supplies are the caller's (qcom_ufs.cpp); this is the PHY's own power-down control
void QmpUfs_PowerOff() { Phy(EPcsPwrDn) &= ~1u; }
void QmpUfs_PowerOn() { Phy(EPcsPwrDn) |= 1; }

TInt QmpUfs_Calibrate()
	{
	Ufs_Reg(EUfsCfg1) |= ECfg1PhySoftReset;
	(void)Ufs_Reg(EUfsCfg1);
	Ufs_Delay(1000);
	WRITE_TABLE(ESerdes, KSerdes);
	WRITE_TABLE(ETx0, KTx);
	WRITE_TABLE(ERx0, KRx);
	WRITE_TABLE(ETx1, KTx);
	WRITE_TABLE(ERx1, KRx);
	WRITE_TABLE(EPcs, KPcs);
	WRITE_TABLE(ESerdes, KSerdesRateB);
	Ufs_Reg(EUfsCfg1) &= ~ECfg1PhySoftReset;
	(void)Ufs_Reg(EUfsCfg1);
	Ufs_Delay(1000);
	Phy(EPcsStart) |= 1;									// SERDES start
	for (TInt t = 0; t <= 10000; t += 200)
		{
		if (Phy(EPcsReady) & 1)
			return KErrNone;
		Ufs_Delay(200);
		}
	Ufs_Log("phy: not ready after 10 ms (ready %08x)", Phy(EPcsReady));
	return KErrTimedOut;
	}

void QmpUfs_Dump()											// the caller checked the PHY's clocks
	{
	Kern::Printf("[ufs] phy: start %08x pwrdn %08x ready %08x", Phy(EPcsStart), Phy(EPcsPwrDn), Phy(EPcsReady));
	}
