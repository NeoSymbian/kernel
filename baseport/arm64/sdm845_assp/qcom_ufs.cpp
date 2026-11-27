// SPDX-License-Identifier: Apache-2.0
//
// qcom_ufs.cpp -- Qualcomm's glue around the UFS host on SDM845 (DTB ufshc@1d84000, "qcom,ufshc", a v3 core): the
// vendor hooks of ufshcd.h. Supplies, the power domain, GCC clocks and resets, the device's reset pad, the host's
// vendor registers and UniPro tweaks, the device quirks. The sequence is Linux ufs-qcom.c's for a v3 core (design and
// values only; docs fajita/ufs-linux-reference.md). The PHY is qcom_ufs_phy.cpp; the generic part ufshcd.cpp.
#include <kernel/kern_priv.h>
#include "board.h"
#include "ufshcd.h"
#include "qcom_ufs_phy.h"
#include "qcom_rpmh.h"
#include "qcom_smmu.h"

const TUint32 KGcc = 0x00100000, KSmmu = 0x15000000;
const TUint32 KResetPad = 0x03d9f000;					// TLMM's ufs_reset pad: control, then I/O (bit 0 = out) at +4
// GCC (gcc-sdm845.c): BCR, GDSC, clock branches (bit 31 = CLK_OFF), root clocks (CMD; CFG at +4)
enum { EBcr = 0x77000, EGdsc = 0x77004, EAxi = 0x7700c, EAhb = 0x77010, ETxSym0 = 0x77014, ERxSym0 = 0x77018,
	EAxiRcg = 0x7701c, EUnipro = 0x77054, EIce = 0x77058, EUniproRcg = 0x77074, EAux = 0x7708c, ERxSym1 = 0x770a8,
	EAggreAxi = 0x82024, EClkref = 0x8c000 };
// RCG configurations from GPLL0's main output (source 1): divider field 2 x div - 1
const TUint32 KAxi200MHz = (1 << 8) | 5, KUnipro150MHz = (1 << 8) | 7;
// the host's vendor registers
enum { EUfsCfg1 = 0xdc, EUfsCfg2 = 0xe0, EHwVersion = 0xe4, EAh8Cfg = 0xfc };
enum { ECfg1QunuproSel = 1 << 0, ECfg1DevRefClk = 1 << 26 };
enum { EVendorSamsung = 0x1ce, EVendorHynix = 0x1ad };

static volatile TUint32& Gcc(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KGcc + aOff); }
static TBool On(TUint32 aBranch) { return !(Gcc(aBranch) >> 31); }

static TBool Wait(TUint32 aOff, TUint32 aMask, TBool aSet, TInt aUs = 10000)
	{
	for (TInt t = 0; t <= aUs; t += 10)
		{
		if (!(Gcc(aOff) & aMask) == !aSet)
			return ETrue;
		Ufs_Delay(10);
		}
	return EFalse;
	}

static TInt Branch(TUint32 aCbcr, TBool aStatus = ETrue)	// enable, then CLK_OFF clear (symbol clocks: no check)
	{
	Gcc(aCbcr) |= 1;
	return !aStatus || Wait(aCbcr, 1u << 31, EFalse) ? KErrNone : KErrTimedOut;
	}

static TInt Rcg(TUint32 aCmd, TUint32 aCfg)				// root clock: configuration, then UPDATE until it latches
	{
	if ((Gcc(aCmd + 4) & 0x71f) == aCfg)
		return KErrNone;
	Ufs_Log("rcg %05x: cfg %08x -> %03x", aCmd, Gcc(aCmd + 4), aCfg);
	Gcc(aCmd + 4) = (Gcc(aCmd + 4) & ~0x71fu) | aCfg;
	Gcc(aCmd) |= 1;
	return Wait(aCmd, 1, EFalse) ? KErrNone : KErrTimedOut;
	}

TBool UfsVop_Clocked() { return (Gcc(EGdsc) >> 31) && On(EAhb) && On(EAxi) && On(EUnipro); }

TInt UfsVop_PowerUp()
	{
	// the supplies (live DT: vcc l20, vccq2 s4, PHY l1 + l26, reference clock l2), at the voltages ABL leaves
	TInt r = Vrm_Vote("ldoa20", 2960);
	if (r == KErrNone) r = Vrm_Vote("smpa4", 1800);
	if (r == KErrNone) r = Vrm_Vote("ldoa1", 880);
	if (r == KErrNone) r = Vrm_Vote("ldoa26", 1200);
	if (r == KErrNone) r = Vrm_Vote("ldoa2", 1200);
	if (r != KErrNone)
		{
		Ufs_Log("supply vote -> %d", r);
		return r;
		}
	// data through aggre1 NoC to DDR (SN9; SN0, SH0, MC0 are voted for USB), registers through the config NoC (CN0)
	Ufs_Log("bus votes: SN9 %d", Bcm_Vote("SN9", 300000, 1500000, 16));
	if (!(Gcc(EGdsc) >> 31))
		{
		Gcc(EGdsc) &= ~1u;									// SW_COLLAPSE off
		if (!Wait(EGdsc, 1u << 31, ETrue, 100000))
			{
			Ufs_Log("gdsc stays off: %08x", Gcc(EGdsc));
			return KErrTimedOut;
			}
		}
	const TUint32 branches[] = { EAhb, EAxi, EAggreAxi, EUnipro, EIce, EAux, EClkref };
	for (TUint i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i)
		if ((r = Branch(branches[i])) != KErrNone)
			{
			Ufs_Log("clock %05x stays off: %08x", branches[i], Gcc(branches[i]));
			return r;
			}
	r = Smmu_IdentityMap(0x100, 0);						// KErrInUse: routed already (on this phone, a bypass)
	return r == KErrInUse ? KErrNone : r;
	}

void UfsVop_DeviceReset()								// active low: 10 us asserted, 10 us to settle
	{
	volatile TUint32& io = *(volatile TUint32*)A64_Io(KResetPad + 4);
	io &= ~1u;
	Ufs_Delay(10);
	io |= 1;
	Ufs_Delay(10);
	}

// Linux ufs_qcom_power_up_sequence: the core reset, the PHY off and on, its tables, then the vendor's UniPro select
// and the lane clocks
TInt UfsVop_HcePre()
	{
	TInt r = Rcg(EAxiRcg, KAxi200MHz);
	if (r == KErrNone)
		r = Rcg(EUniproRcg, KUnipro150MHz);					// LinkPre's 1-us count assumes 150 MHz
	if (r != KErrNone)
		return r;
	Gcc(EBcr) |= 1;
	Ufs_Delay(200);
	Gcc(EBcr) &= ~1u;
	Ufs_Delay(1000);
	QmpUfs_PowerOff();
	QmpUfs_PowerOn();
	if ((r = QmpUfs_Calibrate()) != KErrNone)
		return r;
	Ufs_Reg(EUfsCfg1) |= ECfg1QunuproSel;
	Branch(ETxSym0, EFalse);
	Branch(ERxSym0, EFalse);
	Branch(ERxSym1, EFalse);
	return KErrNone;
	}

static TInt DmeOr(TUint32 aAttr, TUint32 aBits)
	{
	TUint32 v;
	TInt r = Ufs_DmeGet(aAttr, 0, v);
	return r == KErrNone ? Ufs_DmeSet(aAttr, 0, v | aBits) : r;
	}

TInt UfsVop_HcePost()
	{
	TUint32 fsm = 0;
	for (TInt t = 0; t < 100000; t += 200)					// the PHY's TX FSM in HIBERN8 (1)
		{
		if (Ufs_DmeGet(0x0041, 0, fsm) == KErrNone && fsm == 1)
			break;
		Ufs_Delay(200);
		}
	if (fsm != 1)
		{
		Ufs_Log("tx fsm %d, not HIBERN8", fsm);
		return KErrTimedOut;
		}
	Ufs_Reg(EUfsCfg2) |= 0xff;								// the host's hardware clock gating
	TInt r = DmeOr(0xa00b, 0x3ff);							// DL_VS_CLK_CFG
	if (r == KErrNone)
		r = DmeOr(0x9004, 0x1ff);							// PA_VS_CLK_CFG_REG
	if (r == KErrNone)
		r = DmeOr(0xd002, 1 << 9);							// DME_VS_CORE_CLK_CTRL: DME_HW_CGC_EN
	return r;
	}

TInt UfsVop_LinkPre()
	{
	TUint32 v;
	TInt r = Ufs_DmeGet(0xd002, 0, v);						// DME_VS_CORE_CLK_CTRL: unipro cycles per us, no divider
	if (r == KErrNone)
		r = Ufs_DmeSet(0xd002, 0, (v & ~0x1ffu) | 150);
	if (r == KErrNone)
		r = Ufs_DmeSet(0x155e, 0, 0);						// PA_LOCAL_TX_LCC_ENABLE
	return r;
	}

TInt UfsVop_LinkPost()
	{
	if (Ufs_Reg(0x00) & (1 << 23))							// CAP.AH8: the clock requests auto-hibern8 may drop
		Ufs_Reg(EAh8Cfg) |= 0x3c;
	return KErrNone;
	}

TInt UfsVop_GearPre(TUint32 aVendor)
	{
	Ufs_Reg(EUfsCfg1) |= ECfg1DevRefClk;
	Ufs_Delay(1);
	TInt r = KErrNone;
	if (aVendor == EVendorSamsung)
		{
		r = Ufs_DmeSet(0x1552, 0, 0x4a, ETrue);				// the peer's PA_TXHSG1SYNCLENGTH
		for (TUint32 lane = 0; r == KErrNone && lane < 2; ++lane)
			r = Ufs_DmeSet(0x0037, lane, 0);				// TX_HS_EQUALIZER
		}
	else if (aVendor == EVendorHynix)
		r = DmeOr(0x9000, 1 << 12);							// PA_VS_CONFIG_REG1
	Ufs_Log("gear quirks for %04x -> %d", aVendor, r);
	return r;
	}

void UfsVop_Dump()
	{
	Kern::Printf("[ufs] gcc: gdsc %08x ahb %08x axi %08x unipro %08x ice %08x aux %08x tx0 %08x rx0 %08x rx1 %08x "
		"aggre %08x clkref %08x", Gcc(EGdsc), Gcc(EAhb), Gcc(EAxi), Gcc(EUnipro), Gcc(EIce), Gcc(EAux), Gcc(ETxSym0),
		Gcc(ERxSym0), Gcc(ERxSym1), Gcc(EAggreAxi), Gcc(EClkref));
	Kern::Printf("[ufs] rcg: axi %08x/%08x unipro %08x/%08x; reset pad %08x/%08x", Gcc(EAxiRcg), Gcc(EAxiRcg + 4),
		Gcc(EUniproRcg), Gcc(EUniproRcg + 4), *(volatile TUint32*)A64_Io(KResetPad),
		*(volatile TUint32*)A64_Io(KResetPad + 4));
	// the stream matches that cover UFS's streams 0x100-0x10f
	TInt smrs = *(volatile TUint32*)A64_Io(KSmmu + 0x20) & 0xff;
	for (TInt n = 0; n < smrs; ++n)
		{
		TUint32 smr = *(volatile TUint32*)A64_Io(KSmmu + 0x800 + 4 * n);
		TUint32 id = smr & 0x7fff, mask = (smr >> 16) & 0x7fff;
		if ((smr >> 31) && !((id ^ 0x100) & ~(mask | 0xf) & 0x7fff))
			Kern::Printf("[ufs] smmu: smr %d = %08x, s2cr %08x", n, smr, *(volatile TUint32*)A64_Io(KSmmu + 0xc00 + 4 * n));
		}
	if (UfsVop_Clocked())
		Kern::Printf("[ufs] vendor: cfg1 %08x cfg2 %08x ah8 %08x", Ufs_Reg(EUfsCfg1), Ufs_Reg(EUfsCfg2), Ufs_Reg(EAh8Cfg));
	if (On(EAux))
		QmpUfs_Dump();
	}
