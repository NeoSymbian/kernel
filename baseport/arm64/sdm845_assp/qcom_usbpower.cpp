// SPDX-License-Identifier: Apache-2.0
//
// qcom_usbpower.cpp -- power the USB controller and its QUSB2 HS PHY ourselves. ABL does it only for fastboot, so
// after a normal boot (the phone installed) the usb30_prim power domain, its clocks and the PHY are off.
// Every value here is ABL's own, read off this phone in fastboot mode (monitor peek, 2026-11-08): the GCC root clock
// configs and branch enables of the DTB's usb@a6f8800 / phy@88e2000 nodes, and the PHY's 8-bit registers in its
// 0x000-0x2a4 window (qcom,sdm670-qusb2-phy, "qcom,qusb2-v2-phy"). Replaying them needs no driver code from anyone.
// The DTB's offsets (gcc-sdm845 numbering): usb30_prim BCR 0xf000, GDSC 0xf004, master/sleep/mock_utmi branches
// 0xf00c/0xf010/0xf014, master RCG 0xf018, mock_utmi RCG 0xf030, cfg_noc_usb3_prim_axi 0x502c, aggre_usb3_prim_axi
// 0x8201c on SDM845 (0x82010 in the Pixel copy), usb_phy_cfg_ahb2phy 0x6a004, QUSB2 PHY BCR 0x12000, clkref gates 0x8c000-0x8c010.
// The PHY's rails (DTB vdd / vdda-pll / vdda-phy-dpdm = pm660l ldo1 880 mV, pm660 ldo10 1.8 V, pm660l ldo7 3.088 V)
// are RPMh votes: on a normal boot the PLL locked without them (ldo1/ldo10 are shared and on) but no host ever saw the
// phone -- the D+/D- transceiver runs from ldo7, which ABL enables only for fastboot (2026-11-08).
// shortcut: no usb-ddr / apps-usb interconnect vote; added if DMA doesn't reach DDR.
// SDM845 copy (OnePlus 6T, docs fajita.md OP3): the same GCC offsets (gcc-sdm845 serves both SoCs) and the same QUSB2
// v2 PHY, but the rails are PM8998's: l1a 880 mV, l12a 1.8 V, l24a 3.088 V (sdm845-oneplus-common.dtsi usb_1_hsphy).
// shortcut: the PHY values replayed below are the Pixel's (fajita's tuning differs: imp-res 8, HSTX 21.6 mA, 5 %
// pre-emphasis); they run only when ABL left USB off (a normal boot), never on a fastboot RAM boot. Read fajita's
// own in fastboot before an installed boot needs them (OP13).
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_rpmh.h"

TInt UsbVoteMask = 7;
// debug: each step in the log (SDM845: no PHY reads here -- the Pixel's version read the PHY before it was powered)
static void Step(const char* aWhat) { Kern::Printf("[usb] power: %s", aWhat); }
									// which rails A64UsbPowerOn votes (monitor "usb cold <mask>")
static volatile TUint32& G(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }
const TUint32 KGcc = 0x00100000, KQusb = 0x088e2000;

// The QUSB2 v2 PHY init sequence of the 6T's own downstream kernel (live DT qusb@88e2000 qcom,qusb-phy-init-seq, as
// { offset, value } in its order): powered down (0x210 = 0x23), the tuning, powered up (0x210 = 0x22). Its OnePlus
// tuning differs from mainline's table (0x198, 0x220, 0x240, 0x244). shortcut: no eFuse HSTX trim into 0x240[7:4]
// (downstream reads 3 bits at 0x7801e8 bit 25); add it if the host sees a marginal link.
static const TUint16 KPhy[][2] =
	{
	{ 0x210, 0x23 }, { 0x004, 0x03 }, { 0x18c, 0x7c }, { 0x02c, 0x80 }, { 0x184, 0x0a }, { 0x0b4, 0x19 }, { 0x194, 0x40 },
	{ 0x198, 0x12 }, { 0x214, 0x21 }, { 0x220, 0x07 }, { 0x224, 0x58 }, { 0x240, 0x47 }, { 0x244, 0x2b }, { 0x248, 0xca },
	{ 0x24c, 0x04 }, { 0x250, 0x03 }, { 0x23c, 0x00 }, { 0x210, 0x22 },
	};
const TUint32 KPhyPwrCtrl1 = 0x210, KPhyPllStatus = 0x1a0;

static TBool Wait(TUint32 aPhys, TUint32 aMask, TBool aSet)		// up to ~10 ms
	{
	for (TInt i = 0; i < 1000; ++i)
		{
		if (!(G(aPhys) & aMask) == !aSet)
			return ETrue;
		Kern::NanoWait(10000);
		}
	return EFalse;
	}

static TInt Rcg(TUint32 aCmd, TUint32 aCfg)				// root clock: config, then UPDATE until it latches
	{
	G(KGcc + aCmd + 4) = aCfg;
	G(KGcc + aCmd) |= 1;
	return Wait(KGcc + aCmd, 1, EFalse) ? KErrNone : KErrTimedOut;
	}

static TInt Branch(TUint32 aCbcr, TBool aStatus = ETrue)	// clock branch: enable, then CLK_OFF (bit 31) clear
	{
	G(KGcc + aCbcr) |= 1;
	return !aStatus || Wait(KGcc + aCbcr, 1u << 31, EFalse) ? KErrNone : KErrTimedOut;
	}

// thread context, the controller untouched since reset: KErrNone and the PHY's PLL status, or the step that failed
TInt A64UsbPhyInit(TUint32& aPll);

TInt A64UsbPowerOn(TUint32& aPll)
	{
	Step("rail votes (cmd-db lookup, RPMh)");
	if (((UsbVoteMask & 1) && Vrm_Vote("ldoa1", 880) != KErrNone) ||		// the PHY's three rails (see the top)
		((UsbVoteMask & 2) && Vrm_Vote("ldoa12", 1800) != KErrNone) ||
		((UsbVoteMask & 4) && Vrm_Vote("ldoa24", 3088) != KErrNone))
		return -99;
	Step("start");
	G(KGcc + 0xf004) &= ~1u;								// GDSC: SW_COLLAPSE off
	if (!Wait(KGcc + 0xf004, 1u << 31, ETrue))				// PWR_ON
		return -100;
	if (Rcg(0xf018, 0x108) != KErrNone)						// master: GPLL0 / 4.5 (ABL's)
		return -101;
	if (Rcg(0xf030, 0x001) != KErrNone)						// mock_utmi: XO
		return -102;
	const TUint32 branches[] = { 0x0502c, 0x0f00c, 0x8201c, 0x0f010, 0x0f014, 0x6a004 };	// gcc-sdm845 order, aggre at 0x8201c
	for (TUint i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i)
		if (Branch(branches[i]) != KErrNone)
			return -110 - (TInt)i;
	Step("clocks on");
	G(KGcc + 0xf000) = 1;									// controller block reset, pulsed
	Kern::NanoWait(100000);
	G(KGcc + 0xf000) = 0;
	Step("controller reset");
	return A64UsbPhyInit(aPll);
	}

// the QUSB2 PHY from reset, as downstream qusb_phy_init: the GCC block reset pulsed, the sequence above (it powers the
// PHY down, tunes it, powers it up), then CORE_READY (PLL status bit 0). The 6T's ABL leaves the PHY powered down
// (0x210 = 0x23, status 0x04) after `fastboot boot`; a DWC3 endpoint command without its UTMI clock hangs the bus
// and TrustZone resets the phone into CrashDump (docs fajita.md OP3).
TInt A64UsbPhyInit(TUint32& aPll)
	{
	if (!(UsbVoteMask & 8))									// debug: 8 = skip the PHY's reset
		{
		G(KGcc + 0x12000) = 1;								// PHY block reset, pulsed
		Kern::NanoWait(100000);
		G(KGcc + 0x12000) = 0;
		Kern::NanoWait(100000);
		}
	Step("phy reset");
	for (TUint i = 0; i < sizeof(KPhy) / sizeof(KPhy[0]); ++i)
		G(KQusb + KPhy[i][0]) = KPhy[i][1];
	Step("phy table written");
	Kern::NanoWait(1000000);								// the PLL locks within ~150 us
	aPll = G(KQusb + KPhyPllStatus) & 0xff;
	return (aPll & 1) ? KErrNone : KErrNotReady;			// CORE_READY
	}

// the reverse, for the monitor's "usb cold": branches gated, the power domain collapsed, the PHY's 3.1 V rail voted off
void A64UsbPowerOff()
	{
	G(KQusb + KPhyPwrCtrl1) |= 1;
	const TUint32 branches[] = { 0x0f014, 0x0f010, 0x0f00c, 0x6a004, 0x8201c, 0x0502c };
	for (TUint i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i)
		G(KGcc + branches[i]) &= ~1u;
	G(KGcc + 0xf004) |= 1;									// SW_COLLAPSE
	Wait(KGcc + 0xf004, 1u << 31, EFalse);
	Vrm_Off("ldoa24");
	}

static void Fmt(TDes8& aOut, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aOut, aFmt, list);
	VA_END(list);
	}

// GCC and PHY against ABL's fastboot values: the first registers that differ
void A64UsbPowerDiag(TDes8& aOut)
	{
	Fmt(aOut, "gdsc %08x cbcr %08x/%08x/%08x rcg %08x/%08x bcr %08x/%08x; phy diffs:", G(KGcc + 0xf004),
		G(KGcc + 0xf00c), G(KGcc + 0x6a004), G(KGcc + 0x8201c), G(KGcc + 0xf01c), G(KGcc + 0xf034), G(KGcc + 0xf000),
		G(KGcc + 0x12000));
	TInt n = 0;
	for (TUint i = 0; i < sizeof(KPhy) / sizeof(KPhy[0]); ++i)
		{
		const TUint32 v = G(KQusb + KPhy[i][0]) & 0xff;
		if (v != KPhy[i][1] && ++n <= 6)
			Fmt(aOut, " %03x=%02x(%02x)", KPhy[i][0], v, KPhy[i][1]);
		}
	Fmt(aOut, " [%d]", n);
	}
