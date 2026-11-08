// SPDX-License-Identifier: Apache-2.0
// qcom_rpmh.h -- a minimal RPMh bus vote (qcom_rpmh.cpp): the command DB lookup and one active-only BCM vote.
#ifndef QCOM_RPMH_H
#define QCOM_RPMH_H
TInt CmdDb_Find(const char* aName, TUint32& aAddr, TUint32 aAux[2]);	// address + bcm_db aux (unit; width | vcd << 16)
// thread context, polled; aOut (optional) gets the report line
TInt Bcm_Vote(const char* aName, TUint32 aAvgKBps, TUint32 aPeakKBps, TUint32 aBusWidth, TDes8* aOut = NULL);
TInt Arc_Vote(const char* aName, TUint32 aVlvl, TDes8* aOut = NULL);	// a rail ("cx.lvl") at level >= aVlvl
TInt Vrm_Vote(const char* aName, TUint32 aMilliVolts, TDes8* aOut = NULL);	// an LDO/SMPS ("ldob7") on, at aMilliVolts, HPM
#endif
