// SPDX-License-Identifier: Apache-2.0
// t_a64_serial.cpp (NEOCOM.EXE) -- the debug serial port through Symbian's serial LDD (RBusDevComm unit 1, ECOMM +
// the board PDD, common/specific/a64_ecomm.cpp). The host (tools/test-serial-qemu.py) reads "SERIAL READY", sends the
// 256 byte values and must get them back. Checks: open, a second open refused, config, signals, write, read, echo,
// a cancelled read, close, reopen. The result is one RDebug line on the console: "[serial] PASS" or "[serial] FAIL (e)".
#include <e32base.h>
#include <e32debug.h>
#include <d32comm.h>

static TInt Wait(RBusDevComm& aPort, TRequestStatus& aRequest, TBool aRead)	// 30 s, then cancel
	{
	RTimer timer;
	TInt r = timer.CreateLocal();
	if (r != KErrNone)
		{
		aRead ? aPort.ReadCancel() : aPort.WriteCancel();
		User::WaitForRequest(aRequest);
		return r;
		}
	TRequestStatus timeout;
	timer.After(timeout, 30000000);
	User::WaitForRequest(aRequest, timeout);
	if (aRequest == KRequestPending)
		{
		aRead ? aPort.ReadCancel() : aPort.WriteCancel();
		User::WaitForRequest(aRequest);
		r = KErrTimedOut;
		}
	else
		r = aRequest.Int();
	timer.Cancel();
	User::WaitForRequest(timeout);
	timer.Close();
	return r;
	}

static TInt Check(RBusDevComm& aPort)
	{
	TCommConfig config;
	aPort.Config(config);
	config().iRate = EBps115200;
	config().iDataBits = EData8;
	config().iStopBits = EStop1;
	config().iParity = EParityNone;
	config().iHandshake = 0;
	TInt r = aPort.SetConfig(config);
	if (r != KErrNone)
		return r;
	RBusDevComm other;
	r = other.Open(1);
	other.Close();
	if (r != KErrInUse)
		return KErrGeneral - 100;							// a second open must be refused
	const TUint need = KSignalCTS | KSignalDSR | KSignalDCD;	// the PDD's permanent virtual cable
	if ((aPort.Signals() & need) != need)
		return KErrGeneral - 101;
	TRequestStatus status;
	_LIT8(KReady, "SERIAL READY\n");
	aPort.Write(status, KReady);
	r = Wait(aPort, status, EFalse);
	TBuf8<256> data;
	if (r == KErrNone)
		{
		aPort.Read(status, data, 256);
		r = Wait(aPort, status, ETrue);
		}
	if (r == KErrNone && data.Length() != 256)
		r = KErrCorrupt;
	for (TInt i = 0; r == KErrNone && i < data.Length(); ++i)
		if (data[i] != i)
			r = KErrCorrupt;
	if (r == KErrNone)
		{
		aPort.Write(status, data);
		r = Wait(aPort, status, EFalse);
		}
	if (r == KErrNone)
		{
		aPort.Read(status, data, 1);
		aPort.ReadCancel();
		User::WaitForRequest(status);
		if (status != KErrCancel)
			r = KErrGeneral - 102;
		}
	return r;
	}

GLDEF_C TInt E32Main()
	{
	// the drivers are linked into the kernel and installed at boot: no User::Load*Device (see a64_ecomm.cpp)
	RBusDevComm port;
	TInt r = port.Open(1);
	if (r == KErrNone)
		{
		r = Check(port);
		port.Close();
		}
	if (r == KErrNone)
		{
		r = port.Open(1);									// closed means free again
		port.Close();
		}
	if (r == KErrNone)
		RDebug::Print(_L("[serial] PASS"));
	else
		RDebug::Print(_L("[serial] FAIL (%d)"), r);
	return r;
	}
