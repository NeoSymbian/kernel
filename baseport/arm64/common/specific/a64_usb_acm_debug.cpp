// SPDX-License-Identifier: Apache-2.0
//
// a64_usb_acm_debug.cpp -- fshell's serial port on the phone (USBDEBUG=1 builds; docs debug-access.md): a second CDC-ACM
// function (interfaces 2/3; EP 0x03 OUT, 0x83 IN, 0x84 notify) on the device controller core (a64_usb_function.h), and
// the ECOMM PDD over it (Comm.UsbAcm, unit 1; installed by a64_ecomm.cpp): RBusDevComm::Open(1), fshell's
// vt100busdevcons "pdd=euart1,ldd=ecomm,port=1,waitdcd=1". The host's DTR is the carrier (DCD).
// Two contexts: the USB poll (a DFC) moves packets between the endpoints and two byte queues; the doorbell ISR
// (A64Usb_Doorbell: an interrupt, as ECOMM's ...Isr callbacks need) moves bytes between the queues and ECOMM.
// One spinlock guards the queues, never held while calling ECOMM (whose own lock is IrqLow3).
// shortcut: no line coding or flow control on the wire (USB has its own); the notify endpoint sends nothing.
#include <kernel/kern_priv.h>
#include <drivers/comm.h>
#include "a64_usb_function.h"
#include "a64_usb_queue.h"
#include "a64_device.h"

enum { EPhysOut = 6, EPhysIn = 7, EPhysNotify = 9 };
const TInt KPacket = 512, KInMax = 4096, KUnit = 1;

static const TUint8 KFragment[] = {
	9, 4, 2, 0, 1, 0x02, 0x02, 0x01, 0,						// interface 2: CDC communication, ACM
	5, 0x24, 0x00, 0x10, 0x01,								// header 1.10
	5, 0x24, 0x01, 0x00, 3,									// call management: data on interface 3
	4, 0x24, 0x02, 0x02,									// ACM: line coding + control line state
	5, 0x24, 0x06, 2, 3,									// union: 2 controls 3
	7, 5, 0x84, 0x03, 16, 0, 9,								// EP4 IN interrupt, 16 bytes
	9, 4, 3, 0, 2, 0x0a, 0x00, 0x00, 0,						// interface 3: CDC data
	7, 5, 0x03, 0x02, 0x00, 0x02, 0,						// EP3 OUT bulk 512
	7, 5, 0x83, 0x02, 0x00, 0x02, 0,						// EP3 IN bulk 512
	};
static const SA64UsbEndpoint KEndpoints[] = {
	{ EPhysOut, EA64UsbBulk, KPacket, 0, 0x1000 },
	{ EPhysIn, EA64UsbBulk, KPacket, 0, KInMax },
	{ EPhysNotify, EA64UsbInterrupt, 16, 9, 0 },
	};

// ---- shared state (the queues under Lock; flags written by one side, read by the other)
static TSpinLock Lock(TSpinLock::EOrderGenericIrqHigh1);
static TUsbByteQueue<8, KPacket> Rx, Tx;					// host -> ECOMM, ECOMM -> host
static TInt TxBytes;										// what Tx holds
static volatile TBool Dtr, InBusy, OutArmed;
static TUint8 LineCoding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0, 8 };
static TUint32 RxTotal, TxTotal, Isrs, Opens;

class DPort;
static DPort* volatile Port;								// the open channel, for the ISR
static volatile TUint32 InIsr;								// ISRs using Port: a closing channel waits for 0

// ---- the USB function (the poll's DFC)
static void Poll()
	{
	if (!A64Usb_Configured())
		return;
	if (!OutArmed && A64Usb_CanStart(EPhysOut))
		{
		TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
		const TBool room = !Rx.Full();
		__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
		if (room)											// else the host waits (USB flow control)
			OutArmed = A64Usb_Start(EPhysOut, KPacket) == KErrNone;
		}
	if (!InBusy && TxBytes && A64Usb_CanStart(EPhysIn))
		{
		TUint8* buf = A64Usb_Buf(EPhysIn);
		TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
		TInt want = Min(TxBytes, KInMax - 1), got = 0;
		if (want % KPacket == 0)
			--want;											// end every transfer with a short packet
		while (got < want)
			{
			TInt n;
			const TUint8* p = Tx.Front(n);
			if (!p || n <= 0)
				break;										// never spin here with the lock held
			n = Min(n, want - got);
			memcpy(buf + got, p, n);
			Tx.Consume(n);
			got += n;
			}
		TxBytes -= got;
		__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
		if (got)
			{
			InBusy = A64Usb_Start(EPhysIn, got) == KErrNone;
			TxTotal += got;
			A64Usb_Doorbell();								// room in Tx: ECOMM can give more
			}
		}
	}

static void XferComplete(TInt aPhysEp, TInt aBytes)
	{
	if (aPhysEp == EPhysOut)
		{
		OutArmed = EFalse;
		if (aBytes > 0)
			{
			TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
			Rx.Push(A64Usb_Buf(EPhysOut), aBytes);			// room: checked before arming
			__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
			RxTotal += aBytes;
			}
		}
	else if (aPhysEp == EPhysIn)
		InBusy = EFalse;
	A64Usb_Doorbell();
	}

static void Reset()
	{
	Dtr = InBusy = OutArmed = EFalse;
	TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
	Rx.Clear();
	Tx.Clear();
	TxBytes = 0;
	__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
	A64Usb_Doorbell();										// the carrier is gone
	}

static TInt Ready()
	{
	InBusy = OutArmed = EFalse;								// the poll arms OUT
	return KErrNone;
	}

static TInt ClassSetup(const TUint8* aSetup, TUint8* aBuf)	// CDC ACM, to interface 2
	{
	const TUint8 type = aSetup[0];
	const TUint16 index = aSetup[4] | (aSetup[5] << 8), length = aSetup[6] | (aSetup[7] << 8);
	if (index != 2)
		return KA64UsbStall;
	switch (aSetup[1])
		{
	case 0x20: return type == 0x21 && length == 7 ? KA64UsbDataOut : KA64UsbStall;	// SET_LINE_CODING
	case 0x21:												// GET_LINE_CODING
		if (type != 0xa1 || length != 7)
			return KA64UsbStall;
		memcpy(aBuf, LineCoding, 7);
		return 7;
	case 0x22:												// SET_CONTROL_LINE_STATE: bit 0 = DTR
		if (type != 0x21 || length != 0)
			return KA64UsbStall;
		Dtr = aSetup[2] & 1;
		A64Usb_Doorbell();
		return KA64UsbNoData;
	default: return KA64UsbStall;
		}
	}

static void ClassOutData(const TUint8* aSetup, const TUint8* aData)
	{
	if (aSetup[1] == 0x20)
		memcpy(LineCoding, aData, 7);
	}

static void Dump()
	{
	Kern::Printf("[usb-dbg] dtr %d open %d rx %u tx %u isr %u opens %u in %d out %d queued %d", Dtr, Port != NULL,
		RxTotal, TxTotal, Isrs, Opens, InBusy, OutArmed, TxBytes);
	}

static void Isr();

static const SA64UsbFunction KFunction = {
	NULL, 0, NULL, 0, KFragment, sizeof(KFragment), 2, 2, NULL, 0,
	KEndpoints, sizeof(KEndpoints) / sizeof(KEndpoints[0]),
	ClassSetup, ClassOutData, Ready, Reset, XferComplete, Poll, Dump, Isr,
	};
const SA64UsbFunction* A64UsbDebugFunction() { return &KFunction; }

// ---- the ECOMM PDD (Comm.UsbAcm, unit 1)
_LIT(KPddName, "Comm.UsbAcm");
_LIT(KQueueName, "DebugAcm");

static void GetCaps(TDes8& aDes)
	{
	TCommCaps3 caps;
	caps.FillZ();
	TCommCapsV03& c = caps();
	c.iRate = KCapsBps115200 | KCapsBps9600 | KCapsBps57600 | KCapsBps230400 | KCapsBps460800;
	c.iDataBits = KCapsData8;
	c.iStopBits = KCapsStop1;
	c.iParity = KCapsParityNone;
	c.iHandshake = KCapsObeyDCDSupported | KCapsFailDCDSupported | KCapsFreeRTSSupported | KCapsFreeDTRSupported;
	c.iSignals = KCapsSignalCTSSupported | KCapsSignalDSRSupported | KCapsSignalDCDSupported
		| KCapsSignalRTSSupported | KCapsSignalDTRSupported;
	c.iNotificationCaps = KNotifyDataAvailableSupported | KNotifySignalsChangeSupported;
	aDes.FillZ(aDes.MaxLength());
	aDes.Copy(caps.Left(Min(caps.Length(), aDes.MaxLength())));
	}

class DFactory;
class DPort : public DComm
	{
public:
	DPort(DFactory& aFactory, TDfcQue* aQueue) : iFactory(aFactory), iQueue(aQueue) { iTransmitting = EFalse; }
	~DPort();
	TInt Start();
	void Stop(TStopMode);
	void Break(TBool) {}
	void EnableTransmit();
	TUint Signals() const { return iOutputs | (Dtr ? KSignalCTS | KSignalDSR | KSignalDCD : 0); }
	void SetSignals(TUint aSet, TUint aClear) { iOutputs = ((iOutputs | aSet) & ~aClear) & (KSignalRTS | KSignalDTR); }
	TInt ValidateConfig(const TCommConfigV01& aConfig) const
		{ return aConfig.iDataBits == EData8 && aConfig.iParity == EParityNone ? KErrNone : KErrNotSupported; }
	void Configure(TCommConfigV01&) {}
	void Caps(TDes8& aCaps) const { GetCaps(aCaps); }
	void CheckConfig(TCommConfigV01&) {}
	TInt DisableIrqs() { return NKern::DisableAllInterrupts(); }
	void RestoreIrqs(TInt aLevel) { NKern::RestoreInterrupts(aLevel); }
	TDfcQue* DfcQ(TInt) { return iQueue; }
	void DoIsr();
private:
	DFactory& iFactory;
	TDfcQue* iQueue;
	TUint iOutputs = 0, iLastSignals = 0;
	};

// interrupt context: carrier changes, host bytes into ECOMM, ECOMM's bytes into Tx, and the write's completion
void DPort::DoIsr()
	{
	++Isrs;													// (no Kern::Printf in the ISR)
	const TUint signals = Signals();
	if (signals != iLastSignals)
		{
		iLastSignals = signals;
		StateIsr(signals);
		}
	static TUint words[64];									// static, not on the IRQ stack: the ISR runs on CPU 0
	static TUint8 buf[KPacket];								// only (the SPI's route) and doesn't nest
	for (TInt total = 0; total < 4 * KPacket; )				// RX, bounded per interrupt
		{
		TInt n = 0;
		TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
		TInt len;
		const TUint8* p = Rx.Front(len);
		if (p)
			{
			n = Min(len, 64);
			for (TInt i = 0; i < n; ++i)
				words[i] = p[i];
			Rx.Consume(n);
			}
		__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
		if (!n)
			break;
		ReceiveIsr(words, n, 0);
		total += n;
		if (total >= 4 * KPacket)
			A64Usb_Doorbell();								// more next time
		}
	// TX: pull from ECOMM on every doorbell (no flag carried between runs: EnableTransmit on another CPU may refill
	// ECOMM's buffer at any moment, and a stale "drained" lost that wakeup); never take a byte without room for it
	TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
	const TBool room = !Tx.Full();
	__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
	if (!room)
		return;												// the IN completion rings again
	TBool drained = EFalse;
	TInt n = 0;
	for (; n < KPacket; ++n)
		{
		const TInt c = TransmitIsr();
		if (c < 0)
			{
			drained = ETrue;
			break;
			}
		buf[n] = (TUint8)c;
		}
	irq = __SPIN_LOCK_IRQSAVE(Lock);
	if (n)
		{
		Tx.Push(buf, n);
		TxBytes += n;
		}
	const TBool idle = !TxBytes && !InBusy;
	__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
	if (!drained)
		A64Usb_Doorbell();									// ECOMM has more
	else if (idle)											// all sent: ECOMM completes a write (idempotent: it only
		{													// completes an outstanding one, or asks for more of it)
		iTransmitting = EFalse;
		CheckTxBuffer();
		}
	}

static void Isr()
	{
	__e32_atomic_add_ord32(&InIsr, 1);
	DPort* port = Port;
	if (port)
		port->DoIsr();
	__e32_atomic_add_ord32(&InIsr, TUint32(-1));
	}

void DPort::EnableTransmit()
	{
	iTransmitting = ETrue;									// ECOMM may hold its spinlock here: the ISR does the work
	A64Usb_Doorbell();
	}

class DFactory : public DPhysicalDevice
	{
public:
	DFactory()
		{ iUnitsMask = 1u << KUnit; iVersion = TVersion(KCommsMajorVersionNumber, KCommsMinorVersionNumber, KCommsBuildVersionNumber); }
	~DFactory() { if (iQueue) iQueue->Destroy(); }
	TInt Install()
		{
		TInt r = Kern::DynamicDfcQCreate(iQueue, 27, KQueueName);
		return r == KErrNone ? SetName(&KPddName) : r;
		}
	void GetCaps(TDes8& aDes) const { ::GetCaps(aDes); }
	TInt Validate(TInt aUnit, const TDesC8*, const TVersion& aVersion)
		{ return aUnit == KUnit && Kern::QueryVersionSupported(iVersion, aVersion) ? KErrNone : KErrNotSupported; }
	TInt Create(DBase*& aChannel, TInt aUnit, const TDesC8* aInfo, const TVersion& aVersion)
		{
		aChannel = NULL;
		TInt r = Validate(aUnit, aInfo, aVersion);
		if (r != KErrNone)
			return r;
		NKern::FMWait(&iLock);
		const TBool busy = iOpen;
		iOpen = ETrue;
		NKern::FMSignal(&iLock);
		if (busy)
			return KErrInUse;
		aChannel = new DPort(*this, iQueue);
		if (!aChannel)
			Release();
		return aChannel ? KErrNone : KErrNoMemory;
		}
	void Release() { NKern::FMWait(&iLock); iOpen = EFalse; NKern::FMSignal(&iLock); }
private:
	TDynamicDfcQue* iQueue = NULL;
	NFastMutex iLock;
	TBool iOpen = EFalse;
	};

DPort::~DPort()
	{
	Stop(EStopNormal);
	iFactory.Release();
	}

TInt DPort::Start()
	{
	iLastSignals = Signals();
	iLdd->UpdateSignals(iLastSignals);						// thread context, as ECOMM expects for this one
	++Opens;
	Port = this;
	A64Usb_Doorbell();										// bytes the host sent before the open
	return KErrNone;
	}

void DPort::Stop(TStopMode)
	{
	Port = NULL;
	while (InIsr)											// an ISR on another CPU may still hold this port
		{}
	iTransmitting = EFalse;
	}

DPhysicalDevice* A64CreateDebugSerialPdd() { return new DFactory; }

void A64DebugSerial_Status(TDes8& aOut)					// a64_ecomm.cpp's "drivers" line
	{
	A64Device_Format(aOut, ", USB ACM: dtr %d rx %u tx %u isr %u", Dtr, RxTotal, TxTotal, Isrs);
	}
