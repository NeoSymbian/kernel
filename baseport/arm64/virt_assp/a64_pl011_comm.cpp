// SPDX-License-Identifier: Apache-2.0
// a64_pl011_comm.cpp -- the serial PDD (Comm.PL011, unit 1) for QEMU virt's second PL011 (VIRT_UART1: 0x09040000, SPI 8
// = INTID 40; qemu hw/char/pl011.c, the 24 MHz virt uartclk) under Symbian's ECOMM LDD. UART0 stays the console.
// Interrupt-driven, as a UART PDD must be: ECOMM's ReceiveIsr/TransmitIsr/CheckTxBuffer complete requests inline unless
// they run in interrupt context (d_comm.cpp RxComplete), which from a thread self-deadlocks on ECOMM's spinlock.
// shortcut: 115200 8N1 only, no modem lines (QEMU's chardev has none: CTS/DSR/DCD always on).
#include <kernel/kern_priv.h>
#include <drivers/comm.h>
#include <nkernsmp/arm64/a64_platform.h>
#include "board.h"
#include "a64_pl011_io.h"
#include "a64_device.h"

namespace
	{
	const TUint32 KUart = 0x09040000, KIntId = 32 + 8;
	const TInt KUnit = 1, KIsrBudget = 64, KIsrRxMax = 256;
	enum { EIntRx = 1u << 4, EIntTx = 1u << 5, EIntRxTimeout = 1u << 6 };
	_LIT(KPddName, "Comm.PL011");
	_LIT(KQueueName, "DebugSerial");
	class TIo
		{
	public:
		unsigned Read(unsigned aOffset) const
			{ return *reinterpret_cast<volatile TUint32*>(A64_Io(KUart) + aOffset); }
		void Write(unsigned aOffset, unsigned aValue)
			{ *reinterpret_cast<volatile TUint32*>(A64_Io(KUart) + aOffset) = aValue; __e32_memory_barrier(); }
		};
	TIo Io;
	TSpinLock Lock(TSpinLock::EOrderGenericIrqHigh0);	// IMSC and Port; taken inside ECOMM's (IrqLow3), never around it
	class DPort;
	DPort* Port;											// the open channel, for the ISR
	volatile TUint32 InIsr;									// ISRs using Port: a closing channel waits for 0
	TUint32 Isrs, RxBytes, TxBytes, TxDone;					// for the monitor's "drivers"

	void Pend()												// run the ISR again (GICD_ISPENDR)
		{ *(volatile TUint32*)(A64_Io(A64_GicdBase) + 0x200 + 4 * (KIntId / 32)) = 1u << (KIntId % 32); }

	void SetMask(TUint32 aSet, TUint32 aClear)
		{
		TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
		Io.Write(A64Pl011::EImsc, (Io.Read(A64Pl011::EImsc) | aSet) & ~aClear);
		__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
		}

	void GetCaps(TDes8& aDes)
		{
		TCommCaps3 caps;
		caps.FillZ();
		TCommCapsV03& c = caps();
		c.iRate = KCapsBps115200;
		c.iDataBits = KCapsData8; c.iStopBits = KCapsStop1; c.iParity = KCapsParityNone;
		c.iHandshake = KCapsObeyCTSSupported | KCapsFailCTSSupported | KCapsObeyDSRSupported
			| KCapsFailDSRSupported | KCapsObeyDCDSupported | KCapsFailDCDSupported
			| KCapsFreeRTSSupported | KCapsFreeDTRSupported;
		c.iSignals = KCapsSignalCTSSupported | KCapsSignalDSRSupported | KCapsSignalDCDSupported
			| KCapsSignalRTSSupported | KCapsSignalDTRSupported;
		c.iNotificationCaps = KNotifyDataAvailableSupported | KNotifySignalsChangeSupported;
		c.iFifo = KCapsHasFifo;
		aDes.FillZ(aDes.MaxLength());
		aDes.Copy(caps.Left(Min(caps.Length(), aDes.MaxLength())));
		}
	class DFactory;
	class DPort : public DComm
		{
	public:
		DPort(DFactory& aFactory, TDfcQue* aQueue);
		~DPort();
		TInt Start();
		void Stop(TStopMode);
		void Break(TBool);
		void EnableTransmit();
		TUint Signals() const;
		void SetSignals(TUint aSet, TUint aClear);
		TInt ValidateConfig(const TCommConfigV01& aConfig) const;
		void Configure(TCommConfigV01& aConfig);
		void Caps(TDes8& aCaps) const { GetCaps(aCaps); }
		void CheckConfig(TCommConfigV01& aConfig);
		TInt DisableIrqs() { return NKern::DisableAllInterrupts(); }
		void RestoreIrqs(TInt aLevel) { NKern::RestoreInterrupts(aLevel); }
		TDfcQue* DfcQ(TInt) { return iQueue; }
		TInt Next() { TInt c = TransmitIsr(); iTxDrained = c < 0; return c; }	// A64Pl011::Fill's source
		void Isr();
	private:
		DFactory& iFactory;
		TDfcQue* iQueue;
		TUint iOutputs;
		TBool iTxDrained;
		};
	struct TRxSink
		{
		TUint iWords[KIsrBudget];
		TInt iCount;
		void Character(unsigned aDr)
			{
			TUint c = aDr & 0xff;
			if (aDr & (1u << 8)) c |= KReceiveIsrFrameError;
			if (aDr & (1u << 9)) c |= KReceiveIsrParityError;
			if (aDr & (1u << 10)) c |= KReceiveIsrBreakError;
			if (aDr & (1u << 11)) c |= KReceiveIsrOverrunError;
			iWords[iCount++] = c;
			}
		};

	// interrupt context: RX into ECOMM; while it has bytes, TX from it; once it has none and the line is idle, its
	// TX buffer check (completes the write)
	void DPort::Isr()
		{
		Io.Write(A64Pl011::EIcr, EIntRx | EIntRxTimeout | EIntTx);	// clear first: a later byte interrupts again
		++Isrs;
		// QEMU raises RX only as the FIFO fills past its trigger (pl011.c), so bytes left behind would never interrupt:
		// drain to empty, or come back (bounded per interrupt)
		for (TInt got = 0; got < KIsrRxMax; )
			{
			TRxSink rx = {};
			A64Pl011::Drain(Io, rx, KIsrBudget);
			if (!rx.iCount)
				break;
			RxBytes += rx.iCount;
			got += rx.iCount;
			ReceiveIsr(rx.iWords, rx.iCount, 0);
			}
		if (!(Io.Read(A64Pl011::EFr) & A64Pl011::ERxEmpty))
			Pend();
		if (!iTransmitting)
			return;
		iTxDrained = EFalse;
		TxBytes += A64Pl011::Fill(Io, *this, KIsrBudget);
		if (iTxDrained && A64Pl011::TransmitterIdle(Io.Read(A64Pl011::EFr)))
			{
			iTransmitting = EFalse;
			SetMask(0, EIntTx);								// TX is level-triggered on an empty FIFO
			++TxDone;
			CheckTxBuffer();								// may ask for more: EnableTransmit
			}
		}

	void UartIsr(TAny*)
		{
		__e32_atomic_add_ord32(&InIsr, 1);
		DPort* port = Port;
		if (port)
			port->Isr();
		else
			Io.Write(A64Pl011::EImsc, 0);
		__e32_atomic_add_ord32(&InIsr, TUint32(-1));
		}

	class DFactory : public DPhysicalDevice
		{
	public:
		DFactory() : iQueue(NULL), iOpen(EFalse)
			{ iUnitsMask = 1u << KUnit; iVersion = TVersion(KCommsMajorVersionNumber, KCommsMinorVersionNumber, KCommsBuildVersionNumber); }
		~DFactory() { if (iQueue) iQueue->Destroy(); }
		TInt Install()
			{
			Io.Write(A64Pl011::EImsc, 0);
			TInt r = Kern::DynamicDfcQCreate(iQueue, 27, KQueueName);
			if (r == KErrNone)
				r = A64_BindSpi(KIntId, UartIsr, NULL);
			if (r == KErrNone)
				r = SetName(&KPddName);
			return r;
			}
		void GetCaps(TDes8& aDes) const { ::GetCaps(aDes); }
		TInt Validate(TInt aUnit, const TDesC8*, const TVersion& aVersion)
			{ return aUnit == KUnit && Kern::QueryVersionSupported(iVersion, aVersion) ? KErrNone : KErrNotSupported; }
		TInt Create(DBase*& aChannel, TInt aUnit, const TDesC8* aInfo, const TVersion& aVersion)
			{
			aChannel = NULL;
			TInt r = Validate(aUnit, aInfo, aVersion);
			if (r != KErrNone) return r;
			NKern::FMWait(&iLock);
			if (iOpen) { NKern::FMSignal(&iLock); return KErrInUse; }
			iOpen = ETrue;
			NKern::FMSignal(&iLock);
			aChannel = new DPort(*this, iQueue);
			if (!aChannel) { Release(); return KErrNoMemory; }
			return KErrNone;
			}
		void Release()
			{ NKern::FMWait(&iLock); iOpen = EFalse; NKern::FMSignal(&iLock); }
	private:
		TDynamicDfcQue* iQueue;
		NFastMutex iLock;
		TBool iOpen;
		};
	DPort::DPort(DFactory& aFactory, TDfcQue* aQueue)
		: iFactory(aFactory), iQueue(aQueue), iOutputs(0), iTxDrained(EFalse)
		{ iTransmitting = EFalse; }
	DPort::~DPort()
		{
		Stop(EStopNormal);
		iFactory.Release();
		}
	TInt DPort::Start()
		{
		iLdd->UpdateSignals(Signals());						// thread context, as ECOMM expects for this one
		TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
		Port = this;
		Io.Write(A64Pl011::EIcr, 0x7ff);
		Io.Write(A64Pl011::EImsc, EIntRx | EIntRxTimeout);
		__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
		return KErrNone;
		}
	void DPort::Stop(TStopMode)
		{
		TInt irq = __SPIN_LOCK_IRQSAVE(Lock);
		Io.Write(A64Pl011::EImsc, 0);
		if (Port == this)
			Port = NULL;
		__SPIN_UNLOCK_IRQRESTORE(Lock, irq);
		while (InIsr)										// an ISR on another CPU may still hold this port
			{}
		iTransmitting = EFalse;
		}
	void DPort::Break(TBool aState)
		{
		const TUint32 lcr = Io.Read(A64Pl011::ELcr);
		Io.Write(A64Pl011::ELcr, aState ? lcr | 1u : lcr & ~1u);
		}
	void DPort::EnableTransmit()
		{
		// ECOMM may hold its spinlock here, so the ISR sends: unmask TX and pend the SPI in the GIC (GICD_ISPENDR), as
		// QEMU raises TX only after a write (pl011.c), and the first byte has none before it
		iTransmitting = ETrue;
		SetMask(EIntTx, 0);
		Pend();
		}
	TUint DPort::Signals() const
		{
		return iOutputs | KSignalCTS | KSignalDSR | KSignalDCD;	// a permanent virtual cable
		}
	void DPort::SetSignals(TUint aSet, TUint aClear)
		{ iOutputs = ((iOutputs | aSet) & ~aClear) & (KSignalRTS | KSignalDTR); }
	TInt DPort::ValidateConfig(const TCommConfigV01& aConfig) const
		{
		const TUint allowed = KConfigObeyCTS | KConfigFailCTS | KConfigObeyDSR | KConfigFailDSR
			| KConfigObeyDCD | KConfigFailDCD | KConfigFreeRTS | KConfigFreeDTR | KConfigWriteBufferedComplete;
		return aConfig.iRate == EBps115200 && aConfig.iDataBits == EData8 && aConfig.iStopBits == EStop1
			&& aConfig.iParity == EParityNone && aConfig.iSIREnable == ESIRDisable
			&& !(aConfig.iHandshake & ~allowed) ? KErrNone : KErrNotSupported;
		}
	void DPort::CheckConfig(TCommConfigV01& aConfig)
		{ aConfig.iRate = EBps115200; }
	void DPort::Configure(TCommConfigV01&)				// the interrupt mask stays as Start/EnableTransmit set it
		{
		Io.Write(A64Pl011::ECr, 0);
		Io.Write(A64Pl011::EDmacr, 0);
		Io.Write(A64Pl011::ERsr, 0xff);
		Io.Write(A64Pl011::EIbrd, 13);						// 24 MHz / (16 * 115200) = 13.02
		Io.Write(A64Pl011::EFbrd, 1);
		Io.Write(A64Pl011::ELcr, (3u << 5) | (1u << 4));	// 8N1, FIFOs on
		Io.Write(A64Pl011::ECr, 1u | (1u << 8) | (1u << 9));	// UART, TX, RX on
		}
	}

DPhysicalDevice* A64CreateDebugSerialPdd() { return new DFactory; }

void A64DebugSerial_Status(TDes8& aOut)					// a64_ecomm.cpp's "drivers" line
	{
	A64Device_Format(aOut, ", isr %u rx %u tx %u txdone %u open %d imsc %x ris %x fr %x", Isrs, RxBytes,
		TxBytes, TxDone, Port != NULL, Io.Read(A64Pl011::EImsc), Io.Read(0x3c), Io.Read(A64Pl011::EFr));
	}
