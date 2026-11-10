// SPDX-License-Identifier: Apache-2.0
// a64_usb_function.h -- a USB device function (its descriptors, class requests and endpoints) and the device
// controller core that runs it. The core (sdm670_assp/qcom_dwc3.cpp) owns power and bring-up, the event ring, EP0 and
// the standard (Chapter 9) requests, and the endpoints' transfers and DMA buffers; the function
// (common/specific/a64_usb_acm_monitor.cpp: the debug monitor's ACM serial port) supplies the rest.
// shortcut: one function, chosen at link time (KA64UsbFunction); a list when a second function comes (roadmap T8).
#ifndef A64_USB_FUNCTION_H
#define A64_USB_FUNCTION_H

enum TA64UsbEpType { EA64UsbControl = 0, EA64UsbBulk = 2, EA64UsbInterrupt = 3 };
const TInt KA64UsbStall = -1, KA64UsbNoData = -2, KA64UsbDataOut = -3;	// ClassSetup's answers besides a length

struct SA64UsbEndpoint
	{
	TInt iPhysEp;											// the controller's numbering: 2 * number + (IN ? 1 : 0)
	TA64UsbEpType iType;
	TInt iMaxPacket;
	TInt iInterval;											// interrupt endpoints, as bInterval; 0 = none
	TInt iBufSize;											// its DMA buffer in the core's area; 0 = none
	};

struct SA64UsbFunction
	{
	const TUint8* iDevice; TInt iDeviceLen;
	const TUint8* iQualifier; TInt iQualifierLen;
	const TUint8* iConfig; TInt iConfigLen;					// the whole configuration (interfaces, endpoints)
	const char* const* iStrings; TInt iStringCount;			// index 0 unused (the language list is the core's)
	const SA64UsbEndpoint* iEndpoints; TInt iEndpointCount;	// configured in this order
	// a class request: the IN data's length (in aBuf), KA64UsbNoData, KA64UsbDataOut (data follows) or KA64UsbStall
	TInt (*iClassSetup)(const TUint8* aSetup, TUint8* aBuf);
	void (*iClassOutData)(const TUint8* aSetup, const TUint8* aData);	// the data stage of a KA64UsbDataOut request
	TInt (*iConfigured)();									// SET_CONFIGURATION: the endpoints are ready; an error stalls
	void (*iReset)();										// the host's session ended: bus reset, disconnect, a new
															// SET_CONFIGURATION, a controller restart; no transfer is armed
	void (*iXferComplete)(TInt aPhysEp, TInt aBytes);		// aBytes: what an OUT transfer received
	void (*iPoll)();										// every poll (1 ms), after the events
	void (*iDump)();										// status lines in the log
	};
extern const SA64UsbFunction KA64UsbFunction;

// the core, for the function (thread context, the poll's DFC queue)
TUint8* A64Usb_Buf(TInt aPhysEp);							// the endpoint's DMA buffer
TBool A64Usb_CanStart(TInt aPhysEp);						// the controller no longer owns its TRB and buffer
TInt A64Usb_Start(TInt aPhysEp, TInt aLen);					// one transfer from/into its buffer (KErrNotReady: owned)
void A64Usb_EndXfer(TInt aPhysEp);							// asks for a stop; CanStart turns true once it completed
void A64Usb_ProcessEvents();								// for a writer that waits on its IN transfer
TBool A64Usb_Ep0Idle();										// no control transfer in progress
TBool A64Usb_Configured();
struct SA64UsbStats
	{
	TInt iEvents, iResets, iSetups, iEp0, iRestarts;
	TBool iConfigured;
	TUint32 iDsts, iSetupLog[6], iDmaOwned, iDmaPending, iDmaQuarantined;
	};
void A64Usb_Stats(SA64UsbStats& aStats);
void A64Usb_Trb(TInt aPhysEp, TUint32& aSize, TUint32& aCtrl, TUint32& aRscIdx);
#endif
