// SPDX-License-Identifier: Apache-2.0
// a64_usb_dma.h -- who owns a USB endpoint's TRB and buffer, and how EP0 recovers from a bus reset (the controller core,
// sdm670_assp/qcom_dwc3.cpp; host test tools/test_usb_core.cpp). Only the USB DFC changes this state.
// An endpoint the controller may still write stays owned until its transfer completes or its ENDTRANSFER command
// completes (the command-complete event, not CMDACT clearing; a bus reset proves nothing). A failed start or stop
// quarantines it: only a controller halt plus core reset frees it.
#ifndef A64_USB_DMA_H
#define A64_USB_DMA_H

struct TUsbDmaOwnership
	{
	unsigned iOwned, iPending, iQuarantined;				// bit = physical endpoint
	bool CanStart(unsigned aEp) const { return !((iOwned | iPending | iQuarantined) & (1u << aEp)); }
	void Started(unsigned aEp) { iOwned |= 1u << aEp; }
	void StartFailed(unsigned aEp) { iOwned |= 1u << aEp; iQuarantined |= 1u << aEp; }
	bool Completed(unsigned aEp)							// a transfer completed; false: ignore it (a stop is pending)
		{
		const unsigned bit = 1u << aEp;
		if ((iPending | iQuarantined) & bit) return false;
		iOwned &= ~bit;
		return true;
		}
	void StopRequested(unsigned aEp) { iPending |= 1u << aEp; }
	void StopFailed(unsigned aEp) { iQuarantined |= 1u << aEp; }
	bool StopCompleted(unsigned aEp, bool aOk)				// ENDTRANSFER's command-complete event
		{
		const unsigned bit = 1u << aEp;
		if (!(iPending & bit) || (iQuarantined & bit)) return false;
		if (!aOk) { StopFailed(aEp); return false; }
		iPending &= ~bit;
		iOwned &= ~bit;
		return true;
		}
	void ControllerReset() { iOwned = iPending = iQuarantined = 0; }	// only after a proven halt + core reset
	};

// EP0: EDeferred = a SET_CONFIGURATION waiting for its endpoints' stops before its status stage
enum TEp0State { ESetup, EDataIn, EDataOut, EWaitStatus2, EWaitStatus3In, EWaitStatus3Out, EStatus, EDeferred };

// on a bus reset or disconnect: may the SETUP TRB be re-armed, or is a control transfer's DMA possibly still live?
inline bool UsbEp0Rearmable(TEp0State aState)
	{ return aState == EDeferred || aState == EWaitStatus2 || aState == EWaitStatus3In || aState == EWaitStatus3Out; }
inline bool UsbResetNeedsRestart(TEp0State aState, unsigned aQuarantined)
	{ return aQuarantined != 0 || (aState != ESetup && !UsbEp0Rearmable(aState)); }

// a deferred control request's stops (aMask): done, still waiting, or failed (restart the controller)
enum TUsbStopProgress { EUsbStopReady, EUsbStopWait, EUsbStopRestart };
inline TUsbStopProgress UsbStopProgress(unsigned aPending, unsigned aQuarantined, unsigned aMask)
	{
	if (aQuarantined & aMask) return EUsbStopRestart;
	return aPending & aMask ? EUsbStopWait : EUsbStopReady;
	}
#endif
