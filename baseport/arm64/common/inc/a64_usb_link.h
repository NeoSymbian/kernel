// SPDX-License-Identifier: Apache-2.0
// a64_usb_link.h -- the USB core's link-state tracker (sdm670_assp/qcom_dwc3.cpp; host test
// tools/test_usb_core.cpp). Zero-initialise before use.
#ifndef A64_USB_LINK_H
#define A64_USB_LINK_H

// the HS link state in DSTS (bits 21:18): 3 = suspend, 0 = on. Reports each suspend and the resume that follows it once.
struct TUsbResumeTracker
	{
	enum TAction { ENone, ESuspend, EResume };
	bool iSawSuspend;
	void Reset() { iSawSuspend = false; }
	TAction Observe(unsigned aState)
		{
		if (aState == 3 && !iSawSuspend)
			{
			iSawSuspend = true;
			return ESuspend;
			}
		if (aState == 0 && iSawSuspend)
			{
			iSawSuspend = false;
			return EResume;
			}
		return ENone;										// other states (resume signalling, ...) change nothing
		}
	};
#endif
