// SPDX-License-Identifier: Apache-2.0
// test_usb_core.cpp -- host test of the USB core's endpoint ownership and EP0 recovery (common/inc/a64_usb_dma.h):
//   c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined test_usb_core.cpp -o test_usb_core && ./test_usb_core
#include <cassert>
#include <cstdio>
#include "../common/inc/a64_usb_dma.h"

int main()
	{
	TUsbDmaOwnership dma = {};
	assert(dma.CanStart(2));
	dma.Started(2);
	assert(!dma.CanStart(2));
	dma.StopRequested(2);
	assert(!dma.Completed(2));								// a data completion doesn't prove the stop completed
	assert(!dma.CanStart(2));
	assert(dma.StopCompleted(2, true));
	assert(dma.CanStart(2));
	assert(!dma.StopCompleted(2, true));					// a duplicate stop completion
	dma.Started(3);
	dma.StopRequested(3);
	assert(!dma.StopCompleted(3, false));
	assert(!dma.StopCompleted(3, true));					// a late success can't free quarantined storage
	assert(!dma.Completed(3) && !dma.CanStart(3));
	dma.StartFailed(5);
	assert(!dma.CanStart(5) && !dma.Completed(5));
	assert(dma.CanStart(2));								// other endpoints stay usable
	assert(UsbResetNeedsRestart(ESetup, dma.iQuarantined));
	dma.ControllerReset();
	assert(dma.CanStart(3) && dma.CanStart(5));
	dma.Started(3);
	assert(dma.Completed(3) && dma.CanStart(3));

	assert(!UsbEp0Rearmable(ESetup) && !UsbResetNeedsRestart(ESetup, 0));
	const TEp0State idle[] = { EDeferred, EWaitStatus2, EWaitStatus3In, EWaitStatus3Out };
	for (unsigned i = 0; i < sizeof(idle) / sizeof(idle[0]); ++i)
		{
		assert(UsbEp0Rearmable(idle[i]));
		assert(!UsbResetNeedsRestart(idle[i], 0));
		assert(UsbResetNeedsRestart(idle[i], 1u << 3));
		}
	const TEp0State active[] = { EDataIn, EDataOut, EStatus };
	for (unsigned i = 0; i < sizeof(active) / sizeof(active[0]); ++i)
		assert(!UsbEp0Rearmable(active[i]) && UsbResetNeedsRestart(active[i], 0));

	// a stop that never completes (a later SETUP can block it): its timeout quarantines it, which asks for a restart
	TUsbDmaOwnership blocked = {};
	const unsigned bulk = 1u << 3;
	blocked.Started(3);
	blocked.StopRequested(3);
	assert(UsbStopProgress(blocked.iPending, blocked.iQuarantined, bulk) == EUsbStopWait);
	assert(!blocked.Completed(3) && !blocked.CanStart(3));
	blocked.StopFailed(3);
	assert(UsbStopProgress(blocked.iPending, blocked.iQuarantined, bulk) == EUsbStopRestart);
	assert(!blocked.StopCompleted(3, true) && !blocked.CanStart(3));
	assert(UsbStopProgress(blocked.iPending, blocked.iQuarantined, 1u << 2) == EUsbStopReady);
	blocked.ControllerReset();
	assert(UsbStopProgress(blocked.iPending, blocked.iQuarantined, bulk) == EUsbStopReady && blocked.CanStart(3));
	puts("test_usb_core: ok");
	return 0;
	}
