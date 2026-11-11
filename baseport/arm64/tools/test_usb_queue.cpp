// SPDX-License-Identifier: Apache-2.0
// test_usb_queue.cpp -- host test of the USB byte queue (common/inc/a64_usb_queue.h; a64_usb_acm_debug.cpp's RX/TX):
//   c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined test_usb_queue.cpp -o test_usb_queue && ./test_usb_queue
#include <cassert>
#include <cstring>
#include <deque>
#include "../common/inc/a64_usb_queue.h"

int main()
	{
	TUsbByteQueue<3, 1024> q = {};
	unsigned char bytes[1025];
	for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = static_cast<unsigned char>(i);
	int n = -1;
	assert(q.Empty() && !q.Front(n) && n == 0);
	assert(!q.Consume(0) && !q.Consume(1));
	assert(!q.Push(bytes, -1) && !q.Push(bytes, 0) && !q.Push(bytes, 1025));
	assert(!q.Push(0, 1));
	assert(q.Push(bytes, 1)); // single-byte keystrokes and transfer acknowledgements must not be dropped
	assert(q.Push(bytes, 1024) && q.Push(bytes + 3, 13));
	assert(q.Full() && !q.Push(bytes, 1));
	assert(q.Front(n) && n == 1 && *q.Front(n) == 0);
	assert(!q.Consume(-1) && !q.Consume(2));
	assert(q.Consume(1));
	assert(q.Push(bytes + 7, 31)); // wrap the chunk slots
	assert(q.Front(n) && n == 1024);
	assert(q.Consume(511));
	assert(q.Front(n) && n == 513 && !std::memcmp(q.Front(n), bytes + 511, n));
	assert(q.Full() && !q.Push(bytes, 1)); // partial consumption never overwrites a remaining chunk
	assert(q.Consume(0) && q.Consume(513));
	assert(q.Front(n) && n == 13 && !std::memcmp(q.Front(n), bytes + 3, n));
	assert(q.Consume(n));
	assert(q.Front(n) && n == 31 && !std::memcmp(q.Front(n), bytes + 7, n));
	q.Clear(); // a link generation change discards all unconsumed bytes, including a partial chunk
	assert(q.Empty() && !q.Front(n) && n == 0);

	// Independent byte-stream oracle across mixed chunk sizes, partial drains, full queues and wraparound.
	std::deque<unsigned char> expected;
	unsigned random = 0x5a19u;
	for (unsigned step = 0; step < 20000; ++step)
		{
		random = random * 1664525u + 1013904223u;
		if (random & 1)
			{
			const int length = 1 + static_cast<int>((random >> 8) % 1024);
			if (q.Push(bytes, length))
				for (int i = 0; i < length; ++i) expected.push_back(bytes[i]);
			}
		else if (!expected.empty())
			{
			const unsigned char* front = q.Front(n);
			assert(front && n > 0 && static_cast<unsigned>(n) <= expected.size());
			const int take = 1 + static_cast<int>((random >> 8) % static_cast<unsigned>(n));
			for (int i = 0; i < take; ++i)
				{ assert(front[i] == expected.front()); expected.pop_front(); }
			assert(q.Consume(take));
			}
		assert(q.Empty() == expected.empty());
		}
	while (!expected.empty())
		{
		const unsigned char* front = q.Front(n);
		assert(front && n > 0);
		for (int i = 0; i < n; ++i)
			{ assert(front[i] == expected.front()); expected.pop_front(); }
		assert(q.Consume(n));
		}
	assert(q.Empty());
	return 0;
	}
