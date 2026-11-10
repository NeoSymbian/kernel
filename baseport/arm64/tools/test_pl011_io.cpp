// SPDX-License-Identifier: Apache-2.0
// test_pl011_io.cpp -- host test of the PL011 FIFO drain/fill (common/inc/a64_pl011_io.h) against a model FIFO:
//   c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined test_pl011_io.cpp -o test_pl011_io && ./test_pl011_io
#include <cassert>
#include <deque>
#include <vector>
#include "../common/inc/a64_pl011_io.h"

struct TMockIo
	{
	std::deque<unsigned> iRx;
	std::vector<unsigned> iTx;
	unsigned iTxCapacity, iBusy;
	unsigned Read(unsigned aRegister)
		{
		if (aRegister == A64Pl011::EFr)
			return (iRx.empty() ? A64Pl011::ERxEmpty : 0) | (iTx.size() == iTxCapacity ? A64Pl011::ETxFull : 0)
				| (iTx.empty() ? A64Pl011::ETxEmpty : 0) | iBusy;
		assert(aRegister == A64Pl011::EDr && !iRx.empty());
		const unsigned ch = iRx.front(); iRx.pop_front(); return ch;
		}
	void Write(unsigned aRegister, unsigned aValue)
		{
		assert(aRegister == A64Pl011::EDr && iTx.size() < iTxCapacity && aValue <= 255);
		iTx.push_back(aValue);
		}
	};
struct TSink
	{
	std::vector<unsigned> iWords;
	void Character(unsigned aWord) { iWords.push_back(aWord); }
	};
struct TSource
	{
	std::deque<unsigned> iBytes;
	int Next()
		{
		if (iBytes.empty()) return -1;
		const unsigned ch = iBytes.front(); iBytes.pop_front(); return ch;
		}
	};
int main()
	{
	TMockIo io = {}; io.iTxCapacity = 16;
	TSink sink;
	assert(A64Pl011::Drain(io, sink, 64) == 0 && sink.iWords.empty());
	for (unsigned i = 0; i < 256; ++i) io.iRx.push_back(i | ((i & 15) << 8));
	assert(A64Pl011::Drain(io, sink, 0) == 0 && sink.iWords.empty());
	assert(A64Pl011::Drain(io, sink, -1) == 0 && sink.iWords.empty());
	assert(A64Pl011::Drain(io, sink, 64) == 64 && io.iRx.size() == 192);
	while (A64Pl011::Drain(io, sink, 64)) {}
	assert(sink.iWords.size() == 256);
	for (unsigned i = 0; i < 256; ++i) assert(sink.iWords[i] == (i | ((i & 15) << 8)));

	TSource source;
	for (unsigned i = 0; i < 256; ++i) source.iBytes.push_back(i);
	assert(A64Pl011::Fill(io, source, 0) == 0 && source.iBytes.size() == 256);
	assert(A64Pl011::Fill(io, source, 64) == 16 && source.iBytes.size() == 240);
	assert(A64Pl011::Fill(io, source, 64) == 0 && source.iBytes.size() == 240); // full FIFO must retain LDD byte
	for (unsigned i = 0; i < 16; ++i) assert(io.iTx[i] == i);
	io.iTx.clear();
	assert(A64Pl011::Fill(io, source, 1) == 1 && io.iTx[0] == 16);
	io.iTx.clear(); io.iTxCapacity = 512;
	assert(A64Pl011::Fill(io, source, 512) == 239 && source.iBytes.empty());
	for (unsigned i = 0; i < 239; ++i) assert(io.iTx[i] == i + 17);
	assert(A64Pl011::Fill(io, source, 64) == 0); // no invented/padded bytes
	assert(!A64Pl011::TransmitterIdle(A64Pl011::EBusy | A64Pl011::ETxEmpty));
	assert(!A64Pl011::TransmitterIdle(0));
	assert(A64Pl011::TransmitterIdle(A64Pl011::ETxEmpty));
	return 0;
	}
