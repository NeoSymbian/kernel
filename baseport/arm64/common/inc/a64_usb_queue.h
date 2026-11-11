// SPDX-License-Identifier: Apache-2.0
// Bounded byte-stream chunks. The owner supplies synchronization; storage is never a DMA target.
// Zero-initialize before first use. Partial consumption preserves the remaining bytes without copying.
#ifndef A64_USB_QUEUE_H
#define A64_USB_QUEUE_H

template<int Slots, int ChunkMax> struct TUsbByteQueue
	{
	typedef char TValidSizes[(Slots > 0 && ChunkMax > 0) ? 1 : -1];
	unsigned char iData[Slots][ChunkMax];
	int iLength[Slots];
	int iHead, iCount, iOffset;
	void Clear() { iHead = iCount = iOffset = 0; }
	bool Full() const { return iCount == Slots; }
	bool Empty() const { return iCount == 0; }
	bool Push(const unsigned char* aData, int aLength)
		{
		if (!aData || Full() || aLength <= 0 || aLength > ChunkMax)
			return false;
		const int slot = (iHead + iCount) % Slots;
		for (int i = 0; i < aLength; ++i)
			iData[slot][i] = aData[i];
		iLength[slot] = aLength;
		++iCount;
		return true;
		}
	const unsigned char* Front(int& aLength) const
		{
		aLength = Empty() ? 0 : iLength[iHead] - iOffset;
		return Empty() ? 0 : iData[iHead] + iOffset;
		}
	bool Consume(int aLength)
		{
		if (Empty() || aLength < 0 || aLength > iLength[iHead] - iOffset)
			return false;
		iOffset += aLength;
		if (iOffset == iLength[iHead])
			{
			iHead = (iHead + 1) % Slots;
			--iCount;
			iOffset = 0;
			}
		return true;
		}
	};

#endif
