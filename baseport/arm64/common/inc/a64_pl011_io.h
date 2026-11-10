// SPDX-License-Identifier: Apache-2.0
// PL011 FIFO access shared by the QEMU PDD and its host model. No memory allocation or synchronization.
// Register meanings: ARM PL011 TRM DDI0183; the actual target is QEMU hw/char/pl011.c (ARM PL011 model).
#ifndef A64_PL011_IO_H
#define A64_PL011_IO_H

namespace A64Pl011
	{
	enum TRegister
		{
		EDr = 0x00, ERsr = 0x04, EFr = 0x18, EIbrd = 0x24, EFbrd = 0x28, ELcr = 0x2c,
		ECr = 0x30, EImsc = 0x38, EIcr = 0x44, EDmacr = 0x48
		};
	enum TFlag { EBusy = 1u << 3, ERxEmpty = 1u << 4, ETxFull = 1u << 5, ETxEmpty = 1u << 7 };
	inline bool TransmitterIdle(unsigned aFlags)
		{ return (aFlags & (EBusy | ETxEmpty)) == ETxEmpty; }
	// Sink receives the entire DR word, retaining parity/frame/break/overrun status for the native LDD.
	template<class Io, class Sink> int Drain(Io& aIo, Sink& aSink, int aBudget)
		{
		int done = 0;
		while (done < aBudget && !(aIo.Read(EFr) & ERxEmpty))
			{ aSink.Character(aIo.Read(EDr)); ++done; }
		return done;
		}
	// Never consume a native LDD byte until the FIFO has space. A negative source value means no byte available.
	template<class Io, class Source> int Fill(Io& aIo, Source& aSource, int aBudget)
		{
		int done = 0;
		while (done < aBudget && !(aIo.Read(EFr) & ETxFull))
			{
			const int ch = aSource.Next();
			if (ch < 0) break;
			aIo.Write(EDr, static_cast<unsigned>(ch) & 0xff);
			++done;
			}
		return done;
		}
	}

#endif
