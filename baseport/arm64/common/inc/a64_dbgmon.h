// SPDX-License-Identifier: Apache-2.0
// a64_dbgmon.h -- the kernel debug monitor and log ring (a64_dbgmon.cpp), used by its transports.
#ifndef A64_DBGMON_H
#define A64_DBGMON_H

typedef void (*TMonWrite)(const TUint8* aData, TInt aLen);	// a transport's blocking writer

void A64Log_Putc(char aChar);								// every kernel debug character (DebugOutputChar)
void A64Log_Report();										// at boot: a previous boot's log kept (lastlog)
TInt A64Log_Read(TUint32& aPos, TUint8* aBuf, TInt aMax);
TInt A64Log_LastRead(TUint32& aPos, TUint8* aBuf, TInt aMax);	// the previous boot's (lastlog)	// from a reader's cursor (start at 0: the whole ring)
void A64Mon_Line(const TDesC8& aLine, TMonWrite aWrite);	// run one command line; thread context
// "blk put" (telephony C5b): the transport switches to raw input for aBytes (KErrNotSupported where it can't), and
// hands each stage of it back, in order, to A64Mon_PutChunk (aLast on the final one)
TInt A64Usb_RawIn(TInt aBytes);
void A64Mon_PutChunk(const TUint8* aData, TInt aLen, TBool aLast, TMonWrite aWrite);
void A64DumpThreads();
void A64Key_Type(TUint8 aChar);
void A64Mon_Say(const char* aFmt, ...);					// one answer line (< 256 bytes), for board files' commands
void A64Tel_Monitor(TInt aArgc, const TPtrC8* aArg, const TUint64* aNum);	// qcom_telmon.cpp								// one ASCII character as a paced key press (thread)
#endif
