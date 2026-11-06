// SPDX-License-Identifier: Apache-2.0
// a64_dbgmon.h -- the kernel debug monitor and log ring (a64_dbgmon.cpp), used by its transports.
#ifndef A64_DBGMON_H
#define A64_DBGMON_H

typedef void (*TMonWrite)(const TUint8* aData, TInt aLen);	// a transport's blocking writer

void A64Log_Putc(char aChar);								// every kernel debug character (DebugOutputChar)
TInt A64Log_Read(TUint32& aPos, TUint8* aBuf, TInt aMax);	// from a reader's cursor (start at 0: the whole ring)
void A64Mon_Line(const TDesC8& aLine, TMonWrite aWrite);	// run one command line; thread context
void A64DumpThreads();
#endif
