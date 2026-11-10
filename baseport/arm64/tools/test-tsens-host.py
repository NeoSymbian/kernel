#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Compile the actual TSENS driver against fake read-only MMIO; no target hardware needed."""
import os
from pathlib import Path
import subprocess
import tempfile

FILES = {
    'include/a64_dbgmon.h': r'''
#pragma once
void A64Mon_Say(const char*,...);
''',
    'include/a64_device.h': r'''
#pragma once
enum TA64DevState { EA64DevAbsent, EA64DevOff, EA64DevStarting, EA64DevOk, EA64DevPartial, EA64DevError };
void A64Device_Format(TDes8&,const char*,...);
#define A64_DEVICE(a,b,c,d)
''',
    'include/board.h': r'''
#pragma once
TLinAddr A64_Io(TUint32);
''',
    'include/kernel/kern_priv.h': r'''
#pragma once
#include <cstdint>
#include <string>
typedef int TInt;
typedef unsigned int TUint32;
typedef uintptr_t TLinAddr;
const int KErrNone=0, KErrNotSupported=-5, KErrArgument=-6, KErrNotReady=-18;
#define XTRAP(error,kind,operation) do { operation; } while(0)
#define XT_DEFAULT 0
struct TDes8 { std::string text; };
''',
    'test.cpp': r'''
#include <kernel/kern_priv.h>
#include <cassert>
#include <cstdio>
#include <cstdarg>
#include <cstring>
static unsigned regs[4][1024];
static int reads;
static bool mapped=true;
TLinAddr A64_Io(TUint32 p) {
 ++reads;
 if(!mapped) return 0;
 const unsigned base[]={0xc263000,0xc265000,0xc222000,0xc223000};
 for(int i=0;i<4;++i) if(p>=base[i] && p<base[i]+4096) return (TLinAddr)&regs[i][(p-base[i])/4];
 return 0;
}
void A64Device_Format(TDes8& d,const char* f,...) { char b[256];va_list a;va_start(a,f);vsnprintf(b,sizeof(b),f,a);va_end(a);d.text+=b; }
void A64Mon_Say(const char*,...) {}
#include "qcom_tsens.cpp"
static void ready() { memset(regs,0,sizeof(regs)); for(int i=2;i<4;++i){regs[i][0]=0x20000000;regs[i][1]=1|(0xffff<<3);} reads=0;mapped=true; }
int main() {
 int t=123;
 ready();assert(A64ThermalRead(-1,0,t)==KErrArgument && t==123 && reads==0);
 assert(A64ThermalRead(0,13,t)==KErrArgument && reads==0);
 assert(A64ThermalRead(1,8,t)==KErrArgument && reads==0);
 assert(A64ThermalRead(2,0,t)==KErrArgument && reads==0);
 mapped=false;assert(A64ThermalRead(0,0,t)==KErrNotSupported && t==123);
 ready();regs[2][0]=0x10000000;assert(A64ThermalRead(0,0,t)==KErrNotSupported);
 ready();regs[2][1]=0;assert(A64ThermalRead(0,0,t)==KErrNotReady);
 ready();regs[2][1]=1;assert(A64ThermalRead(0,0,t)==KErrNotReady);
 ready();assert(A64ThermalRead(0,0,t)==KErrNotReady && reads==5 && t==123);
 unsigned raw[]={0,250,0xfff,0x800,0x7ff};int expected[]={0,25000,-100,-204800,204700};
 for(int i=0;i<5;++i){ready();regs[0][0xa0/4]=(1<<21)|raw[i];auto before=regs[0][0xa0/4];assert(A64ThermalRead(0,0,t)==0 && t==expected[i] && regs[0][0xa0/4]==before);}
 ready();regs[1][(0xa0+7*4)/4]=(1<<21)|350;assert(A64ThermalRead(1,7,t)==0 && t==35000);
 TDes8 d;assert(ThermalStatus(d)==EA64DevPartial && d.text.find("1/21 valid; max 35000")!=std::string::npos);
 ready();TDes8 off;assert(ThermalStatus(off)==EA64DevOff);
 mapped=false;TDes8 error;assert(ThermalStatus(error)==EA64DevError);
 puts("PASS actual-source TSENS bounds, enable/version, 3-attempt limit, signed conversion, status and no-write checks");
}
''',
}


def main():
    source = Path(__file__).resolve().parents[1] / "sdm670_assp"
    with tempfile.TemporaryDirectory(prefix="neo-tsens-") as directory:
        root = Path(directory)
        for name, content in FILES.items():
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        binary = root / "test"
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra",
                        "-I", str(root / "include"), "-I", str(source),
                        str(root / "test.cpp"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
