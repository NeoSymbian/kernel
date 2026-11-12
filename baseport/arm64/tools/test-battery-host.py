#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Actual-source read-only PM660 battery regression harness (no target hardware)."""
import os
from pathlib import Path
import subprocess
import tempfile
FILES = {
    'include/kernel/kern_priv.h': r'''

#pragma once
#include <cstdint>
#include <vector>
#include <string>
typedef int TInt;typedef unsigned TUint32;typedef unsigned long long TUint64;typedef unsigned char TUint8;typedef bool TBool;typedef void TAny;
const int KErrNone=0,KErrNotSupported=-5,KErrNotReady=-18,KChunkShiftBy0=0;
const bool ETrue=true,EFalse=false;
struct TDes8 { std::string text; std::vector<unsigned char> data; };
template<int N> struct TBuf8:TDes8 { void Append(TUint8 x){data.push_back(x);} };
namespace Kern { inline int& CurrentThread(){static int x;return x;} inline int ThreadDesWrite(int*,void* target,const TDes8& v,int,int,void*){if(!target)return -38;*(TDes8*)target=v;return 0;} }
''',
    'include/a64_device.h': r'''

#pragma once
enum TA64DevState { EA64DevAbsent,EA64DevOff,EA64DevStarting,EA64DevOk,EA64DevPartial,EA64DevError };
void A64Device_Format(TDes8&,const char*,...);
#define A64_DEVICE(a,b,c,d)
''',
    'include/a64_dbgmon.h': r'''
void A64Mon_Say(const char*,...);
''',
    'test.cpp': r'''

#include <kernel/kern_priv.h>
#include <map>
#include <cassert>
#include <cstdarg>
#include <cstdio>
static std::map<unsigned,int> bytes;
static int reads, bus=0x50000000;
static std::map<unsigned,std::vector<int> > scripted;
static std::map<unsigned,unsigned> calls;
int Spmi_Read(unsigned sid,unsigned addr){assert(sid==0);++reads;unsigned index=calls[addr]++;if(scripted.count(addr) && index<scripted[addr].size())return scripted[addr][index];return bytes.count(addr)?bytes[addr]:-1;}
int Spmi_Version(){return bus;}
void A64Device_Format(TDes8& d,const char* f,...){char b[256];va_list a;va_start(a,f);vsnprintf(b,sizeof(b),f,a);va_end(a);d.text+=b;}
void A64Mon_Say(const char*,...){}
#include "qcom_battery.cpp"
void init(){bytes.clear();scripted.clear();calls.clear();reads=0;bus=0x50000000;unsigned addresses[]={0x104,0x105,0x1210,0x4005,0x4105,0x4110,0x4046,0x4048,0x4109,0x4010,0x4009,0x400a,0x41a0,0x41a1,0x41a6,0x41a7,0x1310,0x160b,0x1006,0x1007};int values[]={0x51,0x1b,0,0x10,0x11,0,0x80,0,0,2,128,128,0xcc,0x8c,0xcc,0x8c,0x10,0x11,2,0};for(int i=0;i<20;++i)bytes[addresses[i]]=values[i];}
int main(){unsigned a[8];init();auto original=bytes;assert(A64BatteryRead(a)==0 && a[1]==63 && a[3]==50 && a[4]==4399891 && a[7]==2);assert(bytes==original);for(int raw:{0,1,254,255}){init();bytes[0x4009]=bytes[0x400a]=raw;A64BatteryRead(a);assert(a[3]==(raw==0?0:raw==1?1:raw==254?99:100));}
init();bytes[0x400a]=5;A64BatteryRead(a);assert(!(a[1]&2) && reads<100);
init();bytes[0x41a6]=5;A64BatteryRead(a);assert(!(a[1]&4) && reads<100);
for(unsigned reg:{0x4010,0x4046}){init();bytes[reg]=0;A64BatteryRead(a);assert(!(a[1]&2));}
for(unsigned reg:{0x4048,0x4109}){init();bytes[reg]=reg==0x4048?1:0x80;A64BatteryRead(a);assert(!(a[1]&6));}
init();bytes[0x1210]=0x20;A64BatteryRead(a);assert(a[2]==0 && !(a[1]&6) && a[7]==3);
for(int phase=0;phase<8;++phase){init();bytes[0x1006]=phase;A64BatteryRead(a);assert(a[7]==(phase<=4?2:phase<=6?4:3));}
init();bytes[0x160b]=0;A64BatteryRead(a);assert(a[7]==1 && a[5]==1 && a[6]==0);
init();bytes[0x1007]=0x20;A64BatteryRead(a);assert(a[7]==3);
init();bytes[0x105]=0;assert(A64BatteryRead(a)==KErrNotSupported && a[1]==0);
init();bytes[0x4005]=-1;A64BatteryRead(a);assert(!(a[1]&7) && (a[1]&24));
init();bus=-18;assert(A64BatteryRead(a)==KErrNotReady && reads==0);
init();TDes8 out;assert(A64Battery_Hal(&out)==0 && out.data.size()==32 && out.data[0]==1 && out.data[4]==63 && out.data[12]==50);assert(A64Battery_Hal(0)==-38);
TDes8 detail;assert(BatteryStatus(detail)==EA64DevPartial);init();bytes[0x4010]=0;TDes8 unknown;BatteryStatus(unknown);assert(unknown.text.find("percent unknown")!=std::string::npos);
init();scripted[0x4009]={128,129,130,131,132};scripted[0x400a]={5,6,7,8,9};A64BatteryRead(a);assert(!(a[1]&2) && calls[0x4009]==5 && calls[0x400a]==5);
init();scripted[0x41a0]={0,1,2,3,4};scripted[0x41a6]={5,6,7,8,9};A64BatteryRead(a);assert(!(a[1]&4) && calls[0x41a0]==5 && calls[0x41a6]==5);
init();scripted[0x1310]={16,0,16,0,16,0,16,0,16,0};A64BatteryRead(a);assert(!(a[1]&56) && calls[0x1310]==10);
init();scripted[0x4009]={-34};A64BatteryRead(a);assert(!(a[1]&2) && (a[1]&4) && calls[0x4009]==1);
init();bytes[0x1006]=-34;A64BatteryRead(a);assert((a[1]&8) && !(a[1]&48));
init();bytes[0x104]=-34;assert(A64BatteryRead(a)==-34 && !a[1]);
puts("PASS: battery actual-source identity, readiness, shadows/retry bounds, endpoints, 64-bit voltage, USB/state, partial validity and wire checks");}
''',
}

def main():
    source=Path(__file__).resolve().parents[1]/"sdm670_assp"
    with tempfile.TemporaryDirectory(prefix="neo-battery-") as directory:
        root=Path(directory)
        for name,content in FILES.items():
            p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(content)
        binary=root/"test"
        subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-Wall","-Wextra","-I",str(root/"include"),"-I",str(source),str(root/"test.cpp"),"-o",str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
if __name__=="__main__": main()
