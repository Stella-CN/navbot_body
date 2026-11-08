#!/usr/bin/env python3
"""Hardware protocol checks: zero targets only; briefly arms the speed controller."""
import argparse
import json
import struct
import time
from pathlib import Path
from chassis_serial import Client


def run(port, output):
    c=Client(port); report={'passed':False,'checks':[]}; latest={}
    def wait(seconds=.13):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            f=c.receive()
            if f: latest[f['id']]=f['data']
    def ack(mid,payload=b'',result=0,**kw):
        d=c.request(mid,payload,expected=0x7f,**kw)['data']
        assert d['result']==result,d
        report['checks'].append({'id':mid,'expected':result,'actual':d})
    try:
        report['hello']=c.request(1)['data'];assert report['hello']['firmware'] >> 8 == 0x101
        ack(0x12);wait()
        ack(0x14,struct.pack('<4fH',0,0,0,0,200),4)
        ack(0x14,b'',2)
        ack(0x14,struct.pack('<4fH',float('nan'),0,0,0,200),5)
        ack(0x14,struct.pack('<4fH',13,0,0,0,200),5)
        ack(0x14,struct.pack('<4fH',0,0,0,0,251),5)
        ack(0x31,struct.pack('<ff4b',.0425,.228,0,1,1,1),5)
        ack(0x11,b'\1');wait(.03)
        ack(0x31,struct.pack('<ff4b',.0425,.228,1,1,1,1),4)
        ack(0x14,struct.pack('<4fH',0,0,0,0,200))
        old=c.seq-1
        ack(0x14,struct.pack('<4fH',0,0,0,0,200),3,seq=old)
        wait(.35);assert latest[0x24]['state']==2 and not any(latest[0x24]['duty']),latest
        ack(0x14,struct.pack('<4fH',0,0,0,0,200),4)
        ack(0x11,b'\1',4)
        ack(0x12);wait()
        ack(0x11,b'\1');wait(.03)
        c.session=(c.session%0xffffffff)+1;c.seq=0;c.last_sequence=None;c.pending.clear()
        c.request(1);wait()
        assert latest[0x24]['state']==0 and not any(latest[0x24]['duty']),latest
        ack(0x14,struct.pack('<4fH',0,0,0,0,200),4)
        report['final']=latest[0x24];report['passed']=True
    finally:
        try:c.request(0x12,expected=0x7f)
        finally:
            c.close();output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--port',required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();run(a.port,a.output)
