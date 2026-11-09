#!/usr/bin/env python3
"""Jawnie zlecone zero enkoderów czterech odłączonych wałów, bez ENABLE/ruchu."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import time

AXES = (1, 2, 3, 5)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--zero-decoupled', action='store_true')
    args = parser.parse_args()
    if not args.zero_decoupled:
        parser.error('Wymagana jawna zgoda --zero-decoupled')
    args.output.mkdir(parents=True, exist_ok=True)
    link = json.loads(subprocess.check_output(['ip','-j','-details','link','show','can0'], text=True))[0]
    assert link['linkinfo']['info_data']['bittiming']['bitrate'] == 1000000
    lock = open(f'/tmp/robot-can-{os.getuid()}-can0.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, 0x1fc)
    sock.bind(('can0',));sock.setblocking(False)
    log = (args.output/'zero-frames.jsonl').open('x', buffering=1)
    def query(axis, command):
        assert command in (0x31,0x3a,0x92)
        data=bytes([command,(axis+command)&255]);sock.send(struct.pack('=IB3x8s',axis,2,data))
        log.write(json.dumps({'event':'tx','mono_ns':time.monotonic_ns(),'axis':axis,'data':data.hex()})+'\n')
        end=time.monotonic()+.15
        while time.monotonic()<end:
            if not select.select([sock],[],[],max(0,end-time.monotonic()))[0]:break
            address,size,raw=struct.unpack('=IB3x8s',sock.recv(16));raw=raw[:size]
            log.write(json.dumps({'event':'rx','mono_ns':time.monotonic_ns(),'can_id':address,'data':raw.hex()})+'\n')
            if address&0x20000000:
                recovered=(address&0x1fffffff)==4 and len(raw)==8 and raw[1]==0x40 and not any(raw[i] for i in (0,2,3,4,5))
                if recovered:continue
                raise RuntimeError(f'Błąd CAN {address:x}:{raw.hex()}')
            assert not address&0xc0000000 and len(raw)>=2 and (address+sum(raw[:-1]))&255==raw[-1], 'Nieprawidłowa ramka'
            if address==axis and raw[0]==command:
                assert len(raw)==(8 if command==0x31 else 3)
                return raw[1:-1]
        raise RuntimeError(f'Timeout J{axis}/{command:02x}')
    proof={'date':datetime.now().astimezone().isoformat(),'bitrate':1000000,'axes':{},'commanded_motion':False,'enable_written':False}
    try:
        # Wszystkie warunki sprawdzamy przed pierwszą zmianą zera.
        for axis in AXES:
            assert query(axis,0x3a)==b'\0', f'J{axis}: wymagane ENABLE=0'
            positions=[]
            for _ in range(3):
                positions.append(int.from_bytes(query(axis,0x31),'big',signed=True));time.sleep(.05)
            assert max(positions)-min(positions)<=3, f'J{axis}: brak postoju'
            proof['axes'][axis]={'before_ticks':positions[-1]}
        for axis in AXES:
            assert query(axis,0x3a)==b'\0'
            assert query(axis,0x92)==b'\1', f'J{axis}: brak ACK92'
            after=int.from_bytes(query(axis,0x31),'big',signed=True)
            assert abs(after)<=3, f'J{axis}: niepotwierdzone zero'
            proof['axes'][axis].update(after_ticks=after,enable_after=query(axis,0x3a).hex())
            assert proof['axes'][axis]['enable_after']=='00'
        proof['success']=True
        print(json.dumps(proof,ensure_ascii=False),flush=True)
    finally:
        (args.output/'zero.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2))
        log.close();sock.close();lock.close()

if __name__=='__main__':main()
