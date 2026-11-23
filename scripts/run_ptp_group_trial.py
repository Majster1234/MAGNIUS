#!/usr/bin/env python3
"""Jedna zatwierdzona próba rozsprzęglonych osi; produkcyjna biblioteka PTP.

Nie uruchamia sekwencji bez --execute i --decoupled. Właściciel CAN musi być
zamknięty; blokada SocketCAN nadal obowiązuje. Raporty pozostają w log/.
"""
import argparse
from datetime import datetime
import hashlib
import json
import os
import resource
from pathlib import Path
import signal
import select
import struct
import subprocess
import time
from zoneinfo import ZoneInfo
from test_ptp_hardware import Bench

ROOT=Path(__file__).resolve().parents[1]
DIRECTORY=ROOT/'log/hardware-ptp/group-20261024'
IDS=(1,2,3,5)

class GroupBench(Bench):
    def receive(self,timeout):
        if not select.select([self.sock],[],[],max(0,timeout))[0]:return None
        axis,length,data=struct.unpack('=IB3x8s',self.sock.recv(16));data=data[:length]
        if axis & 0x20000000:
            self.event('can_controller_event',can_id=f'{axis:x}',data=data.hex())
            # CAN_ERR_CRTL_ACTIVE oznacza powrót do stanu aktywnego;
            # ostrzeżenia, error-passive, BUS-OFF i błędy protokołu nadal przerywają.
            if axis==0x20000004 and len(data)==8 and data[1]==0x40 and not any(data[i] for i in (0,2,3,4,5)):
                return None
            raise RuntimeError(f'Ramka błędu CAN {axis:x}: {data.hex()}')
        self.event('rx',axis=axis,data=data.hex())
        if axis & 0xc0000000 or len(data)<2 or (axis+sum(data[:-1]))&255!=data[-1]:
            raise RuntimeError('Nieprawidłowa ramka/CRC')
        if data[0]==0xf5 and data[1] in (0,3):raise RuntimeError(f'CAN {axis}: błąd/krańcówka F5 {data[1]}')
        return axis,data

    def stop_all_zero(self,required=()):
        # Bez oczekiwania na ACK ruchu, także gdy napędy mają CanRSP=Disable.
        errors=[]
        for code,payload in ((0xf7,b''),(0xf6,b'\0\0\x01')):
            for axis in range(1,7):
                try:self.send(axis,code,payload)
                except Exception as exc:errors.append(str(exc))
        self.event('stop_f7_zero_no_ack',errors=errors,note='Nadanie nie potwierdza fizycznego postoju')
        if errors:raise RuntimeError('; '.join(errors))

def raw_session(output,baseline,zero_axes=()):
    bench=GroupBench(output)
    try:
        for axis in IDS:
            saved={code:bench.read(axis,code) for code in (0x82,0x84,0x8c,0x98,0x4a)}
            if int.from_bytes(saved[0x84],'big')!=16:raise RuntimeError(f'CAN {axis}: mikrokrok różny od 16')
            if bench.query(axis,0x3a)!=b'\0':raise RuntimeError(f'CAN {axis}: brak wyłączonego ENABLE')
            if not baseline.exists():bench.saved[axis]=saved
            bench.event('baseline',axis=axis,identity=bench.query(axis,0x40).hex(),position=bench.pos(axis),parameters={str(k):v.hex() for k,v in saved.items()})
        if not baseline.exists():
            baseline.write_text(json.dumps({str(a):{str(c):v.hex() for c,v in p.items()} for a,p in bench.saved.items()},indent=2))
        for axis in zero_axes:
            before=bench.pos(axis);time.sleep(.12);after=bench.pos(axis)
            if abs(after-before)>3:raise RuntimeError(f'CAN {axis}: brak postoju przed zerowaniem')
            bench.write(axis,0x92,b'')
            measured=bench.pos(axis)
            if abs(measured)>3:raise RuntimeError(f'CAN {axis}: niepotwierdzone zero')
            bench.event('zero_confirmed',axis=axis,position=measured)
    finally:
        bench.sock.close();bench.lock.close();bench.log.close()

def cleanup(output,baseline):
    # C++ wcześniej wysłał STOP/ENABLE=0. Daj zakończyć pozostałe odpowiedzi
    # z zamkniętego procesu przed odtworzeniem parametrów przez nowego właściciela.
    time.sleep(.05)
    bench=GroupBench(output)
    bench.saved={int(a):{int(c):bytes.fromhex(v) for c,v in p.items()} for a,p in json.loads(baseline.read_text()).items()}
    return bench.cleanup()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--decoupled',action='store_true')
    parser.add_argument('--axes',choices=('1','1,2','1,2,3,5'),required=True)
    parser.add_argument('--attempt',type=int,required=True);parser.add_argument('--intervention',required=True)
    parser.add_argument('--tick-us',type=int,default=1000);parser.add_argument('--duration-scale',type=float,default=1)
    parser.add_argument('--rezero',action='store_true')
    parser.add_argument('--directory',type=Path,default=DIRECTORY)
    parser.add_argument('--binary',type=Path,default=ROOT/'build/streaming100-dev/ptp_group_hardware')
    parser.add_argument('--library',type=Path,default=ROOT/'install/streaming100-ros/mks_servo_can/lib/libmks_servo_can.so')
    parser.add_argument('--ack-ms',type=int,choices=(5,10),default=10)
    parser.add_argument('--tolerance-deg',type=float,choices=(.25,.5,1.),default=.5)
    args=parser.parse_args()
    if not args.execute or not args.decoupled or not 1<=args.attempt<=6:parser.error('Wymagana zgoda, rozsprzęglone wały i numer 1..6')
    args.binary=args.binary.resolve();args.library=args.library.resolve()
    if not args.binary.is_file() or not args.library.is_file():parser.error('Najpierw zbuduj nowy wykonawca i bibliotekę streaming100; nie używaj starych binariów ACK')
    args.directory=args.directory.resolve()
    directory=args.directory/f'axes-{args.axes.replace(",","-")}-attempt-{args.attempt}'
    directory.mkdir(parents=True,exist_ok=False)
    baseline=args.directory/'drive-baseline.json';start=time.monotonic();started=datetime.now(ZoneInfo('Europe/Prague')).isoformat()
    (directory/'trial-metadata.json').write_text(json.dumps(dict(started_at=started,start_mono_ns=time.monotonic_ns(),
          axes=args.axes,attempt=args.attempt,ack_ms=args.ack_ms,tolerance_deg=args.tolerance_deg,intervention=args.intervention),ensure_ascii=False,indent=2))
    child_cpu=None;child_seconds=None
    result={};exit_code=None;cleanup_ok=False;error=''
    try:
        raw_session(directory/'preflight.jsonl',baseline,tuple(map(int,args.axes.split(','))) if args.rezero else ())
        command=[str(args.binary),'--execute','--decoupled','--axes',args.axes,
                 '--profile',str(args.directory/'profile.txt'),'--tick-us',str(args.tick_us),'--duration-scale',str(args.duration_scale),
                 '--ack-ms',str(args.ack_ms),'--tolerance-deg',str(args.tolerance_deg)]
        with (directory/'runner.jsonl').open('x') as output:
            cpu_before=resource.getrusage(resource.RUSAGE_CHILDREN);child_start=time.monotonic()
            p=subprocess.Popen(command,cwd=ROOT,stdout=output,stderr=subprocess.STDOUT,
                 env={**os.environ,'LD_LIBRARY_PATH':str(args.library.parent)+':'+os.environ.get('LD_LIBRARY_PATH',''),
                      'ROBOT_TRACE_DIR':str(directory),'ROBOT_TRACE_SESSION':directory.name})
            try:exit_code=p.wait(timeout=85)
            except subprocess.TimeoutExpired:
                error='Zewnętrzny limit procesu 85 s';p.send_signal(signal.SIGINT)
                try:exit_code=p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();exit_code=p.wait()
            cpu_after=resource.getrusage(resource.RUSAGE_CHILDREN)
            child_cpu=cpu_after.ru_utime+cpu_after.ru_stime-cpu_before.ru_utime-cpu_before.ru_stime
            child_seconds=time.monotonic()-child_start
            (directory/'process-usage.json').write_text(json.dumps(dict(cpu_seconds=child_cpu,wall_seconds=child_seconds,exit_code=exit_code),indent=2))
        for line in (directory/'runner.jsonl').read_text().splitlines():
            try:
                event=json.loads(line)
                if event.get('event')=='trial_result':result=event
                if event.get('event') in ('failure','startup_error','cleanup_error'):error=event.get('detail','')
            except json.JSONDecodeError:pass
    except Exception as exc:error=str(exc)
    finally:
        if baseline.exists():
            try:cleanup_ok=cleanup(directory/'cleanup.jsonl',baseline)
            except Exception as exc:error+='; cleanup: '+str(exc)
            if not cleanup_ok and not error:error='Błąd końcowej procedury STOP/odtworzenia parametrów; patrz cleanup.jsonl'
    row={'axes':args.axes,'attempt':args.attempt,'started_at':started,'ended_at':datetime.now(ZoneInfo('Europe/Prague')).isoformat(),
         'duration_seconds':time.monotonic()-start,'intervention':args.intervention,'tick_us':args.tick_us,'duration_scale':args.duration_scale,
         'use_ack':False,'sample_ms':10,'ack_timeout_ms':args.ack_ms,'tolerance_deg':args.tolerance_deg,'child_cpu_seconds':child_cpu,'child_wall_seconds':child_seconds,
         'success':exit_code==0 and result.get('success') is True and cleanup_ok,'error':error or result.get('message','Brak wyniku'),
         'exit_code':exit_code,'cleanup_success':cleanup_ok,'result':result,'directory':str(directory.relative_to(ROOT)),
         'source_sha256':hashlib.sha256((ROOT/'scripts/ptp_group_hardware.cpp').read_bytes()).hexdigest(),
         'library_sha256':hashlib.sha256(args.library.read_bytes()).hexdigest()}
    (directory/'summary.json').write_text(json.dumps(row,ensure_ascii=False,indent=2))
    print(json.dumps(row,ensure_ascii=False),flush=True)
    return 0 if row['success'] else 1

if __name__=='__main__':raise SystemExit(main())
