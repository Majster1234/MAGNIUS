"""Porównywalne 150 próbek/s, 20 stanów/s i 0/1/3 odbiorców; bez ROS/CAN/GUI."""
import argparse
import json
import os
from pathlib import Path
import resource
import sys
import tempfile
import time
from types import SimpleNamespace as NS

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_web_bridge'))
from robot_web_bridge.manual import ManualStore
try:
    from robot_web_bridge.shaft_trace import ShaftTraceStore
except ImportError:
    ShaftTraceStore=None


def run(clients, seconds, record_trace=True):
    directory=tempfile.TemporaryDirectory()
    recorder=ShaftTraceStore(directory.name) if ShaftTraceStore and record_trace else None
    manual=ManualStore()
    cursors=[0]*clients
    cpu=lambda:resource.getrusage(resource.RUSAGE_SELF).ru_utime+resource.getrusage(resource.RUSAGE_SELF).ru_stime
    started=time.monotonic();previous_cpu=cpu();previous_time=started
    cpu_samples=[];latencies=[];rows=0
    cycles=round(seconds*25)
    for i in range(cycles):
        stamp=time.monotonic_ns()
        begin=time.perf_counter_ns()
        if i%5!=4:
            axes=[NS(axis=a,available=True,prepared=False,enable_known=True,enabled=False,position_known=True,
                     position_degrees=(i%100)*.01,position_age_seconds=0.,speed_known=True,speed_rpm=0.,
                     speed_age_seconds=0.,enable_age_seconds=0.,can_jog=False,can_zero=False,motion='zatrzymany') for a in range(1,7)]
            manual.update(NS(monotonic_ns=stamp,busy=False,fault=False,active_axis=0,max_rpm=30,
                             operation='idle',message='benchmark',axes=axes))
        if recorder:
            recorder.ingest(NS(dropped_total=0,samples=[NS(axis=a,sequence=i+1,segment=1,
                monotonic_ns=stamp,utc_ns=1_792_700_000_000_000_000+i*40_000_000,
                ticks=(i%100)*a,flags=0) for a in range(1,7)]))
        for client in range(clients):
            manual.snapshot()
            if recorder:
                frame=recorder.live(cursors[client]);cursors[client]=frame['cursor'];json.dumps(frame)
        latencies.append((time.perf_counter_ns()-begin)/1000)
        rows+=6
        time.sleep(max(0.,started+(i+1)*.04-time.monotonic()))
        now=time.monotonic()
        if now-previous_time>=.99:
            current_cpu=cpu();cpu_samples.append((current_cpu-previous_cpu)/(now-previous_time)*100)
            previous_cpu,previous_time=current_cpu,now
    status=recorder.status() if recorder else None
    if recorder:recorder.close()
    result=dict(clients=clients,cycles=cycles,input_samples=rows,cpu_percent=cpu_samples,
                callback_us=latencies,recording=recorder is not None,status=status)
    directory.cleanup()
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--legacy',action='store_true',help='Sam dotychczasowy ManualStore, bez nowego rejestratora')
    args=parser.parse_args()
    result=[run(clients,args.seconds,not args.legacy) for clients in (0,1,3)]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as file:json.dump(result,file)
    print(args.output)
