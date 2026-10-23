#!/usr/bin/env python3
"""Pasywny odbiór: istniejące ramki can0 i paczki ROS, bez nadawania CAN."""
import argparse
from collections import Counter
import json
from pathlib import Path
import select
import socket
import struct
import statistics
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--can-only',action='store_true')
    args=parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('Czas obserwacji musi należeć do zakresu (0, 60] s')
    if args.output.exists(): parser.error('Plik wynikowy już istnieje')
    can=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    can.setsockopt(socket.SOL_SOCKET,35,1)  # Linux SO_TIMESTAMPNS
    can.bind(('can0',));can.setblocking(False)
    rows,points,commands,batches,duplicates=[],[],Counter(),0,0
    received_at,sizes,delays,dropped=[],[],[],[]
    sequences={}
    node=None
    if not args.can_only:
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from robot_control_interfaces.msg import ShaftTraceBatch
        rclpy.init();node=rclpy.create_node('passive_shaft_trace_observer')
        def receive(batch):
            nonlocal batches,duplicates
            batches+=1
            received_at.append(time.monotonic_ns());sizes.append(len(batch.samples));dropped.append(batch.dropped_total)
            for p in batch.samples:
                key=(p.axis,p.segment,p.sequence)
                duplicates+=key in sequences;sequences[key]=True
                points.append((int(p.utc_ns),int(p.axis),int(p.ticks)))
                delays.append((time.time_ns()-int(p.utc_ns))/1e6)
        subscription=node.create_subscription(ShaftTraceBatch,'/robot/diagnostics/shaft_trace',receive,
                                               QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
    started=time.monotonic();ended=started+args.seconds
    try:
        while time.monotonic()<ended:
            if node is not None: rclpy.spin_once(node,timeout_sec=.001)
            else: select.select([can],[],[],.005)
            for _ in range(128):
                try: data,ancillary,_,_=can.recvmsg(16,64)
                except BlockingIOError: break
                if len(data)!=16: continue
                identifier,length,payload=struct.unpack('=IB3x8s',data)
                if identifier&~0x7ff: continue
                address=identifier&0x7ff;payload=payload[:length]
                if not payload: continue
                commands[f'{payload[0]:02x}/{length}']+=1
                if not 1<=address<=6 or length!=8 or payload[0]!=0x31 or sum(bytes([address&255])+payload[:-1])&255!=payload[-1]: continue
                stamp=None
                for level,kind,value in ancillary:
                    if level==socket.SOL_SOCKET and kind==35:
                        seconds,nanoseconds=struct.unpack('ll',value[:struct.calcsize('ll')]);stamp=seconds*1_000_000_000+nanoseconds
                if stamp is not None: rows.append((stamp,address,int.from_bytes(payload[1:7],'big',signed=True)))
    finally:
        can.close()
        if node is not None:node.destroy_node();rclpy.shutdown()
    raw,ros=set(rows),set(points)
    by_axis={}
    for axis in range(1,7):
        stamps=[p[0] for p in rows if p[1]==axis]
        intervals=sorted((b-a)/1e6 for a,b in zip(stamps,stamps[1:]))
        trace_stamps=[p[0] for p in points if p[1]==axis]
        common=[p for p in rows if p[1]==axis and trace_stamps and min(trace_stamps)<=p[0]<=max(trace_stamps)]
        by_axis[str(axis)]=dict(can_samples=len(stamps),ros_samples=sum(p[1]==axis for p in points),
            matched=sum(p[1]==axis for p in raw&ros),
            common_window_can_samples=len(common),common_window_matched=sum(p in ros for p in common),
            encoder_period_median_ms=intervals[len(intervals)//2] if intervals else None)
    def stats(values):
        return dict(minimum=min(values),median=statistics.median(values),p95=sorted(values)[int((len(values)-1)*.95)],maximum=max(values)) if values else None
    result=dict(passive=True,duration_seconds=time.monotonic()-started,can_tx_from_tool=0,
                frames_by_opcode_dlc=dict(commands),batches=batches,duplicate_ros_samples=duplicates,
                matched_samples=len(raw&ros),can_samples=len(rows),ros_samples=len(points),axes=by_axis)
    result.update(batch_period_ms=stats([(b-a)/1e6 for a,b in zip(received_at,received_at[1:])]),
                  samples_per_batch=stats(sizes),delivery_delay_ms=stats(delays),source_dropped=max(dropped,default=0))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as file:json.dump(result,file,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
