#!/usr/bin/env python3
"""Analiza zapisanych prób grupowych PTP; bez dostępu do CAN."""
import collections
import argparse
import json
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parents[1]
DIRECTORY=ROOT/'log/hardware-ptp/group-20261024'

def stats(values):
    return dict(count=len(values),minimum=min(values),maximum=max(values),mean=statistics.mean(values)) if values else None

def analyze(directory):
    summary=json.loads((directory/'summary.json').read_text())
    rows=[json.loads(x) for x in (directory/'can-0.jsonl').read_text().splitlines()]
    run=[json.loads(x) for x in (directory/'runner.jsonl').read_text().splitlines()]
    selected=list(map(int,summary['axes'].split(',')))
    continuous=summary.get('use_ack') is False
    ack_ns=int(summary.get('ack_timeout_ms',5)*1_000_000)
    batches=collections.defaultdict(list)
    for x in rows:
        if x['event']=='reply_ok' and x['command']==245:batches[x['deadline_ns']].append(x)
    ack=[(max(y['request_ns']+y['duration_ns'] for y in group)-(deadline-ack_ns))/1e6
         for deadline,group in batches.items() if sorted(y['axis'] for y in group)==selected]
    incomplete=[d for d,g in batches.items() if sorted(x['axis'] for x in g)!=selected]
    syncs=[x for x in rows if x['event']=='can_tx' and x['command']==75 and x['axis']==0]
    bad_sync=[]
    previous_sync=0
    for sync in syncs:
        preceding=[(d,g) for d,g in batches.items()
                   if previous_sync<d-ack_ns<sync['mono_ns'] and max(x['mono_ns'] for x in g)<sync['mono_ns']]
        if len(preceding)!=1 or preceding[0][0] in incomplete:bad_sync.append(sync['sequence'])
        previous_sync=sync['mono_ns']
    positions={a:[] for a in selected}
    for x in rows:
        if x['event']=='can_rx' and x['command']==49 and x['axis'] in positions:
            data=bytes.fromhex(x['data'])
            if len(data)==8 and (x['axis']+sum(data[:-1]))&255==data[-1]:
                positions[x['axis']].append((x['mono_ns'],int.from_bytes(data[1:7],'big',signed=True)*360/16384))
    legs=[]
    for event in run:
        if event['event']=='leg_pass':
            legs.append(dict(detail=event['detail'],positions={a:next((q for t,q in reversed(positions[a]) if t<=event['mono_ns']),None) for a in selected}))
    streaming_frames=[x for x in rows if x['event']=='can_tx' and x['command']==245 and x['axis'] in selected]
    motion_start=min((x['mono_ns'] for x in (streaming_frames if continuous else syncs)),default=0)
    motion_end=max((x['mono_ns'] for x in run if x['event'] in ('leg_pass','failure')),default=0)
    periods={}
    rpm={}
    for a,points in positions.items():
        query=[x['mono_ns'] for x in rows if x['event']=='can_tx' and x['command']==49 and x['axis']==a and motion_start<=x['mono_ns']<=motion_end]
        periods[a]=stats([(b-a)/1e6 for a,b in zip(query,query[1:])])
        during=[(t,q) for t,q in points if motion_start<=t<=motion_end]
        rpm[a]=stats([abs(q2-q1)*1e9/(6*(t2-t1)) for (t1,q1),(t2,q2) in zip(during,during[1:]) if t2>t1])
    errors=collections.Counter(x['data'] for x in rows if x['event']=='can_error_frame')
    timeouts=collections.Counter((x['axis'],x['command']) for x in rows if x['event']=='request_timeout')
    late_ack=[dict(axis=x['axis'],elapsed_ms=(x['request_ns']+x['duration_ns']-(x['deadline_ns']-ack_ns))/1e6,
                   kernel_to_process_ms=(x['utc_ns']-x['kernel_utc_ns'])/1e6 if x.get('kernel_utc_ns') else None)
              for x in rows if x['event']=='reply_late' and x['command']==245]
    faults=[x['mono_ns'] for x in rows if x['event']=='bus_fault' and 'Koniec próby' not in x['detail']]
    sync_after_fault=sum(x['mono_ns']>min(faults) for x in syncs) if faults else 0
    return dict(axes=selected,attempt=summary['attempt'],success=summary['success'],legs=legs,
                use_ack=not continuous,stream_f5_count=len(streaming_frames),
                batch_ack_ms=stats(ack),incomplete_ack_batches=len(incomplete),syncs=len(syncs),sync_without_full_ack=bad_sync,
                ack_late_or_invalid=sum(x['event'] in ('reply_late','reply_invalid','request_timeout') and x['command']==245 for x in rows),
                late_ack=late_ack,sync_after_fault=sync_after_fault,
                position_query_period_ms=periods,encoder_difference_rpm=rpm,
                timeout_counts=[dict(axis=a,command=f'{c:02x}',count=n) for (a,c),n in timeouts.items()],
                can_errors=dict(errors),can_error_count=sum(errors.values()),max_trace_dropped=max(x.get('dropped_total',0) for x in rows),
                states=dict(collections.Counter(x['detail'].split(' ')[1].rstrip(':') for x in run if x['event']=='state')))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--directory',type=Path,default=DIRECTORY)
    directory=parser.parse_args().directory
    result=[analyze(d) for d in sorted(directory.glob('axes-*')) if (d/'summary.json').exists()]
    (directory/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    for x in result:print(json.dumps(x,ensure_ascii=False))
