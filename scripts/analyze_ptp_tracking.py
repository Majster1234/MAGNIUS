#!/usr/bin/env python3
"""Porównaj zaakceptowane pomiary z trajektorią nominalną i uruchomionymi celami.

Analiza offline; nie otwiera CAN. Stałe pasmo porównania to ±0,5° dla wszystkich
ustawień wykonawcy. Przerwane próby mają wynik tylko dla zmierzonego fragmentu.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics


def reference(start,target,duration,seconds):
    u=max(0.,min(1.,seconds/duration))
    return start+(target-start)*(10*u**3-15*u**4+6*u**5)


def metrics(errors):
    if not errors:return None
    return dict(samples=len(errors),within_half_degree_percent=100*sum(abs(e)<=.5 for e in errors)/len(errors),
                mae_deg=statistics.mean(abs(e) for e in errors),rmse_deg=math.sqrt(statistics.mean(e*e for e in errors)),
                max_abs_error_deg=max(abs(e) for e in errors))


def analyze(directory):
    summary=json.loads((directory/'summary.json').read_text())
    run=[json.loads(x) for x in (directory/'runner.jsonl').read_text().splitlines()]
    rows=[json.loads(x) for x in (directory/'can-0.jsonl').read_text().splitlines()]
    axes=list(map(int,summary['axes'].split(',')))
    latest={};accepted=[]
    for row in rows:
        key=(row['axis'],row['command'])
        if row['event']=='can_rx':latest[key]=row
        if row['event']=='reply_ok' and row['command']==49 and row['axis'] in axes:
            rx=latest[key];data=bytes.fromhex(rx['data'])
            assert len(data)==8 and (row['axis']+sum(data[:-1]))&255==data[-1]
            accepted.append((row['request_ns']+row['duration_ns'],row['axis'],int.from_bytes(data[1:7],'big',signed=True)*360/16384))
    samples=[];legs=[]
    for plan in [x for x in run if x['event']=='trajectory']:
        prepared=next(x for x in run if x['event']=='prepared' and x['detail'].startswith(plan['label']+' '))
        end=next((x for x in run if x.get('mono_ns',0)>prepared['mono_ns'] and
                  (x['event']=='failure' or x['event']=='leg_pass' and x['detail'].startswith(plan['label']+' '))),None)
        if not end:continue
        continuous=plan.get('use_ack') is False
        marker_axis=min(axes)
        sync=[x['mono_ns'] for x in rows if x['event']=='can_tx' and
              ((continuous and x['command']==245 and x['axis']==marker_axis) or
               (not continuous and x['command']==75 and x['axis']==0)) and
              prepared['mono_ns']<=x['mono_ns']<=end['mono_ns']]
        sample_seconds=plan.get('sample_ms',20)/1000
        phase_shift=0 if continuous else sample_seconds
        if not sync:continue
        profiles=plan.get('axis_profiles')
        if continuous and (not profiles or len(profiles)!=6):
            raise ValueError('Analiza streaming wymaga profilu przekładni/kierunku/offsetu w trajectory')
        def joint_degrees(axis,shaft_degrees):
            if not continuous:return shaft_degrees
            p=profiles[axis-1]
            return p['direction']*shaft_degrees/p['gear_ratio']+math.degrees(p['offset_rad'])
        launched_points={a:[] for a in axes}
        if continuous:
            for row in rows:
                if row['event']!='can_tx' or row['command']!=245 or row['axis'] not in launched_points or not sync[0]<=row['mono_ns']<=end['mono_ns']:continue
                data=bytes.fromhex(row['data'])
                assert len(data)==8 and (row['axis']+sum(data[:-1]))&255==data[-1]
                target=int.from_bytes(data[4:7],'big',signed=True)*360/16384
                launched_points[row['axis']].append((row['mono_ns'],joint_degrees(row['axis'],target)))
        assert ' streamed ' in prepared['detail'],'Ten analizator wymaga jawnego mapowania fazy dla streamed'
        leg_samples=[]
        for stamp,axis,q in accepted:
            if not sync[0]<=stamp<=end['mono_ns']:continue
            frame=bisect.bisect_right(sync,stamp)
            elapsed=(stamp-sync[0])/1e9
            # Starszy SYNC uruchamiał t=20 ms. Streaming startuje od t=0;
            # nadany cel odczytujemy z F5, również po pominięciu klatek/lookahead.
            nominal=reference(plan['start_deg'][axis-1],plan['target_deg'],plan['duration'],elapsed+phase_shift)
            if continuous:
                values=launched_points[axis];index=bisect.bisect_right([t for t,_ in values],stamp)-1
                if index<0:continue
                launched=values[index][1];q=joint_degrees(axis,q)
            else:launched=reference(plan['start_deg'][axis-1],plan['target_deg'],plan['duration'],frame*sample_seconds)
            item=dict(leg=plan['label'],axis=axis,mono_ns=stamp,elapsed_s=elapsed,measured_deg=q,
                      nominal_deg=nominal,launched_deg=launched,nominal_error_deg=q-nominal,launched_error_deg=q-launched)
            leg_samples.append(item);samples.append(item)
        legs.append(dict(label=plan['label'],completed=end['event']=='leg_pass',planned_seconds=plan['duration'],
                         actual_seconds=(end['mono_ns']-sync[0])/1e9,syncs=len(sync),expected_syncs=plan['points'],
                         nominal=metrics([x['nominal_error_deg'] for x in leg_samples]),
                         launched=metrics([x['launched_error_deg'] for x in leg_samples])))
    with (directory/'tracking-samples.csv').open('w') as output:
        columns=['leg','axis','mono_ns','elapsed_s','measured_deg','nominal_deg','launched_deg','nominal_error_deg','launched_error_deg']
        writer=csv.DictWriter(output,fieldnames=columns);writer.writeheader();writer.writerows(samples)
    result=dict(attempt=summary['attempt'],tolerance_deg=summary['tolerance_deg'],success=summary['success'],legs=legs,
                nominal=metrics([x['nominal_error_deg'] for x in samples]),launched=metrics([x['launched_error_deg'] for x in samples]),
                per_axis={a:dict(nominal=metrics([x['nominal_error_deg'] for x in samples if x['axis']==a]),
                                launched=metrics([x['launched_error_deg'] for x in samples if x['axis']==a])) for a in axes})
    (directory/'tracking.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    return result


def plot(directory,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    trials=sorted(directory.glob('axes-*/tracking-samples.csv'))
    fig,panels=plt.subplots(len(trials),2,figsize=(13,3*len(trials)),squeeze=False,constrained_layout=True)
    for index,path in enumerate(trials):
        data=list(csv.DictReader(path.open()));meta=json.loads((path.parent/'summary.json').read_text())
        offset=0
        for leg in ('plus20','return0'):
            points=[x for x in data if x['leg']==leg]
            if not points:continue
            for color,axis in enumerate((1,2,3,5)):
                values=[x for x in points if int(x['axis'])==axis];times=[float(x['elapsed_s'])+offset for x in values]
                panels[index,0].plot(times,[float(x['measured_deg']) for x in values],color=f'C{color}',label=f'J{axis}' if leg=='plus20' else None,linewidth=.9)
                panels[index,1].plot(times,[float(x['nominal_error_deg']) for x in values],color=f'C{color}',linewidth=.9)
            j1=[x for x in points if int(x['axis'])==1]
            panels[index,0].plot([float(x['elapsed_s'])+offset for x in j1],[float(x['nominal_deg']) for x in j1],'k--',linewidth=1,label='plan nominalny J1' if leg=='plus20' else None)
            offset+=max(float(x['elapsed_s']) for x in points)+.1
        mode='100 Hz bez ACK' if meta.get('use_ack') is False else f"ACK {meta.get('ack_timeout_ms',5)} ms"
        panels[index,0].set_title(f"Próba {meta['attempt']}, tolerancja ±{meta['tolerance_deg']}°, {mode}")
        panels[index,0].set_ylabel('Kąt przegubu [°]');panels[index,0].legend(ncol=5,fontsize=7)
        panels[index,1].set_ylabel('Błąd względem czasu nominalnego [°]')
        for bound in (-.5,.5):panels[index,1].axhline(bound,color='black',linestyle='--',linewidth=.7)
        for panel in panels[index]:panel.grid(alpha=.25);panel.set_xlabel('Czas ruchu [s], odcinki połączone')
    fig.savefig(output);plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--directory',type=Path,required=True);parser.add_argument('--plot',type=Path)
    args=parser.parse_args();result=[analyze(d) for d in sorted(args.directory.glob('axes-*')) if (d/'summary.json').exists()]
    (args.directory/'tracking.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    if args.plot:plot(args.directory,args.plot)
    for row in result:print(json.dumps({k:row[k] for k in ('attempt','tolerance_deg','success','nominal','launched')},ensure_ascii=False))
