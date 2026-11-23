"""Znane próbki i zegary; brak ROS, SocketCAN i wywołań narzędzi sprzętowych."""
import json
import math
from pathlib import Path
import pytest
from scripts.analyze_ptp_tracking import analyze, reference
from scripts.ptp_tracking_metrics import (percentile, nominal, error_metrics,
                                         desynchronization, settling_time, trajectory_report)


def packet(axis, body):
    return bytes(body+[sum(body+ [axis]) & 255]).hex()


def capture(path, *, complete=True, skipped=(), quiet=False, missing=(), profiles=None,start=0.,goal=1.):
    path.mkdir()
    profiles=profiles or [dict(gear_ratio=1,direction=1,offset_rad=0) for _ in range(6)]
    plan=dict(event='trajectory',label='plus',duration=.2,target_deg=goal,start_deg=[start]*6,
              axis_profiles=profiles,points=21,sample_ms=10,use_ack=False)
    origin=1_000_000_000;end=origin+int((.36 if complete else .15)*1e9)
    run=[plan,dict(event='prepared',mono_ns=origin-1,detail='plus streamed duration=.2'),
         dict(event='leg_pass' if complete else 'failure',mono_ns=end,detail='plus complete')]
    rows=[]
    for k in range(36):
        for axis in (1,2):
            stamp=origin+k*10_000_000+(axis-1)*200_000
            p=profiles[axis-1]
            q=reference(start,goal,.2,k*.01)
            ticks=round((q-math.degrees(p['offset_rad']))*p['gear_ratio']/p['direction']*16384/360)
            if k<=20 and k not in skipped:
                raw=ticks & 0xffffff
                body=[0xf5,0,1,135,raw>>16,(raw>>8)&255,raw&255]
                rows.append(dict(event='can_tx',axis=axis,command=0xf5,mono_ns=stamp,data=packet(axis,body)))
            if axis not in missing:
                body=[0x31,*ticks.to_bytes(6,'big',signed=True)]
                rx=stamp+1_000_000
                rows += [dict(event='can_rx',axis=axis,command=0x31,mono_ns=rx,data=packet(axis,body)),
                         dict(event='reply_ok',axis=axis,command=0x31,mono_ns=rx+1,
                              request_ns=stamp,duration_ns=1_000_000)]
        rows.append(dict(event='can_tx',axis=1,command=0x31,mono_ns=origin+k*10_000_000+3_000_000,data='3132'))
    if quiet:
        rows.append(dict(event='can_tx',axis=2,command=0x32,mono_ns=origin+57_000_000,data='3234'))
    rows.sort(key=lambda r:r['mono_ns'])
    (path/'summary.json').write_text(json.dumps(dict(axes='1,2',attempt=1,tolerance_deg=.5,success=complete,use_ack=False)))
    (path/'runner.jsonl').write_text('\n'.join(map(json.dumps,run)))
    (path/'can-0.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    return plan, rows


def test_exact_statistics_percentiles_and_worst_time():
    values=[dict(leg='out',axis=1,elapsed_s=t,nominal_error_deg=e) for t,e in enumerate((.1,-.2,.6))]
    metric=error_metrics(values)
    assert metric['rmse_deg']==pytest.approx(math.sqrt(.41/3))
    assert metric['mae_deg']==pytest.approx(.3)
    assert metric['p95_error_deg']==pytest.approx(.56)
    assert metric['max_error_time_sec']==2 and metric['max_abs_error_deg']==.6
    assert metric['within_0_5_deg_pct']==pytest.approx(200/3)
    assert metric['within_0_1_deg_pct']==pytest.approx(100/3)
    assert percentile([], .95) is None and percentile([7], .99)==7


def test_general_hermite_uses_recorded_velocities_not_zero_end_quintic():
    points=[dict(seconds=t,positions=[q]*6,velocities=[1]*6,accelerations=[0]*6) for t,q in ((0,0),(1,1),(2,2))]
    plan=dict(trajectory=dict(points=points),trajectory_units='rad')
    for t in (0,.1,.5,1,1.2,2,3):
        assert nominal(plan,1,t)==pytest.approx(math.degrees(min(t,2)))
    points[1]['seconds']=0
    with pytest.raises(ValueError):nominal(plan,1,.5)


def test_desync_aligns_asynchronous_measurements_and_excludes_stationary_axes():
    plan=dict(start_deg=[0]*6,target_deg=[10,10,0,0,0,0])
    samples=[dict(axis=a,elapsed_s=t,measured_deg=q) for a,t,q in ((1,0,0),(1,1,10),(2,.1,1),(2,1.1,11),(3,0,0),(3,1,0))]
    metric,per_axis=desynchronization(samples,[1,2,3],plan)
    assert metric['max_inter_axis_desync_pct']==pytest.approx(0)
    assert per_axis[3]['max_inter_axis_desync_pct'] is None
    for row in samples:
        if row['axis']==2:row['measured_deg']-=1
    metric,_=desynchronization(samples,[1,2,3],plan)
    assert metric['max_inter_axis_desync_pct']==pytest.approx(10)
    assert metric['max_inter_axis_desync_deg']==pytest.approx(1)


def test_report_splits_schedule_and_tracking_and_preserves_inputs(tmp_path):
    directory=tmp_path/'source';capture(directory,skipped=(10,),quiet=True)
    original={p.name:p.read_bytes() for p in directory.iterdir()}
    report=analyze(directory,tmp_path/'report')
    schedule=report['schedule_accuracy'];accuracy=report['trajectory_accuracy']
    assert schedule['skipped_frames_count']==1
    assert schedule['tx_period_max_ms']==pytest.approx(20)
    assert schedule['tx_period_mean_ms']==pytest.approx(200/19)
    assert schedule['batch_tx_skew_max_ms']==pytest.approx(.2)
    assert schedule['rx_feedback_period_mean_ms']==pytest.approx(10)
    assert schedule['quiet_window_violations']==1
    assert schedule['actual_settling_time_sec'] is not None
    assert accuracy['overall']['rmse_deg']<.03
    assert accuracy['overall']['final_error_deg']<.03
    assert 'within_half_degree_percent' in report['nominal']  # Zgodność dawnych konsumentów.
    assert (tmp_path/'report/tracking.md').is_file()
    assert original=={p.name:p.read_bytes() for p in directory.iterdir()}


@pytest.mark.parametrize('missing,complete',[((),False),((2,),True)])
def test_incomplete_feedback_or_aborted_run_never_claims_settling(tmp_path,missing,complete):
    directory=tmp_path/'trial';capture(directory,complete=complete,missing=missing)
    report=analyze(directory)
    assert report['schedule_accuracy']['actual_settling_time_sec'] is None
    assert report['schedule_accuracy']['settling_delay_ms'] is None
    assert report['trajectory_accuracy']['overall']['final_error_deg'] is None
    if missing:assert report['trajectory_accuracy']['per_axis'][2] is None


def test_signed_encoder_ratios_directions_offsets_and_crc_rejection(tmp_path):
    profiles=[dict(gear_ratio=3,direction=-1,offset_rad=-.2) for _ in range(6)]
    directory=tmp_path/'trial';_,rows=capture(directory,profiles=profiles)
    broken=next(r for r in rows if r['event']=='can_rx')
    data=bytearray.fromhex(broken['data']);data[-1]^=1;broken['data']=data.hex()
    (directory/'can-0.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    report=analyze(directory)
    assert report['invalid_position_frames']==1
    assert report['trajectory_accuracy']['overall']['max_abs_error_deg']<.03
    assert report['schedule_accuracy']['quiet_window_violations']==0


def test_settling_requires_two_measurements_and_one_hundred_ms_per_axis():
    plan=dict(target_deg=1,start_deg=[0]*6)
    def rows(times):return [dict(axis=a,elapsed_s=t,measured_deg=1) for t in times for a in (1,2)]
    assert settling_time(rows((.2,)),[1,2],plan) is None
    assert settling_time(rows((.2,.25)),[1,2],plan) is None
    assert settling_time(rows((.2,.25,.31)),[1,2],plan)==pytest.approx(.2)
    data=rows((.2,.25,.31));data[-1]['measured_deg']=2
    assert settling_time(data,[1,2],plan) is None


@pytest.mark.parametrize('direction',[1,-1])
def test_overshoot_and_final_error_are_relative_to_direction(direction):
    plan=dict(duration=1,start_deg=[0]*6,target_deg=direction)
    samples=[dict(leg='leg',axis=1,elapsed_s=t,measured_deg=direction*q,nominal_error_deg=0)
             for t,q in ((0,0),(1,1.1),(1.11,1.02),(1.22,1))]
    report=trajectory_report(samples,[1],plan,True)
    assert report['overall']['overshoot_deg']==pytest.approx(.1)
    assert report['per_axis'][1]['final_error_deg']==0
    assert report['overall']['max_inter_axis_desync_pct'] is None


def test_two_legs_do_not_count_pause_as_tx_or_rx_jitter(tmp_path):
    first=tmp_path/'first';_,a=capture(first)
    second=tmp_path/'second';_,b=capture(second,start=1,goal=0)
    first_run=[json.loads(x) for x in (first/'runner.jsonl').read_text().splitlines()]
    second_run=[json.loads(x) for x in (second/'runner.jsonl').read_text().splitlines()]
    for r in second_run:
        if 'mono_ns' in r:r['mono_ns']+=500_000_000
        if 'label' in r:r['label']='return'
        if 'detail' in r:r['detail']=r['detail'].replace('plus ','return ')
    for r in b:
        r['mono_ns']+=500_000_000
        if 'request_ns' in r:r['request_ns']+=500_000_000
    (first/'runner.jsonl').write_text('\n'.join(map(json.dumps,first_run+second_run)))
    (first/'can-0.jsonl').write_text('\n'.join(map(json.dumps,a+b)))
    report=analyze(first)
    assert len(report['legs'])==2
    assert report['schedule_accuracy']['planned_duration_sec']==pytest.approx(.4)
    assert report['schedule_accuracy']['tx_period_max_ms']==pytest.approx(10)
    assert report['schedule_accuracy']['rx_feedback_period_max_ms']==pytest.approx(10)
    assert report['schedule_accuracy']['skipped_frames_count']==0


def test_rotated_logs_preserve_reply_pairing_and_missing_feedback_is_unknown(tmp_path):
    source=tmp_path/'source';_,rows=capture(source)
    cut=next(i for i,r in enumerate(rows) if r['event']=='can_rx')+1
    (source/'can-0.jsonl').write_text('\n'.join(map(json.dumps,rows[:cut])))
    (source/'can-1.jsonl').write_text('\n'.join(map(json.dumps,rows[cut:])))
    report=analyze(source)
    assert report['trajectory_accuracy']['overall']['samples']==72
    (source/'can-1.jsonl').unlink()
    (source/'can-0.jsonl').write_text('\n'.join(json.dumps(r) for r in rows if r['event']=='can_tx'))
    report=analyze(source)
    assert report['trajectory_accuracy']['overall'] is None
    assert report['schedule_accuracy']['actual_settling_time_sec'] is None
