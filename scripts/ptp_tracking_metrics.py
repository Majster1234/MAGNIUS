"""Metryki offline: czasy z trace użytkownika, bez deklaracji pomiaru na przewodzie."""
import bisect
import math
import statistics


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * fraction
    left = int(index)
    return values[left] + (values[min(left + 1, len(values) - 1)] - values[left]) * (index - left)


def target(plan, axis):
    value = plan['target_deg']
    return float(value[axis - 1] if isinstance(value, list) else value)


def nominal(plan, axis, seconds):
    """Jawne q/v/a ROS w radianach lub dawny quintic z zerowymi v/a końców."""
    trajectory = plan.get('trajectory')
    points = trajectory.get('points') if isinstance(trajectory, dict) else trajectory
    if not points:
        u = max(0., min(1., seconds / plan['duration']))
        start = plan['start_deg'][axis - 1]
        return start + (target(plan, axis) - start) * (10*u**3 - 15*u**4 + 6*u**5)
    def time(point):
        if 'seconds' in point:
            return point['seconds']
        stamp = point['time_from_start']
        return stamp['sec'] + stamp['nanosec'] / 1e9
    times = [time(point) for point in points]
    if len(times) < 2 or times[0] != 0 or any(not math.isfinite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Nieprawidłowy czas zapisanej trajektorii Hermite’a')
    for point in points:
        if any(len(point.get(key, [])) != 6 or any(not math.isfinite(v) for v in point[key])
               for key in ('positions', 'velocities', 'accelerations')):
            raise ValueError('Trajektoria wymaga sześciu skończonych q/v/a')
    j = axis - 1
    if seconds <= 0:
        value = points[0]['positions'][j]
    elif seconds >= times[-1]:
        value = points[-1]['positions'][j]
    else:
        right = bisect.bisect_right(times, seconds)
        a, b = points[right - 1], points[right]
        dt = times[right] - times[right - 1]
        u = (seconds - times[right - 1]) / dt
        q, v, acc = a['positions'][j], a['velocities'][j]*dt, a['accelerations'][j]*dt*dt/2
        d = b['positions'][j] - q
        w, beta = b['velocities'][j]*dt, b['accelerations'][j]*dt*dt
        c3, c4, c5 = 10*d-6*v-3*acc-4*w+beta/2, -15*d+8*v+3*acc+7*w-beta, 6*d-3*v-acc-3*w+beta/2
        value = q + u*(v+u*(acc+u*(c3+u*(c4+u*c5))))
    units = plan.get('trajectory_units', 'rad')
    if units not in ('rad', 'deg'):
        raise ValueError('Nieznane jednostki trajektorii')
    return math.degrees(value) if units == 'rad' else value


def error_metrics(samples):
    if not samples:
        return None
    errors = [s['nominal_error_deg'] for s in samples]
    absolute = [abs(e) for e in errors]
    worst = max(samples, key=lambda s: abs(s['nominal_error_deg']))
    return dict(samples=len(errors), rmse_deg=math.sqrt(statistics.mean(e*e for e in errors)),
                mae_deg=statistics.mean(absolute), p95_error_deg=percentile(absolute, .95),
                max_abs_error_deg=max(absolute), max_error_time_sec=worst['elapsed_s'], max_error_leg=worst['leg'],
                within_0_5_deg_pct=100*sum(e <= .5+1e-12 for e in absolute)/len(errors),
                within_0_1_deg_pct=100*sum(e <= .1+1e-12 for e in absolute)/len(errors))


def desynchronization(samples, axes, plan):
    moving = [a for a in axes if abs(target(plan, a)-plan['start_deg'][a-1]) > 1e-9]
    empty = dict(max_inter_axis_desync_pct=None, max_inter_axis_desync_deg=None)
    per_axis = {a: dict(empty) for a in axes}
    values = {a: sorted((s['elapsed_s'], s['measured_deg']) for s in samples if s['axis'] == a) for a in moving}
    if len(moving) < 2 or any(len(values[a]) < 2 for a in moving):
        return dict(empty), per_axis
    begin, end = max(values[a][0][0] for a in moving), min(values[a][-1][0] for a in moving)
    grid = sorted({t for a in moving for t, _ in values[a] if begin <= t <= end})
    if not grid:
        return dict(empty), per_axis
    stamps = {a: [t for t, _ in values[a]] for a in moving}
    peak = {a: 0. for a in moving}
    for t in grid:
        progress = {}
        for a in moving:
            right = min(len(values[a])-1, max(1, bisect.bisect_left(stamps[a], t)))
            (t0, q0), (t1, q1) = values[a][right-1:right+1]
            q = q0+(q1-q0)*(t-t0)/(t1-t0) if t1 > t0 else q1
            progress[a] = (q-plan['start_deg'][a-1])/(target(plan, a)-plan['start_deg'][a-1])
        lo, hi = min(progress.values()), max(progress.values())
        for a in moving:
            peak[a] = max(peak[a], progress[a]-lo, hi-progress[a])
    for a in moving:
        per_axis[a] = dict(max_inter_axis_desync_pct=peak[a]*100,
                           max_inter_axis_desync_deg=peak[a]*abs(target(plan, a)-plan['start_deg'][a-1]))
    largest = max(peak.values())
    return dict(max_inter_axis_desync_pct=largest*100,
                max_inter_axis_desync_deg=largest*max(abs(target(plan, a)-plan['start_deg'][a-1]) for a in moving)), per_axis


def trajectory_report(samples, axes, plan, completed):
    sync, individual = desynchronization(samples, axes, plan)
    settled = settling_time(samples, axes, plan)
    per_axis = {}
    for a in axes:
        rows = sorted((s for s in samples if s['axis'] == a), key=lambda s: s['elapsed_s'])
        metric = error_metrics(rows)
        if metric is None:
            per_axis[a] = None
            continue
        delta = target(plan, a)-plan['start_deg'][a-1]
        sign = (delta > 0)-(delta < 0)
        metric.update(individual[a])
        metric['final_error_deg'] = rows[-1]['measured_deg']-target(plan, a) if completed and settled is not None and rows[-1]['elapsed_s'] >= plan['duration'] else None
        metric['overshoot_deg'] = max(0., max(sign*(s['measured_deg']-target(plan, a)) for s in rows)) if sign else None
        per_axis[a] = metric
    overall = error_metrics(samples)
    if overall:
        overall.update(sync)
        finals = [m['final_error_deg'] for m in per_axis.values() if m]
        overshoots = [m['overshoot_deg'] for m in per_axis.values() if m and m['overshoot_deg'] is not None]
        overall['final_error_deg'] = max(abs(e) for e in finals) if len(finals) == len(axes) and all(e is not None for e in finals) else None
        overall['overshoot_deg'] = max(overshoots) if overshoots else None
    return dict(per_axis=per_axis, overall=overall)


def settling_time(samples, axes, plan):
    """Początek końcowego pasma, potwierdzonego przez >=100 ms i >=2 próbki każdej osi."""
    latest, entered, counts = {}, None, {a: [] for a in axes}
    for row in sorted(samples, key=lambda s: s['elapsed_s']):
        a, t = row['axis'], row['elapsed_s']
        latest[a] = (row['measured_deg'], t)
        if len(latest) < len(axes) or any(abs(latest[a][0]-target(plan, a)) > .5+1e-12 for a in axes):
            entered = None
            counts = {a: [] for a in axes}
        else:
            if entered is None:
                entered = t
                counts = {axis: [latest[axis][1]] for axis in axes}
            else:
                counts[a].append(t)
    if entered is None or any(len(v) < 2 or v[-1]-v[0] < .1-1e-9 for v in counts.values()):
        return None
    return entered


def schedule_report(rows, samples, axes, origin, finish, plan, continuous):
    tx = [r for r in rows if r['event']=='can_tx' and r['command']==0xf5 and r['axis'] in axes and origin <= r['mono_ns'] <= finish]
    period = plan.get('sample_ms', 10 if continuous else 20)/1000
    batches = []
    for row in tx:
        if not batches or row['axis'] in {v['axis'] for v in batches[-1]} or row['mono_ns']-batches[-1][0]['mono_ns'] >= period*1e9:
            batches.append([])
        batches[-1].append(row)
    if continuous:
        marks = [b[0]['mono_ns'] for b in batches]
        skews = [(b[-1]['mono_ns']-b[0]['mono_ns'])/1e6 for b in batches if {r['axis'] for r in b} == set(axes)]
    else:
        marks = [r['mono_ns'] for r in rows if r['event']=='can_tx' and r['command']==0x4b and r['axis']==0 and origin <= r['mono_ns'] <= finish]
        skews = []  # F5 są stagingiem przed SYNC, a nie strumieniem wykonywanym co 10 ms.
    periods = [(b-a)/1e6 for a, b in zip(marks, marks[1:])]
    rx = []
    for a in axes:
        times = sorted(s['mono_ns'] for s in samples if s['axis']==a)
        rx.extend((b-a)/1e6 for a, b in zip(times, times[1:]))
    quiet = None
    if continuous and marks:
        quiet = sum(1 for row in rows if row['event']=='can_tx' and row['axis'] in axes and row['command'] in (0x31,0x32,0x3a,0x3e,0x40,0xf1,0) and
                    marks[0] <= row['mono_ns'] < marks[-1] and (row['mono_ns']-marks[0]) % round(period*1e9) >= round((period-.004)*1e9))
    settled = settling_time(samples, axes, plan)
    stats = timing_statistics(periods, skews, rx)
    stats.update(skipped_frames_count=max(0, round((marks[-1]-marks[0])/(period*1e9))+1-len(marks)) if len(marks)>1 else 0,
                 incomplete_batches_count=sum({r['axis'] for r in b} != set(axes) for b in batches),
                 quiet_window_violations=quiet, planned_duration_sec=plan['duration'], actual_settling_time_sec=settled,
                 settling_delay_ms=None if settled is None else (settled-plan['duration'])*1000)
    return stats, (periods, skews, rx)


def timing_statistics(periods, skews, feedback):
    return dict(tx_period_mean_ms=statistics.mean(periods) if periods else None,
                tx_jitter_rms_ms=statistics.pstdev(periods) if periods else None,
                tx_period_p99_ms=percentile(periods, .99), tx_period_max_ms=max(periods) if periods else None,
                batch_tx_skew_mean_ms=statistics.mean(skews) if skews else None,
                batch_tx_skew_max_ms=max(skews) if skews else None,
                rx_feedback_period_mean_ms=statistics.mean(feedback) if feedback else None,
                rx_feedback_period_p95_ms=percentile(feedback, .95), rx_feedback_period_max_ms=max(feedback) if feedback else None)


def combine_trajectory(samples, axes, reports):
    per_axis = {}
    for a in axes:
        metric = error_metrics([s for s in samples if s['axis']==a])
        values = [r['per_axis'][a] for r in reports if r['per_axis'].get(a)]
        if metric and values:
            metric['final_error_deg'] = reports[-1]['per_axis'][a]['final_error_deg'] if reports[-1]['per_axis'].get(a) else None
            for key in ('overshoot_deg','max_inter_axis_desync_pct','max_inter_axis_desync_deg'):
                known = [m[key] for m in values if m[key] is not None]
                metric[key] = max(known) if known else None
        per_axis[a] = metric
    overall = error_metrics(samples)
    if overall:
        finals = [m['final_error_deg'] for m in per_axis.values() if m]
        overall['final_error_deg'] = max(abs(v) for v in finals) if len(finals)==len(axes) and all(v is not None for v in finals) else None
        for key in ('overshoot_deg','max_inter_axis_desync_pct','max_inter_axis_desync_deg'):
            values = [r['overall'][key] for r in reports if r['overall'] and r['overall'][key] is not None]
            overall[key] = max(values) if values else None
    return dict(per_axis=per_axis,overall=overall)


def markdown(result):
    def value(v): return '—' if v is None else f'{v:.6g}' if isinstance(v,(int,float)) else str(v)
    lines = [f"# Próba {result['attempt']} — osie {', '.join(map(str,result['per_axis']))}", '',
             '## trajectory_accuracy', '', '| Miara | overall | '+' | '.join('J'+str(a) for a in result['per_axis'])+' |',
             '| --- | ---: | '+' | '.join('---:' for _ in result['per_axis'])+' |']
    group = result['trajectory_accuracy']
    for key in ('rmse_deg','mae_deg','p95_error_deg','max_abs_error_deg','max_error_time_sec','within_0_5_deg_pct','within_0_1_deg_pct',
                'final_error_deg','overshoot_deg','max_inter_axis_desync_pct','max_inter_axis_desync_deg'):
        cells = [group['overall'],*group['per_axis'].values()]
        lines.append('| '+key+' | '+' | '.join(value(c.get(key)) if c else '—' for c in cells)+' |')
    lines += ['', '## schedule_accuracy', '', '| Miara | Wartość |', '| --- | ---: |']
    lines += [f'| {key} | {value(v)} |' for key,v in result['schedule_accuracy'].items() if key!='per_leg']
    lines += ['', 'Czasy TX/RX pochodzą z trace procesu, nie z analizatora przewodu CAN. Brak danych oznacza —.',
              'Synchronizacja: interpolacja pomiarów tylko we wspólnym przedziale, bez ekstrapolacji; skala stopni overall = Δs × największy zakres osi.',
              'Final error overall = maksymalny moduł uchybu ostatniego odcinka, tylko jeśli ukończony; czas maksimum jest względny do odcinka max_error_leg.',
              'Settling: początek ostatniego nieprzerwanego pasma ±0,5°, potwierdzony >=100 ms i >=2 pomiarami każdej osi; opóźnienie może być ujemne.',
              'Pominięte sloty są estymowane z rozpiętości czasu TX i liczby paczek; legacy ACK/SYNC nie ma oceny okna 4 ms.', '']
    return '\n'.join(lines)
