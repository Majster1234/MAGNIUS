#!/usr/bin/env python3
"""Dobór wyprzedzenia celu F5 (lookahead) osobno dla każdej osi (22.11.2026).

Ruch: PTP tylko badanej osi (pozostałe ENABLE=1 w postoju) tam i z powrotem wokół środka osi, przy prędkościach --speeds
[RPM wału] i wartościach wyprzedzenia --leads [ms]. Na czas testu: nastawy wykonania testowe (poza konfiguracją) —
`lookahead_time_sec` = badana wartość, `gain_schedule` = false (inaczej szybki PTP dostaje 50 ms z harmonogramu), reszta z pliku.
Kolejność przeplatana: dla każdej prędkości wszystkie wyprzedzenia po kolei (dryf w czasie nie faworyzuje żadnej wartości).
Każdy blok osi zaczyna się od zerowania MKS; nieudany ruch = wynik (zapis, odzyskanie, ponowne zerowanie, dalej).
Po teście (także po przerwaniu) nastawy wracają do pliku.

Wskaźniki [° wału] z sync_feedback i odczytów 0x31: uchyb maks. w ruchu, opóźnienie i część zmienna na odcinku stałej prędkości,
uchyb na końcu trajektorii, przestrzał po zatrzymaniu (względem położenia spoczynku), czas dolotu (wykonanie − plan), poślizgi.
Ocena: uchyb maks. + 2·część zmienna + przestrzał (awaria = najgorsza); najlepsze wyprzedzenie dla osi i prędkości.

    python3 scripts/axis_lookahead.py --axes 1,2,3,4,5,6                 # suchy przebieg (planowanie)
    python3 scripts/axis_lookahead.py --axes 1,2,3,4,5,6 --execute
    python3 scripts/axis_lookahead.py --analyse log/axis_lookahead/<czas>
"""
import argparse, json, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tcp_motion_quality as q          # noqa: E402
import axis_diagnosis as d              # noqa: E402  (setup, load_trace, wait_recovered)

LEADS_MS = (10.0, 25.0, 40.0, 60.0, 80.0)
SPEEDS_RPM = (5.0, 20.0, 60.0, 120.0, 180.0, 240.0)
ACCELERATION = 200.0                    # °/s² przegubu
CENTER_DEG = {1: 0.0, 2: -60.0, 3: -60.0, 4: 0.0, 5: 45.0, 6: 0.0}   # środek ruchu osi (zapas do granic)
MAX_STROKE_DEG = 70.0
HOLD_S = 1.0
SLIP_SHAFT_DEG = 10.0
MAX_FAILURES = 6


def stroke(v):
    """Skok ruchu [° przegubu]: ~1,5 s stałej prędkości + rozpędzanie i hamowanie, 6–70°."""
    return float(np.clip(v * 1.5 + v * v / ACCELERATION + 2.0, 6.0, MAX_STROKE_DEG))


def vector(axis, deg):
    t = list(q.START_POSE); t[axis - 1] = deg
    return t


def plan_block(axis, gear):
    c = CENTER_DEG[axis]; moves = []
    for rpm in SPEEDS_RPM:
        v = rpm * 6.0 / gear; s = stroke(v)
        moves.append({'label': f'J{axis} dojazd {rpm:g} RPM', 'kind': 'setup', 'target': c - s / 2, 'v': 20.0, 'a': 100.0})
        for lead in LEADS_MS:
            for sign in (1, -1):
                moves.append({'label': f'J{axis} {rpm:g} RPM L{lead:g} {"+" if sign > 0 else "−"}', 'kind': 'test', 'rpm': rpm,
                              'lead_ms': lead, 'direction': sign, 'target': c + sign * s / 2, 'v': v, 'a': ACCELERATION})
    moves.append({'label': f'J{axis} powrót', 'kind': 'setup', 'target': q.START_POSE[axis - 1], 'v': 20.0, 'a': 100.0})
    return moves


def set_lead(robot, lead_ms):
    robot.set_tuning({'lookahead_time_sec': lead_ms / 1000.0, 'gain_schedule': False})


# ---------- analiza ----------
def analyse_move(m, fb, enc, gear):
    r = m['result']; t0, t1, t2 = r['t_exec_ns'], r['t_done_ns'], r['t_hold_end_ns']
    f = fb[(fb[:, 0] >= t0 - 5e7) & (fb[:, 0] <= t1)] if len(fb) else fb
    if len(f) < 15:
        return {'valid': False, 'reason': f'za mało sync_feedback ({len(f)})'}
    _, keep = np.unique(f[:, 0], return_index=True); f = f[keep]
    grid = np.arange(f[0, 0], f[-1, 0], 1e7); t = (grid - grid[0]) / 1e9
    ref = np.degrees(np.interp(grid, f[:, 0], f[:, 1])) * gear
    meas = np.degrees(np.interp(grid, f[:, 0], f[:, 2])) * gear
    err = meas - ref; v_ref = np.gradient(ref, t)
    peak = float(np.max(np.abs(v_ref))) or 1e-9
    plateau = np.abs(v_ref) >= 0.95 * peak; moving = np.abs(v_ref) >= 0.05 * peak
    sgn = float(np.sign(np.mean(v_ref[moving]))) if moving.any() else 1.0
    w = 5
    slips = 0
    if len(err) > w:
        jump = np.abs(err[w:] - err[:-w]) * (plateau[w:] & plateau[:-w])
        slips = int(np.sum(np.diff(np.concatenate([[0], (jump > SLIP_SHAFT_DEG).astype(int)])) == 1))
    out = {'valid': True, 'peak_rpm': round(peak / 6.0, 1),
           'err_max': round(float(np.max(np.abs(err[moving]))) if moving.any() else float(np.max(np.abs(err))), 3),
           'lag_mean': round(float(-np.mean(err[plateau]) * sgn), 3) if plateau.any() else None,   # + = za referencją
           'err_std': round(float(np.std(err[plateau])), 3) if plateau.any() else None,
           'end_err': round(float(err[-1]), 3), 'slips': slips,
           'overrun_s': round((t1 - t0) / 1e9 - float(r.get('duration_s') or 0.0), 3)}
    e = enc[(enc[:, 0] >= t1 - 2e8) & (enc[:, 0] <= t2)] if len(enc) else enc
    if len(e) >= 5:
        shaft = e[:, 1].astype(float) / d.COUNTS_PER_REV * 360.0
        rest = float(np.median(shaft[-max(3, len(shaft) // 5):]))
        out['overshoot'] = round(max(0.0, float(np.max((shaft - rest) * sgn))), 3)
        out['hold_span'] = round(float(np.ptp(shaft[len(shaft) // 2:])), 3)
    return out


def score(x):
    if not x or not x.get('valid'):
        return float('inf')
    return x['err_max'] + 2 * (x.get('err_std') or 0) + x.get('overshoot', 0) + 5 * x.get('slips', 0)


def summarize(data):
    lines = []
    best_overall = {}
    for block in data['blocks']:
        axis = block['axis']; tests = [m for m in block['moves'] if m['kind'] == 'test']
        leads = sorted({m['lead_ms'] for m in tests}); speeds = sorted({m['rpm'] for m in tests})
        lines += ['', f"### J{axis} — ocena (uchyb maks. + 2·część zmienna + przestrzał [° wału], średnio z ruchu tam i z powrotem; "
                      f"✗ = awaria/niezaliczony)", '',
                  '| RPM wału | ' + ' | '.join(f'{l:g} ms' for l in leads) + ' | najlepsze |', '|---' * (len(leads) + 2) + '|']
        rel = {l: [] for l in leads}
        for rpm in speeds:
            row, scores = [], {}
            for l in leads:
                ms = [m for m in tests if m['rpm'] == rpm and m['lead_ms'] == l]
                if any(m.get('error') for m in ms) or not ms:
                    row.append('✗'); scores[l] = float('inf'); continue
                sc = [score(m.get('metrics')) for m in ms]; scores[l] = float(np.mean(sc))
                x = [m['metrics'] for m in ms]
                row.append(f"{scores[l]:.2f} (maks {np.mean([y['err_max'] for y in x]):.2f}, zm {np.mean([y.get('err_std') or 0 for y in x]):.2f}, "
                           f"prz {np.mean([y.get('overshoot', 0) for y in x]):.2f})")
            finite = {l: s for l, s in scores.items() if np.isfinite(s)}
            best = min(finite, key=finite.get) if finite else None
            for l in leads:   # ocena względna do najlepszej w tym wierszu (do wyboru jednej wartości dla osi)
                rel[l].append(scores[l] / finite[best] if best is not None and np.isfinite(scores[l]) and finite[best] > 0 else 3.0)
            lines.append(f'| {rpm:g} | ' + ' | '.join(row) + f" | **{best:g} ms** |" if best is not None else f'| {rpm:g} | ' + ' | '.join(row) + ' | — |')
        overall = {l: float(np.mean(v)) for l, v in rel.items() if v}
        if overall:
            b = min(overall, key=overall.get); best_overall[axis] = b
            lines.append('')
            lines.append(f"Jedna wartość dla J{axis} (średnia ocena względna, 1 = najlepsza w każdym wierszu): " +
                         ', '.join(f"{l:g} ms → {v:.2f}" for l, v in sorted(overall.items())) + f" — **{b:g} ms**")
        fails = [m for m in block['moves'] if m.get('error')]
        if fails:
            lines.append(''); lines += [f"- ✗ {m['label']}: {m['error'][:160]}" for m in fails]
    head = ['## Najlepsze wyprzedzenie celu dla osi', '', '| Oś | Jedna wartość |', '|---|---|'] + \
           [f'| J{a} | {b:g} ms |' for a, b in best_overall.items()]
    return '\n'.join(head + lines)


def analyse_dir(out, gears):
    data = json.loads((out / 'results.json').read_text())
    for k, block in enumerate(data['blocks']):
        axis = block['axis']; done = [m for m in block['moves'] if m.get('result')]
        if not done:
            continue
        trace = out / f'trace-J{axis}-{k + 1}.npz'
        if trace.exists():
            z = np.load(trace); fb, enc = z['fb'], z['enc']
        else:
            F, E = d.load_trace(min(m['result']['t_exec_ns'] for m in done) - 10**8,
                                max(m['result']['t_hold_end_ns'] for m in done) + 10**8, [axis]); fb, enc = F[axis], E[axis]
        for m in done:
            if m['kind'] == 'test':
                m['metrics'] = analyse_move(m, fb, enc, gears[axis - 1])
    (out / 'results.json').write_text(json.dumps(data, ensure_ascii=False, indent=1))
    text = summarize(data); (out / 'summary.md').write_text(text + '\n'); print(text + f'\n\nWyniki: {out}')


def main():
    global LEADS_MS, SPEEDS_RPM
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--axes', default='1,2,3,4,5,6')
    p.add_argument('--leads', default=None, help='wyprzedzenia [ms], domyślnie ' + ','.join(f'{x:g}' for x in LEADS_MS))
    p.add_argument('--speeds', default=None, help='RPM wału, domyślnie ' + ','.join(f'{x:g}' for x in SPEEDS_RPM))
    p.add_argument('--execute', action='store_true')
    p.add_argument('--analyse', default=None, metavar='KATALOG')
    p.add_argument('--output', default=None)
    args = p.parse_args()
    if args.leads:
        LEADS_MS = tuple(float(x) for x in args.leads.split(','))
    if args.speeds:
        SPEEDS_RPM = tuple(float(x) for x in args.speeds.split(','))
    out = Path(args.analyse or args.output or q.ROOT / 'log/axis_lookahead' / datetime.now().strftime('%Y%m%d-%H%M%S'))
    log = q.Log(out); robot = q.Robot(log)
    values = robot.configuration()['active']['document']['values']
    gears = [float(values[f'joint.{j}.gear_ratio']) for j in range(1, 7)]
    if args.analyse:
        analyse_dir(out, gears); return
    if values['motion.ptp_lin_dispatch_mode'] != 'synchronized':
        raise q.Abort('Analiza wymaga trybu PTP/LIN synchronized')
    if any(not 0 <= l <= 200 for l in LEADS_MS):
        raise q.Abort('Wyprzedzenie: 0–200 ms')
    blocks = []
    for axis in (int(x) for x in args.axes.split(',')):
        lower, upper = values[f'joint.{axis}.lower'], values[f'joint.{axis}.upper']
        moves = plan_block(axis, gears[axis - 1])
        for m in moves:
            if not lower + 5 <= m['target'] <= upper - 5 or m['v'] > values[f'joint.{axis}.velocity']:
                raise q.Abort(f"{m['label']}: poza zakresem/limitem osi J{axis}")
        blocks.append({'axis': axis, 'moves': moves})
    ctx = robot.context()
    for b in blocks:   # suchy plan: każdy ruch z celu poprzedniego
        prev = list(q.START_POSE)
        for m in b['moves']:
            tgt = vector(b['axis'], m['target'])
            robot.plan(ctx, prev, 'PTP', tgt, {'preview_velocity_degrees_s': m['v'], 'preview_acceleration_degrees_s2': m['a']}, True)
            prev = tgt
    log('plan_ok', blocks=[{'axis': b['axis'], 'moves': len(b['moves'])} for b in blocks],
        minutes_estimate=round(sum(len(b['moves']) for b in blocks) * 6 / 60))
    if not args.execute:
        return
    try:
        for b in blocks:
            axis = b['axis']; log('block_start', axis=axis)
            robot.set_tuning(None); d.setup(robot, log)
            failures, current = 0, None
            for i, m in enumerate(b['moves']):
                if m['kind'] == 'test' and m['lead_ms'] != current:
                    set_lead(robot, m['lead_ms']); current = m['lead_ms']
                params = {'preview_velocity_degrees_s': m['v'], 'preview_acceleration_degrees_s2': m['a']}
                try:
                    res = q.run_move(robot, 'PTP', vector(axis, m['target']), params, m['label'])
                    time.sleep(HOLD_S); res['t_hold_end_ns'] = time.time_ns(); m['result'] = res
                    log('move', label=m['label'])
                except q.Abort as error:
                    failures += 1; m['error'] = str(error)[:400]; log('move_failed', label=m['label'], reason=m['error'])
                    try:
                        q.api('/api/v1/motion/stop', {})
                    except q.Abort:
                        pass
                    if failures >= MAX_FAILURES:
                        log('block_stopped', axis=axis); break
                    d.wait_recovered(robot); robot.set_tuning(None); current = None
                    d.setup(robot, log)
                    # dalej od strony, z której wyruszy następny ruch (cel nieudanego ruchu)
                    q.run_move(robot, 'PTP', vector(axis, m['target']), {'preview_velocity_degrees_s': 20.0,
                               'preview_acceleration_degrees_s2': 100.0}, 'dojazd po awarii')
            robot.enable_all(False)
            done = [m for m in b['moves'] if m.get('result')]
            if done:   # ślad osi zaraz po bloku (nagranie CAN to bufor kołowy)
                F, E = d.load_trace(min(m['result']['t_exec_ns'] for m in done) - 10**8,
                                    max(m['result']['t_hold_end_ns'] for m in done) + 10**8, [axis])
                np.savez_compressed(out / f'trace-J{axis}-{blocks.index(b) + 1}.npz', fb=F[axis], enc=E[axis])
                log('trace_saved', axis=axis, feedback=len(F[axis]))
    except (q.Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        try:
            d.wait_recovered(robot); robot.set_tuning(None); log('tuning_reset')
        except q.Abort as error:
            log('tuning_reset_failed', reason=str(error)[:200])
            print('!!! Nastawy testowe mogą być aktywne: POST /api/v1/motion/execution-tuning {"reset": true}')
        (out / 'results.json').write_text(json.dumps({'blocks': blocks, 'gears': gears, 'leads_ms': LEADS_MS,
                                                      'speeds_rpm': SPEEDS_RPM}, ensure_ascii=False, indent=1, default=str))
        try:
            if not robot.manual()['fault']:
                robot.enable_all(False)
        except q.Abort:
            pass
    analyse_dir(out, gears)


if __name__ == '__main__':
    try:
        main()
    except q.Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
