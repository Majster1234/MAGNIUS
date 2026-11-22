#!/usr/bin/env python3
"""Diagnoza pojedynczej osi (pierwotnie J4, 22.11.2026): utrata synchronizmu, zależność od kąta wału, trzymanie po ruchu.

Ruch: PTP tylko badanej osi z pozy startowej testu jakości (pozostałe osie ENABLE=1 w postoju). Bez zmian nastaw napędu
i konfiguracji. Każdy blok osi zaczyna się od zerowania MKS (jak test jakości). Kolejne osie z --axes porównują ten sam profil
(np. 4,6,4 = J4, oś odniesienia J6, powtórka J4).

Testy:
- map:   wolny obrót o MAP_RANGE_DEG przegubu tam i z powrotem przy MAP_RPM [RPM wału]; analiza uchybu (pomiar − referencja)
         i prędkości w funkcji kąta wału: amplitudy harmonicznych 1, 2, 4 i 50 na obrót wału (50 = okres elektryczny
         silnika 1,8°). Silna 1/obr → mimośród magnesu / kalibracja enkodera; 50/obr → komutacja.
- speed: odcinki stałej prędkości SPEED_RPM [RPM wału] przy przyspieszeniach SPEED_ACC [°/s² przegubu], ruch ±SPEED_RANGE_DEG
         w obu kierunkach. Wskaźniki na odcinku stałej prędkości: opóźnienie średnie, część zmienna, najmniejszy stosunek
         prędkości (okno 50 ms), poślizgi (wzrost uchybu > SLIP_SHAFT_DEG wału w 50 ms).
- multi: PTP wszystkich osi naraz (MULTI_DELTA), badana oś jedzie MULTI_RPM — czy utrata synchronizmu zależy od ruchu
         pozostałych osi (zasilanie, obciążenie magistrali), skoro w ruchu samej osi nie występuje.
- hold:  po każdym ruchu HOLD_S s obserwacji odczytów 31h (dryf i rozrzut położenia w postoju).

Nieudany ruch (np. nadzór ucieczki) jest wynikiem testu: zapis, odczekanie odzyskania, ponowne zerowanie, dalej.

    python3 scripts/axis_diagnosis.py --axes 4              # suchy przebieg (planowanie)
    python3 scripts/axis_diagnosis.py --axes 4,6,4 --execute
    python3 scripts/axis_diagnosis.py --analyse log/axis_diagnosis/<czas>
"""
import argparse, glob, json, os, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tcp_motion_quality as q   # noqa: E402

COUNTS_PER_REV = 16384            # 31h: wartość narastająca enkodera, 0x4000 na obrót wału
MAP_RPM = (20.0, 60.0)
MAP_RANGE_DEG = 60.0
SPEED_RPM = (60.0, 100.0, 140.0, 180.0, 220.0, 260.0, 300.0)
SPEED_ACC = (100.0, 400.0)
SPEED_RANGE_DEG = 100.0
MULTI_DELTA = (60.0, -20.0, 40.0, 100.0, 30.0, 100.0)   # test multi: PTP wszystkich osi (J4 i J6 dominujące)
MULTI_RPM = (180.0, 220.0, 260.0)
MULTI_REPEATS = 2
HOLD_S = 2.0
SLIP_SHAFT_DEG = 10.0
MAX_FAILURES = 4


def rpm_to_joint(rpm, gear):
    return rpm * 6.0 / gear


def plan_moves(axis, gear, tests):
    """Lista ruchów: (etykieta, cel osi [°], prędkość [°/s], przyspieszenie [°/s²], metadane)."""
    moves = []
    start = q.START_POSE[axis - 1]
    if 'map' in tests:
        for rpm in MAP_RPM:
            v = rpm_to_joint(rpm, gear)
            for sign, name in ((1, 'tam'), (0, 'powrót')):
                moves.append({'label': f'J{axis} map {rpm:g} RPM {name}', 'test': 'map', 'rpm': rpm, 'acc': 100.0,
                              'target_deg': start + sign * MAP_RANGE_DEG, 'v': v, 'a': 100.0})
    if 'speed' in tests:
        moves.append({'label': f'J{axis} dojazd −{SPEED_RANGE_DEG:g}°', 'test': 'setup', 'rpm': None, 'acc': 100.0,
                      'target_deg': start - SPEED_RANGE_DEG, 'v': 20.0, 'a': 100.0})
        for acc in SPEED_ACC:
            for rpm in SPEED_RPM:
                v = rpm_to_joint(rpm, gear)
                for sign in (1, -1):
                    moves.append({'label': f'J{axis} {rpm:g} RPM a{acc:g} {"+" if sign > 0 else "−"}', 'test': 'speed',
                                  'rpm': rpm, 'acc': acc, 'direction': sign,
                                  'target_deg': start + sign * SPEED_RANGE_DEG, 'v': v, 'a': acc})
        moves.append({'label': f'J{axis} powrót do startu', 'test': 'setup', 'rpm': None, 'acc': 100.0,
                      'target_deg': start, 'v': 20.0, 'a': 100.0})
    if 'multi' in tests:
        dominant = max(abs(x) for x in MULTI_DELTA); share = abs(MULTI_DELTA[axis - 1]) / dominant
        for repeat in range(MULTI_REPEATS):
            for acc in SPEED_ACC:
                for rpm in MULTI_RPM:
                    v = rpm_to_joint(rpm, gear) / share          # prędkość osi dominującej, by badana miała rpm
                    for sign in (1, 0):
                        vec = [a + sign * d for a, d in zip(q.START_POSE, MULTI_DELTA)]
                        moves.append({'label': f'J{axis} multi {rpm:g} RPM a{acc:g} {"tam" if sign else "powrót"} #{repeat + 1}',
                                      'test': 'multi', 'rpm': rpm, 'acc': acc, 'target_deg': vec[axis - 1], 'target_vec': vec,
                                      'v': v, 'a': acc})
    return moves


def target_vector(axis, deg):
    t = list(q.START_POSE); t[axis - 1] = deg
    return t


def setup(robot, log):
    robot.ensure_ready()
    robot.enable_all(False)
    robot.zero_and_confirm()
    robot.enable_all(True)
    for attempt in range(3):
        try:
            q.run_move(robot, 'PTP', q.START_POSE, {'preview_velocity_degrees_s': q.SETUP_VELOCITY,
                       'preview_acceleration_degrees_s2': q.SETUP_ACCELERATION}, 'PTP do pozy startowej')
            return
        except q.MoveFailed as error:
            log('setup_failed', attempt=attempt + 1, reason=str(error)[:300])
    raise q.Abort('Nie osiągnięto pozy startowej')


def wait_recovered(robot):
    q.wait(lambda: (lambda m: not m['fault'] and not m['busy'] and m.get('recovery_state') in ('idle', 'idle_disabled'))(
        robot.manual()), 60, 'Brak powrotu do gotowości po awarii ruchu')


# ---------- analiza ----------
def load_trace(t0_ns, t1_ns, axes):
    """Jeden przebieg po nagraniu: sync_feedback (ref, pomiar [rad przegubu]) i odczyty 31h (impulsy wału) badanych osi."""
    fb = {a: [] for a in axes}; enc = {a: [] for a in axes}
    for path in sorted(glob.glob(str(q.CAN_LOG / 'can-*.jsonl')), key=os.path.getmtime):
        if os.path.getmtime(path) * 1e9 < t0_ns:
            continue
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                fbl = '"sync_feedback"' in line
                if not fbl and '"can_rx"' not in line:
                    continue
                r = json.loads(line)
                if r.get('axis') not in fb or not t0_ns <= r['utc_ns'] <= t1_ns:
                    continue
                if fbl:
                    d = dict(kv.split('=') for kv in r['detail'].split())
                    fb[r['axis']].append((r['request_ns'] + (r['utc_ns'] - r['mono_ns']), float(d['q_RX']), float(d['q_measured'])))
                else:
                    data = r.get('data') or ''
                    if data.startswith('31') and len(data) >= 14:
                        v = int(data[2:14], 16)
                        enc[r['axis']].append((r['utc_ns'], v - (1 << 48) if v & (1 << 47) else v))
    return {a: np.array(sorted(v)) if v else np.zeros((0, 3)) for a, v in fb.items()}, \
           {a: np.array(sorted(v), dtype=np.int64) if v else np.zeros((0, 2), dtype=np.int64) for a, v in enc.items()}


def harmonics(theta_deg, err_shaft_deg, orders=(1, 2, 4, 50)):
    th = np.radians(theta_deg); e = err_shaft_deg - np.mean(err_shaft_deg)
    out = {}
    for k in orders:
        c, s = np.cos(k * th), np.sin(k * th)
        a = np.linalg.lstsq(np.stack([c, s], 1), e, rcond=None)[0]
        out[str(k)] = round(float(np.hypot(*a)), 3)
    return out


def analyse_move(move, fb, enc, gear):
    t0, t1, t2 = move['result']['t_exec_ns'], move['result']['t_done_ns'], move['result']['t_hold_end_ns']
    f = fb[(fb[:, 0] >= t0 - 5e7) & (fb[:, 0] <= t1)] if len(fb) else fb
    if len(f) < 20:
        return {'valid': False, 'reason': f'za mało sync_feedback ({len(f)}) — nagranie nadpisane (bufor ~18 min)?'}
    _, keep = np.unique(f[:, 0], return_index=True); f = f[keep]
    grid = np.arange(f[0, 0], f[-1, 0], 1e7)                                  # siatka 10 ms
    t = (grid - grid[0]) / 1e9
    ref = np.degrees(np.interp(grid, f[:, 0], f[:, 1])) * gear                # ° wału
    meas = np.degrees(np.interp(grid, f[:, 0], f[:, 2])) * gear
    out = {'valid': True}
    err = meas - ref
    v_ref = np.gradient(ref, t); v_meas = np.gradient(meas, t)
    peak = float(np.max(np.abs(v_ref)))
    plateau = np.abs(v_ref) >= 0.95 * peak
    moving = np.abs(v_ref) >= 0.1 * peak
    # prędkość z okna 50 ms (5 próbek) — stosunek pomiar/referencja na odcinku stałej prędkości
    w = 5
    if len(t) > w:
        vm = (meas[w:] - meas[:-w]) / (t[w:] - t[:-w]); vr = (ref[w:] - ref[:-w]) / (t[w:] - t[:-w])
        pl = plateau[w:] & plateau[:-w]
        ratio = vm[pl] / vr[pl] if pl.any() else np.array([1.0])
        derr = (err[w:] - err[:-w]) * (moving[w:] | moving[:-w])
        slips = int(np.sum(np.diff(np.concatenate([[0], (np.abs(derr) > SLIP_SHAFT_DEG).astype(int)])) == 1))
        worst_jump = float(np.max(np.abs(derr)))
    else:
        ratio, slips, worst_jump = np.array([1.0]), 0, 0.0
    out.update({'peak_rpm': round(peak / 6.0, 1), 'err_max_shaft_deg': round(float(np.max(np.abs(err))), 2),
                'plateau_lag_mean_shaft_deg': round(float(np.mean(err[plateau]) * np.sign(np.mean(v_ref[plateau]))), 2) if plateau.any() else None,
                'plateau_err_std_shaft_deg': round(float(np.std(err[plateau])), 3) if plateau.any() else None,
                'plateau_speed_ratio_min': round(float(np.min(ratio)), 3), 'plateau_speed_ratio_mean': round(float(np.mean(ratio)), 3),
                'slips': slips, 'err_jump_50ms_max_shaft_deg': round(worst_jump, 2)})
    if move['test'] == 'map' and plateau.sum() > 50:
        out['harmonics_err_shaft_deg'] = harmonics(meas[plateau], err[plateau])
        out['harmonics_speed_pct'] = {k: round(v / (peak) * 100, 2) for k, v in
                                      harmonics(meas[plateau], v_meas[plateau]).items()}
    # trzymanie: odczyty 31h od końca programu do końca obserwacji
    e = enc[(enc[:, 0] >= t1) & (enc[:, 0] <= t2)] if len(enc) else enc
    if len(e) >= 5:
        c = e[:, 1].astype(float)
        out.update({'hold_samples': int(len(e)), 'hold_drift_counts': int(c[-1] - c[0]),
                    'hold_span_counts': int(c.max() - c.min()), 'hold_span_shaft_deg': round((c.max() - c.min()) / COUNTS_PER_REV * 360, 2)})
    return out


def summarize(data):
    lines = ['| Blok | Ruch | Szczyt [RPM] | Opóźnienie śr. [° wału] | Część zmienna [° wału] | Min. v pomiar/ref | Poślizgi | '
             'Skok uchybu 50 ms maks. [° wału] | Uchyb maks. [° wału] | Trzymanie: rozrzut [° wału] | Harmoniczne uchybu 1/2/4/50 [° wału] |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for block in data['blocks']:
        for m in block['moves']:
            x = m.get('metrics') or {}
            if m['test'] == 'setup' and not m.get('error'):
                continue
            if m.get('error'):
                lines.append(f"| {block['label']} | {m['label']} | **NIEUDANY:** {m['error'][:150]} | | | | | | | | |"); continue
            if not x.get('valid'):
                lines.append(f"| {block['label']} | {m['label']} | {x.get('reason')} | | | | | | | | |"); continue
            h = x.get('harmonics_err_shaft_deg')
            lines.append(f"| {block['label']} | {m['label']} | {x['peak_rpm']} | {x['plateau_lag_mean_shaft_deg']} | {x['plateau_err_std_shaft_deg']} | "
                         f"{x['plateau_speed_ratio_min']} | {x['slips']} | {x['err_jump_50ms_max_shaft_deg']} | {x['err_max_shaft_deg']} | "
                         f"{x.get('hold_span_shaft_deg', '—')} | {' / '.join(str(h[k]) for k in ('1', '2', '4', '50')) if h else ''} |")
    return '\n'.join(lines)


def analyse_dir(out, gears):
    data = json.loads((out / 'results.json').read_text())
    for k, block in enumerate(data['blocks']):
        axis = block['axis']
        done = [m for m in block['moves'] if m.get('result')]
        if not done:
            continue
        trace = out / f'trace-block{k + 1}-J{axis}.npz'
        if trace.exists():   # ślad zapisany po bloku (nagranie CAN to bufor kołowy ~18 min)
            z = np.load(trace); fb, enc = {axis: z['fb']}, {axis: z['enc']}
        else:
            fb, enc = load_trace(min(m['result']['t_exec_ns'] for m in done) - 10**8,
                                 max(m['result']['t_hold_end_ns'] for m in done) + 10**8, [axis])
        for m in done:
            m['metrics'] = analyse_move(m, fb[axis], enc[axis], gears[axis - 1])
    (out / 'results.json').write_text(json.dumps(data, ensure_ascii=False, indent=1))
    text = summarize(data)
    (out / 'summary.md').write_text(text + '\n')
    print(text + f'\n\nWyniki: {out}')


def main():
    global MAP_RPM, SPEED_RPM, SPEED_ACC
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--axes', default='4', help='osie po kolei, np. 4,6,4')
    p.add_argument('--tests', default='map,speed', help='map, speed (trzymanie liczone po każdym ruchu)')
    p.add_argument('--speeds', default=None, help=f'RPM wału testu speed (domyślnie {",".join(f"{x:g}" for x in SPEED_RPM)})')
    p.add_argument('--accs', default=None, help=f'przyspieszenia testu speed [°/s²] (domyślnie {",".join(f"{x:g}" for x in SPEED_ACC)})')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--analyse', default=None, metavar='KATALOG')
    p.add_argument('--output', default=None)
    args = p.parse_args()
    if args.speeds:
        SPEED_RPM = tuple(float(x) for x in args.speeds.split(','))
    if args.accs:
        SPEED_ACC = tuple(float(x) for x in args.accs.split(','))
    out = Path(args.analyse or args.output or q.ROOT / 'log/axis_diagnosis' / datetime.now().strftime('%Y%m%d-%H%M%S'))
    log = q.Log(out); robot = q.Robot(log)
    values = robot.configuration()['active']['document']['values']
    gears = [float(values[f'joint.{j}.gear_ratio']) for j in range(1, 7)]
    if args.analyse:
        analyse_dir(out, gears); return
    if values['motion.ptp_lin_dispatch_mode'] != 'synchronized':
        raise q.Abort('Analiza wymaga trybu PTP/LIN synchronized')
    axes = [int(x) for x in args.axes.split(',')]
    tests = {x.strip() for x in args.tests.split(',')}
    blocks = []
    for k, axis in enumerate(axes):
        lower, upper = values[f'joint.{axis}.lower'], values[f'joint.{axis}.upper']
        moves = plan_moves(axis, gears[axis - 1], tests)
        for m in moves:
            if not lower + 5 <= m['target_deg'] <= upper - 5 or m['v'] > values[f'joint.{axis}.velocity'] or m['a'] > values[f'joint.{axis}.acceleration']:
                raise q.Abort(f"{m['label']}: poza zakresem/limitem osi J{axis}")
        blocks.append({'label': f'{k + 1}: J{axis}', 'axis': axis, 'moves': moves})
    # suchy plan: każdy ruch z pozycji końcowej poprzedniego (symulowany start)
    ctx = robot.context()
    for block in blocks:
        prev = list(q.START_POSE)
        for m in block['moves']:
            target = m.get('target_vec') or target_vector(block['axis'], m['target_deg'])
            robot.plan(ctx, prev, 'PTP', target, {'preview_velocity_degrees_s': m['v'], 'preview_acceleration_degrees_s2': m['a']}, True)
            prev = target
    log('plan_ok', blocks=[{'label': b['label'], 'moves': len(b['moves'])} for b in blocks])
    if not args.execute:
        return
    try:
        for block in blocks:
            log('block_start', label=block['label'])
            setup(robot, log)
            failures = 0
            for m in block['moves']:
                params = {'preview_velocity_degrees_s': m['v'], 'preview_acceleration_degrees_s2': m['a']}
                try:
                    result = q.run_move(robot, 'PTP', m.get('target_vec') or target_vector(block['axis'], m['target_deg']), params, m['label'])
                    time.sleep(HOLD_S)
                    result['t_hold_end_ns'] = time.time_ns()
                    m['result'] = result
                    log('move', label=m['label'])
                except q.Abort as error:   # MoveFailed albo awaria (ucieczka): wynik testu
                    failures += 1
                    m['error'] = str(error)[:400]
                    log('move_failed', label=m['label'], reason=m['error'])
                    try:
                        q.api('/api/v1/motion/stop', {})
                    except q.Abort:
                        pass
                    if failures >= MAX_FAILURES:
                        log('block_stopped', label=block['label']); break
                    wait_recovered(robot)
                    setup(robot, log)   # ponowne zerowanie: dalsze ruchy od pozy startowej
                    # po zerowaniu oś jest w pozie startowej; następny ruch speed jedzie przeciwnie — dojazd do celu nieudanego ruchu
                    if m['test'] == 'speed':
                        q.run_move(robot, 'PTP', target_vector(block['axis'], q.START_POSE[block['axis'] - 1] + SPEED_RANGE_DEG * m.get('direction', 1)),
                                   {'preview_velocity_degrees_s': 20.0, 'preview_acceleration_degrees_s2': 100.0}, 'dojazd po awarii')
            robot.enable_all(False)
            done = [m for m in block['moves'] if m.get('result')]
            if done:   # zaraz po bloku: wycięcie śladu osi, zanim bufor nagrania go nadpisze
                fb, enc = load_trace(min(m['result']['t_exec_ns'] for m in done) - 10**8,
                                     max(m['result']['t_hold_end_ns'] for m in done) + 10**8, [block['axis']])
                np.savez_compressed(out / f"trace-block{blocks.index(block) + 1}-J{block['axis']}.npz",
                                    fb=fb[block['axis']], enc=enc[block['axis']])
                log('trace_saved', label=block['label'], feedback=len(fb[block['axis']]), encoder=len(enc[block['axis']]))
    except (q.Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        (out / 'results.json').write_text(json.dumps({'blocks': blocks, 'gears': gears}, ensure_ascii=False, indent=1, default=str))
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
