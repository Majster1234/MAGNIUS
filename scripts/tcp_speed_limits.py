#!/usr/bin/env python3
"""Rampa prędkości PTP i LIN: najwyższa prędkość wykonywana bez awarii i dużych odchyłek.

Używa ruchów, kinematyki i analizy standardowego testu (scripts/tcp_motion_quality.py): ta sama poza startowa,
ten sam cel 400 mm. Każda rampa zaczyna się od zerowania MKS, a na każdym poziomie wykonuje --repeats przejazdów
tam i z powrotem. Skrypt nie zmienia konfiguracji; poziomy LIN powyżej tcp.max_linear_velocity odrzuca.
--tune powtarza te same poziomy dla kolejnych nastaw regulatora wykonania (Kp, wyprzedzenie), a po serii przywraca plik.

Bez --execute: tylko planowanie wszystkich poziomów. Ruch fizyczny wymaga zgody operatora (AGENTS.md).
"""
import argparse, json, time
from datetime import datetime

import numpy as np

import tcp_motion_quality as q

DISTANCE_MM = 400.0
PTP_LEVELS = (40, 60, 80, 100, 130, 160, 200, 230, 260)   # mm/s; 260 mm/s ≈ 80 °/s osi dominującej
LIN_LEVELS = (40, 50, 60, 70, 80, 90, 100)                # mm/s; limit tcp.max_linear_velocity
PTP_ACCELERATION = 100.0                                  # °/s², jak ruch przygotowawczy testu standardowego
ATTEMPTS = 2                                              # awaria na poziomie: jedno powtórzenie po zerowaniu
# (wskaźnik, próg „dobry”, próg „dopuszczalny”). Punkt odniesienia: test 17.11.2026, 40 mm/s, 400 mm:
# odchyłka 0,8–0,9 mm, uchyb osi 0,27–0,49°, tętnienie 18–27%, zatrzymania 0–1,4%, błąd końcowy ≤ 0,4 mm, nadmiar 1,1 s.
LIMITS = (('path_err_max_mm', 1.5, 3.0), ('joint_err_deg', 0.5, 1.0), ('speed_ripple_pct', 30.0, 50.0),
          ('stall_pct', 2.0, 5.0), ('final_err_mm', 0.5, 1.0), ('overrun_s', 1.5, 3.0),
          ('final_approach_corrections', 0, 2))
GRADES = ('dobry', 'dopuszczalny', 'przekroczony', 'nieudany')


def grade(metrics):
    if not metrics.get('valid'):
        return 'nieudany', [metrics.get('reason', 'brak danych')]
    values = {**metrics, 'joint_err_deg': max(metrics['joint_err_max_deg'])}
    worst, reasons = 0, []
    for key, good, allowed in LIMITS:
        value = values.get(key)
        if value is None:
            continue
        level = 0 if value <= good else 1 if value <= allowed else 2
        if level:
            reasons.append(f'{key}={value} (próg {good if level == 1 else allowed})')
        worst = max(worst, level)
    return GRADES[worst], reasons


def worst_grade(grades):
    return GRADES[max(GRADES.index(g) for g in grades)]


def joint_speeds(result, gear_ratios):
    """Szczyt prędkości zadanej każdej osi [°/s] i wału [RPM] z nagrania CAN."""
    rows, _ = q.read_feedback(result['t_exec_ns'] - 50_000_000, result['t_end_ns'])
    degrees, rpm = [], []
    for axis in range(1, 7):
        sample = sorted((r[0], r[2]) for r in rows if r[1] == axis)
        if len(sample) < 5:
            return None
        t = np.array([s[0] for s in sample], dtype=np.int64)
        grid = np.arange(t[0], t[-1], 20_000_000, dtype=np.int64)
        value = np.degrees(np.interp(grid, t, [s[1] for s in sample]))
        peak = float(np.max(np.abs(np.gradient(value, 0.02)))) if len(grid) > 2 else 0.0
        degrees.append(round(peak, 1)); rpm.append(round(peak / 6 * gear_ratios[axis - 1], 0))
    return {'deg_s': degrees, 'motor_rpm': rpm}


def line_error(kin, result, start, target):
    """LIN: największe zejście TCP z prostej start–cel i błąd wzdłuż toru względem punktu zadanego [mm]."""
    rows, _ = q.read_feedback(result['t_exec_ns'] - 50_000_000, result['t_end_ns'])
    if not rows:
        return None
    grid = np.arange(min(r[0] for r in rows), max(r[0] for r in rows), 10_000_000, dtype=np.int64)
    ref, meas = np.zeros((len(grid), 6)), np.zeros((len(grid), 6))
    for j in range(6):
        sample = sorted((r[0], r[2], r[3]) for r in rows if r[1] == j + 1)
        t = np.array([x[0] for x in sample], dtype=np.int64)
        ref[:, j] = np.interp(grid, t, [x[1] for x in sample]); meas[:, j] = np.interp(grid, t, [x[2] for x in sample])
    a, b = kin.positions(np.radians([start]))[0], kin.positions(np.radians([target]))[0]
    u = (b - a) / np.linalg.norm(b - a)
    p_ref, p_meas = kin.positions(ref), kin.positions(meas)
    w = p_meas - a
    off = np.linalg.norm(w - np.outer(w @ u, u), axis=1)
    return {'off_line_max_mm': round(float(off.max()), 3), 'off_line_rms_mm': round(float(np.sqrt(np.mean(off ** 2))), 3),
            'along_max_mm': round(float(np.abs((p_ref - p_meas) @ u).max()), 3)}


def recover(robot, log):
    try:
        q.api('/api/v1/motion/stop', {})
    except q.Abort:
        pass
    def ready():
        m = robot.manual()
        return not m['fault'] and not m['busy'] and m.get('recovery_state') in ('idle', 'idle_disabled')
    q.wait(ready, 40, 'Brak powrotu do gotowości po awarii')
    log('recovered')


def prepare_ramp(robot, log):
    """Każdy przebieg ruchu zaczyna się od zerowania MKS i dojazdu do pozy startowej."""
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


def level_params(kind, speed, dq_max):
    if kind == 'PTP':
        return {'preview_velocity_degrees_s': min(80.0, dq_max * speed / DISTANCE_MM),
                'preview_acceleration_degrees_s2': PTP_ACCELERATION}
    return q.lin_params(speed)


def plan_levels(robot, kind, levels, start, target, dq_max, log):
    usable = []
    ctx = robot.context()
    for speed in levels:
        params = level_params(kind, speed, dq_max)
        try:
            robot.plan(ctx, start, kind, target, params, True)
            robot.plan(ctx, target, kind, start, params, True)
            usable.append(speed)
            log('plan_ok', kind=kind, speed=speed, params=params)
        except q.Abort as error:
            log('plan_rejected', kind=kind, speed=speed, reason=str(error)[:240])
            break
    return usable


def run_level(robot, kin, gears, kind, speed, params, start, target, repeats, log, moves):
    """Dopisuje wyniki do moves na bieżąco: ruchy sprzed awarii zostają w ocenie poziomu."""
    for repeat in range(repeats):
        for name, goal in (('tam', target), ('powrót', start)):
            label = f'{kind} {speed:g} mm/s {name} {repeat + 1}'
            move = {'label': label, 'target': list(goal), 'type': kind, 'speed_mm_s': speed}
            result = q.run_move(robot, kind, goal, params, label)
            metrics = q.analyse(kin, move, result)
            verdict, reasons = grade(metrics)
            speeds = joint_speeds(result, gears)
            line = line_error(kin, result, start, target) if kind == 'LIN' else None
            moves.append({**move, 'metrics': metrics, 'grade': verdict, 'reasons': reasons, 'joint_speed': speeds, 'line': line})
            log('move', label=label, grade=verdict, reasons=reasons, metrics=metrics, joint_speed=speeds, line=line,
                brief={k: metrics.get(k) for k in ('v_peak_ref_mm_s', 'path_err_max_mm', 'speed_ripple_pct', 'stall_pct', 'final_err_mm')})


def run_ramp(robot, kin, gears, kind, levels, start, target, dq_max, repeats, log, attempts=ATTEMPTS, explore=False):
    ramp = {'kind': kind, 'levels': [], 'stopped': None}
    prepare_ramp(robot, log)
    for speed in levels:
        params = level_params(kind, speed, dq_max)
        entry = {'speed_mm_s': speed, 'params': params, 'incidents': [], 'moves': [], 'grade': 'nieudany'}
        ramp['levels'].append(entry)
        for attempt in range(attempts):
            try:
                run_level(robot, kin, gears, kind, speed, params, start, target, repeats, log, entry['moves'])
                entry['grade'] = worst_grade([m['grade'] for m in entry['moves']])
                break
            except q.MoveFailed as error:
                reason = str(error)
                log('move_failed', kind=kind, speed=speed, attempt=attempt + 1, reason=reason[:300])
            except q.Abort as error:
                reason = str(error)
                log('level_fault', kind=kind, speed=speed, attempt=attempt + 1, reason=reason[:300])
            entry['incidents'].append(reason[:400])
            recover(robot, log)
            if attempt + 1 < attempts:
                prepare_ramp(robot, log)
        log('level', kind=kind, speed=speed, grade=entry['grade'], incidents=len(entry['incidents']))
        if entry['grade'] == 'nieudany' or (entry['grade'] == 'przekroczony' and not explore):
            ramp['stopped'] = f"{speed:g} mm/s: {entry['grade']}"
            break
    return ramp


def peak(moves, key):
    values = [m['metrics'][key] for m in moves if m['metrics'].get('valid') and m['metrics'].get(key) is not None]
    return max(values) if values else None


def summary(ramps):
    lines = ['| Ruch | Zadane [mm/s] | Ocena | Szczyt TCP [mm/s] | Maks. oś [°/s] | Maks. wał [RPM] | Odchyłka maks. [mm] | Uchyb osi [°] | '
             'Tętnienie [%] | Zatrzymania [%] | Błąd końcowy [mm] | Incydenty |', '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for ramp in ramps:
        for e in ramp['levels']:
            m = e['moves']; speeds = [x['joint_speed'] for x in m if x.get('joint_speed')]
            deg = max((max(s['deg_s']) for s in speeds), default=None); rpm = max((max(s['motor_rpm']) for s in speeds), default=None)
            jerr = max((max(x['metrics']['joint_err_max_deg']) for x in m if x['metrics'].get('valid')), default=None)
            lines.append(f"| {ramp['kind']} | {e['speed_mm_s']:g} | {e['grade']} | {peak(m, 'v_peak_ref_mm_s')} | {deg} | {rpm} | "
                         f"{peak(m, 'path_err_max_mm')} | {jerr} | {peak(m, 'speed_ripple_pct')} | {peak(m, 'stall_pct')} | "
                         f"{peak(m, 'final_err_mm')} | {len(e['incidents'])} |")
    return '\n'.join(lines)


def run_tuning(robot, kin, gears, specs, kinds, usable, start, target, dq_max, args, log):
    runs = []
    for spec in specs:
        run = {'label': spec['label'], 'tuning': spec['tuning'], 'ramps': []}; runs.append(run)
        log('tuning_start', label=spec['label'], tuning=spec['tuning'])
        robot.enable_all(False)
        run['execution_tuning'] = robot.set_tuning(spec['tuning'])
        for kind in kinds:
            run['ramps'].append(run_ramp(robot, kin, gears, kind, usable[kind], start, target, dq_max, args.repeats, log,
                                         args.attempts, True))
    return runs


def tuning_summary(runs):
    lines = ['| Nastawy | Ruch [mm/s] | Ruchy OK | Uchyb osi śr./maks. [°] | Uchyb na osiach maks. J1–J6 [°] | Odchyłka TCP śr./maks. [mm] | RMS TCP [mm] | '
             'Poza linią śr./maks. [mm] | Koniec traj. [mm] | Błąd końcowy [mm] | Korekty | Awarie |', '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for run in runs:
        for ramp in run['ramps']:
            for e in ramp['levels']:
                ok = [m for m in e['moves'] if m['metrics'].get('valid')]
                if not ok:
                    lines.append(f"| {run['label']} | {ramp['kind']} {e['speed_mm_s']:g} | 0 | — | — | — | — | — | — | — | — | {len(e['incidents'])} |"); continue
                x = [m['metrics'] for m in ok]; joint = np.array([v['joint_err_max_deg'] for v in x]); worst = joint.max(axis=1)
                path = [v['path_err_max_mm'] for v in x]; off = [m['line']['off_line_max_mm'] for m in ok if m.get('line')]
                lines.append(f"| {run['label']} | {ramp['kind']} {e['speed_mm_s']:g} | {len(ok)} | {worst.mean():.3f} / {worst.max():.3f} | "
                             f"{' '.join(f'{v:.2f}' for v in joint.max(axis=0))} | {np.mean(path):.2f} / {max(path):.2f} | "
                             f"{np.mean([v['path_err_rms_mm'] for v in x]):.3f} | {f'{np.mean(off):.2f} / {max(off):.2f}' if off else '—'} | "
                             f"{np.mean([v['end_traj_err_mm'] for v in x]):.3f} | {np.mean([v['final_err_mm'] for v in x]):.3f} | "
                             f"{sum(v['final_approach_corrections'] for v in x)} | {len(e['incidents'])} |")
    return '\n'.join(lines)


def main():
    global PTP_ACCELERATION
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--execute', action='store_true', help='wykonaj ruch fizyczny (inaczej tylko planowanie)')
    parser.add_argument('--kinds', default='PTP,LIN', help='rampy do wykonania')
    parser.add_argument('--ptp', default=','.join(map(str, PTP_LEVELS)), help='poziomy PTP [mm/s]')
    parser.add_argument('--lin', default=','.join(map(str, LIN_LEVELS)), help='poziomy LIN [mm/s]')
    parser.add_argument('--repeats', type=int, default=2, help='przejazdy tam i z powrotem na poziom')
    parser.add_argument('--ptp-acc', type=float, default=PTP_ACCELERATION, help='przyspieszenie PTP [°/s²]')
    parser.add_argument('--attempts', type=int, default=ATTEMPTS, help='próby poziomu przy awarii (każda od zerowania)')
    parser.add_argument('--explore', action='store_true', help='nie kończ rampy po przekroczeniu progów, tylko po awarii we wszystkich próbach')
    parser.add_argument('--tune', action='append', default=[], metavar='KLUCZ=W1,W2', help='seria nastaw: kp, lookahead_ms, ff, ki, ki_limit_deg')
    parser.add_argument('--fixed', action='append', default=[], metavar='KLUCZ=W', help='nastawa wspólna dla serii --tune')
    parser.add_argument('--baseline', action='store_true', help='z --tune: przebieg z nastawami z pliku (i --fixed) na początku')
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    PTP_ACCELERATION = args.ptp_acc
    out = q.Path(args.output) if args.output else q.ROOT / 'log/tcp_speed_limits' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = q.Log(out); robot = q.Robot(log)
    values = robot.configuration()['active']['document']['values']
    if values['motion.ptp_lin_dispatch_mode'] != 'synchronized':
        raise q.Abort('Analiza wymaga trybu PTP/LIN synchronized')
    if args.execute:
        robot.ensure_ready()
    kin = q.Kinematics(values)
    gears = [values[f'joint.{j}.gear_ratio'] for j in range(1, 7)]
    procedure = q.build_procedure(robot, q.START_POSE, log)
    start, target = list(q.START_POSE), procedure['targets'][DISTANCE_MM]
    dq_max = max(abs(a - b) for a, b in zip(target, start))
    levels = {'PTP': [float(x) for x in args.ptp.split(',')], 'LIN': [float(x) for x in args.lin.split(',')]}
    kinds = [k.strip().upper() for k in args.kinds.split(',')]
    tcp_limit = values['tcp.max_linear_velocity']
    if 'LIN' in kinds and max(levels['LIN']) > tcp_limit:
        raise q.Abort(f'LIN powyżej tcp.max_linear_velocity = {tcp_limit:g} mm/s wymaga decyzji operatora o zmianie limitu')
    usable = {kind: plan_levels(robot, kind, levels[kind], start, target, dq_max, log) for kind in kinds}
    log('plan_summary', dq_max_deg=round(dq_max, 2), usable=usable)
    if not args.execute:
        return
    ramps, runs = [], []
    args.acc = ['current']
    specs = q.run_specs(args, values) if args.tune else []
    try:
        if specs:
            runs = run_tuning(robot, kin, gears, specs, kinds, usable, start, target, dq_max, args, log)
        for kind in (kinds if not specs else []):
            log('ramp_start', kind=kind, levels=usable[kind])
            ramps.append(run_ramp(robot, kin, gears, kind, usable[kind], start, target, dq_max, args.repeats, log,
                                  args.attempts, args.explore))
            log('ramp_end', kind=kind, stopped=ramps[-1]['stopped'])
    except (q.Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        if specs:
            try:
                robot.set_tuning(None); log('tuning_reset')
            except q.Abort as error:
                log('tuning_reset_failed', reason=str(error)[:200])
                print(f'!!! Nie przywrócono nastaw z pliku: {error}; POST /api/v1/motion/execution-tuning {{"reset": true}}')
        try:
            if not robot.manual()['fault']:
                robot.enable_all(False)
        except q.Abort as error:
            log('final_disable_failed', reason=str(error)[:200])
        (out / 'results.json').write_text(json.dumps({'procedure': {k: v for k, v in procedure.items() if k != 'moves'},
                                                      'limits': LIMITS, 'ramps': ramps, 'tuning_runs': runs},
                                                     ensure_ascii=False, indent=1, default=str))
        text = tuning_summary(runs) if specs else summary(ramps)
        (out / 'summary.md').write_text(text + '\n')
        print('\n' + text + f'\n\nWyniki: {out}')


if __name__ == '__main__':
    try:
        main()
    except q.Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
