#!/usr/bin/env python3
"""Test progu prędkości osi: czy odwzorowanie planu jest lepsze, gdy żadna oś nie jedzie wolniej niż 2 RPM wału.

Ruch: PTP w przestrzeni przegubów z pozy startowej testu jakości (0/0/0/0/45/0°) o Δq = +30/+20/+25/+30/−20/+30° i z powrotem.
PTP jedzie po prostej w przestrzeni przegubów, więc proporcje prędkości osi są stałe (najwolniejsza = 2/3 najszybszej); poziom
prędkości dobiera się tak, by najwolniejsza oś na odcinku stałej prędkości miała zadane RPM wału (domyślnie 1 — poniżej progu —
oraz 2,5 / 5 / 10). Każdy poziom: ruch tam i z powrotem. Start od zerowania MKS (jak test jakości).

Wskaźniki liczone wyłącznie na odcinku stałej prędkości (prędkość zadana osi dominującej ≥ 95 % maksimum), z sync_feedback:
- uchyb osi (pomiar − referencja) w stopniach wału: średnia (stałe opóźnienie) i odchylenie standardowe (część zmienna — schodki),
- zatrzymania osi (prędkość z okna 50 ms < 50 % zadanej),
- odchyłka TCP od referencji (jak w teście jakości) i jej część zmienna (odchylenie standardowe).

    python3 scripts/axis_speed_floor.py                 # suchy przebieg: plan każdego poziomu i sprawdzenie progu w planie
    python3 scripts/axis_speed_floor.py --execute       # ruch (stała zgoda tylko przy rozsprzęglonych wałach, AGENTS.md)
    python3 scripts/axis_speed_floor.py --execute --min-lead 0,1,2   # seria: minimalne wyprzedzenie celu F5 [° wału], 0 = plik
    python3 scripts/axis_speed_floor.py --execute --acc file,0,230,200,160 --levels 1,2.5,5   # seria kodów acc F5 (test, poza konfiguracją)

Seria --min-lead ustawia nastawę testową `min_lead_shaft_deg` (POST /api/v1/motion/execution-tuning) przed każdym zestawem poziomów
i przywraca nastawy z pliku na końcu (także po przerwaniu).
"""
import argparse, json, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tcp_motion_quality as q   # klient API, zerowanie, wykonanie, odczyt sync_feedback, kinematyka

DELTA_DEG = (30.0, 20.0, 25.0, 30.0, -20.0, 30.0)
LEVELS_RPM = (1.0, 2.5, 5.0, 10.0)     # RPM wału najwolniejszej osi na odcinku stałej prędkości
RAMP_S = 0.4                          # czas rozpędzania (przyspieszenie = prędkość / RAMP_S)
FLOOR_RPM = 2.0


def gear_ratios(values):
    return [float(values[f'joint.{j}.gear_ratio']) for j in range(1, 7)]


def shaft_rpm(joint_deg_s, gears):
    return np.abs(joint_deg_s) * np.array(gears) / 6.0   # °/s przegubu → RPM wału


def plan_profile(robot, ctx, start, target, velocity, gears, simulated=True):
    """Plan PTP i RPM wału osi na odcinku stałej prędkości (z punktów planu)."""
    params = {'preview_velocity_degrees_s': velocity, 'preview_acceleration_degrees_s2': velocity / RAMP_S}
    plan_id = robot.plan(ctx, start, 'PTP', target, params, simulated)
    points = q.api('/api/v1/planning/plans/' + plan_id)['trajectory']['points']
    v = np.degrees(np.array([p['velocities'] for p in points]))
    rpm = shaft_rpm(v, gears); dominant = rpm.max(axis=1); cruise = dominant >= 0.95 * dominant.max()
    t = points[-1]['time_from_start']
    return params, rpm[cruise].min(axis=0), t['sec'] + t['nanosec'] * 1e-9


def calibrate(robot, start, target, gears):
    """Prędkość PTP [°/s] dająca zadane RPM najwolniejszej osi (zależność liniowa, kalibracja jednym planem)."""
    ctx = robot.context()
    _, minimum, _ = plan_profile(robot, ctx, start, target, 2.0, gears)
    return 2.0 / float(minimum.min())   # [°/s] na 1 RPM najwolniejszej osi


def analyse(kin, gears, result):
    rows, _ = q.read_feedback(result['t_exec_ns'] - 50_000_000, result['t_done_ns'])
    if len({r[1] for r in rows}) != 6:
        return {'valid': False, 'reason': 'brak sync_feedback wszystkich osi'}
    t0 = min(r[0] for r in rows); t1 = max(r[0] for r in rows)
    grid = np.arange(t0, t1, 5_000_000, dtype=np.int64)
    ref = np.zeros((len(grid), 6)); meas = np.zeros((len(grid), 6))
    for j in range(6):
        sel = sorted((r[0], r[2], r[3]) for r in rows if r[1] == j + 1)
        t = np.array([s[0] for s in sel], dtype=np.int64)
        ref[:, j] = np.interp(grid, t, [s[1] for s in sel]); meas[:, j] = np.interp(grid, t, [s[2] for s in sel])
    w = 10   # 50 ms
    vr = np.degrees(ref[w:] - ref[:-w]) / 0.05; vm = np.degrees(meas[w:] - meas[:-w]) / 0.05
    rpm = shaft_rpm(vr, gears); dominant = rpm.max(axis=1); cruise = dominant >= 0.95 * dominant.max()
    err_shaft = np.degrees(meas - ref)[w // 2: w // 2 + len(vr)] * np.array(gears)   # ° wału, wyrównane do okna prędkości
    axes = []
    for j in range(6):
        e = err_shaft[cruise, j]
        axes.append({'rpm': round(float(np.median(rpm[cruise, j])), 2), 'lag_shaft_deg': round(float(np.mean(e)), 3),
                     'fluct_shaft_deg': round(float(np.std(e)), 3), 'max_shaft_deg': round(float(np.max(np.abs(e))), 3),
                     'stall_pct': round(float(np.mean(np.abs(vm[cruise, j]) < 0.5 * np.abs(vr[cruise, j])) * 100), 1)})
    p_ref, p_meas = kin.positions(ref), kin.positions(meas)
    d = np.linalg.norm(p_meas - p_ref, axis=1)[w // 2: w // 2 + len(vr)][cruise]
    return {'valid': True, 'cruise_s': round(float(cruise.sum() * 0.005), 2), 'min_axis_rpm': min(a['rpm'] for a in axes),
            'tcp_err_rms_mm': round(float(np.sqrt(np.mean(d ** 2))), 3), 'tcp_err_max_mm': round(float(np.max(d)), 3),
            'tcp_err_fluct_mm': round(float(np.std(d)), 3), 'axes': axes}


def summary(moves):
    lines = ['| acc | Min. wyprzedzenie [° wału] | Poziom (min RPM) | Ruch | Min RPM osi (zmierz. ref.) | TCP RMS / max [mm] | TCP część zmienna [mm] | '
             'Uchyb osi: część zmienna śr. [° wału] | Zatrzymania osi śr. [%] | Osie < 2 RPM |', '|---|---|---|---|---|---|---|---|---|---|']
    for m in moves:
        x = m.get('metrics') or {}
        if not x.get('valid'):
            lines.append(f"| {m.get('acc', 'plik')} | {m.get('min_lead', 0):g} | {m['level']:g} | {m['label']} | — | {x.get('reason', m.get('error', ''))} | | | | |"); continue
        lines.append(f"| {m.get('acc', 'plik')} | {m.get('min_lead', 0):g} | {m['level']:g} | {m['label']} | {x['min_axis_rpm']} | {x['tcp_err_rms_mm']} / {x['tcp_err_max_mm']} | "
                     f"{x['tcp_err_fluct_mm']} | {np.mean([a['fluct_shaft_deg'] for a in x['axes']]):.3f} | "
                     f"{np.mean([a['stall_pct'] for a in x['axes']]):.1f} | {sum(a['rpm'] < FLOOR_RPM for a in x['axes'])} |")
    lines += ['', '| acc | Min. wyprz. | Poziom | Ruch | ' + ' | '.join(f'J{j} RPM / zmienna [° wału] / zatrz. [%]' for j in range(1, 7)) + ' |',
              '|---|---|---|---|' + '---|' * 6]
    for m in moves:
        x = m.get('metrics') or {}
        if x.get('valid'):
            lines.append(f"| {m.get('acc', 'plik')} | {m.get('min_lead', 0):g} | {m['level']:g} | {m['label']} | " +
                         ' | '.join(f"{a['rpm']} / {a['fluct_shaft_deg']} / {a['stall_pct']}" for a in x['axes']) + ' |')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--levels', default=','.join(f'{x:g}' for x in LEVELS_RPM), help='RPM wału najwolniejszej osi, np. 1,2.5,5,10')
    parser.add_argument('--acc', default='file', help='kody acc F5 (nadpisanie testowe 0–255, „file” = konfiguracja), seria np. file,0,230')
    parser.add_argument('--min-lead', default='0', help='minimalne wyprzedzenie celu F5 [° wału], seria np. 0,1,2 (0 = nastawy z pliku)')
    parser.add_argument('--execute', action='store_true', help='wykonaj ruch fizyczny (inaczej suchy przebieg)')
    args = parser.parse_args()
    levels = [float(x) for x in args.levels.split(',')]
    leads = [float(x) for x in args.min_lead.split(',')]
    accs = [None if x.strip() == 'file' else int(x) for x in args.acc.split(',')]
    if any(a is not None and not 0 <= a <= 255 for a in accs):
        raise q.Abort('--acc: 0–255 albo file')
    if any(not 0 <= x <= 10 for x in leads):
        raise q.Abort('--min-lead: 0–10° wału')
    out = q.ROOT / 'log/axis_speed_floor' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = q.Log(out); robot = q.Robot(log)
    values = robot.configuration()['active']['document']['values']
    gears, kin = gear_ratios(values), q.Kinematics(values)
    start = list(q.START_POSE); target = [s + d for s, d in zip(start, DELTA_DEG)]
    per_rpm = calibrate(robot, start, target, gears)
    ctx = robot.context(); plan = []
    for level in levels:
        params, minimum, duration = plan_profile(robot, ctx, start, target, level * per_rpm, gears)
        back = plan_profile(robot, ctx, target, start, level * per_rpm, gears)
        plan.append({'level': level, 'params': params, 'plan_min_rpm': [round(float(x), 2) for x in minimum],
                     'plan_s': round(duration, 1), 'back_min_rpm': [round(float(x), 2) for x in back[1]]})
        log('plan', level=level, velocity_deg_s=round(params['preview_velocity_degrees_s'], 3),
            min_rpm=plan[-1]['plan_min_rpm'], duration_s=plan[-1]['plan_s'])
    if not args.execute:
        log('dry_run_ok', levels=levels, total_s=round(sum(2 * p['plan_s'] for p in plan), 1)); return
    robot.ensure_ready()   # odmawia, gdy trwa przebieg programu
    moves = []
    try:
        robot.enable_all(False); robot.zero_and_confirm(); robot.enable_all(True)
        q.run_move(robot, 'PTP', start, {'preview_velocity_degrees_s': q.SETUP_VELOCITY,
                                         'preview_acceleration_degrees_s2': q.SETUP_ACCELERATION}, 'PTP do pozy startowej')
        for acc, lead in [(a, l) for a in accs for l in leads]:
            tuning = {**({'acceleration_code': acc} if acc is not None else {}), **({'min_lead_shaft_deg': lead} if lead else {})}
            robot.set_tuning(tuning or None)
            for p in plan:
                for label, goal in (('tam', target), ('powrót', start)):
                    name = f"PTP {p['level']:g} RPM {label} (acc {acc if acc is not None else 'plik'}, min. wyprzedzenie {lead:g}°)"
                    try:
                        result = q.run_move(robot, 'PTP', goal, p['params'], name)
                    except q.MoveFailed as error:
                        moves.append({'acc': acc if acc is not None else 'plik', 'min_lead': lead, 'level': p['level'], 'label': label, 'error': str(error)[:300]})
                        log('move_failed', label=name, reason=str(error)[:300]); continue
                    metrics = analyse(kin, gears, result)
                    moves.append({'acc': acc if acc is not None else 'plik', 'min_lead': lead, 'level': p['level'], 'label': label, 'result': result, 'metrics': metrics})
                    log('move', label=name, brief={k: metrics.get(k) for k in ('min_axis_rpm', 'tcp_err_rms_mm', 'tcp_err_fluct_mm')})
        robot.enable_all(False)
    except (q.Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        if any(leads) or any(a is not None for a in accs):
            try:
                robot.set_tuning(None); log('tuning_reset')
            except q.Abort as error:
                log('tuning_reset_failed', reason=str(error)[:200])
                print(f'!!! Nie przywrócono nastaw z pliku: {error}; ręcznie: POST /api/v1/motion/execution-tuning {{"reset": true}}')
        (out / 'results.json').write_text(json.dumps({'plan': plan, 'moves': moves}, ensure_ascii=False, indent=1, default=str))
        (out / 'summary.md').write_text(summary(moves) + '\n')
        print('\n' + summary(moves) + f'\n\nWyniki: {out}')


if __name__ == '__main__':
    try:
        main()
    except q.Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
