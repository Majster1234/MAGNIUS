#!/usr/bin/env python3
"""SPRZĘTOWA próba długich ruchów PTP i LIN (RUCH ROBOTA) — czy nie są przerywane z błahych powodów.

Ta sama ścieżka co programy: planer (`/api/v1/planning/moves`) → wykonawca (`/api/v1/motion/*`, tryb z konfiguracji,
zwykle SYNC). Przebieg: zerowanie MKS i ENABLE (jak scripts/tcp_motion_quality.py), PTP do pozy startowej, potem
w pętli przez zadany czas:
  - PTP o dużych kątach (do ±150° J6, przejście J5 przez 0 — osobliwość nadgarstka nie spowalnia PTP),
    przy 50% i 100% limitu prędkości osi,
  - LIN tam i z powrotem (domyślnie 300 mm) przy 20 i 50 mm/s.
Każdy ruch: wynik (OK / nieudany bez awarii / awaria z ENABLE=0), czas planu i wykonania, komunikat. Po awarii
ponowne ENABLE i kontynuacja. Wynik: log/motion-endurance/<czas>/events.jsonl + summary.json.
Stała zgoda operatora dotyczy wyłącznie rozsprzęglonej mechaniki (AGENTS.md §5).
Użycie: python3 scripts/motion_endurance.py --minutes 10 [--lin-mm 300] [--no-zero]
"""
import argparse, json, math, sys, time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from tcp_motion_quality import Robot, Log, Abort, api, wait, lin_params, START_POSE, DIRECTIONS  # noqa: E402

HOME = [0.0, -30.0, 40.0, 0.0, 45.0, 0.0]
FAR_A = [70.0, -70.0, 20.0, 120.0, -50.0, 150.0]
FAR_B = [-70.0, -5.0, 50.0, -120.0, 70.0, -150.0]
LIN_HOME = list(START_POSE)   # poza testu jakości TCP (0/0/0/0/45/0°): LIN 400 mm wykonalny w kierunkach DIRECTIONS


def ptp(fraction):
    return {'preview_velocity_degrees_s': 80.0 * fraction, 'preview_acceleration_degrees_s2': 500.0 * fraction}


def lin_target(robot, start, distance_mm):
    ctx = robot.context(); pose = robot.ik(ctx, start)['tcp']
    xyz, rpy = pose[:3], pose[3:]
    for d in DIRECTIONS:
        n = math.sqrt(sum(c * c for c in d)); u = [c / n for c in d]
        try:
            answer = robot.ik(ctx, start, [xyz[i] + distance_mm * u[i] for i in range(3)] + list(rpy))
            if answer['position_error_mm'] > 0.01:
                continue
            target = answer['positions_degrees']
            robot.plan(ctx, start, 'LIN', target, lin_params(20), True)
            return target, d
        except Abort:
            continue
    raise Abort(f'Brak wykonalnego kierunku LIN {distance_mm} mm z pozy startowej')


def ready_after_fault(robot):
    wait(lambda: (lambda s: not s['fault'] and s.get('recovery_state') in ('idle', 'idle_disabled'))(robot.manual()), 60,
         'Brak powrotu do gotowości po awarii')
    robot.ensure_ready(); robot.enable_all(True)


def move(robot, log, kind, target, params, label):
    if robot.manual()['fault']:
        ready_after_fault(robot)
    robot.wait_quiet(30)
    if not all(a['enabled'] for a in robot.manual()['axes'] if a['available']):
        robot.enable_all(True)
    ctx = robot.context(); t0 = time.monotonic()
    try:
        plan = robot.plan(ctx, ctx['seed_degrees'], kind, target, params, False)
        t1 = time.monotonic()
        result = robot.execute(plan, label)
        log('move', label=label, outcome='ok', plan_s=round(t1 - t0, 2), exec_s=round(time.monotonic() - t1, 2),
            duration_s=result.get('duration_s'), message=result.get('message'))
        return 'ok'
    except Abort as error:
        m = robot.manual()
        outcome = 'fault' if m['fault'] or m.get('recovery_state') not in ('idle', None) or not all(
            a['enabled'] for a in m['axes'] if a['available']) else 'failed'
        log('move', label=label, outcome=outcome, message=str(error)[:400], manual=str(m.get('message'))[:200])
        if outcome == 'fault':
            ready_after_fault(robot)
        return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--minutes', type=float, default=10)
    parser.add_argument('--lin-mm', type=float, default=400)
    parser.add_argument('--no-zero', action='store_true')
    args = parser.parse_args()
    out = ROOT / 'log/motion-endurance' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = Log(out); robot = Robot(log)
    robot.ensure_ready()
    if not args.no_zero:
        robot.zero_and_confirm()
    robot.enable_all(True)
    if move(robot, log, 'PTP', HOME, ptp(.25), 'PTP do pozy startowej') != 'ok':
        raise SystemExit('Nie osiągnięto pozy startowej')
    target, direction = lin_target(robot, LIN_HOME, args.lin_mm)
    log('lin_direction', direction=direction, target=[round(v, 2) for v in target])
    sequence = [('PTP', FAR_A, ptp(.5), 'PTP dom→A 50%'), ('PTP', FAR_B, ptp(1.), 'PTP A→B 100% (J5 przez 0)'),
                ('PTP', LIN_HOME, ptp(1.), 'PTP B→start LIN 100%'), ('LIN', target, lin_params(20), f'LIN {args.lin_mm:g} mm 20 mm/s'),
                ('LIN', LIN_HOME, lin_params(50), f'LIN powrót 50 mm/s'), ('PTP', FAR_B, ptp(.5), 'PTP start LIN→B 50%'),
                ('PTP', FAR_A, ptp(1.), 'PTP B→A 100% (J5 przez 0)'), ('PTP', LIN_HOME, ptp(.5), 'PTP A→start LIN 50%'),
                ('LIN', target, lin_params(50), f'LIN {args.lin_mm:g} mm 50 mm/s'), ('LIN', LIN_HOME, lin_params(20), 'LIN powrót 20 mm/s'),
                ('PTP', HOME, ptp(.5), 'PTP start LIN→dom 50%')]
    end = time.monotonic() + args.minutes * 60; results = []; cycle = 0
    while time.monotonic() < end:
        cycle += 1
        for kind, goal, params, label in sequence:
            if time.monotonic() >= end:
                break
            results.append((label, move(robot, log, kind, goal, params, f'#{cycle} {label}')))
    summary = {'minutes': args.minutes, 'moves': len(results), 'ok': sum(o == 'ok' for _, o in results),
               'failed': sum(o == 'failed' for _, o in results), 'fault': sum(o == 'fault' for _, o in results)}
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    log('summary', **summary)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
