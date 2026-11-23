#!/usr/bin/env python3
"""Próba wywołania ucieczki J2 (napęd jedzie przeciwnie do celu F5) — seria powtórzeń ruchu z testu jakości TCP.

Dla każdej wartości --acc: zerowanie MKS, poza startowa (0/0/0/0/45/0), potem --pairs razy:
PTP 400 mm 40 mm/s do celu i LIN 400 mm 40 mm/s z powrotem (w LIN J2 zmienia kierunek ruchu).
Każdy ruch analizowany z nagrania CAN (sync_feedback + ramki F5/31 osi): maks. uchyb osi, epizody „ruchu przeciwnie
do celu” (oś oddala się od celu F5 przy rosnącym uchybie), stan tuż przed epizodem. Ruch fizyczny wymaga zgody
operatora (stała zgoda tylko przy rozsprzęglonej mechanice, AGENTS.md §5).
"""
import argparse, glob, json, math, os, time
from datetime import datetime
from pathlib import Path

import tcp_motion_quality as q

SPEED = 40.0
DISTANCE = 400.0
DIVERGE_DEG = 0.5            # uchyb osi uznawany za epizod
TICKS_PER_DEG = 16384 / 360  # wał


def axis_frames(t0_ns, t1_ns):
    """Na oś: sync_feedback (czas, ref°, pomiar°) i TX F5 (czas, rpm, cel imp), RX F1 (czas, stan)."""
    fb, f5, f1 = {}, {}, {}
    for path in glob.glob(str(q.CAN_LOG / 'can-*.jsonl')):
        if os.path.getmtime(path) * 1e9 < t0_ns:
            continue
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                feedback = '"sync_feedback"' in line
                tx = '"can_tx"' in line and '"data":"f5' in line
                rx1 = '"can_rx"' in line and '"data":"f1' in line
                if not (feedback or tx or rx1):
                    continue
                r = json.loads(line)
                if not t0_ns <= r['utc_ns'] <= t1_ns:
                    continue
                t = r['utc_ns']; axis = r['axis']
                if feedback:
                    d = dict(kv.split('=') for kv in r['detail'].split())
                    fb.setdefault(axis, []).append((t, math.degrees(float(d['q_RX'])), math.degrees(float(d['q_measured']))))
                elif tx:
                    b = bytes.fromhex(r['data'])
                    f5.setdefault(axis, []).append((t, ((b[1] & 15) << 8) | b[2], int.from_bytes(b[4:7], 'big', signed=True)))
                else:
                    f1.setdefault(axis, []).append((t, int(r['data'][2:4], 16)))
    for store in (fb, f5, f1):
        for v in store.values():
            v.sort()
    return fb, f5, f1


def episodes(fb, f5, f1, gear=10.0):
    """Epizody na oś: uchyb > próg i oś przesuwa się w stronę przeciwną do celu F5 (pierwsza próbka i kontekst)."""
    out = {}
    for axis, rows in fb.items():
        worst = max(abs(r - m) for _, r, m in rows) if rows else 0
        found = None
        for i in range(5, len(rows)):
            t, ref, meas = rows[i]
            if abs(ref - meas) < DIVERGE_DEG:
                continue
            # kierunek komendy: cel F5 względem pomiaru; kierunek ruchu: zmiana pomiaru w ostatnich 50 ms
            target = next((c for tt, _, c in reversed(f5.get(axis, [])) if tt <= t), None)
            if target is None:
                continue
            target_deg = target / TICKS_PER_DEG / gear
            moving = meas - rows[i - 5][2]
            if (target_deg - meas) * moving < 0:     # oddala się od celu
                # początek: cofnij się do ostatniej próbki, gdzie oś jechała w stronę celu
                start = i
                while start > 1:
                    tt, rr, mm = rows[start - 1]
                    tg = next((c for t5, _, c in reversed(f5.get(axis, [])) if t5 <= tt), target) / TICKS_PER_DEG / gear
                    if (tg - mm) * (mm - rows[start - 2][2]) >= 0:
                        break
                    start -= 1
                t_start = rows[start][0]
                before = [(round((tt - t_start) / 1e6, 1), round(rr, 3), round(mm, 3)) for tt, rr, mm in rows[max(0, start - 10):start + 10]]
                cmds = [(round((tt - t_start) / 1e6, 1), rpm, round(c / TICKS_PER_DEG / gear, 3)) for tt, rpm, c in f5.get(axis, [])
                        if -150e6 <= tt - t_start <= 150e6]
                states = [(round((tt - t_start) / 1e6, 1), s) for tt, s in f1.get(axis, []) if -300e6 <= tt - t_start <= 200e6]
                v_before = (rows[start][2] - rows[max(0, start - 10)][2]) / max(1e-9, (rows[start][0] - rows[max(0, start - 10)][0]) / 1e9)
                found = {'t_ns': t_start, 'meas_deg': round(rows[start][2], 3), 'ref_deg': round(rows[start][1], 3),
                         'joint_speed_before_deg_s': round(v_before, 2), 'feedback': before, 'f5': cmds, 'f1': states}
                break
        out[axis] = {'max_err_deg': round(worst, 3), 'episode': found}
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acc', nargs='+', type=int, default=[254])
    parser.add_argument('--pairs', type=int, default=10)
    args = parser.parse_args()
    out = q.ROOT / 'log/j2_probe' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = q.Log(out); robot = q.Robot(log)
    robot.ensure_ready()
    values = robot.configuration()['active']['document']['values']
    gear = [values[f'joint.{j}.gear_ratio'] for j in range(1, 7)]
    procedure = q.build_procedure(robot, q.START_POSE, log)
    target = procedure['targets'][DISTANCE]
    dq = max(abs(a - b) for a, b in zip(target, q.START_POSE))
    results = []
    try:
        for acc in args.acc:
            log('block_start', acc=acc)
            robot.enable_all(False); robot.set_acceleration(acc); robot.set_tuning(None)
            robot.zero_and_confirm()
            q.run_move(robot, 'PTP', q.START_POSE, {'preview_velocity_degrees_s': q.SETUP_VELOCITY,
                       'preview_acceleration_degrees_s2': q.SETUP_ACCELERATION}, 'PTP do pozy startowej')
            for pair in range(1, args.pairs + 1):
                for kind, goal, params in (('PTP', target, q.ptp_params(dq, DISTANCE, SPEED)),
                                           ('LIN', q.START_POSE, q.lin_params(SPEED))):
                    label = f'acc {acc} para {pair} {kind}'
                    t0 = time.time_ns(); error = None
                    try:
                        q.run_move(robot, kind, goal, params, label)
                    except q.Abort as failure:
                        error = str(failure)
                    t1 = time.time_ns()
                    fb, f5, f1 = axis_frames(t0 - 100_000_000, t1)
                    analysis = {j: v for j, v in episodes(fb, f5, f1).items()}
                    hit = {j: v for j, v in analysis.items() if v['episode']}
                    row = {'acc': acc, 'pair': pair, 'move': kind, 'error': error, 'axes': analysis}
                    results.append(row)
                    log('move', label=label, error=(error or '')[:160],
                        max_err={j: v['max_err_deg'] for j, v in analysis.items()},
                        reversal={j: {k: v['episode'][k] for k in ('meas_deg', 'ref_deg', 'joint_speed_before_deg_s')} for j, v in hit.items()})
                    if error:
                        if 'Awaria' in error and not hit:
                            raise
                        # po ucieczce: powrót do gotowości i do pozy startowej, kontynuacja
                        robot.wait_quiet(30)
                        q.run_move(robot, 'PTP', q.START_POSE, {'preview_velocity_degrees_s': q.SETUP_VELOCITY,
                                   'preview_acceleration_degrees_s2': q.SETUP_ACCELERATION}, 'powrót do pozy startowej')
                        break
            robot.enable_all(False)
    except (q.Abort, KeyboardInterrupt) as failure:
        log('aborted', reason=str(failure)[:300])
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        (out / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=1))
        summary = {}
        for row in results:
            key = (row['acc'], row['move'])
            s = summary.setdefault(key, {'n': 0, 'reversals': 0, 'errors': 0, 'max_j2': 0.0})
            s['n'] += 1; s['errors'] += bool(row['error'])
            s['reversals'] += any(v['episode'] for v in row['axes'].values())
            s['max_j2'] = max(s['max_j2'], row['axes'].get(2, {}).get('max_err_deg', 0))
        lines = ['| acc | ruch | prób | epizody ruchu przeciwnie do celu | nieudane | maks. uchyb J2 [°] |', '|---|---|---|---|---|---|']
        lines += [f'| {a} | {m} | {s["n"]} | {s["reversals"]} | {s["errors"]} | {s["max_j2"]:.2f} |' for (a, m), s in summary.items()]
        (out / 'summary.md').write_text('\n'.join(lines) + '\n')
        print('\n'.join(lines) + f'\n\nWyniki: {out}')


if __name__ == '__main__':
    try:
        main()
    except q.Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
