#!/usr/bin/env python3
"""Test nastaw napędu MKS na jednej osi (tryb pracy, prąd roboczy, prąd trzymania) — porównanie schodków w wolnym ruchu.

Hipoteza (19.11.2026): w SR_vFOC sterownik dobiera prąd do uchybu; bez obciążenia (wały rozsprzęglone) prąd jest mały,
wirnik trzyma się pełnego kroku (1,8°) i przeskakuje go z opóźnieniem — schodki położenia przy kilku RPM wału.
SR_CLOSE pracuje ze stałym prądem roboczym.

Przebieg (domyślnie TYLKO karta i preflight; ruch wyłącznie z --execute-approved po zgodzie operatora):
  1. Zerowanie MKS (jak testy jakości), kopia `pre_test` (obowiązkowa przed ingerencją), tryb izolowany: ENABLE=1 tylko
     badana oś, PTP z maską osi (`participating_axes`).
  2. Ocena A (tryb z pre_test) → zapis trybu testowego (ENABLE=0 osi, kontener test → apply_test, odczyt kontrolny)
     → ocena B → powrót do pre_test (ENABLE=0) → ocena A → zapis → ocena B (przeplatane A/B/A/B).
  3. ZAWSZE powrót do `pre_test` na końcu i przy każdym przerwaniu (odczyt kontrolny, `remaining_differences`), wszystkie
     osie ENABLE=0. Decyzja o pozostawieniu trybu należy do operatora (osobno, przez mechanizm kopii).
Ocena = (a) ruchy skokowe ±2°/±8° 10°/s z twardymi kryteriami z `mks_pid_tuning.py` (oscylacja w postoju p-p > 0,15°,
przeregulowanie > 0,5°, brak ustalenia, awaria/nieudany program), (b) wolne PTP badanej osi +3° i z powrotem przy
1 / 2,5 / 5 RPM wału: część zmienna uchybu (odchylenie standardowe pomiar − referencja, ° wału), maks. uchyb,
zatrzymania (prędkość z okna 50 ms < 50 % zadanej) na odcinku stałej prędkości. Twarde naruszenie w trybie testowym
kończy rundy B (powrót do pre_test).

    python3 scripts/drive_mode_test.py --axis 1                       # karta + preflight (bez ruchu i zapisu)
    python3 scripts/drive_mode_test.py --axis 1 --execute-approved    # test (karta zaakceptowana przez operatora)
    python3 scripts/drive_mode_test.py --axis 1 --set hold_current_ratio=8       # prąd trzymania 90 % (0–8 = 10–90 %)
    python3 scripts/drive_mode_test.py --axis 1 --set work_current_ma=1800       # prąd roboczy [mA] — nie powyżej znamionowego
"""
import argparse, copy, json, math, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mks_pid_tuning as pt          # tryb izolowany, PTP jednej osi z nadzorem, ocena skoków, operacje na kopiach
import tcp_motion_quality as q       # zerowanie MKS z potwierdzeniem odniesienia

MODES = {'close': (4, 'SR_CLOSE'), 'vfoc': (5, 'SR_vFOC')}
SLOW_RPM = (1.0, 2.5, 5.0)
SLOW_DEG = 3.0
RAMP_S = 0.4


def run_ptp_isolated(robot, q0, target_axis_deg, args, log, label):
    """PTP tylko badanej osi: cel pozostałych osi = start planu z tego samego zapytania (wymóg maski osi w motion/prepare);
    nadzór zakresu (`pt.guard`) nadal względem pozycji z początku testu q0."""
    ctx = pt.api('/api/v1/planning/context')
    seed = ctx['seed_degrees']
    target = list(seed); target[robot.axis - 1] = target_axis_deg
    request = {'model_id': ctx['model_id'], 'configuration_revision': ctx['configuration_revision'],
               'settings_revision': ctx['settings_revision'], 'seed_degrees': seed, 'simulated_start': [False] * 6,
               'preview_velocity_degrees_s': args.velocity, 'preview_acceleration_degrees_s2': args.acceleration,
               'motion_type': 'PTP', 'target_degrees': target}
    job = pt.api('/api/v1/planning/moves', request); end = time.monotonic() + 15
    while job['status'] == 'running':
        if time.monotonic() > end:
            raise pt.Abort('Timeout planera')
        time.sleep(0.05); job = pt.api('/api/v1/planning/jobs/' + job['id'])
    if job['status'] != 'succeeded':
        raise pt.Abort(f'Planowanie nieudane: {job}')
    pt.api('/api/v1/motion/prepare', {'plan_id': job['plan_id'], 'participating_axes': [robot.axis]})
    end = time.monotonic() + 15
    while True:
        st = pt.api('/api/v1/motion/status'); phase = st['preparation']['state']
        if phase in ('failed', 'cancelled'):
            raise (pt.Hard if pt.at_rest_problem(st['preparation']) else pt.Abort)(f"Przygotowanie nieudane: {st['preparation']}")
        if phase == 'ready' and st['state'] == 'ready':
            break
        if time.monotonic() > end:
            raise pt.Abort('Timeout przygotowania programu')
        time.sleep(0.05)
    prepared = pt.api('/api/v1/motion/prepared')
    t_exec = time.time_ns()
    pt.api('/api/v1/motion/execute', {'program_id': prepared['program_id'], 'execution_key': f'mode-{pt.uuid.uuid4().hex}'})
    robot.running = True
    end = time.monotonic() + 60
    while True:
        pt.guard(robot, q0, args)
        st = pt.api('/api/v1/motion/status')
        if st['state'] == 'succeeded' and not st['busy']:
            break
        if st['state'] in ('failed', 'fault', 'cancelled', 'aborted'):
            robot.running = False
            raise (pt.Hard if pt.at_rest_problem(st['message']) else pt.Abort)(f"Program zakończony: {st['state']} {st['message']}")
        if time.monotonic() > end:
            raise pt.Abort('Program przekroczył 60 s')
        time.sleep(0.04)
    robot.running = False
    t_done = time.time_ns()
    end = time.monotonic() + pt.SETTLE_OBS_S
    while time.monotonic() < end:
        pt.guard(robot, q0, args); time.sleep(0.05)
    return {'label': label, 'target_deg': target_axis_deg, 'start_deg': seed[robot.axis - 1],
            't_exec_ns': t_exec, 't_done_ns': t_done, 't_end_ns': time.time_ns()}


pt.run_ptp = run_ptp_isolated   # ocena skoków z mks_pid_tuning (pt.evaluate) używa tej samej funkcji


def slow_metrics(robot, move, gear):
    """Część zmienna uchybu, maks. uchyb i zatrzymania (° wału) na odcinku stałej prędkości wolnego ruchu."""
    axis = robot.axis
    meas = pt.ring(f'axis-{axis}.bin', move['t_exec_ns'], move['t_done_ns'])
    ref = pt.ring(f'reference-{axis}.bin', move['t_exec_ns'], move['t_done_ns'])
    if len(meas) < 20 or len(ref) < 20:
        return {'valid': False, 'reason': 'za mało próbek śladu wału'}
    tm = np.array([t for t, _ in meas], float) / 1e9; vm = np.array([v for _, v in meas], float)
    tr = np.array([t for t, _ in ref], float) / 1e9; vr = np.array([v for _, v in ref], float)
    grid = np.arange(max(tm[0], tr[0]), min(tm[-1], tr[-1]), 0.005)
    if len(grid) < 40:
        return {'valid': False, 'reason': 'za krótki wspólny zakres'}
    M = np.interp(grid, tm, vm); R = np.interp(grid, tr, vr)   # ° wału
    w = 10
    sm = (M[w:] - M[:-w]) / 0.05; sr = (R[w:] - R[:-w]) / 0.05
    # Odcinek stałej prędkości: ±25 % zadanej prędkości wału (referencja z zapisu ma rozdzielczość 0,01° — próg względem
    # maksimum zaszumionej pochodnej wybierał tylko 0,3–0,4 s; poprawione 19.11.2026).
    target = np.median(np.abs(sr)[np.abs(sr) > 0.5 * np.percentile(np.abs(sr), 90)])
    cruise = np.abs(np.abs(sr) - target) < 0.25 * target
    if cruise.sum() < 40:
        return {'valid': False, 'reason': 'za krótki odcinek stałej prędkości'}
    err = (M - R)[w // 2: w // 2 + len(sr)][cruise]
    return {'valid': True, 'rpm': round(float(np.median(np.abs(sr[cruise]))) / 6, 2),
            'fluct_shaft_deg': round(float(np.std(err)), 3), 'max_shaft_deg': round(float(np.max(np.abs(err - np.mean(err)))), 3),
            'lag_shaft_deg': round(float(np.mean(err)), 3),
            'stall_pct': round(float(np.mean(np.abs(sm[cruise]) < 0.5 * np.abs(sr[cruise])) * 100), 1)}


def slow_evaluation(robot, q0, args, log, name, gear):
    home = q0[robot.axis - 1]; out = []
    for rpm in SLOW_RPM:
        joint = rpm * 6 / gear
        a = copy.copy(args); a.velocity = joint; a.acceleration = joint / RAMP_S
        for label, target in ((f'{rpm:g} RPM tam', home + SLOW_DEG), (f'{rpm:g} RPM powrót', home)):
            robot.wait_idle()
            move = run_ptp_isolated(robot, q0, target, a, log, label)
            move['metrics'] = slow_metrics(robot, move, gear)
            out.append({'label': label, **move['metrics']})
    robot.wait_idle()
    valid = [m for m in out if m.get('valid')]
    summary = {'name': name, 'moves': out,
               **{k: round(float(np.mean([m[k] for m in valid])), 3) for k in ('fluct_shaft_deg', 'max_shaft_deg', 'stall_pct')}} if valid else {'name': name, 'moves': out}
    log('slow_evaluation', **summary)
    return summary


def evaluate(robot, q0, args, log, name, gear):
    steps = pt.evaluate(robot, q0, args, log, name + '-skoki')
    slow = slow_evaluation(robot, q0, args, log, name + '-wolne', gear)
    return {'name': name, 'steps': steps, 'slow': slow}


def capture_pre_test(robot, log, attempts=20):
    """Kopia pre_test po zakończeniu operacji ręcznych (sterownik kończy sekwencje ENABLE chwilę po stanie w API)."""
    for attempt in range(attempts):
        robot.wait_idle(30)
        try:
            return pt.backup_op({'action': 'capture_pre_test'}, log)
        except pt.Abort as error:
            if 'operacja ręczna w toku' not in str(error) or attempt == attempts - 1:
                raise
            time.sleep(0.5)


def wait_mode(robot, log, mode_code, timeout=20):
    """Po zapisie trybu sterownik odczytuje parametry napędu od nowa: czekamy na zgłoszony tryb i bezczynność
    (19.11.2026: przygotowanie zaraz po zapisie — „wymaga postoju, zakończonego skanu i wolnego CAN”)."""
    name = {code: n for code, n in MODES.values()}[mode_code]
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        dev = pt.api('/api/v1/can/status')['devices'][robot.axis - 1]['values']
        m = robot.manual()
        if dev.get('mode') == name and not m['busy'] and not m['fault']:
            log('mode_confirmed', axis=robot.axis, mode=name); return
        time.sleep(0.2)
    raise pt.Abort(f'J{robot.axis}: sterownik nie zgłosił trybu {name} w {timeout} s')


def write_settings(robot, log, settings):
    """Zapis nastaw testowych przez mechanizm kopii (kontener test zastępowany w całości): tylko przy ENABLE=0 badanej osi,
    odczyt kontrolny w apply_test; po zapisie trybu — potwierdzenie trybu przez sterownik; potem ponowne ENABLE."""
    robot.wait_idle()
    robot.set_enable(robot.axis, False)
    pt.api('/api/v1/drive-backup/test', {'axes': {str(robot.axis): settings}}, method='PUT')
    result = pt.backup_op({'action': 'apply_test'}, log)
    if not result.get('applied'):
        raise pt.Abort(f'apply_test nieudany: {result}')
    log('settings_written', axis=robot.axis, settings=settings)
    if 'work_mode' in settings:
        wait_mode(robot, log, settings['work_mode'])
    else:
        robot.wait_idle(30); time.sleep(1.0)   # sterownik odczytuje parametry napędu po zapisie
    robot.set_enable(robot.axis, True)
    robot.wait_idle(30)


def restore(robot, log, enable_after):
    for _ in range(150):
        m = robot.manual()
        if not m['fault'] and not m['busy'] and m['recovery_state'] in ('idle', 'idle_disabled'):
            break
        time.sleep(0.1)
    if robot.manual()['axes'][robot.axis - 1]['enabled']:
        robot.set_enable(robot.axis, False)
    result = pt.restore_pre_test(robot, log)
    if result.get('remaining_differences'):
        raise pt.Abort(f"Po powrocie do pre_test zostały różnice: {result['remaining_differences']}")
    wait_mode(robot, log, MODES['vfoc'][0])
    if enable_after:
        robot.set_enable(robot.axis, True)
        robot.wait_idle(30)
    return result


LABELS = {'work_mode': lambda v: {4: 'SR_CLOSE', 5: 'SR_vFOC'}.get(v, v), 'work_current_ma': lambda v: f'{v} mA',
          'hold_current_ratio': lambda v: f'{10 * (v + 1)}%'}


def card(robot, args, pre, mode_name, gear):
    base = pre['base']['axes'][str(robot.axis)] if pre['base'] else {}
    q0 = pre['q0'][robot.axis - 1]
    slow_s = sum(2 * SLOW_DEG / (r * 6 / gear) for r in SLOW_RPM)
    change = ', '.join(f"{k} {base.get(k, {}).get('value', '?')} → {LABELS[k](v)}" for k, v in args.settings.items())
    keep = [k for k in ('work_mode', 'work_current_ma', 'hold_current_ratio') if k not in args.settings]
    return '\n'.join([
        f'KARTA TESTU NASTAW — J{robot.axis} (adres CAN {robot.axis}): {change}; bez zmian: '
        + ', '.join(f"{k} {base.get(k, {}).get('value', '?')}" for k in keep) +
        f", mikrokrok {base.get('microstep', {}).get('value', '?')}, PID, kierunek, adres, bitrate, kalibracja",
        '  kopie: base (nietykalna) musi istnieć; świeża kopia pre_test tuż przed pierwszym zapisem; zapis tylko przy ENABLE=0 osi',
        f'  start: zerowanie MKS J1–J6, tryb izolowany (ENABLE=1 tylko J{robot.axis}, pozostałe ENABLE=0, niekomendowane);'
        f' J{robot.axis} start {q0:.2f}° (po zerowaniu 0°)',
        f'  ruchy J{robot.axis} (każda ocena): skoki ±2°, ±8° z powrotem, 10°/s, 50°/s²; wolne PTP +{SLOW_DEG:g}° i z powrotem '
        f'przy 1 / 2,5 / 5 RPM wału ({", ".join(f"{r * 6 / gear:.2f}" for r in SLOW_RPM)}°/s przegubu)',
        f'  oceny: A (pre_test) → B ({mode_name}) → A → B; ≈ 4 × (≈45 s skoki + ≈{slow_s + 15:.0f} s wolne) '
        f'+ zapisy ≈ {4 * (60 + slow_s + 15) / 60 + 1:.0f} min',
        f'  STOP i powrót do pre_test: awaria/nadzór postoju/ucieczka, nieudany program, J{robot.axis} > 9° od startu, inna oś > 3°,',
        '  oscylacja w postoju p-p > 0,15°, przeregulowanie > 0,5°, brak ustalenia (w trybie testowym: koniec rund B), Ctrl+C;',
        f'  na końcu ZAWSZE powrót do pre_test z odczytem kontrolnym i ENABLE=0 wszystkich osi; pozostawienie nastawy — osobna decyzja.',
    ])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--axis', type=int, default=1, choices=range(1, 7))
    p.add_argument('--mode', choices=('close',), default=None, help='skrót: --set work_mode=4 (SR_CLOSE)')
    p.add_argument('--set', action='append', default=[], metavar='NAZWA=WARTOŚĆ',
                   help='nastawa testowa: work_mode (4/5), work_current_ma (1–3000, ≤ znamionowy!), hold_current_ratio (0–8)')
    p.add_argument('--execute-approved', action='store_true', help='wykonaj test (po zgodzie operatora na kartę)')
    args = p.parse_args()
    args.steps, args.velocity, args.acceleration, args.isolated = [2.0, 8.0], 10.0, 50.0, True
    args.settings = {}
    if args.mode:
        args.settings['work_mode'] = MODES[args.mode][0]
    for item in args.set:
        name, _, value = item.partition('=')
        if name not in LABELS or not value.lstrip('-').isdigit():
            sys.exit(f'--set {item}: dozwolone work_mode, work_current_ma, hold_current_ratio z wartością całkowitą')
        args.settings[name] = int(value)
    if not args.settings:
        sys.exit('Podaj --mode close albo --set NAZWA=WARTOŚĆ')
    mode_name = ', '.join(f'{k}={LABELS[k](v)}' for k, v in args.settings.items())
    out = q.ROOT / 'log' / f"drive-mode-{datetime.now().strftime('%Y%m%d-%H%M%S')}-J{args.axis}"
    log = pt.Log(out) if args.execute_approved else (lambda *a, **k: None)
    robot = pt.Robot(args.axis, log if args.execute_approved else None); robot.running = False
    gear = float(robot.gear[args.axis - 1])
    pre = pt.preflight(robot, args)
    print(card(robot, args, pre, mode_name, gear))
    run = q.api('/api/v1/program-run')
    if run.get('active'):
        pre['problems'].append(f"trwa przebieg programu ({run.get('state')})")
    if pre['problems']:
        sys.exit('PREFLIGHT NIEUDANY:\n  ' + '\n  '.join(pre['problems']))
    print('Preflight: OK (bez ruchu i zapisu).')
    if not args.execute_approved:
        return
    report = {'axis': args.axis, 'mode': mode_name, 'evaluations': []}; wrote = False
    try:
        zero = q.Robot(log); zero.enable_all(False); zero.zero_and_confirm(); zero.enable_all(False)
        pre = pt.preflight(robot, args); q0 = pre['q0']; report['q0'] = q0
        if pre['problems']:
            raise pt.Abort('Preflight po zerowaniu: ' + '; '.join(pre['problems']))
        report['pre_test'] = capture_pre_test(robot, log)
        robot.isolate()
        for round_ in (1, 2):
            report['evaluations'].append(('A', evaluate(robot, q0, args, log, f'A{round_}', gear)))
            wrote = True; write_settings(robot, log, args.settings)
            b = evaluate(robot, q0, args, log, f'B{round_}', gear); report['evaluations'].append(('B', b))
            restore(robot, log, enable_after=True)
            if b['steps']['hard']:
                log('hard_violation', mode=mode_name, hard=b['steps']['hard']); report['hard'] = b['steps']['hard']; break
        log('finished')
    except (pt.Abort, KeyboardInterrupt) as error:
        log('abort', reason=repr(error)); report['abort'] = repr(error)
        if robot.running:
            try:
                pt.api('/api/v1/motion/stop', {})
            except Exception:
                pass
    finally:
        if wrote:
            try:
                report['restore'] = restore(robot, log, enable_after=False)
            except Exception as error:
                log('RESTORE_FAILED', error=repr(error))
                print('!!! Powrót do pre_test nieudany — ręcznie: POST /api/v1/drive-backup/operations '
                      f'{{"action":"restore","source":"pre_test","axes":[{args.axis}]}}', file=sys.stderr)
        try:
            q.Robot(log).enable_all(False)
        except Exception as error:
            log('disable_failed', error=repr(error))
        state = pt.api('/api/v1/drive-backup')['containers']
        report['sha256'] = {k: (v or {}).get('sha256') for k, v in state.items() if isinstance(v, dict)}
        log.save('report.json', report)
        lines = ['| Ocena | Skoki: koszt / twarde | Wolne: część zmienna [° wału] | maks. [° wału] | zatrzymania [%] |', '|---|---|---|---|---|']
        for kind, e in report['evaluations']:
            s = e['slow']
            lines.append(f"| {e['name']} ({kind}) | {e['steps']['cost']} / {', '.join(e['steps']['hard']) or '—'} | "
                         f"{s.get('fluct_shaft_deg', '—')} | {s.get('max_shaft_deg', '—')} | {s.get('stall_pct', '—')} |")
        (out / 'summary.md').write_text('\n'.join(lines) + '\n')
        print('\n'.join(lines) + f'\n\nRaport: {out / "report.json"}')


if __name__ == '__main__':
    main()
