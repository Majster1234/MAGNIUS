#!/usr/bin/env python3
"""Strojenie i weryfikacja nastaw PID napędu MKS (jedna oś) — po zmianie mechaniki.

Tryby (domyślnie TYLKO preflight i karta testu; ruch wyłącznie z --execute-approved):
  tune    — linia bazowa → przeszukiwanie PID vFOC (Kp, Ki, Kd, Kv) w granicach [lo, hi]× kopii base →
            weryfikacja przeplatana (bazowe/najlepsze ×2) → zostawienie najlepszego albo powrót do pre_test.
  verify  — weryfikacja zmiany z bieżącego kontenera test: linia bazowa → apply_test → ocena → decyzja.

Każda ingerencja w nastawy: kopia base musi istnieć, przed pierwszym zapisem automatyczna kopia pre_test,
zapis wyłącznie przez /api/v1/drive-backup (odczyt kontrolny), przy przerwaniu automatyczny powrót do pre_test.

Tryb izolowany (domyślny): ENABLE=1 tylko strojona oś, pozostałe ENABLE=0; program PTP z maską osi
(participating_axes) — inne osie nie są komendowane ani brane pod uwagę w ocenie (jedynie zabezpieczenie: przesunięcie
wyłączonej osi > 3° przerywa test, np. opadanie pod grawitacją).
Ocena jakości = ruchy PTP tylko badanej osi (tam i z powrotem, ±małe i ±duże przemieszczenie), pomiar ze śladu wału
(~/.local/state/robot/shaft-trace). Wskaźnik (mniejszy = lepszy) na ruch:
  1·(czas ustalenia/0,5 s) + 2·(przeregulowanie/ε) + 3·(oscylacja w postoju p-p/ε) + 3·(uchyb ustalony/ε) + 0,5·(RMS śledzenia/ε)
Twarde odrzucenie: awaria/nieudany program, oscylacja p-p > 0,15°, przeregulowanie > 0,5°, brak ustalenia w oknie.
"""
import argparse, json, math, os, re, struct, sys, time, uuid, zlib
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler

HOST = os.environ.get('ROBOT_API', 'http://192.168.1.69:8080')
TRACE = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'robot/shaft-trace'
RECORD = struct.Struct('<QQQQQqII')
PID_KEYS = ('kp', 'ki', 'kd', 'kv')
BAND_DEG = 0.05          # ε: pasmo ustalenia [° przegubu]
SETTLE_OBS_S = 1.5       # obserwacja postoju po zakończeniu programu
HARD_PP_DEG, HARD_OS_DEG = 0.15, 0.5
WEIGHTS = dict(settle=1.0, overshoot=2.0, hold=3.0, steady=3.0, tracking=0.5)
op = build_opener(ProxyHandler({}))


class Abort(RuntimeError):
    pass


class Hard(Abort):
    """Kandydat nie do przyjęcia (np. oscylacje osi blokują przygotowanie ruchu) — powrót do najlepszego PID."""


def at_rest_problem(text):
    return re.search(r'post[oó]j', str(text), re.IGNORECASE) is not None


def api(path, body=None, method=None, timeout=4):
    req = Request(HOST + path, data=None if body is None else json.dumps(body).encode(), method=method,
                  headers={'Content-Type': 'application/json'})
    try:
        with op.open(req, timeout=timeout) as r:
            return json.load(r)
    except HTTPError as e:
        raise Abort(f'{method or ("POST" if body is not None else "GET")} {path}: HTTP {e.code} {e.read().decode(errors="replace")[:300]}')


class Log:
    def __init__(self, directory):
        self.dir = directory; directory.mkdir(parents=True, exist_ok=True)
        self.stream = (directory / 'events.jsonl').open('a', encoding='utf-8')

    def __call__(self, event, **data):
        row = {'utc_ns': time.time_ns(), 'event': event, **data}
        self.stream.write(json.dumps(row, ensure_ascii=False) + '\n'); self.stream.flush()
        brief = {k: v for k, v in data.items() if k not in ('metrics', 'moves', 'status')}
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {event} {json.dumps(brief, ensure_ascii=False)[:240]}", flush=True)

    def save(self, name, value):
        (self.dir / name).write_text(json.dumps(value, ensure_ascii=False, indent=1))


# ---------- ślad wału ----------
def ring(name, t0_ns, t1_ns):
    data = (TRACE / name).read_bytes(); out = []
    for off in range(4096, len(data) - RECORD.size + 1, RECORD.size):
        utc = struct.unpack_from('<Q', data, off + 32)[0]
        if not t0_ns <= utc <= t1_ns:
            continue
        rec = data[off:off + RECORD.size]
        if zlib.crc32(rec[:-4]) != struct.unpack_from('<I', rec, RECORD.size - 4)[0]:
            continue
        index, _, _, _, utc, pos, _, _ = RECORD.unpack(rec)
        if index:
            out.append((utc, pos / 100.0))  # stopnie wału
    out.sort(); return out


# ---------- stan i bezpieczeństwo ----------
class Robot:
    def __init__(self, axis, log):
        self.axis, self.log = axis, log
        cfg = api('/api/v1/configuration')['active']['document']['values']
        self.gear = [cfg[f'joint.{j}.gear_ratio'] for j in range(1, 7)]
        self.direction = [cfg[f'joint.{j}.direction'] for j in range(1, 7)]
        self.offset = [cfg[f'joint.{j}.zero_offset'] or 0 for j in range(1, 7)]

    def joint(self, j, shaft_deg):
        return self.direction[j - 1] * shaft_deg / self.gear[j - 1] + self.offset[j - 1]

    def manual(self):
        return api('/api/v1/manual/status')

    def q(self):
        return [a['joint_position_degrees'] for a in self.manual()['axes']]

    def set_enable(self, axis, enabled, timeout=8):
        api(f'/api/v1/manual/axes/{axis}/enable', {'enabled': enabled})
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.manual(); a = m['axes'][axis - 1]
            if m['fault']:
                raise Abort(f'Awaria przy ENABLE J{axis}: {m["message"]}')
            if not m['busy'] and a['enable_known'] and a['enabled'] == enabled:
                return
            time.sleep(0.1)
        raise Abort(f'Brak potwierdzenia ENABLE={int(enabled)} J{axis}')

    def isolate(self):
        """Tylko strojona oś z ENABLE=1; pozostałe wyłączane pojedynczo (postój potwierdza sterowanie)."""
        for j, a in enumerate(self.manual()['axes'], start=1):
            if j != self.axis and a['enabled']:
                self.set_enable(j, False)
        if not self.manual()['axes'][self.axis - 1]['enabled']:
            self.set_enable(self.axis, True)
        if self.log:
            self.log('isolated', enabled=[a['enabled'] for a in self.manual()['axes']])

    def recover(self, isolated):
        """Po awarii (np. limit potwierdzenia postoju): zakończone odzyskiwanie, ponownie ENABLE strojonej osi."""
        end = time.monotonic() + 15
        while time.monotonic() < end:
            m = self.manual()
            if not m['fault'] and not m['busy'] and m['recovery_state'] in ('idle', 'idle_disabled'):
                break
            time.sleep(0.2)
        else:
            raise Abort('Odzyskiwanie po awarii nie zakończyło się')
        if isolated:
            self.isolate()
        elif not all(a['enabled'] for a in self.manual()['axes']):
            raise Abort('Po awarii osie wyłączone — tryb nieizolowany wymaga decyzji operatora')

    def wait_idle(self, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.manual(); mo = api('/api/v1/motion/status')
            if m['fresh'] and not m['busy'] and not m['fault'] and not mo['busy']:
                return m
            time.sleep(0.1)
        raise Abort('Brak stanu bezczynności')


def preflight(robot, args):
    m = robot.manual(); c = api('/api/v1/can/status'); t = api('/api/v1/diagnostics/trace/status')
    mo = api('/api/v1/motion/status'); db = api('/api/v1/drive-backup'); ctx = api('/api/v1/planning/context')
    problems = []
    if not m['fresh'] or m['busy'] or m['fault'] or mo['busy']:
        problems.append('sterowanie niegotowe (busy/fault)')
    if m['recovery_state'] not in ('idle', 'idle_disabled'):
        problems.append('trwa odzyskiwanie')
    for a in m['axes']:
        needs_enable = not args.isolated  # tryb izolowany: ENABLE strojonej osi włącza narzędzie po kopii pre_test
        if not (a['available'] and a['calibration_valid'] and a['joint_position_known']) or (needs_enable and not a['enabled']):
            problems.append(f"J{a['axis']}: wymagane {'ENABLE=1, ' if needs_enable else ''}kalibracja i znana pozycja")
    if not (all(ctx['available']) and all(s == 'measured' for s in ctx['seed_sources'])):
        problems.append('planer bez sześciu zmierzonych pozycji')
    if not c['fresh'] or c['summary']['values']['fault'] or c['summary']['values']['bitrate'] != 1000000:
        problems.append('CAN niegotowy')
    dev = c['devices'][robot.axis - 1]['values']
    if dev['mode'] != 'SR_vFOC':
        problems.append(f"J{robot.axis}: strojenie PID vFOC wymaga trybu SR_vFOC (jest {dev['mode']})")
    if not (t['recording'] and t['connected']):
        problems.append('ślad wału nie jest nagrywany')
    base = db['containers']['base']
    if not base or not base.get('valid'):
        problems.append('brak ważnej, nietykalnej kopii base — zapis nastaw zablokowany')
    if db['operation']['phase'] == 'running':
        problems.append('trwa operacja na nastawach')
    q0 = [a['joint_position_degrees'] for a in m['axes']]
    a = m['axes'][robot.axis - 1]
    big = max(args.steps)
    if not (a['joint_lower_degrees'] + big + 2 <= q0[robot.axis - 1] <= a['joint_upper_degrees'] - big - 2):
        problems.append(f'J{robot.axis}: za mały zapas do limitu mechanicznego dla ±{big}°')
    return {'q0': q0, 'base': base, 'problems': problems, 'context': ctx}


def card(robot, args, pre):
    axis = robot.axis; q0 = pre['q0'][axis - 1]
    base = pre['base']['axes'][str(axis)] if pre['base'] else None
    moves = moves_plan(args)
    est_eval = 2 * len(moves) * 3.6  # ruch i powrót
    pid = (base['vfoc_kp_ki']['value'] + base['vfoc_kd_kv']['value']) if base else None
    lines = [
        (f'KARTA TESTU — tryb {args.mode}, oś J{axis} (adres CAN {axis}); TRYB IZOLOWANY: pozostałe osie ENABLE=0, '
         'niekomendowane, poza oceną (przerwanie, gdy wyłączona oś przesunie się > 3°)') if args.isolated else
        f'KARTA TESTU — tryb {args.mode}, oś J{axis} (adres CAN {axis}); pozostałe osie stoją (PTP z celem = pozycja bieżąca)',
        f'  start J{axis} = {q0:.3f}°; ruchy: ' + ', '.join(f'{d:+.1f}°' for d in moves) + ' (każdy wraca do startu)',
        f'  prędkość {args.velocity}°/s, przyspieszenie {args.acceleration}°/s²; maks. wychylenie ±{max(args.steps)}°',
        f'  ocena: {2 * len(moves)} ruchów (z powrotami) ≈ {est_eval:.0f} s; obserwacja postoju {SETTLE_OBS_S} s po każdym ruchu',
    ]
    if args.mode == 'tune':
        lines.append(f'  PID vFOC base Kp/Ki/Kd/Kv = {pid}; zakres {args.lo}–{args.hi}× base; '
                     f'maks. {args.max_candidates} kandydatów (+ 2 linie bazowe + 4 weryfikacji) ≈ '
                     f'{(args.max_candidates + 6) * est_eval / 60:.0f} min')
    lines += [
        '  kopie: base musi istnieć; automatyczna kopia pre_test przed pierwszym zapisem; zapis PID przy ENABLE=1 bez ruchu',
        f'  STOP i powrót do pre_test: awaria/nadzór postoju, nieudany program, J{axis} poza ±{max(args.steps) + 1}° od startu,',
        f'  inna oś > {"3" if args.isolated else "0,5"}° od startu, oscylacja p-p > 0,15°, przeregulowanie > 0,5°, Ctrl+C;',
        '  brak potwierdzenia postoju w 3 s (awaria) = odrzucenie kandydata, ponowne ENABLE strojonej osi, powrót do najlepszego PID',
    ]
    return '\n'.join(lines)


def moves_plan(args):
    out = []
    for s in args.steps:
        out += [s, -s]  # przemieszczenie względem startu; po każdym ruch powrotny
    return out


# ---------- ruch PTP jednej osi ----------
def run_ptp(robot, q0, target_axis_deg, args, log, label):
    ctx = api('/api/v1/planning/context')
    seed = ctx['seed_degrees']
    target = list(q0); target[robot.axis - 1] = target_axis_deg
    request = {'model_id': ctx['model_id'], 'configuration_revision': ctx['configuration_revision'],
               'settings_revision': ctx['settings_revision'], 'seed_degrees': seed, 'simulated_start': [False] * 6,
               'preview_velocity_degrees_s': args.velocity, 'preview_acceleration_degrees_s2': args.acceleration,
               'motion_type': 'PTP', 'target_degrees': target}
    job = api('/api/v1/planning/moves', request); end = time.monotonic() + 15
    while job['status'] == 'running':
        if time.monotonic() > end:
            raise Abort('Timeout planera')
        time.sleep(0.05); job = api('/api/v1/planning/jobs/' + job['id'])
    if job['status'] != 'succeeded':
        raise Abort(f'Planowanie nieudane: {job}')
    body = {'plan_id': job['plan_id']}
    if args.isolated:
        body['participating_axes'] = [robot.axis]
    api('/api/v1/motion/prepare', body); end = time.monotonic() + 15
    while True:
        st = api('/api/v1/motion/status'); phase = st['preparation']['state']
        if phase in ('failed', 'cancelled'):
            kind = Hard if at_rest_problem(st['preparation']) else Abort
            raise kind(f"Przygotowanie nieudane: {st['preparation']}")
        if phase == 'ready' and st['state'] == 'ready':
            break
        if time.monotonic() > end:
            raise Abort('Timeout przygotowania programu')
        time.sleep(0.05)
    prepared = api('/api/v1/motion/prepared')
    t_exec = time.time_ns()
    api('/api/v1/motion/execute', {'program_id': prepared['program_id'], 'execution_key': f'pid-{uuid.uuid4().hex}'})
    robot.running = True
    end = time.monotonic() + 30
    while True:
        guard(robot, q0, args)
        st = api('/api/v1/motion/status')
        if st['state'] == 'succeeded' and not st['busy']:
            break
        if st['state'] in ('failed', 'fault', 'cancelled', 'aborted'):
            robot.running = False
            kind = Hard if at_rest_problem(st['message']) else Abort
            raise kind(f"Program zakończony: {st['state']} {st['message']}")
        if time.monotonic() > end:
            raise Abort('Program przekroczył 30 s')
        time.sleep(0.04)
    robot.running = False
    t_done = time.time_ns()
    end = time.monotonic() + SETTLE_OBS_S
    while time.monotonic() < end:
        guard(robot, q0, args); time.sleep(0.05)
    return {'label': label, 'target_deg': target_axis_deg, 'start_deg': robot_q_axis_from(seed, robot.axis),
            't_exec_ns': t_exec, 't_done_ns': t_done, 't_end_ns': time.time_ns()}


def robot_q_axis_from(seed, axis):
    return seed[axis - 1]


def guard(robot, q0, args):
    m = robot.manual()
    if m['fault']:
        raise Abort('Awaria: ' + str(m['message']))
    for j, a in enumerate(m['axes'], start=1):
        if not a.get('joint_position_known'):
            continue
        limit = max(args.steps) + 1.0 if j == robot.axis else (3.0 if args.isolated else 0.5)
        if abs(a['joint_position_degrees'] - q0[j - 1]) > limit:
            raise Abort(f"J{j} poza zakresem karty: {a['joint_position_degrees']:.3f}° (start {q0[j - 1]:.3f}°)")


def metrics(robot, move):
    axis = robot.axis
    shaft = ring(f'axis-{axis}.bin', move['t_exec_ns'] - 200_000_000, move['t_end_ns'])
    ref = ring(f'reference-{axis}.bin', move['t_exec_ns'] - 200_000_000, move['t_end_ns'])
    meas = [(t, robot.joint(axis, v)) for t, v in shaft]
    refj = [(t, robot.joint(axis, v)) for t, v in ref]
    qf, q_start = move['target_deg'], move['start_deg']
    sign = 1 if qf >= q_start else -1
    if len(meas) < 10 or len(refj) < 5:
        return {'valid': False, 'reason': 'za mało próbek śladu wału'}
    # Koniec referencji: od końca, ostatnia próbka poza celem (ruch powrotny zaczyna się od celu poprzedniego ruchu).
    active = [(t, v) for t, v in refj if t >= move['t_exec_ns']]
    if not active:
        return {'valid': False, 'reason': 'brak referencji w trakcie programu'}
    away = [i for i, (t, v) in enumerate(active) if abs(v - qf) > 2e-3]
    t_ref_end = active[away[-1] + 1][0] if away and away[-1] + 1 < len(active) else active[-1][0] if away else active[0][0]
    after = [(t, v) for t, v in meas if t >= t_ref_end]
    during = [(t, v) for t, v in meas if move['t_exec_ns'] <= t < t_ref_end]
    outside = [t for t, v in after if abs(v - qf) > BAND_DEG]
    settle = ((outside[-1] - t_ref_end) / 1e9) if outside else 0.0
    overshoot = max([0.0] + [sign * (v - qf) for t, v in after])
    tail = [v for t, v in after if t >= move['t_end_ns'] - 1_000_000_000]
    last = [v for t, v in after if t >= move['t_end_ns'] - 500_000_000]
    hold_pp = (max(tail) - min(tail)) if tail else float('inf')
    steady = abs(sum(last) / len(last) - qf) if last else float('inf')

    def ref_at(t):
        for (t0, v0), (t1, v1) in zip(refj, refj[1:]):
            if t0 <= t <= t1:
                return v0 + (v1 - v0) * (t - t0) / max(t1 - t0, 1)
        return None
    errs = [v - r for t, v in during if (r := ref_at(t)) is not None]
    tracking = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else 0.0
    settled = bool(after) and not (outside and outside[-1] >= after[-1][0] - 300_000_000)
    hard = []
    if hold_pp > HARD_PP_DEG:
        hard.append(f'oscylacja p-p {hold_pp:.3f}°')
    if overshoot > HARD_OS_DEG:
        hard.append(f'przeregulowanie {overshoot:.3f}°')
    if not settled:
        hard.append('brak ustalenia w oknie obserwacji')
    cost = (WEIGHTS['settle'] * settle / 0.5 + WEIGHTS['overshoot'] * overshoot / BAND_DEG + WEIGHTS['hold'] * hold_pp / BAND_DEG +
            WEIGHTS['steady'] * steady / BAND_DEG + WEIGHTS['tracking'] * tracking / BAND_DEG)
    return {'valid': True, 'settle_s': round(settle, 3), 'overshoot_deg': round(overshoot, 4), 'hold_pp_deg': round(hold_pp, 4),
            'steady_deg': round(steady, 4), 'tracking_rms_deg': round(tracking, 4), 'cost': round(cost, 3), 'hard': hard,
            'samples': len(meas)}


def evaluate(robot, q0, args, log, name):
    """Pełny zestaw ruchów; zwraca średni wskaźnik i metryki ruchów (wyjątek Abort przerywa test)."""
    robot.wait_idle()
    results = []
    home = q0[robot.axis - 1]
    for d in moves_plan(args):
        for label, tgt in ((f'{d:+.1f}', home + d), (f'{d:+.1f}→0', home)):
            mv = run_ptp(robot, q0, tgt, args, log, label)
            mv['metrics'] = metrics(robot, mv)
            results.append(mv)
            robot.wait_idle()
    valid = [m['metrics'] for m in results if m['metrics'].get('valid')]
    if len(valid) != len(results):
        raise Abort('Niekompletny ślad wału — ocena nieważna')
    hard = sorted({h for m in valid for h in m['hard']})
    summary = {'name': name, 'cost': round(sum(m['cost'] for m in valid) / len(valid), 3), 'hard': hard,
               **{k: round(sum(m[k] for m in valid) / len(valid), 4) for k in ('settle_s', 'overshoot_deg', 'hold_pp_deg', 'steady_deg', 'tracking_rms_deg')},
               'max_hold_pp_deg': max(m['hold_pp_deg'] for m in valid), 'max_overshoot_deg': max(m['overshoot_deg'] for m in valid)}
    log('evaluation', **{k: v for k, v in summary.items()}, moves=[{'label': m['label'], **m['metrics']} for m in results])
    return summary


# ---------- nastawy przez mechanizm kopii ----------
def backup_op(body, log, timeout=60):
    started = api('/api/v1/drive-backup/operations', body)
    end = time.monotonic() + timeout
    while True:
        st = api('/api/v1/drive-backup')['operation']
        if st['id'] == started['id'] and st['phase'] != 'running':
            log('drive_backup', action=body['action'], success=st['success'], message=st['message'])
            if not st['success']:
                raise Abort(f"{body['action']}: {st['message']} {json.dumps(st['result'], ensure_ascii=False)[:400]}")
            return st['result']
        if time.monotonic() > end:
            raise Abort(f"{body['action']}: przekroczony czas")
        time.sleep(0.2)


def current_pid(axis):
    st = api('/api/v1/drive-backup')
    pre = st['containers']['pre_test']['axes'][str(axis)]
    return dict(zip(PID_KEYS, pre['vfoc_kp_ki']['value'] + pre['vfoc_kd_kv']['value']))


def apply_pid(axis, pid, log):
    api('/api/v1/drive-backup/test', {'axes': {str(axis): {'vfoc_kp_ki': [pid['kp'], pid['ki']],
                                                         'vfoc_kd_kv': [pid['kd'], pid['kv']]}}}, method='PUT')
    result = backup_op({'action': 'apply_test'}, log)
    if not result.get('applied'):
        raise Abort(f'apply_test nieudany: {result}')
    log('pid_applied', axis=axis, pid=pid)


def restore_pre_test(robot, log):
    for _ in range(100):  # po awarii czekamy na zakończenie odzyskiwania
        m = robot.manual()
        if not m['fault'] and not m['busy'] and m['recovery_state'] in ('idle', 'idle_disabled'):
            break
        time.sleep(0.1)
    result = backup_op({'action': 'restore', 'source': 'pre_test', 'axes': [robot.axis]}, log)
    log('restored_pre_test', remaining=result.get('remaining_differences'))
    return result


# ---------- przeszukiwanie ----------
def tune(robot, q0, args, log, base_pid):
    bounds = {k: (max(0, round(base_pid[k] * args.lo)), min(1024, round(base_pid[k] * args.hi))) for k in PID_KEYS}
    start = current_pid(robot.axis)
    log('search_bounds', bounds=bounds, start=start)
    b1 = evaluate(robot, q0, args, log, 'baseline-1'); b2 = evaluate(robot, q0, args, log, 'baseline-2')
    if b1['hard'] or b2['hard']:
        log('baseline_hard', hard=b1['hard'] + b2['hard'])
    base_cost = (b1['cost'] + b2['cost']) / 2
    noise = abs(b1['cost'] - b2['cost']) / max(base_cost, 1e-9)
    margin = max(0.05, 2 * noise)
    log('baseline', cost=base_cost, noise=round(noise, 3), margin=round(margin, 3))
    best, best_cost, factor, evaluated = dict(start), base_cost, 1.4, 0
    history = [{'pid': dict(start), 'cost': base_cost, 'hard': b1['hard'] + b2['hard'], 'note': 'baseline'}]
    while factor >= 1.1 and evaluated < args.max_candidates:
        improved = False
        for key in PID_KEYS:
            for mult in (factor, 1 / factor):
                if evaluated >= args.max_candidates:
                    break
                cand = dict(best); cand[key] = int(round(min(max(best[key] * mult, bounds[key][0]), bounds[key][1])))
                if cand == best or any(h['pid'] == cand for h in history):
                    continue
                apply_pid(robot.axis, cand, log)
                try:
                    res = evaluate(robot, q0, args, log, f'cand-{evaluated + 1}')
                except Hard as error:
                    res = {'cost': float('inf'), 'hard': [str(error)]}
                    log('candidate_blocked', pid=cand, reason=str(error))
                    robot.recover(args.isolated)
                evaluated += 1
                history.append({'pid': cand, 'cost': res['cost'], 'hard': res['hard']})
                if res['hard']:
                    apply_pid(robot.axis, best, log)  # natychmiast z powrotem do najlepszego
                    time.sleep(1.0)
                    continue
                if res['cost'] < best_cost * (1 - margin / 2):
                    best, best_cost, improved = cand, res['cost'], True
                    log('new_best', pid=best, cost=best_cost)
                    break
            # po zmianie najlepszego następny parametr startuje od nowego punktu
        if not improved:
            factor = math.sqrt(factor)
    apply_pid(robot.axis, best, log)
    return best, base_cost, margin, history


def verify_pair(robot, q0, args, log, before_apply, after_apply, margin):
    """Weryfikacja przeplatana A/B/A/B; zwraca (decyzja, wyniki)."""
    runs = []
    for i in range(2):
        before_apply(); runs.append(('before', evaluate(robot, q0, args, log, f'verify-before-{i + 1}')))
        after_apply(); runs.append(('after', evaluate(robot, q0, args, log, f'verify-after-{i + 1}')))
    before = [r for k, r in runs if k == 'before']; after = [r for k, r in runs if k == 'after']
    cb = sum(r['cost'] for r in before) / 2; ca = sum(r['cost'] for r in after) / 2
    hard = sorted({h for r in after for h in r['hard']})
    accept = not hard and ca < cb * (1 - margin)
    log('verification', before_cost=cb, after_cost=ca, improvement=round(1 - ca / cb, 3) if cb else None,
        margin=margin, hard=hard, accept=accept)
    return accept, {'before_cost': cb, 'after_cost': ca, 'hard': hard, 'runs': runs}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=('tune', 'verify'))
    p.add_argument('--axis', type=int, required=True, choices=range(1, 7))
    p.add_argument('--steps', type=float, nargs='+', default=[2.0, 8.0], help='przemieszczenia [° przegubu]')
    p.add_argument('--velocity', type=float, default=10.0, help='°/s przegubu')
    p.add_argument('--acceleration', type=float, default=50.0, help='°/s² przegubu')
    p.add_argument('--lo', type=float, default=0.5); p.add_argument('--hi', type=float, default=2.0)
    p.add_argument('--max-candidates', type=int, default=20)
    p.add_argument('--no-isolated', dest='isolated', action='store_false',
                   help='ruch programu ze wszystkimi osiami (domyślnie: tylko strojona oś, pozostałe ENABLE=0)')
    p.add_argument('--execute-approved', action='store_true', help='wykonaj ruch (po zgodzie operatora na kartę)')
    args = p.parse_args()
    if not (0.25 <= args.lo < 1 < args.hi <= 2.0) or max(args.steps) > 10 or args.velocity > 20 or args.acceleration > 100:
        sys.exit('Parametry poza bezpiecznym zakresem narzędzia (lo≥0,25, hi≤2, krok≤10°, v≤20°/s, a≤100°/s²)')
    out = Path(__file__).resolve().parents[1] / 'log' / f"pid-{args.mode}-{datetime.now().strftime('%Y%m%d-%H%M%S')}-J{args.axis}"
    log = Log(out) if args.execute_approved else None
    robot = Robot(args.axis, log); robot.running = False
    pre = preflight(robot, args)
    print(card(robot, args, pre))
    if pre['problems']:
        sys.exit('PREFLIGHT NIEUDANY:\n  ' + '\n  '.join(pre['problems']))
    print('Preflight: OK (bez ruchu).')
    if not args.execute_approved:
        return
    q0 = pre['q0']; wrote = False; report = {'mode': args.mode, 'axis': args.axis, 'q0': q0, 'args': vars(args)}
    base_pid = dict(zip(PID_KEYS, pre['base']['axes'][str(args.axis)]['vfoc_kp_ki']['value'] +
                        pre['base']['axes'][str(args.axis)]['vfoc_kd_kv']['value']))
    try:
        log('start', q0=q0, base_pid=base_pid)
        report['pre_test'] = backup_op({'action': 'capture_pre_test'}, log)  # obowiązkowa kopia przed ingerencją
        if args.isolated:
            robot.isolate()
        before_pid = current_pid(args.axis); report['before_pid'] = before_pid
        if args.mode == 'tune':
            wrote = True
            best, base_cost, margin, history = tune(robot, q0, args, log, base_pid)
            report.update(best_pid=best, baseline_cost=base_cost, history=history)
            if best == before_pid:
                accept, ver = False, {'note': 'brak lepszego kandydata'}
            else:
                accept, ver = verify_pair(robot, q0, args, log, lambda: apply_pid(args.axis, before_pid, log),
                                          lambda: apply_pid(args.axis, best, log), margin)
        else:
            test = api('/api/v1/drive-backup')['containers']['test']
            if not test or not test['axes'].get(str(args.axis)):
                raise Abort('Brak nastaw testowych dla tej osi w kontenerze test')
            b1 = evaluate(robot, q0, args, log, 'baseline-1'); b2 = evaluate(robot, q0, args, log, 'baseline-2')
            margin = max(0.05, 2 * abs(b1['cost'] - b2['cost']) / max((b1['cost'] + b2['cost']) / 2, 1e-9))
            wrote = True
            accept, ver = verify_pair(robot, q0, args, log, lambda: restore_pre_test(robot, log),
                                      lambda: backup_op({'action': 'apply_test'}, log), margin)
        report['verification'] = ver; report['accepted'] = accept
        if not accept:
            report['restore'] = restore_pre_test(robot, log)
        log('finished', accepted=accept)
    except (Abort, KeyboardInterrupt) as error:
        log('abort', reason=repr(error))
        if robot.running:
            try:
                api('/api/v1/motion/stop', {}); log('motion_stop_sent')
            except Exception as stop_error:
                log('motion_stop_failed', error=repr(stop_error))
        report['abort'] = repr(error)
        if wrote:
            try:
                report['restore'] = restore_pre_test(robot, log)
            except Exception as restore_error:
                log('RESTORE_FAILED', error=repr(restore_error))
                print('!!! Przywrócenie pre_test nieudane — wykonaj ręcznie: POST /api/v1/drive-backup/operations '
                      f'{{"action":"restore","source":"pre_test","axes":[{args.axis}]}}', file=sys.stderr)
        report['accepted'] = False
    finally:
        log.save('report.json', report)
        print(f'Raport: {out / "report.json"}')
    sys.exit(0 if report.get('accepted') else 2)


if __name__ == '__main__':
    main()
