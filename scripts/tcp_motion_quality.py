#!/usr/bin/env python3
"""Procedura jakości ruchu TCP w trybie planowanego ruchu (docs/tcp_motion_quality_procedure.md).

Przebieg jednego testu (dla każdej wartości --acc):
  1. ENABLE=0 wszystkich osi, zapis `motion.f5_acceleration_code` (puste = parametry wykonania).
  2. ENABLE=1, zerowanie pozycji sterowników MKS (komenda 92, J1–J6), ENABLE=0 i potwierdzenie odniesienia → przeguby 0°.
  3. ENABLE=1, PTP przygotowawczy do pozy startowej (domyślnie 0/0/0/0/45/0°).
  4. 12 ruchów: dystans 20 i 400 mm × prędkość 5/15/40 mm/s; w każdej parze PTP start→cel i LIN cel→start.
  5. ENABLE=0, analiza z nagrania CAN (`log/manual-can`, zdarzenia sync_feedback trybu SYNC).

Bez --execute wykonuje się tylko suchy przebieg: IK i planowanie podglądu wszystkich ruchów z pozy startowej,
bez zerowania, ENABLE i ruchu. Ruch fizyczny wymaga zgody operatora (AGENTS.md); stała zgoda obowiązuje
wyłącznie przy rozsprzęglonej mechanice.
"""
import argparse, glob, json, math, os, re, time, uuid
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler

import numpy as np

HOST = os.environ.get('ROBOT_API', 'http://192.168.1.69:8080')
ROOT = Path(__file__).resolve().parent.parent
CAN_LOG = Path(os.environ.get('ROBOT_TRACE_DIR', str(ROOT / 'log/manual-can')))
F5_KEY = 'motion.f5_acceleration_code'
START_POSE = [0.0, 0.0, 0.0, 0.0, 45.0, 0.0]
DISTANCES_MM = (20.0, 400.0)
SPEEDS_MM_S = (5.0, 15.0, 40.0)
TCP_ACCELERATION = 100.0            # mm/s² (minimum planera LIN; konfiguracja tcp.linear_acceleration)
TCP_ANGULAR_VELOCITY = 30.0         # °/s (orientacja stała)
SETUP_VELOCITY, SETUP_ACCELERATION = 20.0, 100.0   # PTP do pozy startowej [°/s, °/s²]
FINAL_APPROACH_SOFT_DEG = 5.0      # nieudany dolot do tego uchybu = ruch niezaliczony, powyżej = przerwanie serii
SETTLE_OBS_S = 1.0                  # obserwacja po zakończeniu programu (wliczana do analizy)
# Kierunki ruchu w układzie bazy; wybierany pierwszy wykonalny dla wszystkich ruchów.
DIRECTIONS = [(1, 1, -1), (-1, 1, -1), (1, -1, -1), (-1, -1, -1), (1, 1, 1), (-1, 1, 1), (1, -1, 1), (-1, -1, 1),
              (1, 0, -1), (-1, 0, -1), (0, 1, -1), (0, -1, -1), (0, 2, -1), (0, -2, -1), (-1, 2, -1), (-1, -2, -1),
              (1, 0, 0), (0, 1, 0), (0, -1, 0), (-1, 1, 0), (-1, -1, 0), (0, 0, -1)]
JOINT_AXES = ((0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0))   # robot_configuration/schema.py AXES
op = build_opener(ProxyHandler({}))


class Abort(RuntimeError):
    pass


def api(path, body=None, method=None, timeout=10):
    req = Request(HOST + path, data=None if body is None else json.dumps(body).encode(), method=method,
                  headers={'Content-Type': 'application/json'})
    try:
        with op.open(req, timeout=timeout) as r:
            return json.load(r)
    except HTTPError as e:
        raise Abort(f'{method or ("POST" if body is not None else "GET")} {path}: HTTP {e.code} '
                    f'{e.read().decode(errors="replace")[:300]}')


class Log:
    def __init__(self, directory):
        self.dir = directory; directory.mkdir(parents=True, exist_ok=True)
        self.stream = (directory / 'events.jsonl').open('a', encoding='utf-8')

    def __call__(self, event, **data):
        self.stream.write(json.dumps({'utc_ns': time.time_ns(), 'event': event, **data}, ensure_ascii=False) + '\n')
        self.stream.flush()
        brief = {k: v for k, v in data.items() if k not in ('metrics', 'plan')}
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {event} {json.dumps(brief, ensure_ascii=False)[:260]}", flush=True)


def wait(predicate, timeout, message, period=0.1):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(period)
    raise Abort(message)


# ---------- kinematyka prosta (do oceny drogi TCP) ----------
def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rotation(axis, q):
    """Macierze obrotu wokół osi jednostkowej dla wektora kątów q (N) → (N,3,3)."""
    x, y, z = axis; c, s = np.cos(q), np.sin(q); t = 1 - c
    return np.stack([np.stack([c + x * x * t, x * y * t - z * s, x * z * t + y * s], -1),
                     np.stack([y * x * t + z * s, c + y * y * t, y * z * t - x * s], -1),
                     np.stack([z * x * t - y * s, z * y * t + x * s, c + z * z * t], -1)], -2)


class Kinematics:
    def __init__(self, values):
        self.origins = []
        for j in range(1, 7):
            xyz = np.array([values[f'joint.{j}.origin_{k}'] or 0.0 for k in 'xyz'])
            rpy = [math.radians(values[f'joint.{j}.origin_{k}'] or 0.0) for k in ('roll', 'pitch', 'yaw')]
            self.origins.append((xyz, rpy_matrix(*rpy)))
        self.tool = (np.array([values[f'tool.tcp_{k}'] or 0.0 for k in 'xyz']),
                     rpy_matrix(*[math.radians(values[f'tool.tcp_{k}'] or 0.0) for k in ('roll', 'pitch', 'yaw')]))

    def positions(self, q_rad):
        """Pozycje TCP [mm] dla macierzy kątów (N,6) w radianach."""
        q = np.atleast_2d(q_rad); n = len(q)
        p = np.zeros((n, 3)); r = np.tile(np.eye(3), (n, 1, 1))
        for j, (xyz, rot) in enumerate(self.origins):
            p = p + r @ xyz; r = r @ rot @ axis_rotation(JOINT_AXES[j], q[:, j])
        return p + r @ self.tool[0]


# ---------- robot ----------
class Robot:
    def __init__(self, log):
        self.log = log

    def manual(self):
        return api('/api/v1/manual/status')

    def configuration(self):
        return api('/api/v1/configuration')

    def q(self):
        return [a['joint_position_degrees'] for a in self.manual()['axes']]

    def check_fault(self):
        m = self.manual()
        if m['fault']:
            raise Abort('Awaria: ' + str(m['message']))
        return m

    def wait_quiet(self, timeout=10):
        def quiet():
            m = self.manual()
            if m['fault']:
                raise Abort('Awaria: ' + str(m['message']))
            return m['fresh'] and not m['busy'] and not api('/api/v1/motion/status')['busy'] and m
        return wait(quiet, timeout, 'Brak stanu bezczynności sterowania')

    def enable_all(self, enabled, timeout=15):
        for attempt in range(100):   # do ~30 s: przy łagodnym acc osie dojeżdżają jeszcze po końcu programu
            self.wait_quiet(30)
            try:
                api('/api/v1/manual/enable-all', {'enabled': enabled}); break
            except Abort as error:
                # Sterownik C++ kończy poprzednią sekwencję chwilę po stanie widocznym w API.
                if 'HTTP 409' not in str(error) or attempt == 99:
                    raise
                time.sleep(0.3)
        def done():
            m = self.manual()
            if m['fault']:
                raise Abort('Awaria przy ENABLE: ' + str(m['message']))
            return not m['busy'] and all(a['enable_known'] and a['enabled'] == enabled for a in m['axes'] if a['available'])
        wait(done, timeout, f'Brak potwierdzenia ENABLE={int(enabled)} wszystkich osi')
        self.log('enable_all', enabled=enabled)

    def apply(self, document, revision, confirm=(), timeout=20):
        body = {'document': document, 'expected_revision': revision, 'confirm_calibration_axes': list(confirm)}
        for attempt in range(20):
            self.wait_quiet()
            try:
                return self._apply_once(body, timeout)
            except Abort as error:
                # Sterownik C++ kończy sekwencję ENABLE/zerowania chwilę po stanie widocznym w API.
                if not any(t in str(error) for t in ('nieaktywnego sterowania', 'potwierdzony postój')) or attempt == 19:
                    raise
                self.log('apply_retry', reason=str(error)[:120]); time.sleep(0.5)

    def _apply_once(self, body, timeout):
        result = api('/api/v1/configuration/apply', body)
        operation = result.get('operation', {}).get('id')
        def finished():
            status = self.configuration()
            op_ = status['operation']
            if operation and op_['id'] != operation:
                return False
            if op_['phase'] in ('failed', 'uncertain') or op_.get('success') is False:
                raise Abort(f"Zapis konfiguracji nieudany: {op_['message']}")
            return op_['phase'] in ('done', 'idle') and op_.get('success') and status
        return wait(finished, timeout, 'Zapis konfiguracji nie zakończył się')

    def ensure_ready(self):
        """Uzgodnienie po przerwanym teście: nieudana operacja konfiguracji lub nieważna kalibracja po zerowaniu."""
        status = self.configuration()
        invalid = [i + 1 for i, flag in enumerate(status['calibration_invalid']) if flag]
        if status['operation']['phase'] in ('failed', 'uncertain') or invalid:
            self.enable_all(False)
            self.apply(status['active']['document'], status['active']['revision'], invalid)
            self.log('configuration_reconciled', confirmed=invalid)

    def set_acceleration(self, code):
        status = self.configuration()
        document = status['active']['document']
        if document['values'].get(F5_KEY) == code:
            self.log('acc_unchanged', acc=code); return
        document['values'][F5_KEY] = code
        self.apply(document, status['active']['revision'])
        if self.configuration()['active']['document']['values'].get(F5_KEY) != code:
            raise Abort('acc F5 nie został zapisany')
        self.log('acc_set', acc=code)

    def set_tuning(self, tuning):
        """Nastawy regulatora wykonania poza konfiguracją: najpierw wartości z pliku, potem ewentualna zmiana testowa."""
        def post(body):
            for attempt in range(50):   # sterownik C++ kończy sekwencję ENABLE chwilę po stanie widocznym w API
                self.wait_quiet(30)
                try:
                    return api('/api/v1/motion/execution-tuning', body)
                except Abort as error:
                    if 'wymaga postoju' not in str(error) or attempt == 49:
                        raise
                    time.sleep(0.3)
        result = api('/api/v1/motion/execution-tuning')
        if result.get('test'):
            result = post({'reset': True})
        if tuning:
            result = post(tuning)
        self.log('execution_tuning', test=result.get('test'), active=result.get('active'))
        return result

    def zero_and_confirm(self):
        """Zerowanie MKS (92) J1–J6 i potwierdzenie odniesienia; przeguby = offset konfiguracji.

        Zerowanie wymaga osi przygotowanej (ENABLE=1: szybkie odczyty prędkości i F1 potwierdzają postój),
        a potwierdzenie odniesienia — ENABLE=0. Kończy z ENABLE=0."""
        self.enable_all(True)
        for axis in range(1, 7):
            self.wait_quiet()
            for attempt in range(10):
                try:
                    api(f'/api/v1/manual/axes/{axis}/zero', {}); break
                except Abort as error:
                    if attempt == 9:
                        raise
                    self.log('zero_retry', axis=axis, reason=str(error)[:160]); time.sleep(0.5)
            wait(lambda: (lambda a: not self.manual()['busy'] and a['position_known'] and abs(a['position_degrees']) < 0.1)(
                self.manual()['axes'][axis - 1]), 10, f'J{axis}: brak potwierdzenia zera')
        self.enable_all(False)
        status = self.configuration()
        invalid = [i + 1 for i, flag in enumerate(status['calibration_invalid']) if flag]
        if invalid:
            self.apply(status['active']['document'], status['active']['revision'], invalid)
            if any(self.configuration()['calibration_invalid']):
                raise Abort('Odniesienie nie zostało potwierdzone')
        self.log('zeroed', confirmed=invalid, q=[round(v, 3) for v in self.q()])

    # ----- planowanie i wykonanie -----
    def context(self):
        return api('/api/v1/planning/context')

    def ik(self, ctx, seed, pose=None):
        body = {'model_id': ctx['model_id'], 'configuration_revision': ctx['configuration_revision'], 'seed_degrees': seed,
                'preview_velocity_degrees_s': SETUP_VELOCITY, 'preview_acceleration_degrees_s2': SETUP_ACCELERATION}
        body.update({'forward_only': True} if pose is None else {'target': pose})
        return api('/api/v1/planning/ik', body)

    def plan(self, ctx, seed, motion_type, target, params, simulated):
        request = {'model_id': ctx['model_id'], 'configuration_revision': ctx['configuration_revision'],
                   'settings_revision': ctx['settings_revision'], 'seed_degrees': seed, 'simulated_start': [simulated] * 6,
                   'motion_type': motion_type, 'target_degrees': target, **params}
        job = api('/api/v1/planning/moves', request)
        end = time.monotonic() + 30
        while job['status'] == 'running':
            if time.monotonic() > end:
                raise Abort('Timeout planera')
            time.sleep(0.05); job = api('/api/v1/planning/jobs/' + job['id'])
        if job['status'] != 'succeeded':
            raise Abort(f"Planowanie {motion_type} nieudane: {job.get('message') or job.get('phase')}")
        return job['plan_id']

    def execute(self, plan_id, label):
        api('/api/v1/motion/prepare', {'plan_id': plan_id})
        def ready():
            st = api('/api/v1/motion/status'); prep = st['preparation']
            if prep['state'] in ('failed', 'cancelled'):
                raise Abort(f"{label}: przygotowanie nieudane: {prep.get('message')}")
            return prep['state'] == 'ready' and st['state'] == 'ready' and st
        wait(ready, 20, f'{label}: timeout przygotowania', 0.05)
        prepared = api('/api/v1/motion/prepared')
        t_exec = time.time_ns()
        api('/api/v1/motion/execute', {'program_id': prepared['program_id'], 'execution_key': f'tcpq-{uuid.uuid4().hex}'})
        limit = None
        end = time.monotonic() + 30
        while True:
            st = api('/api/v1/motion/status')
            if limit is None and st.get('duration'):
                limit = 2 * float(st['duration']) + 30   # 400 mm przy 5 mm/s trwa ~80 s
                end = time.monotonic() + limit
            if st['state'] == 'succeeded' and not st['busy']:
                break
            if st['state'] in ('failed', 'fault', 'cancelled', 'aborted', 'stopped'):
                raise Abort(f"{label}: program zakończony {st['state']}: {st['message']}")
            if self.manual()['fault']:
                raise Abort(f"{label}: awaria {self.manual()['message']}")
            if time.monotonic() > end:
                raise Abort(f'{label}: program przekroczył limit czasu ({limit or 30:.0f} s)')
            time.sleep(0.05)
        t_done = time.time_ns()
        time.sleep(SETTLE_OBS_S)
        rest = self.check_fault()
        return {'t_exec_ns': t_exec, 't_done_ns': t_done, 't_end_ns': time.time_ns(),
                'rest_q_deg': [a['joint_position_degrees'] for a in rest['axes']],
                'duration_s': st.get('duration'), 'elapsed_s': st.get('elapsed'), 'message': st.get('message')}


class MoveFailed(Abort):
    """Program zakończony bez sukcesu bez awarii sprzętowej (np. brak potwierdzenia postoju po dolocie)."""


def run_move(robot, motion_type, target, params, label):
    """Plan z bieżącej pozycji i wykonanie; nieudany program → MoveFailed po ponownym załączeniu osi."""
    robot.wait_quiet(20)
    if not all(a['enabled'] for a in robot.manual()['axes'] if a['available']):
        robot.enable_all(True)
    ctx = robot.context()
    plan = robot.plan(ctx, ctx['seed_degrees'], motion_type, target, params, False)
    try:
        return robot.execute(plan, label)
    except Abort as error:
        text = str(error)
        # Nieudany dolot z małym uchybem (np. stick-slip J5) to ruch niezaliczony; duży uchyb (ucieczka osi) przerywa serię.
        errors = [abs(float(v)) for v in re.findall(r'J\d=(-?[\d.]+)°', text)] if 'dolot końcowy nieudany' in text else []
        if errors and max(errors) <= FINAL_APPROACH_SOFT_DEG:
            wait(lambda: (lambda m: not m['fault'] and not m['busy'] and m.get('recovery_state') in ('idle', 'idle_disabled'))(
                robot.manual()), 30, 'Brak powrotu do gotowości po nieudanym dolocie')
            robot.wait_quiet(20)
            raise MoveFailed(text) from None
        if robot.manual()['fault'] or 'awaria' in text.lower():
            raise
        if 'program zakończony' not in str(error):
            raise
        robot.wait_quiet(20)
        raise MoveFailed(str(error)) from None


# ---------- plan procedury ----------
def ptp_params(dq_max_deg, distance, speed):
    """PTP w przestrzeni przegubów o czasie zbliżonym do LIN: skala dominującej osi = Δq·v/d (oraz Δq·a/d)."""
    return {'preview_velocity_degrees_s': max(0.01, min(80.0, dq_max_deg * speed / distance)),
            'preview_acceleration_degrees_s2': max(0.01, min(500.0, dq_max_deg * TCP_ACCELERATION / distance))}


def lin_params(speed):
    return {'tcp_velocity_mm_s': speed, 'tcp_acceleration_mm_s2': TCP_ACCELERATION,
            'tcp_angular_velocity_degrees_s': TCP_ANGULAR_VELOCITY}


def build_procedure(robot, start, log, directions=None):
    """IK celów dla pierwszego wykonalnego kierunku i suchy plan wszystkich 12 ruchów (podgląd z symulowanym startem)."""
    ctx = robot.context()
    pose = robot.ik(ctx, start)['tcp']
    xyz, rpy = pose[:3], pose[3:]   # [x, y, z mm, roll, pitch, yaw °]
    problems = []
    for direction in (directions or DIRECTIONS):
        u = np.array(direction, float); u /= np.linalg.norm(u)
        try:
            targets = {}
            for d in DISTANCES_MM:
                answer = robot.ik(ctx, start, [*(np.array(xyz) + d * u).tolist(), *rpy])
                if answer['position_error_mm'] > 0.01:
                    raise Abort(f'IK {d} mm: błąd {answer["position_error_mm"]:.3f} mm')
                targets[d] = answer['positions_degrees']
            moves = []
            for d in DISTANCES_MM:
                dq = max(abs(a - b) for a, b in zip(targets[d], start))
                for v in SPEEDS_MM_S:
                    ptp = ptp_params(dq, d, v)
                    robot.plan(ctx, start, 'PTP', targets[d], ptp, True)
                    robot.plan(ctx, targets[d], 'LIN', start, lin_params(v), True)
                    moves.append({'label': f'PTP {d:g}mm {v:g}mm/s', 'type': 'PTP', 'distance_mm': d, 'speed_mm_s': v,
                                  'target': targets[d], 'params': ptp})
                    moves.append({'label': f'LIN {d:g}mm {v:g}mm/s', 'type': 'LIN', 'distance_mm': d, 'speed_mm_s': v,
                                  'target': list(start), 'params': lin_params(v)})
            log('procedure', direction=[round(x, 3) for x in u], start_tcp=pose,
                targets={str(k): [round(x, 3) for x in v] for k, v in targets.items()})
            return {'direction': u.tolist(), 'start_tcp': pose, 'targets': targets, 'moves': moves}
        except Abort as error:
            problems.append(f'{direction}: {str(error)[:120]}')
    raise Abort('Brak wykonalnego kierunku ruchu: ' + ' | '.join(problems))


# ---------- analiza z nagrania CAN ----------
def read_feedback(t0_ns, t1_ns):
    """sync_feedback i sync_final_approach z plików can-*.jsonl w oknie UTC."""
    rows, approaches = [], 0
    for path in sorted(glob.glob(str(CAN_LOG / 'can-*.jsonl')), key=os.path.getmtime):
        if os.path.getmtime(path) * 1e9 < t0_ns:
            continue
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                if '"sync_feedback"' not in line and '"sync_final_approach"' not in line:
                    continue
                r = json.loads(line)
                if not t0_ns <= r['utc_ns'] <= t1_ns:
                    continue
                if r['event'] == 'sync_final_approach':
                    approaches += 1; continue
                d = dict(kv.split('=') for kv in r['detail'].split())
                stamp = r['request_ns'] + (r['utc_ns'] - r['mono_ns'])   # chwila pomiaru w UTC
                rows.append((stamp, r['axis'], float(d['q_RX']), float(d['q_measured'])))
    return rows, approaches


def analyse(kin, move, result):
    rows, approaches = read_feedback(result['t_exec_ns'] - 50_000_000, result['t_end_ns'])
    axes = sorted({r[1] for r in rows})
    if len(axes) != 6 or len(rows) < 60:
        return {'valid': False, 'reason': f'za mało danych sync_feedback ({len(rows)} wierszy, osie {axes}); '
                                          'wymagany tryb synchronized i nagrywanie --record'}
    t0 = min(r[0] for r in rows); t1 = max(r[0] for r in rows)
    grid = np.arange(t0, t1, 10_000_000, dtype=np.int64)
    ref = np.zeros((len(grid), 6)); meas = np.zeros((len(grid), 6)); jerr = []
    for j in range(6):
        sel = sorted((r[0], r[2], r[3]) for r in rows if r[1] == j + 1)
        t = np.array([s[0] for s in sel], dtype=np.int64)
        ref[:, j] = np.interp(grid, t, [s[1] for s in sel]); meas[:, j] = np.interp(grid, t, [s[2] for s in sel])
        jerr.append(float(np.degrees(np.max(np.abs(np.array([s[1] - s[2] for s in sel]))))))
    p_ref, p_meas = kin.positions(ref), kin.positions(meas)
    dt = 0.01
    v_ref = np.linalg.norm(np.gradient(p_ref, dt, axis=0), axis=1)
    v_meas = np.linalg.norm(np.gradient(p_meas, dt, axis=0), axis=1)
    peak = float(np.max(v_ref))
    moving = v_ref > 0.2 * peak
    cruise = v_ref > 0.5 * peak
    path = np.linalg.norm(p_meas - p_ref, axis=1)
    ripple = float(np.sqrt(np.mean((v_meas[moving] - v_ref[moving]) ** 2))) if moving.any() else 0.0
    stall = float(np.mean(v_meas[cruise] < 0.5 * v_ref[cruise]) * 100) if cruise.any() else 0.0
    target = kin.positions(np.radians([move['target']]))[0]
    end_err = float(np.linalg.norm(p_meas[-1] - target))   # ostatni sync_feedback = koniec trajektorii, przed dolotem
    rest = result.get('rest_q_deg')
    final_err = float(np.linalg.norm(kin.positions(np.radians([rest]))[0] - target)) if rest else None
    planned = result.get('duration_s') or 0.0
    elapsed = (result['t_done_ns'] - result['t_exec_ns']) / 1e9
    return {'valid': True, 'samples': len(grid), 'v_peak_ref_mm_s': round(peak, 2),
            'path_err_max_mm': round(float(np.max(path)), 3), 'path_err_rms_mm': round(float(np.sqrt(np.mean(path ** 2))), 3),
            'speed_ripple_rms_mm_s': round(ripple, 2), 'speed_ripple_pct': round(ripple / peak * 100, 1) if peak else None,
            'stall_pct': round(stall, 1), 'end_traj_err_mm': round(end_err, 3),
            'final_err_mm': round(final_err, 3) if final_err is not None else None,
            'joint_err_max_deg': [round(e, 3) for e in jerr], 'final_approach_corrections': approaches,
            'planned_s': round(planned, 2), 'elapsed_s': round(elapsed, 2), 'overrun_s': round(elapsed - planned, 2)}


def summary_table(runs):
    lines = ['| Przebieg | Ruch | Plan [s] | Wykonanie [s] | Tętnienie v [mm/s] | Tętnienie [%] | Zatrzymania [%] | '
             'Odchyłka drogi max/RMS [mm] | Uchyb na końcu trajektorii [mm] | Błąd końcowy po dolocie [mm] | Korekty dolotu |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for run in runs:
        for m in run['moves']:
            x = dict(m.get('metrics') or {})
            if x.get('valid') and 'end_traj_err_mm' not in x:   # wyniki sprzed 08.11 13:55: final_err = koniec trajektorii
                x['end_traj_err_mm'], x['final_err_mm'] = x.get('final_err_mm'), None
            if not x.get('valid'):
                lines.append(f"| {run.get('label', run['acc'])} | {m['label']} | — | — | {x.get('reason', m.get('error', 'brak danych'))} | | | | | | |")
                continue
            lines.append(f"| {run.get('label', run['acc'])} | {m['label']} | {x['planned_s']} | {x['elapsed_s']} | {x['speed_ripple_rms_mm_s']} | "
                         f"{x['speed_ripple_pct']} | {x['stall_pct']} | {x['path_err_max_mm']} / {x['path_err_rms_mm']} | "
                         f"{x.get('end_traj_err_mm')} | {'—' if x.get('final_err_mm') is None else x['final_err_mm']} | {x['final_approach_corrections']} |")
        ok = [dict(m['metrics']) for m in run['moves'] if (m.get('metrics') or {}).get('valid')]
        for x in ok:
            if 'end_traj_err_mm' not in x:
                x['end_traj_err_mm'], x['final_err_mm'] = x.get('final_err_mm'), None
        if ok:
            lines.append(f"| {run.get('label', run['acc'])} | **średnio ({len(ok)} ruchów)** | | | "
                         f"{np.mean([x['speed_ripple_rms_mm_s'] for x in ok]):.2f} | {np.mean([x['speed_ripple_pct'] for x in ok]):.1f} | "
                         f"{np.mean([x['stall_pct'] for x in ok]):.1f} | {np.mean([x['path_err_max_mm'] for x in ok]):.3f} / "
                         f"{np.mean([x['path_err_rms_mm'] for x in ok]):.3f} | {np.mean([x['end_traj_err_mm'] for x in ok]):.3f} | "
                         f"{np.mean([x['final_err_mm'] for x in ok if x.get('final_err_mm') is not None] or [float('nan')]):.3f} | "
                         f"{sum(x['final_approach_corrections'] for x in ok)} |")
    timing = [run for run in runs if run.get('sync_timing')]
    if timing:
        lines += ['', '| Przebieg | Odstęp SYNC mediana/p99/max [ms] | Spóźnienie SYNC p99/max [ms] | F5→ACK mediana/p99/max [ms] |',
                  '|---|---|---|---|']
        for run in timing:
            t = run['sync_timing']; f = lambda x, keys: ' / '.join(str(x[k]) for k in keys) if x else '—'
            lines.append(f"| {run.get('label', run['acc'])} | {f(t['sync_interval_ms'], ('median', 'p99', 'max'))} | "
                         f"{f(t['sync_lateness_ms'], ('p99', 'max'))} | {f(t['f5_ack_ms'], ('median', 'p99', 'max'))} |")
    return '\n'.join(lines)


TUNE_KEYS = {'kp': ('position_kp', 1.0), 'ff': ('velocity_ff_scale', 1.0), 'lookahead_ms': ('lookahead_time_sec', 1e-3),
             'ki': ('integral_gain', 1.0), 'ki_limit_deg': ('integral_limit_rad', math.pi / 180),
             'acc': ('acceleration_code', None)}   # testowe nadpisanie acc F5, także 0 (bez rampy)


def run_specs(args, values):
    """Lista przebiegów: seria acc albo seria nastaw regulatora (--tune) przy jednym acc."""
    def acc_spec(raw):
        code = None if raw == 'exec' else (values.get(F5_KEY) if raw == 'current' else int(raw))
        if code is not None and not 1 <= code <= 255:
            raise Abort(f'acc poza zakresem 1–255: {code}')
        return {'acc_raw': raw, 'acc': code, 'acc_label': 'exec' if code is None else code}
    if not args.tune:
        return [{**acc_spec(raw), 'tuning': None, 'label': f"acc {acc_spec(raw)['acc_label']}"} for raw in args.acc]
    if len(args.acc) != 1:
        raise Abort('--tune wymaga jednej wartości --acc (domyślnie bieżąca)')
    acc = acc_spec(args.acc[0]); specs = []
    fixed = {}
    for item in getattr(args, 'fixed', []):
        key, _, text = item.partition('=')
        if key not in TUNE_KEYS or not text:
            raise Abort(f'--fixed {item}: dozwolone klucze {", ".join(TUNE_KEYS)}')
        name, scale = TUNE_KEYS[key]
        fixed[name] = int(text) if scale is None else float(text) * scale
    if args.baseline:
        specs.append({**acc, 'tuning': dict(fixed) or None, 'label': 'nastawy z pliku' + (f' + {fixed}' if fixed else '')})
    for item in args.tune:
        key, _, raw_values = item.partition('=')
        if key not in TUNE_KEYS or not raw_values:
            raise Abort(f'--tune {item}: dozwolone klucze {", ".join(TUNE_KEYS)} w postaci klucz=w1,w2')
        name, scale = TUNE_KEYS[key]
        for text in raw_values.split(','):
            value = int(text) if scale is None else float(text) * scale
            specs.append({**acc, 'tuning': {**fixed, name: value}, 'label': f'{key}={text}' + (f' ({", ".join(args.fixed)})' if fixed else '')})
    return specs


def sync_timing(t0_ns, t1_ns):
    """Czas wysyłki zestawów F5/SYNC z nagrania CAN: odstęp SYNC, spóźnienie względem terminu, przygotowanie→ACK [ms]."""
    submitted, late, ack = [], [], []
    for path in glob.glob(str(CAN_LOG / 'can-*.jsonl')):
        if os.path.getmtime(path) * 1e9 < t0_ns:
            continue
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                if '"sync_submitted"' not in line and '"sync_axis_accepted"' not in line:
                    continue
                r = json.loads(line)
                if not t0_ns <= r['utc_ns'] <= t1_ns:
                    continue
                if r['event'] == 'sync_submitted':
                    submitted.append(r['mono_ns']); late.append((r['mono_ns'] - r['deadline_ns']) / 1e6)
                else:
                    ack.append((r['mono_ns'] - r['request_ns']) / 1e6)
    submitted.sort()
    interval = [(b - a) / 1e6 for a, b in zip(submitted, submitted[1:]) if b - a < 50_000_000]
    def summary(values):
        if not values:
            return None
        v = sorted(values)
        return {'median': round(v[len(v) // 2], 3), 'p99': round(v[int(len(v) * .99)], 3), 'max': round(v[-1], 3), 'n': len(v)}
    return {'sync_interval_ms': summary(interval), 'sync_lateness_ms': summary(late), 'f5_ack_ms': summary(ack)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--acc', nargs='+', default=['current'],
                        help='kody acc F5 1–255; "current" = bez zmiany, "exec" = puste (parametry wykonania)')
    parser.add_argument('--tune', action='append', default=[], metavar='KLUCZ=W1,W2',
                        help='seria nastaw regulatora wykonania (poza konfiguracją robota): kp, ff, lookahead_ms, ki, '
                             'ki_limit_deg; każda wartość = osobny przebieg, pozostałe nastawy z pliku parametrów')
    parser.add_argument('--fixed', action='append', default=[], metavar='KLUCZ=W',
                        help='nastawa stała dla wszystkich przebiegów serii --tune (np. acc=0)')
    parser.add_argument('--baseline', action='store_true', help='z --tune: dodatkowy przebieg z nastawami z pliku na początku')
    parser.add_argument('--direction', default=None, metavar='X,Y,Z',
                        help='kierunek ruchu w układzie bazy (np. -1,1,-1); domyślnie pierwszy wykonalny z listy')
    parser.add_argument('--execute', action='store_true', help='wykonaj ruch fizyczny (inaczej suchy przebieg)')
    parser.add_argument('--output', default=None, help='katalog wyników (domyślnie log/tcp_quality/<czas>)')
    args = parser.parse_args()
    out = Path(args.output) if args.output else ROOT / 'log/tcp_quality' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = Log(out); robot = Robot(log)
    status = robot.configuration(); values = status['active']['document']['values']
    if values['motion.ptp_lin_dispatch_mode'] != 'synchronized':
        log('warning', message='Tryb PTP/LIN inny niż synchronized: analiza z sync_feedback będzie niedostępna')
    if args.execute:
        robot.ensure_ready()
    kin = Kinematics(values)
    directions = [tuple(float(x) for x in args.direction.split(','))] if args.direction else None
    procedure = build_procedure(robot, START_POSE, log, directions)
    fk = robot.ik(robot.context(), START_POSE)['tcp']
    own = kin.positions(np.radians([START_POSE]))[0]
    fk_gap = float(np.linalg.norm(own - np.array(fk[:3])))
    log('fk_check', difference_mm=round(fk_gap, 4))
    if fk_gap > 0.5:
        raise Abort(f'Kinematyka analizy różni się od planera o {fk_gap:.2f} mm')
    if not args.execute:
        log('dry_run_ok', moves=[m['label'] for m in procedure['moves']])
        return
    specs = run_specs(args, values)
    runs = []
    try:
        for spec in specs:
            run = {'label': spec['label'], 'acc': spec['acc_label'], 'tuning': spec['tuning'], 'moves': []}; runs.append(run)
            run['t_start_ns'] = time.time_ns()
            log('run_start', label=spec['label'], acc=spec['acc_label'], tuning=spec['tuning'])
            robot.enable_all(False)
            if spec['acc_raw'] != 'current':
                robot.set_acceleration(spec['acc'])
            run['execution_tuning'] = robot.set_tuning(spec['tuning'])
            robot.zero_and_confirm()
            robot.enable_all(True)
            ctx = robot.context()
            for attempt in range(3):
                try:
                    run_move(robot, 'PTP', START_POSE, {'preview_velocity_degrees_s': SETUP_VELOCITY,
                             'preview_acceleration_degrees_s2': SETUP_ACCELERATION}, 'PTP do pozy startowej')
                    break
                except MoveFailed as error:
                    log('setup_failed', attempt=attempt + 1, reason=str(error)[:300])
                    if attempt == 2:
                        raise Abort(f'Nie osiągnięto pozy startowej: {error}')
            failures = 0
            for move in procedure['moves']:
                try:
                    result = run_move(robot, move['type'], move['target'], move['params'], move['label'])
                except MoveFailed as error:
                    failures += 1
                    run['moves'].append({**move, 'error': str(error)[:300], 'metrics': {'valid': False, 'reason': 'NIEZALICZONY: ' + str(error)[:160]}})
                    log('move_failed', label=move['label'], reason=str(error)[:300])
                    if failures > 3:
                        raise Abort('Więcej niż 3 niezaliczone ruchy w przebiegu')
                    continue
                metrics = analyse(kin, move, result)
                run['moves'].append({**move, 'result': result, 'metrics': metrics})
                log('move', label=move['label'], metrics=metrics,
                    brief={k: metrics.get(k) for k in ('speed_ripple_pct', 'stall_pct', 'path_err_max_mm', 'final_err_mm')})
            run['failures'] = failures
            robot.enable_all(False)
            run['t_end_ns'] = time.time_ns()
            run['sync_timing'] = sync_timing(run['t_start_ns'], run['t_end_ns'])
            log('sync_timing', label=run['label'], **run['sync_timing'])
    except (Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            api('/api/v1/motion/stop', {})
        except Abort:
            pass
        raise
    finally:
        if any(spec['tuning'] for spec in specs):
            try:
                robot.set_tuning(None); log('tuning_reset')
            except Abort as error:
                log('tuning_reset_failed', reason=str(error)[:200])
                print(f'!!! Nie przywrócono nastaw regulatora z pliku: {error}\n'
                      '    Ręcznie: curl -X POST $ROBOT_API/api/v1/motion/execution-tuning -d \'{"reset": true}\' albo restart usługi.')
        (out / 'results.json').write_text(json.dumps({'procedure': {k: v for k, v in procedure.items() if k != 'moves'},
                                                     'runs': runs}, ensure_ascii=False, indent=1, default=str))
        (out / 'summary.md').write_text(summary_table(runs) + '\n')
        print('\n' + summary_table(runs) + f'\n\nWyniki: {out}')


if __name__ == '__main__':
    try:
        main()
    except Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
