#!/usr/bin/env python3
"""Odtwarzanie ucieczek osi: ruch oddalający wał od aktywnego celu F5.

Dwa obciążenia, wybrane z migawek awarii z 18.11.2026:
- creep: bardzo wolny PTP wszystkich osi (0,3–3 °/s), bo ucieczki „przeciwnie do referencji” dotyczyły osi wolnych;
- arrival: seria krótkich PTP, bo ucieczki „w dolocie” zaczynały się tuż przy celu, w strumieniu trzymania.

Dla każdej konfiguracji regulatora skrypt zeruje MKS, wykonuje oba obciążenia i zapisuje okna czasowe ruchów. Analiza
(`--analyse KATALOG`) czyta ramki z nagrania CAN i liczy epizody, w których wał oddala się od celu ostatniej ramki F5.
Awaria ruchu jest wynikiem testu: skrypt zapisuje ją, czeka na odzyskanie i kontynuuje od ponownego zerowania.

Zestawy --suite porównują warianty w obrębie jednego zerowania, bo cofnięcia zależą od przebiegu: skrypt szuka zerowania,
po którym zwykłe nastawy dają cofnięcia, a potem przeplata warianty z blokami kontrolnymi:
- hold: rzadsze powtarzanie identycznej ramki F5, prędkość trzymania, brak rampy;
- phase: cele przesunięte o ułamki okresu elektrycznego silnika;
- profile: prędkość, przyspieszenie i długość dojazdu;
- confirm: powtórzenia acc 0 oraz dojazdów 10 i 40 °/s;
- finish: acc 0 tylko dla końca ruchu (ramki dolotu, opcjonalnie końcówka trajektorii) oraz STOP po dojechaniu do celu;
- window: okno 150–230 ms przed końcem trajektorii (acc 0 albo prędkość ramek ×0,6 i ×1,5) oraz cisza po jednej ramce celu.
  Tabela pokazuje, do której ramki trajektorii wraca wał i czy jego prędkość idzie za oznaczoną prędkością ramki;
- scale: mnożniki prędkości ramek okna 0,9–0,6 na zwykłych dojazdach;
- scale_profile: najlepszy mnożnik z zestawu scale (albo --mark) na innych długościach, prędkościach i przyspieszeniach,
  każdy profil porównany z własną kontrolą.

Bez --execute: tylko planowanie. Ruch fizyczny wymaga zgody operatora (AGENTS.md).
"""
import argparse, collections, glob, json, os, time
from datetime import datetime

import tcp_motion_quality as q
import tcp_speed_limits as sl

CREEP_SPEEDS = (2.0, 1.0, 0.5, 1.5, 0.3, 3.0)      # °/s, J1–J6
CREEP_SECONDS = 20.0
CREEP_ACCELERATION = 20.0                          # °/s²
ARRIVAL_STEP = (5.0, -5.0, 5.0, -5.0, 5.0, 5.0)    # °
ARRIVAL_VELOCITY, ARRIVAL_ACCELERATION = 20.0, 100.0
DEFAULT_CONFIGS = ('base', 'lookahead_ms=10', 'lookahead_ms=50', 'lookahead_ms=100', 'kp=0,ki=0', 'acc=241', 'acc=0', 'base')
SMALL_TICKS, LARGE_TICKS = 30, 150                 # odejście od celu: 0,66° i 3,3° wału
WINDOW_MS = 60.0
ELECTRICAL_PERIOD_TICKS = 16384 / 50               # silnik 1,8°: 50 par biegunów
HOLD_VARIANTS = ('hold_period=2', 'hold_period=5', 'hold_period=20', 'hold_period=100', 'hold_rpm=5', 'hold_rpm=10', 'hold_rpm=40', 'acc=0')
ARRIVAL_SCALE = 1.0                                # mnożnik długości dojazdu bloków zestawu (--step-scale)
SELECTED_VARIANTS = None                           # --variants zastępuje listę zestawu finish
FINISH_VARIANTS = ('hold_acc=0', 'hold_acc=0,hold_period=100', 'hold_acc=0,hold_tail=100', 'hold_acc=0,hold_tail=300',
                   'hold_stop_ticks=150,hold_stop=1', 'hold_stop_ticks=150,hold_stop=2')   # odległość przed trybem: walidacja po kolei
WINDOW_VARIANTS = ('hold_acc=0,hold_tail=230,hold_tail_end=150', 'hold_tail=230,hold_tail_end=150,hold_mark=0.6',
                   'hold_tail=230,hold_tail_end=150,hold_mark=1.5', 'hold_silent=1')
TAIL_WINDOW = 'hold_tail=230,hold_tail_end=150'
SCALE_MARKS = (0.9, 0.8, 0.7, 0.6)
SCALE_PROFILES = (('krok 2,5°', None, 100, .5), ('krok 10°', None, 100, 2), ('20 °/s', 20, 100, 1), ('a=50', None, 50, 1), ('a=300', None, 300, 1))
PROFILE_MARK = None                                # mnożnik zestawu scale_profile: --mark albo wynik zestawu scale
PROFILE_VARIANTS = (('v=10', 10, 100, 1), ('v=40', 40, 100, 1), ('a=50', 20, 50, 1), ('a=300', 20, 300, 1), ('krok×2', 20, 100, 2), ('krok×0,5', 20, 100, .5))


def tuning_of(text):
    if text == 'base':
        return None
    tuning = {}
    for item in text.split(','):
        key, _, value = item.partition('=')
        if key not in q.TUNE_KEYS or not value:
            raise q.Abort(f'Nieznana nastawa {item}; dozwolone: {", ".join(q.TUNE_KEYS)}')
        name, scale = q.TUNE_KEYS[key]
        tuning[name] = int(value) if scale is None else float(value) * scale
    return tuning


def workload(creep_cycles, arrivals):
    start = list(q.START_POSE)
    creep_target = [s + v * CREEP_SECONDS for s, v in zip(start, CREEP_SPEEDS)]
    creep = {'preview_velocity_degrees_s': max(CREEP_SPEEDS), 'preview_acceleration_degrees_s2': CREEP_ACCELERATION}
    arrival_target = [s + d for s, d in zip(start, ARRIVAL_STEP)]
    arrival = {'preview_velocity_degrees_s': ARRIVAL_VELOCITY, 'preview_acceleration_degrees_s2': ARRIVAL_ACCELERATION}
    moves = []
    for n in range(creep_cycles):
        moves += [('creep', f'creep tam {n + 1}', creep_target, creep), ('creep', f'creep powrót {n + 1}', start, creep)]
    for n in range(arrivals):
        moves.append(('arrival', f'dojazd {n + 1}', arrival_target if n % 2 == 0 else start, arrival))
    if arrivals % 2:
        moves.append(('arrival', 'dojazd powrót', start, arrival))
    return moves


def arrival_block(count, offset=None, velocity=None, acceleration=ARRIVAL_ACCELERATION, scale=None):
    """Parzysta liczba dojazdów tam i z powrotem; blok kończy się w punkcie bazowym (poza startowa + przesunięcie)."""
    base = [s + d for s, d in zip(q.START_POSE, offset or [0.0] * 6)]
    away = [b + d * (scale or ARRIVAL_SCALE) for b, d in zip(base, ARRIVAL_STEP)]
    params = {'preview_velocity_degrees_s': velocity or ARRIVAL_VELOCITY, 'preview_acceleration_degrees_s2': acceleration}
    moves = []
    for n in range(count // 2):
        moves += [('arrival', f'tam {n + 1}', away, params), ('arrival', f'powrót {n + 1}', base, params)]
    return moves


def safe_prepare(robot, log, attempts=4):
    """Zerowanie i dojazd do pozy startowej; nieudane ENABLE (ramka gubiona przy błędach CAN) nie kończy serii."""
    for attempt in range(attempts):
        try:
            sl.prepare_ramp(robot, log); return
        except q.Abort as error:
            log('prepare_failed', attempt=attempt + 1, reason=str(error)[:220])
            if attempt + 1 == attempts:
                raise
            sl.recover(robot, log)


def run_config(robot, name, tuning, moves, log):
    log('config_start', config=name, tuning=tuning)
    robot.enable_all(False)
    tuning_state = robot.set_tuning(tuning)
    safe_prepare(robot, log)
    run = run_moves(robot, name, tuning, moves, log)
    run['execution_tuning'] = tuning_state
    return run


def run_moves(robot, name, tuning, moves, log):
    run = {'config': name, 'tuning': tuning, 'moves': [], 'faults': []}
    for kind, label, target, params in moves:
        try:
            result = q.run_move(robot, 'PTP', target, params, label)
            errors = [abs(a - b) for a, b in zip(result['rest_q_deg'], target)]
            run['moves'].append({'kind': kind, 'label': label, 't_exec_ns': result['t_exec_ns'], 't_done_ns': result['t_done_ns'],
                                 't_end_ns': result['t_end_ns'], 'duration_s': result['duration_s'], 'final_err_deg': max(errors),
                                 'overrun_s': (result['t_done_ns'] - result['t_exec_ns']) / 1e9 - (result['duration_s'] or 0)})
        except q.Abort as error:
            text = str(error)
            run['faults'].append({'kind': kind, 'label': label, 'utc_ns': time.time_ns(), 'reason': text[:400]})
            log('move_fault', config=name, label=label, reason=text[:300])
            sl.recover(robot, log)
            robot.set_tuning(None)          # zerowanie i dojazd do pozy startowej zawsze na zwykłych nastawach
            safe_prepare(robot, log)
            robot.set_tuning(tuning)
    log('config_end', config=name, moves=len(run['moves']), faults=len(run['faults']))
    return run


# ---------- analiza ----------
def can_frames(t0_ns, t1_ns):
    """Ramki F5 (TX), 0x31 i F1 (RX) z nagrania, posortowane po czasie."""
    rows = []
    for path in glob.glob(str(q.CAN_LOG / 'can-*.jsonl')):
        if os.path.getmtime(path) * 1e9 < t0_ns:
            continue
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                if '"data":"f5' not in line and '"data":"31' not in line and '"data":"f1' not in line:
                    continue
                r = json.loads(line)
                if t0_ns <= r['utc_ns'] <= t1_ns and r['event'] in ('can_tx', 'can_rx') and 1 <= r.get('axis', 0) <= 6:
                    rows.append((r['utc_ns'], r['event'], r['axis'], r['data']))
    rows.sort()
    return rows


def signed(text, bits):
    value = int(text, 16)
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def episodes(samples):
    """Epizody cofania: w oknie 60 ms wał oddala się od celu leżącego po tej samej stronie, a cel nie jedzie w przeciwną stronę.

    Bez warunku kierunku celu liczyłoby się zwykłe wyprzedzenie celu przy pracy skokowej (wał przeskakuje przed cel i czeka)."""
    found, first, last = [], 0, -1e18
    for k, (t, position, target, rpm) in enumerate(samples):
        while samples[first][0] < t - WINDOW_MS:
            first += 1
        t0, p0, c0, r0 = samples[first]
        if t - t0 < WINDOW_MS * .6 or c0 == p0:
            continue
        sign = 1 if c0 > p0 else -1
        away = -(position - p0) * sign
        if away >= SMALL_TICKS and (target - position) * sign > 0 and (target - c0) * sign >= 0 and t - last > 300:
            # Szczyt epizodu: największe oddalenie, dopóki wał nie zacznie wracać (najwyżej 1 s).
            peak = away
            for later in samples[k + 1:]:
                if later[0] > t + 1000:
                    break
                peak = max(peak, -(later[1] - p0) * sign)
            found.append({'t_ms': t, 'gap': c0 - p0, 'peak': peak, 'rpm': r0})  # gap: cel − pozycja na starcie okna
            last = t
    return found


def reversal_detail(series, frames, episode, planned_end_ms):
    """Dokąd i jak szybko cofa się wał oraz która ramka trajektorii miała cel najbliższy punktowi zwrotnemu."""
    start = episode['t_ms'] - WINDOW_MS
    sign = 1 if episode['gap'] > 0 else -1
    part = [s for s in series if start <= s[0] <= start + 400]
    earlier = [f for f in frames if f[0] < start]
    if len(part) < 3 or not earlier:
        return {}
    origin = part[0][1]
    turn = max(range(len(part)), key=lambda k: -(part[k][1] - origin) * sign)
    point = part[turn][1]; reach = max(abs(point - origin), 1)
    middle = [s for s in part[:turn + 1] if .2 <= abs(s[1] - origin) / reach <= .8]
    first_hold = next(f[0] for f in earlier if f[1] == earlier[-1][1])
    trajectory = [f for f in earlier if f[0] < first_hold]
    progress = [(s[1] - origin) * sign for s in part[:turn + 1]]        # początek: ostatnia próbka najbliżej celu
    onset = len(progress) - 1 - progress[::-1].index(max(progress))
    detail = {'start_after_end_ms': round(part[onset][0] - planned_end_ms)}
    if len(middle) > 2 and middle[-1][0] > middle[0][0]:
        detail['shaft_rpm'] = round(abs(middle[-1][1] - middle[0][1]) / (middle[-1][0] - middle[0][0]) * 60000 / 16384, 1)
    if trajectory:
        index = min(range(len(trajectory)), key=lambda k: abs(trajectory[k][1] - point))
        detail.update(frames_before_end=len(trajectory) - index, frame_rpm=trajectory[index][2], frame_miss=abs(trajectory[index][1] - point))
    return detail


def stopping_ticks(rpm, acceleration_code, microstep_scale=.125):
    """Droga hamowania rampy F5 [impulsy enkodera]; kod 0 = bez rampy."""
    if not acceleration_code:
        return 0.0
    ramp = microstep_scale / ((256 - acceleration_code) * 50e-6)   # RPM/s
    return (rpm / 60) ** 2 / (2 * ramp / 60) * 16384


def analyse_run(run):
    """Nagranie CAN jest pierścieniem, dlatego przebieg trzeba policzyć zaraz po wykonaniu."""
    if not run['moves']:
        return {'config': run['config'], 'moves': 0, 'faults': run['faults'], 'phases': {}, 'per_axis': {}, 'hold_gaps': [], 'backward': []}
    rows = can_frames(min(m['t_exec_ns'] for m in run['moves']) - 100_000_000, max(m['t_end_ns'] for m in run['moves']))
    if True:
        totals = collections.defaultdict(lambda: collections.Counter())
        per_axis = collections.Counter(); hold_gaps = []; backward = []; hold_frames = []; tracking = []
        for move in run['moves']:
            samples = {a: [] for a in range(1, 7)}; target, rpm = {}, {}; frames = {a: [] for a in range(1, 7)}
            begin = move['t_exec_ns']; planned_end = begin + int((move['duration_s'] or 0) * 1e9)
            for t, event, axis, text in rows[bisect(rows, begin - 50_000_000):bisect(rows, move['t_done_ns'])]:
                if event == 'can_tx' and text[:2] == 'f5':
                    target[axis] = signed(text[8:14], 24); rpm[axis] = (int(text[2:6], 16) & 0xfff) * .125
                    frames[axis].append((t / 1e6, target[axis], rpm[axis]))
                elif event == 'can_rx' and text[:2] == '31' and axis in target:
                    samples[axis].append((t / 1e6, signed(text[2:14], 48), target[axis], rpm[axis]))
            # Koniec trajektorii liczony od pierwszej ramki F5: uzbrojenie trwa ok. 155 ms po komendzie wykonania.
            first_frame = min((f[0][0] for f in frames.values() if f), default=planned_end / 1e6)
            trajectory_end = first_frame + (move['duration_s'] or 0) * 1e3
            for axis, series in samples.items():
                if len(series) < 5:
                    continue
                # Ramki po końcu trajektorii (margines 20 ms): kontrola, że wariant ciszy rzeczywiście milczy.
                hold_frames.append(sum(f[0] > trajectory_end + 20 for f in frames[axis]))
                # Odległość wału od celu ostatniej ramki w trakcie ruchu: porównanie wariantów, cel zawiera wyprzedzenie.
                lag = [abs(s[2] - s[1]) for s in series if first_frame <= s[0] <= trajectory_end]
                if lag:
                    tracking.append(max(lag))
                for phase, part in (('ruch', [s for s in series if s[0] * 1e6 <= planned_end]),
                                    ('trzymanie', [s for s in series if s[0] * 1e6 > planned_end])):
                    if len(part) < 5:
                        continue
                    key = (move['kind'], phase)
                    totals[key]['axis_s'] += (part[-1][0] - part[0][0]) / 1000
                    for e in episodes(part):
                        totals[key]['small'] += 1; totals[key]['large'] += e['peak'] >= LARGE_TICKS
                        totals[key]['peak_sum'] += e['peak']; per_axis[axis] += 1
                        if phase == 'trzymanie' and e['peak'] >= LARGE_TICKS:
                            hold_gaps.append(abs(e['gap']))
                            # Cofnięcie: wał odchodzi w stronę, z której przyjechał; inaczej to przejazd za cel.
                            approach = series[-1][2] - series[0][1]
                            if approach * e['gap'] > 0:
                                backward.append({'axis': axis, 'peak': e['peak'], 'gap': abs(e['gap']), 'move': move['label'],
                                                 **reversal_detail(series, frames[axis], e, trajectory_end)})
        return {'config': run['config'], 'moves': len(run['moves']), 'faults': run['faults'],
                'phases': {f'{k[0]}/{k[1]}': dict(v) for k, v in totals.items()}, 'per_axis': dict(per_axis), 'hold_gaps': hold_gaps, 'backward': backward,
                'hold_frames_mean': sum(hold_frames) / len(hold_frames) if hold_frames else 0,
                'tracking_mean': sum(tracking) / len(tracking) if tracking else None, 'tracking_max': max(tracking, default=None)}


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2] if ordered else None


def run_block(robot, name, tuning, moves, log):
    robot.wait_quiet(20)
    tuning_state = robot.set_tuning(tuning)
    run = run_moves(robot, name, tuning, moves, log)
    run['execution_tuning'] = tuning_state
    run['analysis'] = analyse_run(run)
    found = run['analysis']['backward']
    log('block', block=name, moves=len(run['moves']), backward=len(found), axes=dict(collections.Counter(e['axis'] for e in found)),
        peak=median([e['peak'] for e in found]), faults=len(run['faults']))
    return run


def find_session(robot, moves, need, sessions, runs, log):
    """Zerowanie, po którym zwykłe nastawy odtwarzają cofnięcia; bez tego porównanie wariantów nic nie mówi."""
    for attempt in range(sessions):
        robot.enable_all(False); robot.set_tuning(None); safe_prepare(robot, log)
        probe = run_block(robot, f'próba zerowania {attempt + 1}', None, moves, log)
        runs.append(probe)
        if len(probe['analysis']['backward']) >= need and not probe['faults']:
            return True
    log('no_active_session', sessions=sessions, need=need)
    return False


def mark_variant(mark):
    return f'{TAIL_WINDOW},hold_mark={mark}'


def best_mark(runs):
    """Najmniej cofnięć i awarii; przy remisie mnożnik najbliższy 1, czyli najmniejsza ingerencja w ruch."""
    scores = {}
    for run in runs:
        for mark in SCALE_MARKS:
            if run['config'] == mark_variant(mark):
                scores[mark] = scores.get(mark, 0) + len(run['analysis']['backward']) + len(run['faults'])
    return min(scores, key=lambda mark: (scores[mark], -mark)) if scores else None


def suite_blocks(suite, count, gear_ratios):
    if suite == 'scale':
        return [(mark_variant(m), tuning_of(mark_variant(m)), arrival_block(count)) for _ in range(3) for m in SCALE_MARKS]
    if suite == 'scale_profile':   # każdy profil ma własną kontrolę; przed wyborem mnożnika lista służy tylko do planowania
        mark = PROFILE_MARK or SCALE_MARKS[0]; blocks = []
        for _ in range(3):
            for label, velocity, acceleration, scale in SCALE_PROFILES:
                moves = arrival_block(count, velocity=velocity, acceleration=acceleration, scale=scale)
                blocks.append((f'{label}: ×{mark}', tuning_of(mark_variant(mark)), moves, (f'{label}: kontrola', moves)))
        return blocks
    if suite == 'hold':
        return [(v, tuning_of(v), arrival_block(count)) for v in HOLD_VARIANTS]
    if suite == 'profile':
        return [(label, None, arrival_block(count, velocity=v, acceleration=a, scale=scale)) for label, v, a, scale in PROFILE_VARIANTS]
    if suite == 'finish':    # każdy wariant trzy razy, przeplatany blokami kontrolnymi
        return [(v, tuning_of(v), arrival_block(count)) for _ in range(3) for v in (SELECTED_VARIANTS or FINISH_VARIANTS)]
    if suite == 'window':    # okno ramek, do których wraca wał, i cisza po ramce celu
        return [(v, tuning_of(v), arrival_block(count)) for _ in range(3) for v in (SELECTED_VARIANTS or WINDOW_VARIANTS)]
    if suite == 'confirm':   # powtórzenia wariantów, które w pojedynczym bloku nie dały cofnięć albo dały ich więcej
        return ([('acc=0', tuning_of('acc=0'), arrival_block(count))] * 6 +
                [('v=10', None, arrival_block(count, velocity=10))] * 3 + [('v=40', None, arrival_block(count, velocity=40))] * 3)
    if suite == 'phase':
        blocks = []
        for k in range(1, 8):
            ticks = ELECTRICAL_PERIOD_TICKS * k / 8
            offset = [ticks / 16384 * 360 / gear for gear in gear_ratios]
            blocks.append((f'faza {k}/8', None, arrival_block(count, offset=offset)))
        return blocks
    raise q.Abort(f'Nieznany zestaw {suite}')


def run_suite(robot, suite, count, need, sessions, gear_ratios, log, runs):
    control = arrival_block(count)
    if not find_session(robot, control, need, sessions, runs, log):
        return runs
    for name, tuning, moves, *own_control in suite_blocks(suite, count, gear_ratios):
        control_name, control_moves = own_control[0] if own_control else ('kontrola', control)
        runs.append(run_block(robot, name, tuning, moves, log))
        broken = bool(runs[-1]['faults'])
        if not broken:
            runs.append(run_block(robot, control_name, None, control_moves, log)); broken = bool(runs[-1]['faults'])
        if broken and not find_session(robot, control, need, sessions, runs, log):   # awaria kończy się nowym zerowaniem
            break
    return runs


def suite_summary(name, runs):
    lines = [f'### Zestaw {name}', '', '| Blok | Dojazdy | Cofnięcia | Osie | Mediana szczytu [imp.] | Mediana odległości od celu [imp.] | Start po końcu [ms] | '
             'Ramka powrotu [cykli przed końcem] | Wał / ramka [RPM] | Ramki F5 po końcu (śr. na oś) | Błąd końcowy osi śr./maks. [°] | Nadmiar czasu śr. [s] | Awarie |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for run in runs:
        found = run['analysis']['backward']
        pick = lambda key: median([e[key] for e in found if key in e])
        speeds = f"{pick('shaft_rpm')} / {pick('frame_rpm')}" if pick('shaft_rpm') is not None and pick('frame_rpm') is not None else '—'
        axes = ' '.join(f'J{a}×{n}' for a, n in sorted(collections.Counter(e['axis'] for e in found).items())) or '—'
        final = [m['final_err_deg'] for m in run['moves'] if 'final_err_deg' in m]; extra = [m['overrun_s'] for m in run['moves'] if 'overrun_s' in m]
        accuracy = f'{sum(final) / len(final):.3f} / {max(final):.3f}' if final else '—'
        lines.append(f"| {run['config']} | {len(run['moves'])} | {len(found)} | {axes} | {median([e['peak'] for e in found]) or '—'} | "
                     f"{median([e['gap'] for e in found]) or '—'} | {pick('start_after_end_ms') if pick('start_after_end_ms') is not None else '—'} | "
                     f"{pick('frames_before_end') or '—'} | {speeds} | {run['analysis'].get('hold_frames_mean', 0):.1f} | "
                     f"{accuracy} | {sum(extra) / len(extra) if extra else 0:.2f} | {len(run['faults'])} |")
    return '\n'.join(lines) + '\n\n' + group_summary(runs)


def group_summary(runs):
    """Suma bloków tego samego wariantu; bloki próbne służą tylko do wyboru zerowania."""
    groups = {}
    for run in runs:
        if not run['config'].startswith('próba'):
            groups.setdefault(run['config'], []).append(run)
    lines = ['| Wariant łącznie | Bloki | Dojazdy | Cofnięcia | Osie | Odległość wału od celu F5 w ruchu śr./maks. [imp.] | Błąd końcowy osi śr./maks. [°] | Nadmiar czasu śr. [s] | Awarie |',
             '|---|---|---|---|---|---|---|---|---|']
    for name, items in groups.items():
        found = [e for run in items for e in run['analysis']['backward']]
        moves = [m for run in items for m in run['moves']]
        final = [m['final_err_deg'] for m in moves if 'final_err_deg' in m]; extra = [m['overrun_s'] for m in moves if 'overrun_s' in m]
        lag = [run['analysis'] for run in items if run['analysis'].get('tracking_mean') is not None]
        axes = ' '.join(f'J{a}×{n}' for a, n in sorted(collections.Counter(e['axis'] for e in found).items())) or '—'
        tracking = f"{sum(a['tracking_mean'] for a in lag) / len(lag):.0f} / {max(a['tracking_max'] for a in lag)}" if lag else '—'
        accuracy = f'{sum(final) / len(final):.3f} / {max(final):.3f}' if final else '—'
        lines.append(f"| {name} | {len(items)} | {len(moves)} | {len(found)} | {axes} | {tracking} | {accuracy} | "
                     f"{sum(extra) / len(extra) if extra else 0:.2f} | {sum(len(run['faults']) for run in items)} |")
    return '\n'.join(lines)


def analyse(directory):
    runs = json.loads((directory / 'windows.json').read_text())['runs']
    report = [run.get('analysis') or analyse_run(run) for run in runs]
    lines = ['| Konfiguracja | Obciążenie / faza | Czas osi [s] | Epizody /100 s | Średni szczyt [imp.] | Duże ≥ 150 imp. | Awarie |', '|---|---|---|---|---|---|---|']
    for item in report:
        for phase, v in sorted(item['phases'].items()):
            seconds = v.get('axis_s', 0); small = v.get('small', 0)
            lines.append(f"| {item['config']} | {phase} | {seconds:.0f} | {100 * small / seconds if seconds else 0:.1f} | "
                         f"{v.get('peak_sum', 0) / small if small else 0:.0f} | {v.get('large', 0)} | {len(item['faults'])} |")
    text = '\n'.join(lines)
    (directory / 'analysis.json').write_text(json.dumps(report, ensure_ascii=False, indent=1))
    (directory / 'summary.md').write_text(text + '\n')
    print(text)
    for item in report:
        gaps = sorted(item.get('hold_gaps', []))
        print(item['config'], 'epizody na osiach:', dict(sorted(item['per_axis'].items())),
              '| duże w trzymaniu: mediana |cel−poz| przed cofnięciem', gaps[len(gaps) // 2] if gaps else '—', 'imp., n =', len(gaps))
        for fault in item['faults']:
            print('   AWARIA', fault['label'], fault['reason'][:200])


def bisect(rows, t_ns):
    low, high = 0, len(rows)
    while low < high:
        middle = (low + high) // 2
        if rows[middle][0] < t_ns:
            low = middle + 1
        else:
            high = middle
    return low


def main_suite(args, out, robot, log):
    global PROFILE_MARK
    suites = ['hold', 'phase', 'profile'] if args.suite == 'all' else [x.strip() for x in args.suite.split(',')]
    PROFILE_MARK = args.mark
    if 'scale_profile' in suites and args.mark is None and ('scale' not in suites or suites.index('scale') > suites.index('scale_profile')):
        raise q.Abort('Zestaw scale_profile wymaga --mark albo wcześniejszego zestawu scale w tym samym uruchomieniu')
    values = robot.configuration()['active']['document']['values']
    gears = [values[f'joint.{j}.gear_ratio'] for j in range(1, 7)]
    ctx = robot.context(); count = args.block_moves - args.block_moves % 2
    for suite in suites:
        for name, _, moves, *_own_control in suite_blocks(suite, count, gears):
            position = list(q.START_POSE)
            for _kind, _label, target, params in moves[:2]:
                robot.plan(ctx, position, 'PTP', target, params, True); position = list(target)
    log('plan_ok', suites=suites, block_moves=count)
    if not args.execute:
        return
    results, text, interrupted = {}, [], False
    try:
        robot.ensure_ready()
        for suite in suites:
            log('suite_start', suite=suite)
            if suite == 'scale_profile' and PROFILE_MARK is None:
                PROFILE_MARK = best_mark(results.get('scale', []))
                if PROFILE_MARK is None:
                    raise q.Abort('Zestaw scale nie dał wyniku, nie ma z czego wybrać mnożnika')
                log('mark_selected', mark=PROFILE_MARK)
            results[suite] = []   # lista uzupełniana na bieżąco: przerwanie nie kasuje wykonanych bloków
            run_suite(robot, suite, count, args.need, args.sessions, gears, log, results[suite])
            text.append(suite_summary(suite, results[suite]))
            (out / 'suites.json').write_text(json.dumps(results, ensure_ascii=False))
    except (q.Abort, KeyboardInterrupt) as error:
        interrupted = True
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        (out / 'suites.json').write_text(json.dumps(results, ensure_ascii=False))
        text = text or [suite_summary(k, v) for k, v in results.items()]
        (out / 'summary.md').write_text('\n\n'.join(text) + '\n')
        print('\n\n'.join(text) + f'\n\nWyniki: {out}')
        try:
            if interrupted or robot.manual()['fault']:
                sl.recover(robot, log)
            robot.set_tuning(None); robot.enable_all(False); log('finished')
        except q.Abort as error:
            log('cleanup_failed', reason=str(error)[:200])


def main():
    global ARRIVAL_VELOCITY, ARRIVAL_SCALE, SELECTED_VARIANTS
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--execute', action='store_true', help='wykonaj ruch fizyczny (inaczej tylko planowanie)')
    parser.add_argument('--configs', default=';'.join(DEFAULT_CONFIGS), help='konfiguracje rozdzielone średnikiem, np. "base;kp=0,ki=0"')
    parser.add_argument('--creep-cycles', type=int, default=3)
    parser.add_argument('--arrivals', type=int, default=24)
    parser.add_argument('--suite', default=None, help='hold, phase, profile, confirm, finish, window, scale, scale_profile albo all (po przecinku także kilka)')
    parser.add_argument('--mark', type=float, default=None, help='mnożnik prędkości ramek okna dla zestawu scale_profile (0,25–1)')
    parser.add_argument('--block-moves', type=int, default=12, help='dojazdy w bloku zestawu (parzyste)')
    parser.add_argument('--velocity', type=float, default=ARRIVAL_VELOCITY, help='prędkość dojazdów bloków zestawu [°/s]')
    parser.add_argument('--step-scale', type=float, default=1.0, help='mnożnik kroku 5° dojazdów zestawu')
    parser.add_argument('--variants', default=None, help='warianty zestawu finish lub window rozdzielone średnikiem')
    parser.add_argument('--need', type=int, default=2, help='cofnięcia w bloku próbnym wymagane do porównania')
    parser.add_argument('--sessions', type=int, default=6, help='najwięcej prób zerowania')
    parser.add_argument('--analyse', default=None, metavar='KATALOG', help='tylko analiza zapisanego przebiegu')
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    ARRIVAL_VELOCITY = args.velocity
    ARRIVAL_SCALE = args.step_scale
    SELECTED_VARIANTS = [v.strip() for v in args.variants.split(';') if v.strip()] if args.variants else None
    if args.mark is not None and not .25 <= args.mark <= 1:
        raise q.Abort('--mark: dozwolone 0,25–1')
    if args.analyse:
        analyse(q.Path(args.analyse)); return
    out = q.Path(args.output) if args.output else q.ROOT / 'log/axis_runaway' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = q.Log(out); robot = q.Robot(log)
    if args.suite:
        return main_suite(args, out, robot, log)
    configs = [c.strip() for c in args.configs.split(';') if c.strip()]
    tunings = [(c, tuning_of(c)) for c in configs]
    moves = workload(args.creep_cycles, args.arrivals)
    ctx = robot.context(); position = list(q.START_POSE)
    for kind, label, target, params in moves[:2] + moves[2 * args.creep_cycles:2 * args.creep_cycles + 2]:
        robot.plan(ctx, position, 'PTP', target, params, True); position = list(target)
    log('plan_ok', moves=len(moves), configs=configs)
    if not args.execute:
        return
    runs = []
    try:
        robot.ensure_ready()
        for name, tuning in tunings:
            runs.append(run_config(robot, name, tuning, moves, log))
            runs[-1]['analysis'] = analyse_run(runs[-1])
            (out / 'windows.json').write_text(json.dumps({'runs': runs}, ensure_ascii=False))
    except (q.Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/motion/stop', {})
        except q.Abort:
            pass
        raise
    finally:
        (out / 'windows.json').write_text(json.dumps({'runs': runs}, ensure_ascii=False))
        try:
            if robot.manual()['fault']:
                sl.recover(robot, log)
            robot.set_tuning(None); robot.enable_all(False); log('finished')
        except q.Abort as error:
            log('cleanup_failed', reason=str(error)[:200])
    analyse(out)
    print(f'Wyniki: {out}')


if __name__ == '__main__':
    try:
        main()
    except q.Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
