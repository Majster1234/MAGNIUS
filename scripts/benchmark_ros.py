#!/usr/bin/env python3
"""Benchmarki deweloperskie ROS/backendu, poza uruchomieniem aplikacji."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 1
SETTINGS = {'samples': 7, 'warmup_seconds': 1.0, 'min_series_ms': 200.0}


def command(arguments, timeout=10):
    return subprocess.run(arguments, cwd=ROOT, check=True, text=True,
                          capture_output=True, timeout=timeout).stdout.strip()


def digest_file(relative):
    path = ROOT / relative
    if not path.is_file() or not path.resolve().is_relative_to(ROOT):
        raise ValueError(f'Brak pliku w repozytorium: {relative}')
    return hashlib.sha256(path.read_bytes()).hexdigest()


def hardware():
    model = platform.machine()
    for line in Path('/proc/cpuinfo').read_text().splitlines():
        if line.startswith('model name'):
            model = line.partition(':')[2].strip()
            break
    governors = set()
    for path in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_governor'):
        try:
            governors.add(path.read_text().strip())
        except OSError:
            governors.add('unavailable')
    return {'system': platform.system(), 'kernel': platform.release(),
            'architecture': platform.machine(), 'cpu_model': model,
            'logical_cpus': os.cpu_count(), 'affinity': sorted(os.sched_getaffinity(0)),
            'governors': sorted(governors)}


def provenance(sources=()):
    try:
        revision = command(['git', 'rev-parse', 'HEAD'])
        dirty = bool(command(['git', 'status', '--porcelain']))
        diff_hash = hashlib.sha256(command(['git', 'diff', '--binary', 'HEAD']).encode()).hexdigest()
    except (subprocess.SubprocessError, OSError):
        revision, dirty, diff_hash = None, None, None
    return {'revision': revision, 'dirty': dirty, 'tracked_diff_sha256': diff_hash,
            'sources_sha256': {source: digest_file(source) for source in sorted(set(sources))}}


def record_base(kind, context, sources=()):
    return {'schema_version': SCHEMA, 'kind': kind, 'status': 'complete',
            'recorded_at': datetime.now(timezone.utc).isoformat(), 'context': context,
            'hardware': hardware(), 'code': provenance(sources),
            'observer_python': platform.python_version(), 'load_average': list(os.getloadavg())}


def positive_number(value, name, integer=False):
    valid = type(value) is int if integer else type(value) in (int, float)
    if not valid or not math.isfinite(value) or value <= 0:
        raise ValueError(f'Nieprawidłowe pole {name}')
    return value


def summarize(samples, iterations):
    positive_number(iterations, 'iterations', integer=True)
    if not isinstance(samples, list) or not samples:
        raise ValueError('Brak serii pomiarowych')
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError('Nieprawidłowa seria')
        positive_number(sample.get('wall_ns'), 'wall_ns')
        positive_number(sample.get('cpu_ns'), 'cpu_ns')
    wall = [sample['wall_ns'] / iterations for sample in samples]
    cpu = [sample['cpu_ns'] / iterations for sample in samples]
    def spread(values):
        return {'median': statistics.median(values), 'min': min(values), 'max': max(values)}
    return {'wall_ns_per_cycle': spread(wall), 'cpu_ns_per_cycle': spread(cpu),
            'cycles_per_second': spread([1e9 / value for value in wall])}


def load_json(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f'Oczekiwano obiektu JSON: {path}')
    return value


def validate_record(record):
    """Sprawdzenie danych wejściowych przed jakimkolwiek porównaniem."""
    if record.get('schema_version') != SCHEMA:
        raise ValueError('Nieobsługiwana wersja pomiaru')
    if record.get('status') != 'complete':
        raise ValueError('Pomiar jest niepełny lub zakończony błędem')
    if not isinstance(record.get('context'), str) or not record['context']:
        raise ValueError('Brak opisu warunków pomiaru')
    machine = record.get('hardware')
    if not isinstance(machine, dict) or not all(
            key in machine for key in ('cpu_model', 'kernel', 'affinity', 'logical_cpus')):
        raise ValueError('Brak identyfikacji sprzętu')
    if record.get('kind') == 'run':
        if (not isinstance(record.get('case'), dict) or not all(
                record['case'].get(key) for key in ('id', 'language', 'sources', 'workload'))):
            raise ValueError('Brak identyfikacji scenariusza')
        if not isinstance(record.get('settings'), dict) or record['settings'] != SETTINGS:
            raise ValueError('Nieobsługiwane ustawienia benchmarku')
        if not isinstance(record.get('runtime'), dict) or not record['runtime']:
            raise ValueError('Brak identyfikacji środowiska wykonawczego')
        if not isinstance(record.get('harness_sha256'), dict) or not record['harness_sha256']:
            raise ValueError('Brak identyfikacji pomocników pomiarowych')
        if not isinstance(record.get('samples'), list) or len(record['samples']) != record['settings']['samples']:
            raise ValueError('Niepełna liczba serii')
        summarize(record['samples'], record.get('iterations'))
    elif record.get('kind') == 'watch':
        positive_number(record.get('duration_seconds'), 'duration_seconds')
        positive_number(record.get('interval_seconds'), 'interval_seconds')
        if not isinstance(record.get('processes'), dict) or not record['processes']:
            raise ValueError('Brak pomiarów procesów')
        for process in record['processes'].values():
            if (not isinstance(process, dict) or not isinstance(process.get('samples'), list)
                    or not process['samples']):
                raise ValueError('Brak próbek procesu')
            for sample in process['samples']:
                if not isinstance(sample, dict):
                    raise ValueError('Nieprawidłowa próbka procesu')
                positive_number(sample.get('elapsed_seconds'), 'elapsed_seconds')
                for field in ('cpu_seconds', 'cpu_percent'):
                    value = sample.get(field)
                    if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
                        raise ValueError(f'Nieprawidłowe pole {field}')
                if not math.isclose(sample['cpu_percent'],
                                    100 * sample['cpu_seconds'] / sample['elapsed_seconds'],
                                    rel_tol=1e-9, abs_tol=1e-9):
                    raise ValueError('Niespójne obciążenie CPU w próbce')
            if not process.get('executable') or not isinstance(process.get('affinity'), list):
                raise ValueError('Brak identyfikacji procesu')
    else:
        raise ValueError('Nieznany rodzaj pomiaru')


def run_case(case_id, registry, context='offline', baseline=None):
    cases = load_json(registry)
    if case_id not in cases or not isinstance(cases[case_id], dict):
        raise ValueError(f'Nieznany scenariusz: {case_id}')
    case = cases[case_id]
    if (case.get('language') not in ('python', 'cpp')
            or not isinstance(case.get('workload'), str) or not case['workload'].strip()):
        raise ValueError('Scenariusz wymaga language i workload')
    sources = case.get('sources')
    if not isinstance(sources, list) or not sources or any(
            not isinstance(item, dict) or not isinstance(item.get('path'), str) or not item['path']
            or not isinstance(item.get('symbols'), list) or not item['symbols']
            or not all(isinstance(symbol, str) and symbol for symbol in item['symbols'])
            for item in sources):
        raise ValueError('Scenariusz wymaga powiązania z plikami i funkcjami')
    adapters = case.get('adapter_sources')
    if not isinstance(adapters, list) or not adapters:
        raise ValueError('Scenariusz wymaga adapter_sources')
    helper = 'benchmarks/harness.py' if case['language'] == 'python' else 'benchmarks/harness.hpp'
    harness = {name: digest_file(name) for name in [*adapters, helper, 'scripts/benchmark_ros.py']}
    record = record_base('run', context, [item['path'] for item in sources])
    record.update(case={**case, 'id': case_id}, settings=SETTINGS.copy(), harness_sha256=harness)
    if case['language'] == 'python':
        module = case.get('module')
        if not isinstance(module, str) or not re.fullmatch(r'[A-Za-z_][\w]*(\.[A-Za-z_][\w]*)*', module):
            raise ValueError('Nieprawidłowy moduł adaptera')
        invocation = [sys.executable, *(['-' + 'O' * sys.flags.optimize] if sys.flags.optimize else []),
                      '-m', module]
        record['runtime'] = {'python': platform.python_version(),
                             'implementation': platform.python_implementation(),
                             'optimize': sys.flags.optimize}
    else:
        target = case.get('target')
        if not isinstance(target, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', target):
            raise ValueError('Nieprawidłowy target CMake')
        harness['benchmarks/CMakeLists.txt'] = digest_file('benchmarks/CMakeLists.txt')
        build = Path(os.environ.get('ROBOT_BENCH_BUILD_DIR', ROOT / 'build/benchmarks')).resolve()
        command(['cmake', '-S', str(ROOT / 'benchmarks'), '-B', str(build),
                 '-DCMAKE_BUILD_TYPE=Release'], timeout=120)
        command(['cmake', '--build', str(build), '--config', 'Release', '--target', target], timeout=120)
        cache = {}
        for line in (build / 'CMakeCache.txt').read_text().splitlines():
            if '=' in line and ':' in line.partition('=')[0]:
                key, _, value = line.partition('=')
                cache[key.partition(':')[0]] = value
        compiler = cache['CMAKE_CXX_COMPILER']
        record['runtime'] = {'compiler': command([compiler, '--version']).splitlines()[0],
                             'build_type': cache['CMAKE_BUILD_TYPE'],
                             'flags': [cache.get('CMAKE_CXX_FLAGS', ''),
                                       cache.get('CMAKE_CXX_FLAGS_RELEASE', '')]}
        invocation = [str(build / target)]
    iterations = 0
    if baseline is not None:
        validate_record(baseline)
        for key in ('kind', 'case', 'context', 'hardware', 'settings', 'harness_sha256', 'runtime'):
            if baseline.get(key) != record.get(key):
                raise ValueError(f'Pomiar bazowy nie pasuje: {key}')
        iterations = baseline['iterations']
    try:
        raw = json.loads(command([*invocation, '--iterations', str(iterations)], timeout=30))
        if not isinstance(raw, dict):
            raise ValueError('Adapter nie zwrócił obiektu JSON')
        record['iterations'] = raw.get('iterations')
        record['samples'] = raw.get('samples')
        validate_record(record)
        if iterations and record['iterations'] != iterations:
            raise ValueError('Adapter zmienił liczbę cykli względem pomiaru bazowego')
        record['summary'] = summarize(record['samples'], record['iterations'])
    except (subprocess.SubprocessError, ValueError, TypeError) as error:
        record['status'] = 'error'
        # Uszkodzone dane (np. NaN) nie mogą uniemożliwić zapisania statusu błędu.
        record.pop('iterations', None)
        record.pop('samples', None)
        record['error'] = ('Przekroczono limit 30 s' if isinstance(error, subprocess.TimeoutExpired)
                           else f'Błąd pomiaru: {type(error).__name__}: {error}')
    return record


def read_process(pid, proc_root=Path('/proc')):
    """utime/stime obejmują wątki procesu, bez czasów procesów potomnych."""
    path = proc_root / str(pid)
    text = (path / 'stat').read_text()
    # Nazwa procesu w nawiasach może zawierać spacje i znaki ')'.
    fields = text[text.rfind(')') + 2:].split()
    if len(fields) < 20 or fields[0] in ('Z', 'X', 'x'):
        raise ValueError('Proces zakończony lub nieprawidłowy /proc/stat')
    return {'ticks': int(fields[11]) + int(fields[12]), 'start_ticks': int(fields[19]),
            'executable': os.readlink(path / 'exe'), 'timestamp': time.monotonic()}


def cpu_interval(before, after, ticks_per_second):
    if (before['start_ticks'], before['executable']) != (after['start_ticks'], after['executable']):
        raise ValueError('Zmiana tożsamości procesu (PID użyty ponownie lub exec)')
    elapsed = after['timestamp'] - before['timestamp']
    ticks = after['ticks'] - before['ticks']
    if elapsed <= 0 or ticks < 0:
        raise ValueError('Nieprawidłowy przyrost czasu procesu')
    cpu = ticks / ticks_per_second
    return {'elapsed_seconds': elapsed, 'cpu_seconds': cpu, 'cpu_percent': 100 * cpu / elapsed}


def process_summary(samples):
    elapsed = sum(sample['elapsed_seconds'] for sample in samples)
    cpu = sum(sample['cpu_seconds'] for sample in samples)
    return {'cpu_seconds': cpu, 'elapsed_seconds': elapsed,
            'cpu_percent_mean': 100 * cpu / elapsed if elapsed else None,
            'cpu_percent_max': max((sample['cpu_percent'] for sample in samples), default=None)}


def watch_processes(targets, context, duration=30.0, interval=1.0,
                    reader=read_process, clock=time.monotonic, sleep=time.sleep):
    positive_number(duration, 'duration')
    positive_number(interval, 'interval')
    if not targets or len(set(targets.values())) != len(targets):
        raise ValueError('Wymagane różne, jawnie wskazane PID-y')
    record = record_base('watch', context)
    record.update(duration_seconds=duration, interval_seconds=interval, processes={}, errors=[])
    previous = {}
    ticks_per_second = os.sysconf('SC_CLK_TCK')
    start = clock()
    deadline = start + duration
    try:
        for label, pid in targets.items():
            previous[label] = reader(pid)
            record['processes'][label] = {
                'pid': pid, 'start_ticks': previous[label]['start_ticks'],
                'executable': Path(previous[label]['executable']).name,
                'affinity': sorted(os.sched_getaffinity(pid)), 'samples': []}
        next_sample = min(start + interval, deadline)
        while True:
            sleep(max(0, next_sample - clock()))
            for label, pid in targets.items():
                current = reader(pid)
                if sorted(os.sched_getaffinity(pid)) != record['processes'][label]['affinity']:
                    raise ValueError(f'{label}: zmiana affinity procesu')
                sample = cpu_interval(previous[label], current, ticks_per_second)
                record['processes'][label]['samples'].append(sample)
                previous[label] = current
            if clock() >= deadline:
                break
            next_sample = min(next_sample + interval, deadline)
            # Po opóźnieniu próbkowania nie generuj serii niemal zerowych odstępów.
            if next_sample <= clock():
                next_sample = min(clock() + interval, deadline)
    except (OSError, ValueError) as error:
        record['status'] = 'incomplete'
        record['errors'].append(f'{type(error).__name__}: {error}')
    for process in record['processes'].values():
        process['summary'] = process_summary(process['samples'])
    return record


def cost_reduction(before, after):
    return 100 * (before - after) / before if before > 0 else None


def compare(before, after):
    result = {'schema_version': SCHEMA, 'kind': 'compare', 'status': 'not_comparable',
              'recorded_at': datetime.now(timezone.utc).isoformat(),
              'before': before, 'after': after, 'reasons': [], 'metrics': {}}
    try:
        validate_record(after)
        if before is not None:
            validate_record(before)
    except (ValueError, TypeError, KeyError) as error:
        result['reasons'].append(str(error))
        return result
    if before is None:
        result.update(status='no_baseline', reasons=['Brak pomiaru bazowego; brak różnic procentowych'])
        return result
    keys = ['kind', 'context', 'hardware']
    if before['kind'] == after['kind'] == 'run':
        keys += ['case', 'settings', 'harness_sha256', 'runtime', 'iterations']
    for key in keys:
        if before.get(key) != after.get(key):
            result['reasons'].append(f'Różne pole: {key}')
    if result['reasons']:
        return result
    if before['kind'] == 'run':
        first = summarize(before['samples'], before['iterations'])
        second = summarize(after['samples'], after['iterations'])
        for name in ('wall_ns_per_cycle', 'cpu_ns_per_cycle'):
            result['metrics'][name] = {'before': first[name], 'after': second[name],
                                      'cost_reduction_percent': cost_reduction(
                                          first[name]['median'], second[name]['median'])}
        result['metrics']['cycles_per_second'] = {'before': first['cycles_per_second'],
                                                  'after': second['cycles_per_second']}
    else:
        if (before['duration_seconds'], before['interval_seconds']) != (
                after['duration_seconds'], after['interval_seconds']):
            result['reasons'].append('Różny czas obserwacji lub okres próbkowania')
        if before['processes'].keys() != after['processes'].keys():
            result['reasons'].append('Różne etykiety obserwowanych procesów')
        for label, process in before['processes'].items():
            other = after['processes'].get(label, {})
            if any(process.get(key) != other.get(key) for key in ('executable', 'affinity')):
                result['reasons'].append(f'{label}: inne executable lub affinity')
        if result['reasons']:
            return result
        for label, process in before['processes'].items():
            first = process_summary(process['samples'])
            second = process_summary(after['processes'][label]['samples'])
            result['metrics'][label] = {'before': first, 'after': second,
                                      'cpu_mean_reduction_percent': cost_reduction(
                                          first['cpu_percent_mean'], second['cpu_percent_mean'])}
    result['status'] = 'comparable'
    return result


def save_record(record, output=None):
    if output is None:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        output = ROOT / 'log/benchmarks' / f'{stamp}-{record["kind"]}.json'
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Nie nadpisuj przypadkiem pomiaru bazowego.
    payload = json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    with path.open('x') as stream:
        stream.write(payload)
    return path


def parse_targets(values):
    targets = {}
    for value in values:
        label, separator, pid = value.partition('=')
        if not separator or not re.fullmatch(r'[A-Za-z0-9_.-]+', label) or label in targets:
            raise ValueError('Podaj unikalne --pid ETYKIETA=PID')
        if not pid.isdecimal() or int(pid) <= 0:
            raise ValueError('PID musi być dodatnią liczbą całkowitą')
        targets[label] = int(pid)
    return targets


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='action', required=True)
    run = subparsers.add_parser('run', help='Wybrany podprogram; bez automatycznego wyboru całego repo')
    run.add_argument('--case', required=True)
    run.add_argument('--registry', type=Path, default=ROOT / 'benchmarks/cases.json')
    run.add_argument('--baseline', type=Path)
    run.add_argument('--context', default='offline')
    watch = subparsers.add_parser('watch', help='Odczyt CPU już działających procesów')
    watch.add_argument('--pid', action='append', required=True, metavar='ETYKIETA=PID')
    watch.add_argument('--context', required=True, help='Opis porównywalnego obciążenia')
    watch.add_argument('--duration', type=float, default=30.0)
    watch.add_argument('--interval', type=float, default=1.0)
    comparison = subparsers.add_parser('compare', help='Porównanie wyników przed/po')
    comparison.add_argument('--before', type=Path)
    comparison.add_argument('--after', type=Path, required=True)
    for subparser in (run, watch, comparison):
        subparser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.action == 'run':
            if not args.context.strip():
                raise ValueError('Opis warunków nie może być pusty')
            result = run_case(args.case, args.registry, args.context,
                              load_json(args.baseline) if args.baseline else None)
        elif args.action == 'watch':
            if not args.context.strip():
                raise ValueError('Opis warunków nie może być pusty')
            result = watch_processes(parse_targets(args.pid), args.context, args.duration, args.interval)
        else:
            result = compare(load_json(args.before) if args.before else None, load_json(args.after))
        path = save_record(result, args.output)
        print(f'{result["status"]}: {path}')
        if result.get('summary'):
            print(json.dumps(result['summary'], ensure_ascii=False, indent=2))
        if result.get('metrics'):
            print(json.dumps(result['metrics'], ensure_ascii=False, indent=2))
        if result.get('processes'):
            print(json.dumps({label: process['summary'] for label, process in result['processes'].items()},
                             ensure_ascii=False, indent=2))
        for reason in result.get('reasons', []) + result.get('errors', []):
            print(reason)
        if result.get('error'):
            print(result['error'])
        return 0 if result['status'] in ('complete', 'comparable') else 2
    except (OSError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as error:
        print(f'Błąd: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
