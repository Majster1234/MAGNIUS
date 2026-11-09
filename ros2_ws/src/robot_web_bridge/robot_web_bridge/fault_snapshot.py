"""Migawki awarii: jeden zwarty plik z oknami przed i po awarii ze wszystkich źródeł diagnostycznych.

Wyzwalacze `faults/trigger-*.json` tworzą rejestratory (C++ CAN: bus_fault, manual_fault, …; most: narastające
zbocze stanu awarii) dopiero po zapisaniu zdarzenia na dysk. Po upływie okna „po awarii” proces dopisuje migawkę
jako rekord do `faults/faults.rfs` (bezstratne kodowanie + xz, opis formatu w `fault_codec.py`):

- ramki i zdarzenia CAN (`can-*.jsonl`), zdarzenia mostka/panelu (`bridge-*.jsonl`),
- komunikaty `/rosout` wszystkich węzłów ROS (bufor w pamięci tego procesu),
- dziennik usługi systemd (stdout/stderr procesów, np. wyjątki Pythona) i jądra (gs_usb, USB),
- w nagłówku: wyzwalacze, liczności źródeł, konfigurację robota i plik parametrów wykonania.

Plik przechowuje co najmniej `keep_s` (domyślnie 900 s) okien awarii, najwyżej `max_bytes`. Odczyt:
`python3 -m robot_web_bridge.fault_snapshot --list` oraz `--dump <snapshot|last> [--output plik.jsonl]`.
Proces działa z obniżonym priorytetem i nie dotyka wątków CAN; nagranie ciągłe trwa dalej.
"""
import argparse
from collections import deque
from datetime import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

from .fault_codec import FORMAT, RecordStore, decode_snapshot, encode_snapshot

UTC_FIELD = re.compile(rb'"utc_ns":\s*(\d+)')
SOURCE_FIELD = re.compile(rb'"source":\s*"([^"]*)"')
SEGMENT_PATTERNS = ('can-*.jsonl', 'bridge-*.jsonl')
SECOND = 1_000_000_000
FAULTS_FILE = 'faults.rfs'


def local_time(stamp):
    return datetime.fromtimestamp(stamp / SECOND).strftime('%Y-%m-%d %H:%M:%S.%f')


def snapshot_id(stamp):
    return datetime.fromtimestamp(stamp / SECOND).strftime('%Y%m%d-%H%M%S-%f')[:-3]


def window_seconds(header):
    window = header.get('window') or {}
    try:
        return (window['end_utc_ns'] - window['start_utc_ns']) / SECOND
    except (KeyError, TypeError):
        return 0.0


def line_utc(line):
    match = UTC_FIELD.search(line)
    return int(match.group(1)) if match else None


def segment_rows(path, start, end):
    """Wiersze segmentu z utc_ns w [start, end]; segment wyraźnie poza oknem nie jest czytany w całości."""
    try:
        if path.stat().st_mtime_ns < start - 2 * SECOND:
            return []
        rows = []
        with path.open('rb') as file:
            first = line_utc(file.readline())
            if first is not None and first > end + 2 * SECOND:
                return []
            file.seek(0)
            for line in file:
                if not line.endswith(b'\n'):
                    break  # wiersz jeszcze zapisywany
                stamp = line_utc(line)
                if stamp is not None and start <= stamp <= end:
                    rows.append((stamp, line.rstrip(b'\n')))
        return rows
    except OSError:
        return []


def recording_rows(directories, start, end):
    rows, files = [], []
    for directory in directories:
        for pattern in SEGMENT_PATTERNS:
            for path in sorted(Path(directory).glob(pattern)):
                found = segment_rows(path, start, end)
                if found:
                    files.append(str(path))
                    rows += found
    return rows, files


def journal_unit():
    """Nazwa usługi systemd z cgroup procesu; poza systemd brak (dziennik usługi pominięty)."""
    try:
        for line in Path('/proc/self/cgroup').read_text().splitlines():
            for part in reversed(line.rsplit(':', 1)[-1].split('/')):
                if part.endswith('.service'):
                    return part
    except OSError:
        pass
    return None


def journal_rows(start, end, unit, run=subprocess.run):
    rows = []
    queries = [('kernel', ['-k'])]
    if unit:
        queries.insert(0, ('journal', ['-u', unit]))
    for source, selector in queries:
        command = ['journalctl', *selector, '-o', 'json', '--no-pager',
                   '--since', f'@{start // SECOND}', '--until', f'@{end // SECOND + 1}']
        try:
            output = run(command, capture_output=True, timeout=15, check=False).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        for line in output.splitlines():
            try:
                entry = json.loads(line)
                stamp = int(entry.get('__REALTIME_TIMESTAMP', 0)) * 1000
            except (ValueError, TypeError):
                continue
            if not start <= stamp <= end:
                continue
            message = entry.get('MESSAGE', '')
            if isinstance(message, list):
                message = bytes(message).decode('utf-8', 'replace')
            row = dict(source=source, event='log', utc_ns=stamp, priority=entry.get('PRIORITY'),
                       process=entry.get('SYSLOG_IDENTIFIER') or entry.get('_COMM'), pid=entry.get('_PID'),
                       message=message)
            rows.append((stamp, json.dumps(row, ensure_ascii=False, separators=(',', ':')).encode('utf-8')))
    return rows


class RosoutBuffer:
    """Ostatnie komunikaty /rosout wszystkich węzłów; starsze niż keep_ns są odrzucane."""
    LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

    def __init__(self, keep_ns):
        self.keep_ns = keep_ns
        self.rows = deque()
        self.lock = threading.Lock()

    def add(self, stamp, level, node, message, file='', function='', line=0):
        row = dict(source='rosout', event='log', utc_ns=stamp, level=self.LEVELS.get(level, level), node=node,
                   message=message, file=file, function=function, line=line)
        encoded = json.dumps(row, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        with self.lock:
            self.rows.append((stamp, encoded))
            limit = time.time_ns() - self.keep_ns
            while self.rows and self.rows[0][0] < limit:
                self.rows.popleft()

    def window(self, start, end):
        with self.lock:
            return [row for row in self.rows if start <= row[0] <= end]


def read_triggers(faults):
    triggers = []
    for path in sorted(Path(faults).glob('trigger-*.json')):
        try:
            data = json.loads(path.read_text(encoding='utf-8', errors='replace'))
            data['utc_ns'] = int(data['utc_ns'])
        except (OSError, ValueError, KeyError, TypeError):
            data = dict(source='unknown', event='invalid_trigger', utc_ns=path.stat().st_mtime_ns, detail=path.name)
        data['path'] = path
        triggers.append(data)
    return sorted(triggers, key=lambda trigger: trigger['utc_ns'])


def due_groups(triggers, now, post_ns, grace_ns):
    """Wyzwalacze w oknie `post` od pierwszego tworzą jedną migawkę; gotowa po końcu okna + zapas na zapis."""
    groups, current = [], []
    for trigger in triggers:
        if current and trigger['utc_ns'] > current[0]['utc_ns'] + post_ns:
            groups.append(current)
            current = []
        current.append(trigger)
    if current:
        groups.append(current)
    return [group for group in groups if now >= group[-1]['utc_ns'] + post_ns + grace_ns]


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def read_text(path):
    try:
        return Path(path).read_text(encoding='utf-8')
    except (OSError, TypeError):
        return None


class Collector:
    def __init__(self, directory, pre_s=10.0, post_s=5.0, keep_s=900.0, max_bytes=256 * 1024 * 1024,
                 configuration_dir=None, execution_parameters=None, unit=None, rosout=None, run=subprocess.run):
        self.directory = Path(directory)
        self.faults = self.directory / 'faults'
        self.faults.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.pre_ns, self.post_ns = int(pre_s * SECOND), int(post_s * SECOND)
        self.grace_ns = int(1.5 * SECOND)  # wątki zapisu opróżniają kolejki co ≤ 100 ms
        self.path = self.faults / FAULTS_FILE
        self.store = RecordStore(self.path, keep_s, max_bytes)
        self.configuration_dir = Path(configuration_dir) if configuration_dir else None
        self.execution_parameters = execution_parameters
        self.unit = unit
        self.rosout = rosout
        self.run = run

    def poll(self, now=None, force=False):
        now = time.time_ns() if now is None else now
        triggers = read_triggers(self.faults)
        groups = due_groups(triggers, float('inf') if force else now, self.post_ns, self.grace_ns)
        written = []
        for group in groups:
            written.append(self.snapshot(group))
            for trigger in group:
                try:
                    trigger['path'].unlink()
                except OSError:
                    pass
        return written

    def snapshot(self, group):
        first, last = group[0]['utc_ns'], group[-1]['utc_ns']
        start, end = first - self.pre_ns, last + self.post_ns
        rows, files = recording_rows([self.directory, self.directory / 'previous'], start, end)
        counts = {}
        rows += journal_rows(start, end, self.unit, self.run)
        if self.rosout is not None:
            rows += self.rosout.window(start, end)
        for _, line in rows:
            match = SOURCE_FIELD.search(line)
            source = match.group(1).decode('utf-8', 'replace') if match else 'unknown'
            source = 'can' if source == 'can_cpp' else source
            counts[source] = counts.get(source, 0) + 1
        rows.sort(key=lambda row: row[0])
        session = read_json(self.directory / 'session.json') or {}
        header = dict(
            source='fault_snapshot', event='fault_snapshot', utc_ns=first, created_utc_ns=time.time_ns(),
            recording_id=session.get('recording_id', ''),
            window=dict(pre_s=self.pre_ns / SECOND, post_s=self.post_ns / SECOND, start_utc_ns=start, end_utc_ns=end),
            triggers=[{key: value for key, value in trigger.items() if key != 'path'} for trigger in group],
            counts=counts, recording_files=files, journal_unit=self.unit,
            configuration=None if self.configuration_dir is None else dict(
                active=read_json(self.configuration_dir / 'active.json'),
                system_settings=read_json(self.configuration_dir / 'system_settings.json')),
            execution_parameters=dict(path=self.execution_parameters, text=read_text(self.execution_parameters)),
            session=session)
        snapshot = snapshot_id(first)
        header = dict(header, format=FORMAT, snapshot=snapshot, time=local_time(first), duration_s=(end - start) / SECOND,
                      rows=len(rows))
        decoded = [json.loads(line) for _, line in rows]
        self.store.append(self.meta(header), encode_snapshot(header, decoded))
        print(f'Migawka awarii {snapshot}: {self.path} ({len(rows)} wierszy, źródła {counts})', flush=True)
        return snapshot

    @staticmethod
    def meta(header):
        return dict(format=FORMAT, snapshot=header.get('snapshot'), time=header.get('time'), utc_ns=header.get('utc_ns'),
                    duration_s=header.get('duration_s', window_seconds(header)), rows=header.get('rows'),
                    counts=header.get('counts'),
                    triggers=[dict(source=t.get('source'), event=t.get('event'), detail=t.get('detail'))
                              for t in header.get('triggers', [])])

    def import_plain(self):
        """Jednorazowo przenosi migawki z dawnych plików JSONL (faults.jsonl, fault-*.jsonl) do faults.rfs."""
        for path in [*sorted(self.faults.glob('fault-*.jsonl')), self.faults / 'faults.jsonl']:
            if not path.exists():
                continue
            try:
                snapshots = []
                for line in path.read_bytes().splitlines():
                    if not line.strip():
                        continue
                    row = json.loads(line)
                    if row.get('source') == 'fault_snapshot':
                        snapshots.append((row, []))
                    elif snapshots:
                        row.pop('snapshot', None)
                        row.pop('time', None)
                        snapshots[-1][1].append(row)
                for header, rows in snapshots:
                    first = header['utc_ns']
                    header = dict(header, format=FORMAT, snapshot=header.get('snapshot') or snapshot_id(first),
                                  time=local_time(first), duration_s=window_seconds(header), rows=len(rows))
                    self.store.append(self.meta(header), encode_snapshot(header, rows))
                path.unlink()
            except (OSError, ValueError, KeyError) as error:
                print(f'Pominięto {path}: {error}', flush=True)


def list_snapshots(path):
    entries, _ = RecordStore(path).index()
    return [meta for _, _, meta in entries]


def dump_snapshot(path, snapshot, output):
    """Dekoduje migawkę (identyfikator lub 'last') do JSONL: nagłówek, potem wiersze z polem `time`."""
    store = RecordStore(path)
    entries, _ = store.index()
    chosen = entries[-1] if snapshot == 'last' and entries else next(
        (entry for entry in entries if entry[2].get('snapshot') == snapshot), None)
    if chosen is None:
        raise SystemExit(f'Brak migawki {snapshot} w {path}')
    _, payload = store.load(chosen[0])
    header, rows = decode_snapshot(payload)
    output.write(json.dumps(header, ensure_ascii=False) + '\n')
    for row in rows:
        output.write(json.dumps(dict(time=local_time(row['utc_ns']), **row), ensure_ascii=False) + '\n')
    return len(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Migawki awarii z nagrania diagnostycznego')
    parser.add_argument('--directory', default=os.environ.get('ROBOT_TRACE_DIR'))
    parser.add_argument('--pre', type=float, default=float(os.environ.get('ROBOT_FAULT_PRE_S', '10')))
    parser.add_argument('--post', type=float, default=float(os.environ.get('ROBOT_FAULT_POST_S', '5')))
    parser.add_argument('--keep-seconds', type=float, default=float(os.environ.get('ROBOT_FAULT_KEEP_S', '900')))
    parser.add_argument('--max-mib', type=float, default=float(os.environ.get('ROBOT_FAULT_MAX_MIB', '256')))
    parser.add_argument('--list', action='store_true', help='lista migawek w pliku')
    parser.add_argument('--dump', metavar='SNAPSHOT', help="identyfikator migawki lub 'last' → JSONL")
    parser.add_argument('--output', help='plik wyjściowy dla --dump (domyślnie stdout)')
    parser.add_argument('--configuration-dir', default=None)
    parser.add_argument('--execution-parameters', default=None)
    args, _ = parser.parse_known_args(sys.argv[1:] if argv is None else argv)  # pomija --ros-args
    if args.list or args.dump:
        path = Path(args.directory or 'log/manual-can') / 'faults' / FAULTS_FILE
        if args.list:
            for meta in list_snapshots(path):
                trigger = (meta.get('triggers') or [{}])[0]
                print(meta.get('snapshot'), meta.get('time'), f"{meta.get('duration_s', 0):.1f} s", meta.get('rows'),
                      trigger.get('event'), (trigger.get('detail') or '')[:120])
        else:
            if args.output:
                with open(args.output, 'w', encoding='utf-8') as output:
                    count = dump_snapshot(path, args.dump, output)
                print(f'{count} wierszy → {args.output}')
            else:
                dump_snapshot(path, args.dump, sys.stdout)
        return 0
    if not args.directory:
        print('Migawki awarii wyłączone: brak ROBOT_TRACE_DIR', flush=True)
        return 0
    try:
        os.nice(10)
    except OSError:
        pass
    rosout = RosoutBuffer(int((args.pre + args.post + 60) * SECOND))
    collector = Collector(args.directory, args.pre, args.post, args.keep_seconds, int(args.max_mib * 1024 * 1024),
                          args.configuration_dir, args.execution_parameters, journal_unit(), rosout)
    collector.import_plain()
    stop = threading.Event()

    def worker():
        while not stop.wait(0.5):
            try:
                collector.poll()
            except Exception as error:  # migawka nie może zatrzymać zbierania kolejnych
                print('Błąd migawki awarii:', error, flush=True)

    thread = threading.Thread(target=worker, name='fault-snapshot', daemon=True)
    thread.start()
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rcl_interfaces.msg import Log
    except ImportError as error:
        rclpy = None
        print('Bez /rosout w migawkach:', error, flush=True)
    try:
        if rclpy is None:
            while True:
                time.sleep(1)
        rclpy.init(args=sys.argv if argv is None else None)
        node = rclpy.create_node('fault_snapshot')

        def on_log(message):
            rosout.add(message.stamp.sec * SECOND + message.stamp.nanosec, message.level, message.name,
                       message.msg, message.file, message.function, message.line)

        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(Log, '/rosout', on_log, qos)
        print(f'Migawki awarii: {collector.path} ({args.pre:g} s przed, {args.post:g} s po, '
              f'co najmniej {args.keep_seconds:g} s okien, maks. {args.max_mib:g} MiB)', flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # ExternalShutdownException przy SIGINT z launch
        if type(error).__name__ != 'ExternalShutdownException':
            print('Migawki awarii zakończone błędem:', error, flush=True)
    finally:
        stop.set()
        thread.join(timeout=2)
        collector.poll(force=True)  # zamknięcie: zapisz oczekujące wyzwalacze z dostępnymi danymi
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
