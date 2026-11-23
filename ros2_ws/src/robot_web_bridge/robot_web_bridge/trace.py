"""Opcjonalna, ograniczona rejestracja poza executorem ROS i pętlą HTTP.

Pętla bridge-0…bridge-(N−1).jsonl nie zatrzymuje się po awarii. Narastające zbocze stanu awarii zapisuje
`fault_trigger`, a po jego zapisaniu na dysk wątek zapisu tworzy faults/trigger-*.json dla migawki awarii
(`fault_snapshot.py`)."""
import atexit
import json
import os
from pathlib import Path
import queue
import threading
import time


class TraceRecorder:
    def __init__(self, directory, max_bytes=8 * 1024 * 1024, segments=4):
        if max_bytes < 2048:
            raise ValueError('Za mały limit nagrania')
        if segments < 2:
            raise ValueError('Pętla nagrania wymaga co najmniej 2 segmentów')
        self.segments = segments
        self.directory = Path(directory)
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.max_bytes = max_bytes
        self.recording_id = os.environ.get('ROBOT_TRACE_SESSION', '')[:128]
        self.queue = queue.Queue(maxsize=512)
        self.dropped = 0
        self.failed = None
        self.fault_active = False
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._write, name='diagnostic-writer', daemon=True)
        self.thread.start()

    def record(self, event, **fields):
        if self.stopped.is_set() or self.failed:
            return
        row = dict(source='bridge', event=event, mono_ns=time.monotonic_ns(),
                   utc_ns=time.time_ns(), pid=os.getpid(), recording_id=self.recording_id,
                   dropped_total=self.dropped, **fields)
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)

    def fault_state(self, active, reason=''):
        """Wyzwalacz migawki tylko na narastającym zboczu awarii (stan przychodzi 20 Hz)."""
        active = bool(active)
        if active and not self.fault_active:
            self.record('fault_trigger', reason=str(reason)[:500])
        self.fault_active = active

    def _write_trigger(self, row):
        faults = self.directory / 'faults'
        try:
            faults.mkdir(mode=0o700, exist_ok=True)
            path = faults / f"trigger-{row['utc_ns']}-bridge-{os.getpid()}.json"
            temporary = path.with_name(path.name + '.tmp')
            temporary.write_text(json.dumps(dict(source='bridge', event=row['event'], utc_ns=row['utc_ns'],
                                                 mono_ns=row['mono_ns'], recording_id=self.recording_id,
                                                 detail=row.get('reason', '')), ensure_ascii=False) + '\n', encoding='utf-8')
            temporary.replace(path)
        except OSError as error:
            print('Wyzwalacz migawki awarii niedostępny:', error, flush=True)

    def _write(self):
        slot, size, file = 0, 0, None
        def write_row(row):
            nonlocal slot, size, file
            line = json.dumps(row, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n'
            length = len(line.encode('utf-8'))
            if length > self.max_bytes:
                self.dropped += 1
                line = json.dumps(dict(source='bridge', event='oversized_event', original_event=row['event'],
                                       mono_ns=time.monotonic_ns(), utc_ns=time.time_ns(), dropped_total=self.dropped)) + '\n'
                length = len(line.encode('utf-8'))
            if size + length > self.max_bytes:
                file.close()
                slot, size = (slot + 1) % self.segments, 0
                file = (self.directory / f'bridge-{slot}.jsonl').open('w', encoding='utf-8')
            file.write(line)
            size += length
        try:
            file = (self.directory / 'bridge-0.jsonl').open('w', encoding='utf-8')
            while not self.stopped.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    file.flush()
                    continue
                write_row(row)
                if self.queue.empty() or row['event'] == 'fault_trigger':
                    file.flush()
                if row['event'] == 'fault_trigger':
                    self._write_trigger(row)
            write_row(dict(source='bridge', event='recording_finished', recording_id=self.recording_id, mono_ns=time.monotonic_ns(),
                           utc_ns=time.time_ns(), dropped_total=self.dropped))
        except Exception as error:
            self.failed = str(error)
            print('Zatrzymano zapis diagnostyczny backendu:', error, flush=True)
        finally:
            if file is not None:
                file.close()


_recorder = None
_initialized = False
_init_lock = threading.Lock()


def recorder():
    global _recorder, _initialized
    if not _initialized:
        with _init_lock:
            if not _initialized:
                directory = os.environ.get('ROBOT_TRACE_DIR')
                if directory:
                    try:
                        # ROBOT_TRACE_BRIDGE_MIB: łączny budżet pętli (domyślnie 1024 MiB = 128 × 8 MiB).
                        try:
                            total = int(os.environ.get('ROBOT_TRACE_BRIDGE_MIB', '1024'))
                        except ValueError:
                            total = 1024
                        _recorder = TraceRecorder(directory, segments=max(2, min(4096, total // 8)))
                        atexit.register(_recorder.close)
                    except (OSError, ValueError) as error:
                        print('Zapis diagnostyczny backendu niedostępny:', error, flush=True)
                _initialized = True
    return _recorder


def recording():
    instance = recorder()
    return instance is not None and not instance.failed and not instance.stopped.is_set()


def trace(event, **fields):
    instance = recorder()
    if instance is not None:
        instance.record(event, **fields)


def fault_state(active, reason=''):
    instance = recorder()
    if instance is not None:
        instance.fault_state(active, reason)
