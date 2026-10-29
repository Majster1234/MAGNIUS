"""Ciągły trace wałów. Pliki o stałym rozmiarze; dysk poza callbackiem ROS."""
from collections import deque
import csv
import fcntl
import io
import os
from pathlib import Path
import queue
import struct
import threading
import time
import zlib

LIMIT = 20_000_000
HEADER_BYTES = 4096
HEADER = struct.Struct('<8sIIQQ')
RECORD = struct.Struct('<QQQQQqII')
MAGIC = b'SHAFT01\0'
BLOCK = 256


def centidegrees(ticks):
    value = (abs(int(ticks)) * 36000 + 8192) // 16384
    return -value if ticks < 0 else value


def degrees_text(value):
    return ('-' if value < 0 else '') + f'{abs(value)//100}.{abs(value)%100:02d}'


def reduce_points(points, maximum):
    """Min/max w kolejności próbek. Luki w koszyku nigdy nie stają się linią."""
    if len(points) <= maximum:
        return points
    buckets = max(1, maximum // 2)
    result = []
    for i in range(buckets):
        chunk = points[i*len(points)//buckets:(i+1)*len(points)//buckets]
        chosen = sorted({min(range(len(chunk)), key=lambda j: chunk[j]['position_cdeg']),
                         max(range(len(chunk)), key=lambda j: chunk[j]['position_cdeg'])})
        gap = any(p['gap'] for p in chunk)
        for j in chosen:
            point = dict(chunk[j])
            point['gap'] = point['gap'] or gap
            result.append(point)
    return result


def point_from_record(data, ordinal, axis):
    if len(data) != RECORD.size or zlib.crc32(data[:-4]) != struct.unpack_from('<I', data, RECORD.size-4)[0]:
        return None
    index, sequence, segment, mono, utc, position, flags, _ = RECORD.unpack(data)
    if index != ordinal:
        return None
    return dict(axis=axis, sequence=str(sequence), segment=str(segment), monotonic_ns=str(mono),
                utc_ns=str(utc), time_ms=utc/1e6, position_cdeg=position, gap=bool(flags & 3), ordinal=ordinal)


class AxisRing:
    def __init__(self, path, axis, maximum=LIMIT):
        if maximum < HEADER_BYTES + 2*RECORD.size or maximum > LIMIT:
            raise ValueError('Nieprawidłowy limit pliku pozycji')
        self.axis, self.maximum = axis, maximum
        self.capacity = (maximum-HEADER_BYTES)//RECORD.size
        self.lock = threading.RLock()
        self.blocks = {}
        self.corrupt = 0
        self.pruned_block = 0
        self.dirty = False
        self.fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            size = os.fstat(self.fd).st_size
            if size not in (0, maximum):
                raise ValueError('Niezgodny rozmiar istniejącego pliku pozycji')
            if not size:
                os.ftruncate(self.fd, maximum)
                self.next = 1
                self._header()
            else:
                headers = []
                for offset in (0, 64):
                    data = os.pread(self.fd, HEADER.size+4, offset)
                    if len(data) != HEADER.size+4 or zlib.crc32(data[:-4]) != struct.unpack_from('<I', data, HEADER.size)[0]:
                        continue
                    magic, version, record_size, capacity, next_ = HEADER.unpack(data[:-4])
                    if magic == MAGIC and version == 1 and record_size == RECORD.size and capacity == self.capacity and next_ >= 1:
                        headers.append(next_)
                if not headers:
                    raise ValueError('Uszkodzone nagłówki pliku pozycji; zachowano plik')
                self.next = max(headers)
                # Odzysk kompletnych rekordów zapisanych tuż przed nagłówkiem.
                for _ in range(self.capacity):
                    if not self.read(self.next):
                        break
                    self.next += 1
                snapshot = os.pread(self.fd, maximum, 0)
                for ordinal in range(max(1, self.next-self.capacity), self.next):
                    point = self.read(ordinal, snapshot)
                    if point:
                        self._index(point)
                    else:
                        self.corrupt += 1
                self.pruned_block = (max(1, self.next-self.capacity)-1)//BLOCK
        except BaseException:
            os.close(self.fd)
            raise

    def _offset(self, ordinal):
        return HEADER_BYTES + ((ordinal-1) % self.capacity)*RECORD.size

    def _header(self):
        payload = HEADER.pack(MAGIC, 1, RECORD.size, self.capacity, self.next)
        data = payload + struct.pack('<I', zlib.crc32(payload))
        if os.pwrite(self.fd, data, (self.next % 2)*64) != len(data):
            raise OSError('Niepełny zapis nagłówka trace')

    def read(self, ordinal, snapshot=None):
        offset = self._offset(ordinal)
        data = os.pread(self.fd, RECORD.size, offset) if snapshot is None else snapshot[offset:offset+RECORD.size]
        return point_from_record(data, ordinal, self.axis)

    def _index(self, point):
        block = (point['ordinal']-1)//BLOCK
        current = self.blocks.get(block)
        if current is None:
            self.blocks[block] = dict(first=point, last=point, minimum=point, maximum=point,
                                      begin=point['time_ms'], end=point['time_ms'], gap=point['gap'])
        else:
            current['last'] = point
            current['begin'] = min(current['begin'], point['time_ms'])
            current['end'] = max(current['end'], point['time_ms'])
            current['gap'] |= point['gap']
            if point['position_cdeg'] < current['minimum']['position_cdeg']:
                current['minimum'] = point
            if point['position_cdeg'] > current['maximum']['position_cdeg']:
                current['maximum'] = point

    def append(self, point):
        with self.lock:
            payload = struct.pack('<QQQQQqI', self.next, int(point['sequence']), int(point['segment']),
                                  int(point['monotonic_ns']), int(point['utc_ns']), point['position_cdeg'],
                                  2 if point['gap'] else 0)
            data = payload + struct.pack('<I', zlib.crc32(payload))
            if os.pwrite(self.fd, data, self._offset(self.next)) != len(data):
                raise OSError('Niepełny zapis pozycji')
            self._index(dict(point, ordinal=self.next))
            self.next += 1
            first_block = (max(1, self.next-self.capacity)-1)//BLOCK
            if first_block != self.pruned_block:
                for block in list(self.blocks):
                    if block < first_block:
                        del self.blocks[block]
                self.pruned_block = first_block
            self._header()
            self.dirty = True

    def history(self, start, end, maximum, after=0):
        with self.lock:
            first, last = max(1, self.next-self.capacity, after+1), self.next-1
            blocks = [(key, dict(value)) for key, value in self.blocks.items()
                      if value['end'] >= start and value['begin'] <= end]
        points = []
        estimated = sum(min(last, (key+1)*BLOCK)-max(first, key*BLOCK+1)+1 for key, _ in blocks)
        gap = False
        for key, block in sorted(blocks):
            low, high = max(first, key*BLOCK+1), min(last, (key+1)*BLOCK)
            if low > high:
                continue
            if estimated > maximum and low == key*BLOCK+1 and high == (key+1)*BLOCK and start <= block['begin'] and end >= block['end']:
                chosen = {p['ordinal']: p for p in (block['first'], block['minimum'], block['maximum'], block['last'])}
                for _, p in sorted(chosen.items()):
                    checked = self.read(p['ordinal'])
                    if checked:
                        points.append(dict(checked, gap=checked['gap'] or block['gap'] or gap))
                        gap = False
                    else:
                        gap = True
            else:
                for ordinal in range(low, high+1):
                    p = self.read(ordinal)
                    if p is None:
                        gap = True
                    elif start <= p['time_ms'] <= end:
                        points.append(dict(p, gap=p['gap'] or gap))
                        gap = False
        return reduce_points(points, maximum)

    def snapshot(self):
        with self.lock:
            return os.pread(self.fd, self.maximum, 0), max(1, self.next-self.capacity), self.next

    def page(self, start, end, maximum, after=0):
        """Strona surowych próbek; redukowany podgląd ma osobną ścieżkę."""
        with self.lock:
            first, last = max(1, self.next-self.capacity, after+1), self.next-1
            blocks = [(key, dict(value)) for key, value in self.blocks.items()
                      if value['end'] >= start and value['begin'] <= end]
        points, gap = [], False
        for key, _ in sorted(blocks):
            for ordinal in range(max(first,key*BLOCK+1), min(last,(key+1)*BLOCK)+1):
                p = self.read(ordinal)
                if p is None:
                    gap = True
                elif start <= p['time_ms'] <= end:
                    if len(points) == maximum:
                        return points, True
                    points.append(dict(p, gap=p['gap'] or gap))
                    gap = False
        return points, False

    def status(self):
        with self.lock:
            first = max(1, self.next-self.capacity)
            times = list(self.blocks.values())
            first_point = self.read(first) if first < self.next else None
            return dict(file_bytes=os.fstat(self.fd).st_size, retained_records=self.next-first,
                        capacity=self.capacity, corrupt_records=self.corrupt,
                        first_ms=first_point['time_ms'] if first_point else min((b['begin'] for b in times), default=None),
                        last_ms=max((b['end'] for b in times), default=None))

    def flush(self):
        if self.dirty:
            os.fsync(self.fd)
            self.dirty = False

    def close(self):
        os.close(self.fd)


class ShaftTraceStore:
    def __init__(self, directory=None, maximum=LIMIT, queue_size=4096, clock_ns=time.monotonic_ns):
        default = Path(os.environ.get('XDG_STATE_HOME') or str(Path.home()/'.local/state'))/'robot/shaft-trace'
        self.directory = Path(directory) if directory else default
        self.maximum, self.clock_ns = maximum, clock_ns
        self.lock = threading.RLock()
        self.queue = queue.Queue(maxsize=queue_size)
        self.stopping = threading.Event()
        self.rings, self.errors, self.latest, self.previous = {}, {}, {}, {}
        self.points = deque(maxlen=4096)
        self.cursor = 0
        self.source_dropped = self.transport_dropped = self.writer_dropped = 0
        self.last_source_dropped = 0
        self.last_batch = None
        self.saved = 0
        self.start_error = None
        try:
            self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
            for axis in range(1, 7):
                path = self.directory/f'axis-{axis}.bin'
                if path.exists():
                    try:
                        self.rings[axis] = AxisRing(path, axis, maximum)
                    except (OSError, ValueError) as error:
                        self.errors[axis] = str(error)
        except OSError as error:
            self.start_error = str(error)
        self.thread = threading.Thread(target=self._write, name='shaft-position-writer', daemon=True)
        self.thread.start()

    def ingest(self, batch):
        with self.lock:
            self.last_batch = self.clock_ns()
            dropped = int(batch.dropped_total)
            self.source_dropped += dropped-self.last_source_dropped if dropped >= self.last_source_dropped else dropped
            self.last_source_dropped = dropped
            for sample in batch.samples[:256]:
                axis = int(sample.axis)
                if axis not in range(1, 7) or sample.monotonic_ns <= 0 or sample.utc_ns <= 0:
                    continue
                previous = self.previous.get(axis)
                segment, sequence = int(sample.segment), int(sample.sequence)
                if previous and previous[0] == segment and sequence <= previous[1]:
                    continue
                missing = max(0, sequence-previous[1]-1) if previous and previous[0] == segment else 0
                self.transport_dropped += missing
                gap = bool(sample.flags & 1) or previous is None or previous[0] != segment or missing > 0
                if previous and not 0 < int(sample.monotonic_ns)-previous[2] <= 80_000_000:
                    gap = True
                point = dict(axis=axis, sequence=str(sequence), segment=str(segment),
                             monotonic_ns=str(int(sample.monotonic_ns)), utc_ns=str(int(sample.utc_ns)),
                             time_ms=int(sample.utc_ns)/1e6, position_cdeg=centidegrees(sample.ticks), gap=gap)
                self.previous[axis] = (segment, sequence, int(sample.monotonic_ns))
                self.latest[axis] = point
                self.cursor += 1
                self.points.append((self.cursor, point))
                try:
                    self.queue.put_nowait(point)
                except queue.Full:
                    self.writer_dropped += 1

    def _write(self):
        flushed = time.monotonic()
        last_written = {}
        while not self.stopping.is_set() or not self.queue.empty():
            try:
                point = self.queue.get(timeout=.05)
            except queue.Empty:
                point = None
            if point is not None:
                axis = point['axis']
                try:
                    if self.start_error or axis in self.errors:
                        raise OSError(self.start_error or self.errors[axis])
                    ring = self.rings.get(axis)
                    if ring is None:
                        ring = AxisRing(self.directory/f'axis-{axis}.bin', axis, self.maximum)
                        with self.lock:
                            self.rings[axis] = ring
                    old = last_written.get(axis)
                    if old and (old[0] != point['segment'] or int(point['sequence']) != old[1]+1):
                        point = dict(point, gap=True)
                    ring.append(point)
                    last_written[axis] = (point['segment'], int(point['sequence']))
                    self.saved += 1
                except (OSError, ValueError) as error:
                    with self.lock:
                        self.errors[axis] = str(error)
                        self.writer_dropped += 1
                finally:
                    self.queue.task_done()
            if time.monotonic()-flushed >= 1:
                self._flush()
                flushed = time.monotonic()
        self._flush()

    def _flush(self):
        for axis, ring in list(self.rings.items()):
            try:
                ring.flush()
            except OSError as error:
                with self.lock:
                    self.errors[axis] = str(error)

    def wait_idle(self):
        self.queue.join()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=5)
        if not self.thread.is_alive():
            for ring in self.rings.values():
                ring.close()

    def status(self):
        with self.lock:
            now = self.clock_ns()
            axes = []
            for axis in range(1, 7):
                point = self.latest.get(axis)
                age = (now-int(point['monotonic_ns']))/1e9 if point else None
                ring = self.rings.get(axis)
                info = ring.status() if ring else dict(file_bytes=0, retained_records=0, first_ms=None, last_ms=None, corrupt_records=0)
                axes.append(dict(axis=axis, limit_bytes=self.maximum, recording=not self.start_error and axis not in self.errors,
                                 error=self.start_error or self.errors.get(axis), fresh=age is not None and 0 <= age < .25,
                                 age_seconds=age, latest=point, **info))
            return dict(recording=not self.stopping.is_set() and not self.start_error and not self.errors,
                        server_time_ms=time.time_ns()/1e6,
                        connected=self.last_batch is not None and 0 <= now-self.last_batch < 500_000_000,
                        source_dropped=self.source_dropped, missing_samples=self.transport_dropped,
                        writer_dropped=self.writer_dropped, queued=self.queue.qsize(), saved=self.saved,
                        cursor=self.cursor, axes=axes)

    def live(self, cursor=0, maximum=2000):
        with self.lock:
            expired = bool(self.points and cursor and cursor < self.points[0][0]-1)
            # Zwykle przybywa kilka punktów: nie przeglądaj całej historii na klienta.
            values = []
            for item in reversed(self.points):
                if item[0] <= cursor:
                    break
                values.append(item)
            values.reverse()
            values = values[:maximum]
            last = values[-1][0] if values else self.cursor
            return dict(type='shaft_trace', cursor=last, expired=expired,
                        samples=[p for _, p in values], status=self.status())

    def history(self, axes, start, end, maximum=2000, after=None, reduction='minmax'):
        result = []
        for axis in axes:
            ring = self.rings.get(axis)
            cursor = (after or {}).get(axis, 0)
            more = False
            if ring and reduction == 'none':
                points, more = ring.page(start, end, maximum, cursor)
            else:
                points = ring.history(start, end, maximum, cursor) if ring else []
            result.append(dict(axis=axis, samples=points, has_more=more,
                               next_cursor=points[-1]['ordinal'] if more and points else None))
        return dict(axes=result, status=self.status(), reduced=reduction != 'none')

    def export(self, axis, start=float('-inf'), end=float('inf')):
        ring = self.rings.get(axis)
        snapshot = ring.snapshot() if ring else (b'', 1, 1)
        def rows():
            output = io.StringIO()
            writer = csv.writer(output, lineterminator='\n')
            writer.writerow(['axis', 'sequence', 'segment', 'monotonic_ns', 'utc_ns', 'position_degrees', 'gap'])
            yield output.getvalue()
            gap = False
            for ordinal in range(snapshot[1], snapshot[2]):
                point = ring.read(ordinal, snapshot[0])
                if point is None:
                    gap = True
                if point and start <= point['time_ms'] <= end:
                    output.seek(0);output.truncate(0)
                    writer.writerow([axis, point['sequence'], point['segment'], point['monotonic_ns'],
                                     point['utc_ns'], degrees_text(point['position_cdeg']), int(point['gap'] or gap)])
                    yield output.getvalue()
                    gap = False
        return rows()
