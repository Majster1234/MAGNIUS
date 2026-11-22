"""Migawki awarii w panelu: lista, karta awarii (oś, wyciąg zdarzeń CAN, krótki przebieg pomiar/plan, wzorzec),
strony zdarzeń, pełny przebieg i pobranie JSONL.

Źródło: `$ROBOT_TRACE_DIR/faults/faults.rfs` (zapisywany przez proces `fault_snapshot`). Dekodowanie migawki
(xz, ~30 tys. wierszy) odbywa się w wątku; ostatnie migawki są w pamięci podręcznej. Wzorzec jest prostą,
jawną regułą na zdarzeniach migawki (przerwy w odpowiedziach osi, skok licznika, ponowione ENABLE=0) —
panel oznacza go jako automatyczną analizę, nie diagnozę."""
import asyncio
from collections import OrderedDict
import json
from pathlib import Path
import re
import statistics
import threading
from typing import Literal

from fastapi import HTTPException, Query
from fastapi.responses import JSONResponse, StreamingResponse

from .fault_codec import RecordStore, decode_snapshot
from .fault_snapshot import FAULTS_FILE, local_time
from .shaft_trace import centidegrees

MS = 1_000_000
SNAPSHOT_ID = re.compile(r'^\d{8}-\d{6}-\d{3}$')
AXIS_TEXT = re.compile(r'\bJ([1-6])\b')
SOURCES = {'can_cpp': 'CAN', 'bridge': 'Mostek', 'rosout': 'ROS', 'journal': 'Dziennik', 'kernel': 'Jądro', 'shaft': 'Przebieg'}
KEY_WORDS = ('fault', 'timeout', 'violation', 'recovery', 'error', 'stall', 'reject', 'failed')
POSITION_COMMAND = 0x31  # odczyt licznika położenia (int48, impulsy 16384/obr. wału)
MINI_BEFORE, MINI_AFTER = 8, 8


def is_key(row):
    source, event = row.get('source'), str(row.get('event') or '')
    if source == 'shaft':
        return False
    if source in ('journal', 'kernel'):
        try:
            return int(row.get('priority')) <= 3
        except (TypeError, ValueError):
            return False
    if source == 'rosout':
        return row.get('level') in ('ERROR', 'FATAL')
    return any(word in event for word in KEY_WORDS)


def level(row):
    event = str(row.get('event') or '')
    if any(word in event for word in ('fault', 'failed', 'violation')) or row.get('level') in ('ERROR', 'FATAL'):
        return 'error'
    if any(word in event for word in ('timeout', 'retry', 'reject')) or row.get('level') == 'WARN':
        return 'warning'
    return ''


def row_axis(row):
    axis = row.get('axis')
    return axis if isinstance(axis, int) and 1 <= axis <= 6 else None


def row_detail(row):
    for key in ('detail', 'reason', 'message'):
        if row.get(key):
            return str(row[key])
    if row.get('event') == 'request_timeout' and row.get('deadline_ns') is not None and row.get('request_ns') is not None:
        return f"brak odpowiedzi w {(int(row['deadline_ns']) - int(row['request_ns'])) / MS:.0f} ms"
    if row.get('source') == 'shaft':
        return f"{row.get('channel')}: {row.get('value', 0) / 100:.2f}"
    if row.get('source') == 'can_cpp' and row.get('event') in ('can_rx', 'can_tx'):
        command = row.get('command')
        if command == POSITION_COMMAND and row.get('event') == 'can_rx':
            ticks = position_ticks(row.get('data'))
            return 'odczyt licznika położenia' if ticks is None else f'licznik {ticks} imp. ({centidegrees(ticks) / 100:.2f}° wału)'
    return ''


def position_ticks(data):
    try:
        raw = bytes.fromhex(str(data))
    except ValueError:
        return None
    if len(raw) < 7 or raw[0] != POSITION_COMMAND:
        return None
    return int.from_bytes(raw[1:7], 'big', signed=True)


def event_row(row, t0):
    stamp = int(row.get('utc_ns') or 0)
    command = row.get('command')
    data = str(row.get('data') or '')
    return dict(t_ms=round((stamp - t0) / MS, 1), time=local_time(stamp)[11:23], source=SOURCES.get(row.get('source'), row.get('source') or '?'),
                event=row.get('event') or '', axis=row_axis(row), command=f'{command:02X}' if isinstance(command, int) and row.get('source') == 'can_cpp' and command else '',
                data=' '.join(data[i:i + 2] for i in range(0, len(data), 2)).upper(), detail=row_detail(row), level=level(row), key=is_key(row))


def fault_axis(header, rows, t0, with_source=False):
    """Oś z opisu wyzwalacza; bez niej — z pierwszego zdarzenia kluczowego z osią w ±1 s (wniosek, nie pewność)."""
    found = None, None
    for trigger in header.get('triggers') or []:
        match = AXIS_TEXT.search(str(trigger.get('detail') or ''))
        if match:
            found = int(match.group(1)), 'trigger'
            break
    else:
        for row in rows:
            if is_key(row) and row_axis(row) and abs(int(row.get('utc_ns') or 0) - t0) <= 1000 * MS:
                found = row_axis(row), 'event'
                break
    return found if with_source else found[0]


def measured(rows, axis, t0):
    """[(t_ms, ° wału, przerwa)] z zapisu wałów; starsze migawki bez niego — z ramek 31h tej osi."""
    shaft = [(round((r['utc_ns'] - t0) / MS, 1), r['value'] / 100, bool(r.get('gap'))) for r in rows
             if r.get('source') == 'shaft' and r.get('channel') == 'position' and r.get('axis') == axis]
    if shaft:
        return shaft, 'shaft'
    frames = []
    for row in rows:
        if row.get('source') == 'can_cpp' and row.get('event') == 'can_rx' and row.get('command') == POSITION_COMMAND and row_axis(row) == axis:
            ticks = position_ticks(row.get('data'))
            if ticks is not None:
                frames.append((round((int(row['utc_ns']) - t0) / MS, 1), centidegrees(ticks) / 100, False))
    return frames, 'can' if frames else None


def planned(rows, axis, t0):
    for channel in ('reference', 'command'):
        points = [(round((r['utc_ns'] - t0) / MS, 1), r['value'] / 100) for r in rows
                  if r.get('source') == 'shaft' and r.get('channel') == channel and r.get('axis') == axis]
        if points:
            return points, channel
    return [], None


def gaps(points, start, end):
    """Przedziały bez odpowiedzi osi: odstęp > 3× mediana kroku (min. 100 ms) albo brak próbek po awarii."""
    times = [p[0] for p in points]
    steps = [b - a for a, b in zip(times, times[1:]) if b > a]
    limit = max(100.0, 3 * statistics.median(steps)) if steps else 100.0
    found = [[a, b] for a, b in zip(times, times[1:]) if b - a > limit and b >= start and a <= end]
    after = [t for t in times if t >= 0]
    if times and not after and end > times[-1]:
        found.append([times[-1], end])
    return found


def decimate(points, maximum):
    if len(points) <= maximum:
        return points
    step = len(points) / maximum
    return [points[int(i * step)] for i in range(maximum)] + [points[-1]]


def mini_trace(rows, axis, t0, end_ms):
    """Kilka próbek przed i po awarii. Brak osi lub < 3 próbek przed awarią → brak wykresu (karta mówi dlaczego)."""
    if not axis:
        return None, 'błąd nie wskazuje osi'
    points, source = measured(rows, axis, t0)
    before = [p for p in points if p[0] < 0][-MINI_BEFORE:]
    after = [p for p in points if p[0] >= 0][:MINI_AFTER]
    if len(before) < 3:
        return None, 'brak próbek osi przed awarią w migawce'
    start = before[0][0]
    end = max(after[-1][0] if after else 0.0, min(end_ms, before[-1][0] + 1500.0) if not after else 0.0) + 40.0
    plan, kind = planned(rows, axis, t0)
    plan = [p for p in plan if start <= p[0] <= end]
    if not plan:
        recent = [p[1] for p in points if p[0] < -100][-25:]  # próbka wyzwalająca bywa tuż przed zgłoszeniem
        if recent and max(recent) - min(recent) <= 2.0:  # oś stała przed awarią: plan = pozycja trzymania
            hold = statistics.median(recent[-10:])
            plan, kind = [(start, hold), (end, hold)], 'hold'
        else:
            kind = None
    return dict(axis=axis, unit='° wału', source=source, plan_kind=kind, start_ms=start, end_ms=end,
                samples=[[t, round(v, 3)] for t, v, _ in before + after], plan=[[t, round(v, 3)] for t, v in plan],
                gaps=gaps(before + after, start, end)), None


def full_trace(rows, axis, t0):
    result = []
    for current in range(1, 7):
        points, source = measured(rows, current, t0)
        if not points:
            continue
        plan, kind = planned(rows, current, t0)
        limit = 900 if current == axis else 200
        result.append(dict(axis=current, source=source, plan_kind=kind, samples=[[t, round(v, 3)] for t, v, _ in decimate(points, limit)],
                           plan=[[t, round(v, 3)] for t, v in decimate(plan, limit)], gaps=gaps(points, points[0][0], points[-1][0])))
    return result


def excerpt(rows, axis, t0):
    """Zdarzenia, które wywołały błąd: do 3 przed awarią i do 5 od chwili awarii (oś błędu albo bez osi)."""
    window = [row for row in rows if is_key(row) and row.get('source') in ('can_cpp', 'bridge')
              and row_axis(row) in (axis, None) and -2000 * MS <= int(row.get('utc_ns') or 0) - t0 <= 6000 * MS]
    before = [row for row in window if int(row['utc_ns']) < t0][-3:]
    later = [row for row in window if int(row['utc_ns']) >= t0]
    after = later[:4]
    final = next((row for row in reversed(later) if level(row) == 'error'), None)
    if final is not None and all(final is not row for row in after):
        after.append(final)  # zakończenie (np. nieudane odzyskiwanie) zawsze widoczne
    return [event_row(row, t0) for row in before + after]


def hint(rows, axis, t0, end_ms):
    if not axis:
        return None
    facts, restart = [], 0
    timeouts = [row for row in rows if row.get('event') == 'request_timeout' and row_axis(row) == axis and -500 * MS <= int(row['utc_ns']) - t0 < 0]
    if timeouts:
        moments = ', '.join('%.0f ms' % ((int(r['utc_ns']) - t0) / MS) for r in timeouts[:3])
        noun = 'zapytanie' if len(timeouts) == 1 else 'zapytania' if len(timeouts) < 5 else 'zapytań'
        facts.append(f'tuż przed awarią J{axis} nie odpowiedział na {len(timeouts)} {noun} ({moments})')
        restart += 1
    replies = sorted(int(row['utc_ns']) for row in rows if row.get('source') == 'can_cpp' and row.get('event') == 'can_rx'
                     and row_axis(row) == axis and int(row['utc_ns']) >= t0)
    marks = [t0, *replies] if replies else [t0, t0 + int(end_ms * MS)]
    longest = max((b - a for a, b in zip(marks, marks[1:])), default=0) / MS
    if longest >= 500:
        facts.append(f'po awarii brak odpowiedzi J{axis} przez {longest / 1000:.2f} s'.replace('.', ','))
        restart += 1
    points, _ = measured(rows, axis, t0)
    normal = [p for p in points if p[0] < -100]
    later = [p for p in points if p[0] >= -100]
    if normal and later and abs(later[0][1] - normal[-1][1]) >= 1.0:
        facts.append(f'licznik położenia skoczył o {later[0][1] - normal[-1][1]:+.1f}° wału'.replace('.', ','))
        restart += 1
    if any(row.get('event') == 'recovery_disable_retry' and row_axis(row) == axis for row in rows):
        facts.append(f'po STOP J{axis} zgłaszał ENABLE=1 (ponowione ENABLE=0)')
        restart += 1
    if not facts:
        return None
    text = 'Z nagrania: ' + '; '.join(facts) + '.'
    if restart >= 2 and (timeouts or longest >= 500):
        text += f' To typowy obraz restartu sterownika J{axis} — sprawdź zasilanie i złącze tej osi.'
    return dict(text=text, facts=facts)


def snapshot_axis(meta):
    for trigger in meta.get('triggers') or []:
        match = AXIS_TEXT.search(str(trigger.get('detail') or ''))
        if match:
            return int(match.group(1))
    return None


class FaultSnapshotStore:
    def __init__(self, directory, cache_size=3):
        self.path = Path(directory) / 'faults' / FAULTS_FILE if directory else None
        self.cache, self.cache_size, self.lock = OrderedDict(), cache_size, threading.Lock()

    @property
    def available(self):
        return self.path is not None and self.path.exists()

    def _entries(self):
        return RecordStore(self.path).index()[0] if self.available else []

    def list(self, limit=50):
        entries = self._entries()[-limit:]
        return [dict(id=meta.get('snapshot'), time=meta.get('time'), utc_ns=meta.get('utc_ns'), duration_s=meta.get('duration_s'),
                     rows=meta.get('rows'), counts=meta.get('counts') or {}, axis=snapshot_axis(meta),
                     triggers=meta.get('triggers') or []) for _, _, meta in reversed(entries)]

    def load(self, snapshot):
        if not SNAPSHOT_ID.match(snapshot or ''):
            raise KeyError(snapshot)
        with self.lock:
            if snapshot in self.cache:
                self.cache.move_to_end(snapshot)
                return self.cache[snapshot]
        entry = next((entry for entry in self._entries() if entry[2].get('snapshot') == snapshot), None)
        if entry is None:
            raise KeyError(snapshot)
        _, payload = RecordStore(self.path).load(entry[0])
        header, rows = decode_snapshot(payload)
        with self.lock:
            self.cache[snapshot] = (header, rows)
            while len(self.cache) > self.cache_size:
                self.cache.popitem(last=False)
        return header, rows

    def summary(self, snapshot):
        header, rows = self.load(snapshot)
        t0 = int(header['utc_ns'])
        window = header.get('window') or {}
        end_ms = (int(window.get('end_utc_ns', t0)) - t0) / MS
        axis, axis_source = fault_axis(header, rows, t0, with_source=True)
        mini, missing = mini_trace(rows, axis, t0, end_ms)
        active = ((header.get('configuration') or {}).get('active') or {})
        return dict(id=header.get('snapshot'), time=header.get('time'), utc_ns=t0, duration_s=header.get('duration_s'),
                    window=dict(pre_s=window.get('pre_s'), post_s=window.get('post_s')), rows=header.get('rows'), counts=header.get('counts') or {},
                    triggers=[dict(source=t.get('source'), event=t.get('event'), detail=t.get('detail')) for t in header.get('triggers') or []],
                    axis=axis, axis_source=axis_source, excerpt=excerpt(rows, axis, t0), key_events=sum(1 for row in rows if is_key(row)), hint=hint(rows, axis, t0, end_ms),
                    mini=mini, mini_missing=missing, has_shaft=any(row.get('source') == 'shaft' for row in rows),
                    geometry=(active.get('document') or {}).get('values'), configuration_revision=active.get('revision'))

    def events(self, snapshot, kind='key', axis=None, query='', offset=0, limit=200):
        header, rows = self.load(snapshot)
        t0, text = int(header['utc_ns']), query.strip().lower()
        sources = dict(can=('can_cpp',), bridge=('bridge',), log=('journal', 'kernel', 'rosout'), shaft=('shaft',))
        selected = []
        for row in rows:
            if kind == 'key' and not is_key(row):
                continue
            if kind in sources and row.get('source') not in sources[kind]:
                continue
            if axis and row_axis(row) not in (axis, None if kind == 'key' else axis):
                continue
            selected.append(row)
        if text:
            selected = [row for row in selected if text in f"{row.get('event', '')} {row_detail(row)} {row.get('source', '')}".lower()]
        return dict(total=len(selected), offset=offset, rows=[event_row(row, t0) for row in selected[offset:offset + limit]])

    def trace(self, snapshot):
        header, rows = self.load(snapshot)
        t0 = int(header['utc_ns'])
        return dict(axis=fault_axis(header, rows, t0), unit='° wału', axes=full_trace(rows, fault_axis(header, rows, t0), t0))

    def jsonl(self, snapshot):
        header, rows = self.load(snapshot)
        yield json.dumps(header, ensure_ascii=False) + '\n'
        for row in rows:
            yield json.dumps(dict(time=local_time(int(row['utc_ns'])), **row), ensure_ascii=False) + '\n'


def add_fault_routes(app, store):
    base = '/api/v1/faults'
    headers = {'Cache-Control': 'no-store'}

    def require(snapshot):
        if store is None or not store.available:
            raise HTTPException(404, 'Migawki awarii są niedostępne (brak nagrywania diagnostyki)')
        if not SNAPSHOT_ID.match(snapshot):
            raise HTTPException(422, 'Nieprawidłowy identyfikator migawki')

    async def run(function, *args):
        try:
            return await asyncio.to_thread(function, *args)
        except KeyError:
            raise HTTPException(404, 'Brak migawki') from None

    @app.get(base)
    async def snapshots(limit: int = Query(50, ge=1, le=200)):
        if store is None or not store.available:
            return JSONResponse(dict(available=False, snapshots=[]), headers=headers)
        return JSONResponse(dict(available=True, snapshots=await asyncio.to_thread(store.list, limit)), headers=headers)

    @app.get(base + '/{snapshot}')
    async def summary(snapshot: str):
        require(snapshot)
        return JSONResponse(await run(store.summary, snapshot), headers=headers)

    @app.get(base + '/{snapshot}/events')
    async def events(snapshot: str, kind: Literal['key', 'can', 'bridge', 'log', 'shaft', 'all'] = 'key', axis: int = Query(None, ge=1, le=6),
                     q: str = Query('', max_length=120), offset: int = Query(0, ge=0), limit: int = Query(200, ge=1, le=500)):
        require(snapshot)
        return JSONResponse(await run(store.events, snapshot, kind, axis, q, offset, limit), headers=headers)

    @app.get(base + '/{snapshot}/trace')
    async def trace(snapshot: str):
        require(snapshot)
        return JSONResponse(await run(store.trace, snapshot), headers=headers)

    @app.get(base + '/{snapshot}/download')
    async def download(snapshot: str):
        require(snapshot)
        await run(store.load, snapshot)
        return StreamingResponse(store.jsonl(snapshot), media_type='application/x-ndjson',
                                 headers={**headers, 'Content-Disposition': f'attachment; filename="awaria-{snapshot}.jsonl"'})
