"""Migawki awarii w panelu: okno przebiegów bez blokady, przebiegi w migawce, karta awarii i trasy HTTP."""
import asyncio
import json
import time

from fastapi import FastAPI

from robot_web_bridge.fault_api import FaultSnapshotStore, add_fault_routes, hint, mini_trace
from robot_web_bridge.fault_snapshot import Collector
from robot_web_bridge.shaft_trace import HEADER_BYTES, RECORD, AxisRing, centidegrees, ring_window, window_rows

S, MS = 1_000_000_000, 1_000_000
T = time.time_ns() - 30 * S  # chwila awarii (segmenty z bieżącym mtime obejmują okno)


def ring_point(n, utc, position, gap=False):
    return dict(sequence=str(n), segment='1', monotonic_ns=str(n), utc_ns=str(utc), time_ms=utc / 1e6, position_cdeg=position, gap=gap)


def write_ring(path, points, capacity=4096):
    ring = AxisRing(path, 1, HEADER_BYTES + RECORD.size * capacity)
    for n, (utc, position) in enumerate(points, 1):
        ring.append(ring_point(n, utc, position))
    ring.flush()
    return ring


def test_ring_window_reads_time_range_without_lock_and_after_wrap(tmp_path):
    path = tmp_path / 'axis-1.bin'
    ring = write_ring(path, [(T + n * 40 * MS, n * 10) for n in range(300)], capacity=128)  # 300 > 128: pierścień zawinięty
    try:
        window = ring_window(path, T + 250 * 40 * MS, T + 260 * 40 * MS)  # plik zablokowany przez zapis
        assert [p['value'] for p in window] == [n * 10 for n in range(250, 261)]
        assert ring_window(path, T, T + 100 * 40 * MS) == []  # nadpisane
        assert ring_window(tmp_path / 'brak.bin', T, T + S) == []
    finally:
        import os
        os.close(ring.fd)


def test_window_rows_label_channels(tmp_path):
    for name, offset in (('axis-1.bin', 0), ('reference-1.bin', 500)):
        ring = write_ring(tmp_path / name, [(T + n * 40 * MS, n + offset) for n in range(10)])
        import os
        os.close(ring.fd)
    rows = [json.loads(line) for _, line in window_rows(tmp_path, T, T + 5 * 40 * MS)]
    assert {(r['channel'], r['axis']) for r in rows} == {('position', 1), ('reference', 1)}
    assert sorted(r['value'] for r in rows if r['channel'] == 'reference') == [500, 501, 502, 503, 504, 505]


def can_rx_position(stamp, ticks, axis=1):
    data = bytes([0x31]) + ticks.to_bytes(6, 'big', signed=True) + b'\x00'
    return dict(source='can_cpp', event='can_rx', utc_ns=stamp, axis=axis, command=0x31, data=data.hex())


def j1_restart_rows():
    """Wzorzec z 19.11.2026 20:49:23: dwa timeouty J1, skok licznika, przerwa w odpowiedziach, ENABLE=1 po STOP."""
    rows = [can_rx_position(T - 2 * S + n * 40 * MS, 40545) for n in range(48)]  # postój do −80 ms
    rows += [dict(source='can_cpp', event='request_timeout', utc_ns=T - 76 * MS, axis=1, command=0x82, request_ns=0, deadline_ns=24 * MS),
             dict(source='can_cpp', event='request_timeout', utc_ns=T - 27 * MS, axis=1, command=0x3A, request_ns=0, deadline_ns=24 * MS),
             can_rx_position(T - 1 * MS, 40545 + 742),
             dict(source='can_cpp', event='hold_violation', utc_ns=T, axis=1, detail='J1 (CAN ID 1): odchyłka 742 imp. wału'),
             dict(source='can_cpp', event='bus_fault', utc_ns=T + 1000, axis=0, detail='J1 (CAN ID 1): ruch bez komendy w postoju'),
             dict(source='bridge', event='fault_trigger', utc_ns=T + 28 * MS, reason='STOP wysłany'),
             dict(source='can_cpp', event='recovery_disable_retry', utc_ns=T + 1296 * MS, axis=1, detail='ENABLE=1 po STOP'),
             dict(source='can_cpp', event='recovery_failed', utc_ns=T + 4900 * MS, axis=0, detail='przekroczono termin etapu verifying_disabled')]
    rows += [can_rx_position(T + 1265 * MS + n * 120 * MS, 40545 + 742 + 400 * n) for n in range(10)]
    rows += [dict(source='can_cpp', event='can_rx', utc_ns=T - S + n * 40 * MS, axis=2, command=0x31,
                  data=(bytes([0x31]) + (1000).to_bytes(6, 'big', signed=True) + b'\x00').hex()) for n in range(5)]
    return sorted(rows, key=lambda row: row['utc_ns'])


def make_snapshot(tmp_path, rows, shaft_dir=None, detail='J1 (CAN ID 1): ruch bez komendy w postoju'):
    (tmp_path / 'can-0.jsonl').write_bytes(b''.join(json.dumps(row).encode() + b'\n' for row in rows))
    faults = tmp_path / 'faults'
    faults.mkdir(exist_ok=True)
    (faults / f'trigger-{T}-can.json').write_text(json.dumps(dict(source='can_cpp', event='bus_fault', utc_ns=T, detail=detail)))
    collector = Collector(tmp_path, run=lambda *a, **k: type('R', (), {'stdout': b''})(), shaft_dir=shaft_dir)
    [snapshot] = collector.poll(force=True)
    return snapshot


def test_summary_finds_axis_excerpt_restart_pattern_and_hold_plan(tmp_path):
    snapshot = make_snapshot(tmp_path, j1_restart_rows())
    store = FaultSnapshotStore(tmp_path)
    [entry] = store.list()
    assert entry['id'] == snapshot and entry['axis'] == 1
    summary = store.summary(snapshot)
    assert summary['axis'] == 1 and summary['axis_source'] == 'trigger' and not summary['has_shaft']
    events = [row['event'] for row in summary['excerpt']]
    assert events[:3] == ['request_timeout', 'request_timeout', 'hold_violation'] and 'bus_fault' in events and events[-1] == 'recovery_failed'
    assert summary['excerpt'][0]['detail'] == 'brak odpowiedzi w 24 ms' and summary['excerpt'][0]['command'] == '82'
    text = summary['hint']['text']
    assert '2 zapytania (-76 ms, -27 ms)' in text and ('1,26 s' in text or '1,27 s' in text)
    assert 'skoczył o +16,3° wału' in text and 'ENABLE=1' in text and 'restartu sterownika J1' in text
    mini = summary['mini']
    assert mini['plan_kind'] == 'hold' and mini['source'] == 'can' and mini['unit'] == '° wału'
    assert len([s for s in mini['samples'] if s[0] < 0]) == 8 and len([s for s in mini['samples'] if s[0] >= 0]) == 8
    assert mini['plan'][0][1] == mini['plan'][1][1] == centidegrees(40545) / 100
    assert any(a < 0 < b for a, b in mini['gaps'])


def test_events_filters_paging_trace_and_jsonl(tmp_path):
    snapshot = make_snapshot(tmp_path, j1_restart_rows())
    store = FaultSnapshotStore(tmp_path)
    key = store.events(snapshot, 'key')
    assert key['total'] == 7 and all(row['key'] for row in key['rows'])
    page = store.events(snapshot, 'can', 1, '', 5, 10)
    assert page['offset'] == 5 and len(page['rows']) == 10 and all(row['axis'] == 1 for row in page['rows'])
    assert store.events(snapshot, 'all', None, 'verifying')['total'] == 1
    trace = store.trace(snapshot)
    assert trace['axis'] == 1 and [axis['axis'] for axis in trace['axes']] == [1, 2]
    lines = list(store.jsonl(snapshot))
    assert json.loads(lines[0])['snapshot'] == snapshot and len(lines) == 1 + len(j1_restart_rows())


def test_shaft_rows_in_snapshot_give_planned_reference(tmp_path):
    shaft = tmp_path / 'shaft'
    shaft.mkdir()
    import os
    for name, values in (('axis-1.bin', lambda n: 1000 + n * 50), ('reference-1.bin', lambda n: 1000 + n * 52)):
        ring = write_ring(shaft / name, [(T - 2 * S + n * 40 * MS, values(n)) for n in range(70)])
        os.close(ring.fd)
    rows = [dict(source='can_cpp', event='bus_fault', utc_ns=T, axis=0, detail='J1 (CAN ID 1): ucieczka osi')]
    snapshot = make_snapshot(tmp_path, rows, shaft_dir=shaft, detail='J1 (CAN ID 1): ucieczka osi — uchyb śledzenia')
    summary = FaultSnapshotStore(tmp_path).summary(snapshot)
    assert summary['has_shaft'] and summary['mini']['source'] == 'shaft' and summary['mini']['plan_kind'] == 'reference'
    assert len(summary['mini']['plan']) > 2


def test_no_axis_means_no_chart_and_reason():
    mini, reason = mini_trace([], None, T, 5000)
    assert mini is None and reason == 'błąd nie wskazuje osi'
    assert mini_trace([can_rx_position(T - 10 * MS, 5)], 1, T, 5000) == (None, 'brak próbek osi przed awarią w migawce')
    assert hint([], None, T, 5000) is None


async def request(app, path, query=b''):
    result, sent = [], False

    async def receive():
        nonlocal sent
        if not sent:
            sent = True
            return {'type': 'http.request', 'body': b'', 'more_body': False}
        await asyncio.Future()

    async def send(event):
        result.append(event)
    await app(dict(type='http', asgi={'version': '3.0'}, http_version='1.1', scheme='http', method='GET', path=path, raw_path=path.encode(),
                   query_string=query, headers=[], root_path='', server=('test', 80), client=('127.0.0.1', 1234)), receive, send)
    headers = dict((k.decode(), v.decode()) for k, v in result[0].get('headers', []))
    return result[0]['status'], b''.join(e.get('body', b'') for e in result), headers


def test_http_routes(tmp_path):
    snapshot = make_snapshot(tmp_path, j1_restart_rows())
    app = FastAPI()
    add_fault_routes(app, FaultSnapshotStore(tmp_path))
    status, body, _ = asyncio.run(request(app, '/api/v1/faults'))
    assert status == 200 and json.loads(body)['available'] and json.loads(body)['snapshots'][0]['id'] == snapshot
    status, body, _ = asyncio.run(request(app, f'/api/v1/faults/{snapshot}'))
    assert status == 200 and json.loads(body)['axis'] == 1
    status, body, _ = asyncio.run(request(app, f'/api/v1/faults/{snapshot}/events', b'kind=can&axis=1&limit=3'))
    assert status == 200 and len(json.loads(body)['rows']) == 3
    assert asyncio.run(request(app, f'/api/v1/faults/{snapshot}/events', b'kind=nope'))[0] == 422
    assert asyncio.run(request(app, '/api/v1/faults/20990101-000000-000'))[0] == 404
    assert asyncio.run(request(app, '/api/v1/faults/zly-id'))[0] == 422
    status, body, headers = asyncio.run(request(app, f'/api/v1/faults/{snapshot}/download'))
    assert status == 200 and f'awaria-{snapshot}.jsonl' in headers['content-disposition'] and body.count(b'\n') == 1 + len(j1_restart_rows())
    empty = FastAPI()
    add_fault_routes(empty, FaultSnapshotStore(None))
    status, body, _ = asyncio.run(request(empty, '/api/v1/faults'))
    assert status == 200 and json.loads(body) == dict(available=False, snapshots=[])
    assert asyncio.run(request(empty, f'/api/v1/faults/{snapshot}'))[0] == 404
