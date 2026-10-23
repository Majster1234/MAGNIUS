"""Pliki cykliczne, kompletność i API; bez CAN i bez poleceń ruchu."""
import asyncio
import csv
import errno
import io
import json
import os
import threading
import time
from types import SimpleNamespace as NS

import pytest

from robot_web_bridge.shaft_trace import (
    AxisRing, ShaftTraceStore, LIMIT, HEADER_BYTES, RECORD, centidegrees,
    degrees_text, reduce_points,
)
from robot_web_bridge.shaft_trace_api import add_shaft_trace_routes
from fastapi import FastAPI


def sample(n=1, axis=1, ticks=0, segment=1, mono=None):
    return NS(sequence=n, axis=axis, ticks=ticks, segment=segment, flags=0,
              monotonic_ns=mono or 1_000_000_000+n*40_000_000,
              utc_ns=1_792_700_000_000_000_000+n*40_000_000)


def batch(*samples, dropped=0):
    return NS(dropped_total=dropped, samples=samples)


def point(n, position=None):
    return dict(axis=1, sequence=str(n), segment='1', monotonic_ns=str(n*40_000_000),
                utc_ns=str(n*40_000_000), time_ms=n*40., position_cdeg=n if position is None else position,
                gap=False)


@pytest.fixture
def store(tmp_path):
    value = ShaftTraceStore(tmp_path, maximum=HEADER_BYTES+RECORD.size*1024,
                            clock_ns=lambda: 1_200_000_000)
    yield value
    value.close()


@pytest.mark.parametrize('ticks,expected', [(0,0),(1,2),(-1,-2),(256,563),(-256,-563),
                                           (16384,36000),(-16384,-36000)])
def test_exact_rounding_once_half_away_from_zero(ticks, expected):
    assert centidegrees(ticks) == expected
    assert degrees_text(expected) == f'{expected/100:.2f}'


def test_150_samples_per_second_no_duplicate_cache_and_all_axes(store):
    for n in range(1,126):  # pięć sekund, 25 Hz × sześć osi
        value = batch(*(sample(n,a,(-1)**a*256*a) for a in range(1,7)))
        store.ingest(value)
        store.ingest(value)  # ponowny odbiór tej samej paczki nie powiela pomiarów
    store.wait_idle()
    assert store.saved == 750
    assert store.status()['writer_dropped'] == 0
    history = store.history(range(1,7), float('-inf'), float('inf'))
    assert [len(a['samples']) for a in history['axes']] == [125]*6
    for axis in range(1,7):
        rows = list(csv.DictReader(store.export(axis)))
        assert len(rows) == 125
        assert rows[0]['position_degrees'] == degrees_text(centidegrees((-1)**axis*256*axis))


def test_segment_reset_source_restart_missing_samples_and_80ms_gap(store):
    store.ingest(batch(sample(1,ticks=100), sample(2,ticks=-256), sample(4,ticks=0),
                       sample(5,mono=1_400_000_001), sample(6,segment=2,ticks=0),
                       sample(1,segment=3,ticks=3), dropped=2))
    store.wait_idle()
    values = store.history([1],0,float('inf'))['axes'][0]['samples']
    assert [p['gap'] for p in values] == [True,False,True,True,True,True]
    assert values[1]['position_cdeg'] == -563
    assert store.status()['source_dropped'] == 2
    assert store.status()['missing_samples'] == 1
    assert store.status()['axes'][5]['latest'] is None


@pytest.mark.parametrize('capacity', [7, (LIMIT-HEADER_BYTES)//RECORD.size])
def test_multiple_wraps_restart_stable_export_and_file_limit(tmp_path, capacity):
    maximum = LIMIT if capacity>7 else HEADER_BYTES+capacity*RECORD.size
    path = tmp_path/'axis-1.bin'
    ring = AxisRing(path,1,maximum)
    for n in range(1,capacity*2+18):
        ring.append(point(n))
    data, first, end = ring.snapshot()
    assert end-first == capacity
    assert len(data) == os.stat(path).st_size == maximum <= LIMIT
    assert ring.read(first,data)['sequence'] == str(first)
    assert ring.read(end-1,data)['sequence'] == str(end-1)
    # Zapis równoległy nie zmienia kopii będącej źródłem eksportu.
    worker = threading.Thread(target=lambda:[ring.append(point(n)) for n in range(end,end+50)])
    worker.start()
    assert ring.read(first,data)['sequence'] == str(first)
    worker.join();ring.flush();next_ = ring.next;ring.close()
    ring = AxisRing(path,1,maximum)
    assert ring.next == next_
    assert ring.status()['retained_records'] == capacity
    assert os.stat(path).st_size == maximum
    history = ring.history(0,float('inf'),2000)
    assert len(history) <= 2000
    assert min(p['position_cdeg'] for p in history) == next_-capacity
    assert max(p['position_cdeg'] for p in history) == next_-1
    ring.close()


def test_corrupt_record_is_gap_and_bad_header_is_not_overwritten(tmp_path):
    path = tmp_path/'axis-1.bin';maximum=HEADER_BYTES+RECORD.size*16
    ring = AxisRing(path,1,maximum)
    for n in range(1,6): ring.append(point(n))
    os.pwrite(ring.fd,b'\xff',ring._offset(3)+20)
    ring.close();ring=AxisRing(path,1,maximum)
    rows=ring.history(0,float('inf'),2000)
    assert [p['sequence'] for p in rows] == ['1','2','4','5']
    assert rows[2]['gap']
    assert ring.corrupt == 1
    os.pwrite(ring.fd,b'X'*128,0);ring.close()
    before=path.read_bytes()
    with pytest.raises(ValueError,match='nagłówki'): AxisRing(path,1,maximum)
    assert path.read_bytes() == before


def test_second_writer_cannot_corrupt_open_ring(tmp_path):
    path=tmp_path/'axis-1.bin';ring=AxisRing(path,1)
    ring.append(point(1))
    with pytest.raises(BlockingIOError): AxisRing(path,1)
    assert ring.read(1)['position_cdeg']==1
    ring.close()


def test_min_max_keeps_short_overshoots_and_breaks():
    values=[dict(point(i,0),ordinal=i) for i in range(5000)]
    values[2345]['position_cdeg']=12345;values[2346]['position_cdeg']=-8765
    values[2000]['gap']=True
    result=reduce_points(values,100)
    assert len(result)<=100
    assert max(p['position_cdeg'] for p in result)==12345
    assert min(p['position_cdeg'] for p in result)==-8765
    assert any(p['gap'] for p in result)


def test_disk_full_keeps_live_readings_and_reports_loss(store,monkeypatch):
    store.ingest(batch(sample(1)));store.wait_idle()
    def full(*args): raise OSError(errno.ENOSPC,'Brak miejsca')
    monkeypatch.setattr(os,'pwrite',full)
    store.ingest(batch(sample(2,ticks=-256)));store.wait_idle()
    status=store.status()
    assert not status['recording']
    assert status['writer_dropped']==1
    assert 'Brak miejsca' in status['axes'][0]['error']
    assert store.live(1)['samples'][0]['position_cdeg']==-563


def test_writer_queue_overload_is_bounded_and_marks_next_saved_gap(tmp_path,monkeypatch):
    entered,release=threading.Event(),threading.Event()
    append=AxisRing.append
    def slow(self,p):
        entered.set();release.wait(3);return append(self,p)
    monkeypatch.setattr(AxisRing,'append',slow)
    store=ShaftTraceStore(tmp_path,queue_size=2)
    try:
        store.ingest(batch(sample(1)));assert entered.wait(1)
        for n in range(2,8): store.ingest(batch(sample(n)))
        assert store.queue.qsize()==2
        assert store.status()['writer_dropped']==4
        release.set();store.wait_idle();store.ingest(batch(sample(8)));store.wait_idle()
        rows=store.history([1],0,float('inf'))['axes'][0]['samples']
        assert [p['sequence'] for p in rows]==['1','2','3','8']
        assert rows[-1]['gap']
    finally:
        release.set();store.close()


def test_restart_uses_same_files_and_starts_new_display_segment(tmp_path):
    store=ShaftTraceStore(tmp_path);store.ingest(batch(sample(1)));store.wait_idle();store.close()
    store=ShaftTraceStore(tmp_path)
    try:
        store.ingest(batch(sample(2)));store.wait_idle()
        assert len(list(tmp_path.iterdir()))==1
        rows=list(csv.DictReader(store.export(1)))
        assert len(rows)==2 and rows[-1]['gap']=='1'
    finally: store.close()


async def request(app,path,query=b''):
    result=[];sent=False
    async def receive():
        nonlocal sent
        if not sent:
            sent=True;return {'type':'http.request','body':b'','more_body':False}
        await asyncio.Future()
    async def send(event): result.append(event)
    await app(dict(type='http',asgi={'version':'3.0'},http_version='1.1',scheme='http',method='GET',
                   path=path,raw_path=path.encode(),query_string=query,headers=[],root_path='',
                   server=('test',80),client=('127.0.0.1',1234)),receive,send)
    return result[0]['status'],b''.join(e.get('body',b'') for e in result)


def test_http_history_cursor_validation_and_unreduced_csv(store):
    store.ingest(batch(*(sample(n,ticks=-256) for n in range(1,126))));store.wait_idle()
    app=FastAPI();add_shaft_trace_routes(app,store);base='/api/v1/diagnostics/trace'
    status,body=asyncio.run(request(app,base));assert status==200 and len(json.loads(body)['axes'])==6
    status,body=asyncio.run(request(app,base+'/history',b'axis=1&max_points=10'))
    rows=json.loads(body)['axes'][0];assert status==200 and len(rows['samples'])<=10
    cursor=0;sequences=[]
    while True:
        status,body=asyncio.run(request(app,base+'/history',f'axis=1&max_points=10&reduction=none&after={cursor}'.encode()))
        page=json.loads(body)['axes'][0];sequences.extend(p['sequence'] for p in page['samples'])
        if not page['has_more']: break
        cursor=page['next_cursor']
    assert sequences==[str(n) for n in range(1,126)]
    status,body=asyncio.run(request(app,base+'/history',b'axis=1&after=120'))
    assert [p['sequence'] for p in json.loads(body)['axes'][0]['samples']]==['121','122','123','124','125']
    for query in (b'max_points=2001',b'axis=7',b'start_ms=nan',b'start_ms=2&end_ms=1'):
        assert asyncio.run(request(app,base+'/history',query))[0]==422
    status,body=asyncio.run(request(app,base+'/axes/1/export.csv'))
    assert status==200
    csv_rows=list(csv.DictReader(io.StringIO(body.decode())))
    assert len(csv_rows)==125 and all(p['position_degrees']=='-5.63' for p in csv_rows)


def test_websocket_initial_heartbeat_new_samples_and_disconnect(store):
    app=FastAPI();add_shaft_trace_routes(app,store)
    async def scenario():
        inputs=asyncio.Queue();await inputs.put({'type':'websocket.connect'});outputs=[]
        async def send(event):
            outputs.append(event)
            if event['type']=='websocket.send':
                frame=json.loads(event['text'])
                if not frame['samples']: store.ingest(batch(sample(1)))
                else: await inputs.put({'type':'websocket.disconnect','code':1000})
        await asyncio.wait_for(app(dict(type='websocket',asgi={'version':'3.0'},scheme='ws',path='/api/v1/diagnostics/trace/ws',
                                       raw_path=b'/api/v1/diagnostics/trace/ws',query_string=b'',headers=[],root_path='',
                                       client=('127.0.0.1',12),server=('test',80),subprotocols=[]),inputs.get,send),1)
        frames=[json.loads(e['text']) for e in outputs if e['type']=='websocket.send']
        assert frames[0]['samples']==[]
        assert frames[-1]['samples'][0]['sequence']=='1'
    asyncio.run(scenario())
