import json
import time
from types import SimpleNamespace

import io

from robot_web_bridge.fault_codec import RecordStore
from robot_web_bridge.fault_snapshot import Collector, RosoutBuffer, due_groups, dump_snapshot, list_snapshots, segment_rows

S = 1_000_000_000
T = time.time_ns() - 30 * S  # chwila awarii (segmenty z bieżącym mtime obejmują okno)


def write_lines(path, rows, tail=b''):
    path.write_bytes(b''.join(json.dumps(row).encode() + b'\n' for row in rows) + tail)


def trigger(directory, stamp, source='can_cpp', detail='awaria'):
    faults = directory / 'faults'
    faults.mkdir(exist_ok=True)
    (faults / f'trigger-{stamp}-{source}.json').write_text(
        json.dumps(dict(source=source, event='bus_fault', utc_ns=stamp, detail=detail)))


def test_triggers_within_post_window_form_one_snapshot():
    triggers = [dict(utc_ns=T), dict(utc_ns=T + 2 * S), dict(utc_ns=T + 20 * S)]
    assert due_groups(triggers, T + 5 * S, 5 * S, S) == []
    groups = due_groups(triggers, T + 9 * S, 5 * S, S)
    assert [[t['utc_ns'] for t in group] for group in groups] == [[T, T + 2 * S]]
    assert len(due_groups(triggers, T + 30 * S, 5 * S, S)) == 2


def test_segment_skips_incomplete_line_and_rows_outside_window(tmp_path):
    path = tmp_path / 'can-0.jsonl'
    write_lines(path, [dict(source='can_cpp', utc_ns=T - 11 * S, kernel_utc_ns=T), dict(source='can_cpp', utc_ns=T)],
                tail=b'{"source":"can_cpp","utc_ns":%d' % T)
    rows = segment_rows(path, T - 10 * S, T + 5 * S)
    assert [stamp for stamp, _ in rows] == [T]


def test_snapshot_merges_all_sources_in_time_order(tmp_path):
    (tmp_path / 'session.json').write_text(json.dumps({'recording_id': 'r1'}))
    write_lines(tmp_path / 'can-0.jsonl', [
        dict(source='can_cpp', event='can_tx', utc_ns=T - 12 * S),  # przed oknem
        dict(source='can_cpp', event='can_tx', utc_ns=T - 9 * S),
        dict(source='can_cpp', event='bus_fault', utc_ns=T)])
    write_lines(tmp_path / 'can-1.jsonl', [dict(source='can_cpp', event='can_rx', utc_ns=T + 4 * S)])
    write_lines(tmp_path / 'bridge-0.jsonl', [dict(source='bridge', event='ws_command_received', utc_ns=T - S)])
    (tmp_path / 'previous').mkdir()
    write_lines(tmp_path / 'previous' / 'bridge-3.jsonl', [dict(source='bridge', event='old', utc_ns=T - 100 * S)])
    rosout = RosoutBuffer(10 ** 15)
    rosout.add(T - 2 * S, 30, 'robot_can_diagnostics', 'ostrzeżenie')
    configuration = tmp_path / 'configuration'
    configuration.mkdir()
    (configuration / 'active.json').write_text(json.dumps({'revision': 'abc'}))
    parameters = tmp_path / 'execution.yaml'
    parameters.write_text('execution_lookahead_ms: 85\n')
    calls = []

    def journal(command, **_):
        calls.append(command)
        if '-k' in command:
            return SimpleNamespace(stdout=b'')
        entry = dict(__REALTIME_TIMESTAMP=str((T - 3 * S) // 1000), MESSAGE='[bridge-3] Traceback',
                     SYSLOG_IDENTIFIER='bash', PRIORITY='3')
        return SimpleNamespace(stdout=json.dumps(entry).encode() + b'\n')

    collector = Collector(tmp_path, 10, 5, configuration_dir=configuration, execution_parameters=str(parameters),
                          unit='robot-web-status.service', rosout=rosout, run=journal)
    trigger(tmp_path, T)
    trigger(tmp_path, T + S, source='bridge')
    assert collector.poll(now=T + 5 * S) == []  # okno „po” jeszcze trwa
    assert collector.poll(now=T + 7 * S) == []  # zapas 1,5 s na opróżnienie kolejek zapisu
    [snapshot] = collector.poll(now=T + 8 * S)
    [meta] = list_snapshots(collector.path)
    assert meta['snapshot'] == snapshot and meta['duration_s'] == 16 and meta['triggers'][0]['detail'] == 'awaria'
    output = io.StringIO()
    count = dump_snapshot(collector.path, 'last', output)
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    header, body = rows[0], rows[1:]
    assert header['snapshot'] == snapshot and header['rows'] == len(body) == count
    assert len(body[0]['time']) == len('2026-11-05 21:19:35.786123')
    assert header['event'] == 'fault_snapshot' and header['recording_id'] == 'r1'
    assert len(header['triggers']) == 2
    assert header['window']['start_utc_ns'] == T - 10 * S and header['window']['end_utc_ns'] == T + 6 * S
    assert header['counts'] == {'can': 3, 'bridge': 1, 'journal': 1, 'rosout': 1}
    assert header['configuration']['active'] == {'revision': 'abc'}
    assert header['execution_parameters']['text'].startswith('execution_lookahead_ms')
    assert [row['utc_ns'] for row in body] == sorted(row['utc_ns'] for row in body)
    assert [row['source'] for row in body] == ['can_cpp', 'journal', 'rosout', 'bridge', 'can_cpp', 'can_cpp']
    assert not list((tmp_path / 'faults').glob('trigger-*'))
    assert any('-u' in command and 'robot-web-status.service' in command for command in calls)


def test_plain_snapshot_files_are_imported_once(tmp_path):
    faults = tmp_path / 'faults'
    faults.mkdir()
    header = dict(source='fault_snapshot', event='fault_snapshot', utc_ns=T, recording_id='r1',
                  window=dict(start_utc_ns=T - 10 * S, end_utc_ns=T + 5 * S), triggers=[dict(detail='J1')])
    write_lines(faults / 'faults.jsonl', [
        dict(header, snapshot='20261105-223707-711'),
        dict(snapshot='20261105-223707-711', time='x', source='can_cpp', event='bus_fault', utc_ns=T, recording_id='r1')])
    write_lines(faults / 'fault-20261105-223800-000.jsonl', [dict(header, utc_ns=T + 60 * S),
                                                             dict(source='can_cpp', event='can_tx', utc_ns=T + 60 * S)])
    collector = Collector(tmp_path, 10, 5, run=lambda *a, **k: SimpleNamespace(stdout=b''))
    collector.import_plain()
    assert not list(faults.glob('*.jsonl'))
    metas = list_snapshots(collector.path)
    assert [m['duration_s'] for m in metas] == [15, 15] and len(metas) == 2
    output = io.StringIO()
    dump_snapshot(collector.path, '20261105-223707-711', output)
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert rows[1]['event'] == 'bus_fault' and rows[1]['recording_id'] == 'r1' and 'snapshot' not in rows[1]


def test_collector_retention_uses_store(tmp_path):
    write_lines(tmp_path / 'can-0.jsonl', [dict(source='can_cpp', event='can_tx', utc_ns=T + i * S) for i in range(-20, 200)])
    collector = Collector(tmp_path, 10, 5, keep_s=40, run=lambda *a, **k: SimpleNamespace(stdout=b''))
    for i in range(6):
        trigger(tmp_path, T + i * 30 * S)
        collector.poll(now=T + i * 30 * S + 7 * S)
    assert [m['duration_s'] for m in list_snapshots(collector.path)] == [15, 15, 15]
    assert isinstance(collector.store, RecordStore)
