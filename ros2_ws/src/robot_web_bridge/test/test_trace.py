import importlib.util
import json
import time

import pytest
from pathlib import Path
from robot_web_bridge.trace import TraceRecorder


def test_writer_timestamps_rotation_and_shutdown(tmp_path):
    writer = TraceRecorder(tmp_path, max_bytes=2048)
    for i in range(60):
        writer.record('sample', axis=1, sample=i, message='Test "pozycji"\n')
    writer.close()
    files = list(tmp_path.glob('bridge-*.jsonl'))
    assert 1 <= len(files) <= 4
    rows = []
    for file in files:
        assert file.stat().st_size <= 2048
        rows += [json.loads(line) for line in file.read_text().splitlines()]
    assert rows
    assert all(row['mono_ns'] > 0 and row['utc_ns'] > 0 for row in rows)
    assert not writer.thread.is_alive()
    assert writer.failed is None


def test_analysis_links_timeout_and_global_fault(tmp_path):
    path = Path(__file__).resolve().parents[4] / 'scripts/analyze_can_recording.py'
    spec = importlib.util.spec_from_file_location('recording_analysis', path)
    analysis = importlib.util.module_from_spec(spec); spec.loader.exec_module(analysis)
    rows = [dict(event='request_timeout', mono_ns=1000000000, utc_ns=1700000000000000000,
                 axis=2, command=0xf6, duration_ns=21000000),
            dict(event='bus_fault', mono_ns=1001000000, utc_ns=1700000000001000000,
                 detail='Brak odpowiedzi'),
            dict(event='io_loop_gap', mono_ns=1002000000, utc_ns=1700000000002000000, duration_ns=5000000)]
    (tmp_path / 'can-0.jsonl').write_text('\n'.join(json.dumps(row) for row in rows) + '\n{incomplete')
    report = analysis.analyze(tmp_path)
    assert report['events'] == 3
    assert report['timeouts']['CAN 2 / 0xF6 (JOG)'] == 1
    assert len(report['fault_window']) == 3
    assert 'globalny błąd' in analysis.render(report)


def test_fault_edge_creates_one_trigger_and_recording_continues(tmp_path):
    writer = TraceRecorder(tmp_path, segments=3)
    writer.fault_state(False)
    writer.fault_state(True, 'J5: uchyb')
    writer.fault_state(True, 'J5: uchyb')  # stan 20 Hz: bez kolejnych wyzwalaczy
    writer.record('after_fault')
    writer.fault_state(False)
    writer.fault_state(True, 'J2: druga awaria')
    writer.close()
    triggers = sorted((tmp_path / 'faults').glob('trigger-*.json'))
    assert [json.loads(path.read_text())['detail'] for path in triggers] == ['J5: uchyb', 'J2: druga awaria']
    rows = [json.loads(line) for line in (tmp_path / 'bridge-0.jsonl').read_text().splitlines()]
    assert [row['event'] for row in rows].count('fault_trigger') == 2
    assert 'after_fault' in [row['event'] for row in rows]
    assert rows[-1]['event'] == 'recording_finished'


def test_ring_uses_configured_segment_count(tmp_path):
    writer = TraceRecorder(tmp_path, max_bytes=2048, segments=6)
    for i in range(400):
        writer.record('sample', sample=i)
        if i % 50 == 0:
            time.sleep(0.01)
    writer.close()
    assert len(list(tmp_path.glob('bridge-*.jsonl'))) == 6
    with pytest.raises(ValueError):
        TraceRecorder(tmp_path, segments=1)


def test_analysis_does_not_mix_recordings_or_previous_boots(tmp_path):
    path = Path(__file__).resolve().parents[4] / 'scripts/analyze_can_recording.py'
    spec = importlib.util.spec_from_file_location('session_analysis', path)
    analysis = importlib.util.module_from_spec(spec); spec.loader.exec_module(analysis)
    (tmp_path / 'session.json').write_text(json.dumps({'recording_id': 'new', 'started_mono_ns': 100}))
    rows = [dict(event='old', recording_id='old', mono_ns=1000000),
            dict(event='new', recording_id='new', mono_ns=120),
            dict(event='before', recording_id='new', mono_ns=80)]
    (tmp_path / 'can-0.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    assert [row['event'] for row in analysis.events(tmp_path)] == ['new']
