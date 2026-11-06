import json

from robot_web_bridge.fault_codec import RecordStore, decode_snapshot, encode_snapshot

S = 1_000_000_000
T = 1_793_909_975_786_745_081


def sample_rows():
    rows = []
    for i in range(300):
        mono = 8_273_000_000_000 + i * 1_000_123
        rows.append(dict(source='can_cpp', pid=18340, event='can_tx' if i % 2 else 'reply_ok', recording_id='r1',
                         sequence=175_000 + i, mono_ns=mono, utc_ns=T + i * 1_000_000, axis=i % 6 + 1, command=245,
                         flags=0, dropped_total=0, request_ns=mono - 2_500_000, deadline_ns=mono + 8_000_000,
                         kernel_utc_ns=T + i * 1_000_000 - 400_000, data='f50047f1002be744', detail=''))
    rows.append(dict(source='can_cpp', event='bus_fault', recording_id='r1', sequence=175_400, mono_ns=8_273_400_000_000,
                     utc_ns=T + 300 * 1_000_000, axis=0, request_ns=0, detail='J5: uchyb śledzenia "−1,61°"\n'))
    rows.append(dict(source='bridge', event='ros_state_received', mono_ns=8_267_743_806_299, utc_ns=T + 301_000_000,
                     pid=18342, recording_id='inny', dropped_total=0,
                     state={'busy': True, 'axes': [{'axis': 5, 'speed_rpm': -10.0, 'ok': False}], 'message': None}))
    rows.append(dict(source='journal', event='log', utc_ns=T + 302_000_000, priority='3', process='bash', pid=None,
                     message='[bridge-3] Traceback'))
    rows.append(dict(source='rosout', event='log', utc_ns=T + 302_000_000, level='WARN', node='planner',
                     message='ostrzeżenie', file='', function='', line=0))
    rows.append(dict(source='kernel', utc_ns=T + 303_000_000, message='gs_usb: bez pola event', flag=True))
    return rows


def test_roundtrip_is_lossless_and_compact():
    rows = sample_rows()
    header = dict(source='fault_snapshot', event='fault_snapshot', recording_id='r1', utc_ns=T,
                  configuration={'active': {'revision': 'abc'}})
    payload = encode_snapshot(header, rows)
    decoded_header, decoded = decode_snapshot(payload)
    assert decoded_header == header
    assert decoded == rows
    plain = sum(len(json.dumps(row, separators=(',', ':'))) for row in rows)
    assert len(payload) * 8 < plain


def record(i, duration=15):
    return dict(snapshot=f's{i}', duration_s=duration, utc_ns=T + i * S), encode_snapshot({'n': i}, [])


def test_store_keeps_at_least_requested_time(tmp_path):
    store = RecordStore(tmp_path / 'faults.rfs', keep_s=40)
    for i in range(6):
        store.append(*record(i))
    entries, end = store.index()
    assert [meta['snapshot'] for _, _, meta in entries] == ['s3', 's4', 's5']  # 2 × 15 s < 40 s
    assert end == (tmp_path / 'faults.rfs').stat().st_size
    assert decode_snapshot(store.load(entries[-1][0])[1])[0] == {'n': 5}
    assert not list(tmp_path.glob('*.tmp'))


def test_store_size_cap_keeps_newest(tmp_path):
    store = RecordStore(tmp_path / 'faults.rfs', keep_s=1e9, max_bytes=1)
    for i in range(3):
        store.append(*record(i))
    entries, _ = store.index()
    assert [meta['snapshot'] for _, _, meta in entries] == ['s2']


def test_torn_record_at_end_is_dropped_on_next_append(tmp_path):
    path = tmp_path / 'faults.rfs'
    store = RecordStore(path, keep_s=1e9)
    store.append(*record(0))
    good = path.stat().st_size
    with path.open('ab') as file:
        file.write(b'RFS1\x05\x00')  # przerwany zapis
    assert [meta['snapshot'] for _, _, meta in store.index()[0]] == ['s0']
    store.append(*record(1))
    entries, end = store.index()
    assert [meta['snapshot'] for _, _, meta in entries] == ['s0', 's1']
    assert end == path.stat().st_size and entries[1][0] == good
