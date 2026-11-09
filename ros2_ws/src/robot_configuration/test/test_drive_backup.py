import json
import os
import stat
import time

import pytest

from robot_configuration.drive_backup import AXES, SPECS, Conflict, DriveBackups

BY_CODE = {(s['code'], s['sub']): name for name, s in SPECS.items()}


class FakeDrives:
    """Atrapa sześciu napędów: nastawy per adres, blokady jak w C++ (ENABLE=0, kody tylko do odczytu)."""

    def __init__(self):
        self.values = {a: {} for a in AXES}
        for a in AXES:
            for name, spec in SPECS.items():
                if name in ('home_parameters', 'pulse_divider_output'):
                    continue  # firmware bez odczytu (FFh FFh)
                data = [a] * spec['length']
                if spec['sub'] >= 0:
                    data[0] = spec['sub']
                self.values[a][name] = data
            self.values[a]['work_mode'] = [5]
            self.values[a]['work_current_ma'] = [0x06, 0x40]
        self.enabled = {a: False for a in AXES}
        self.writes, self.fail_write = [], None

    def read(self, address, code, sub):
        name = BY_CODE[(code, sub)]
        if name not in self.values[address]:
            return False, []
        return True, list(self.values[address][name])

    def write(self, address, code, sub, value):
        name = BY_CODE[(code, sub)]
        if not SPECS[name]['writable']:
            raise RuntimeError('Nastawa tylko do odczytu')
        if self.enabled[address]:
            raise RuntimeError('zapis nastaw wymaga potwierdzonego ENABLE=0 osi')
        if self.fail_write == (address, name):
            raise RuntimeError('Napęd odrzucił zapis (status 0)')
        self.values[address][name] = list(value)
        self.writes.append((address, name, bytes(value).hex()))


@pytest.fixture
def backups(tmp_path):
    drives = FakeDrives()
    return DriveBackups(tmp_path / 'drive-backups', drives), drives


def test_base_is_written_once_readonly_and_checksummed(backups):
    store, drives = backups
    result = store.capture('base')
    path = store.path('base')
    assert result['unsupported']['1'] == ['home_parameters', 'pulse_divider_output']
    assert not os.stat(path).st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH)
    with pytest.raises(Conflict, match='nietykalna'):
        store.capture('base')
    loaded = store.load('base')
    assert loaded['axes']['4']['address'] == 4 and loaded['axes']['4']['parameters']['work_current_ma'] == '0640'
    os.chmod(path, 0o600)
    data = json.loads(path.read_text()); data['axes']['2']['parameters']['work_current_ma'] = '0bb8'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='SHA-256'):
        store.load('base')


def test_pre_test_capture_archives_previous(backups):
    store, drives = backups
    store.capture('pre_test'); time.sleep(.01); store.capture('pre_test')
    assert len(list((store.root / 'archive').glob('pre_test-*.json'))) == 1


@pytest.mark.parametrize('params,match', [
    ({'direction': 1}, 'niedozwolona'), ({'work_current_ma': 4000}, '1–3000'), ({'microstep': 100}, 'dozwolone'),
    ({'work_mode': 2}, 'dozwolone'), ({'vfoc_kp_ki': [220, 2000]}, '0–1024'), ({'position_error_protection': [1, 2]}, 'wymagane')])
def test_test_container_validates_names_and_ranges(backups, params, match):
    store, _ = backups
    with pytest.raises(ValueError, match=match):
        store.set_test({'axes': {'2': params}})


def test_apply_test_requires_base_and_fresh_pre_test_then_restore_returns_own_values(backups):
    store, drives = backups
    store.set_test({'axes': {'2': {'work_current_ma': 2000, 'vfoc_kp_ki': [300, 120]}, '3': {'work_mode': 4}}})
    with pytest.raises(Conflict, match='base'):
        store.apply_test()
    store.capture('base')
    with pytest.raises(Conflict, match='pre_test'):
        store.apply_test()
    store.capture('pre_test')
    drives.values[2]['hold_current_ratio'] = [7]  # zmiana poza kopią przed testem
    with pytest.raises(Conflict, match='różni się'):
        store.apply_test()
    store.capture('pre_test')
    drives.enabled[2] = True
    result = store.apply_test()
    assert result['applied'] is False and 'ENABLE=0' in result['axes']['2']['error']
    drives.enabled[2] = False
    drives.writes.clear()
    result = store.apply_test()
    assert result['applied'] is True
    assert drives.values[2]['work_current_ma'] == [0x07, 0xd0] and drives.values[3]['work_mode'] == [4]
    assert [w[:2] for w in drives.writes] == [(2, 'work_current_ma'), (2, 'vfoc_kp_ki'), (3, 'work_mode')]
    assert store.verify('test')['differences'] == {}
    restored = store.restore('pre_test')
    assert restored['remaining_differences'] == {}
    assert drives.values[2]['work_current_ma'] == [0x06, 0x40] and drives.values[3]['work_mode'] == [5]
    assert drives.values[2]['hold_current_ratio'] == [7], 'pre_test zawiera stan sprzed testu, nie bazę'
    store.restore('base', axes=[2])
    assert drives.values[2]['hold_current_ratio'] == [2] and drives.values[1]['hold_current_ratio'] == [1]


def test_restore_writes_only_differences_and_reports_read_only_mismatch(backups):
    store, drives = backups
    store.capture('base')
    drives.values[5]['microstep'] = [16]; drives.values[5]['can_bitrate'] = [9]; drives.values[1]['work_mode'] = [4]
    drives.writes.clear()
    result = store.restore('base')
    assert sorted(drives.writes) == [(1, 'work_mode', '05'), (5, 'microstep', '05')]
    assert list(result['remaining_differences']) == ['5'] and list(result['remaining_differences']['5']) == ['can_bitrate']
    assert result['remaining_differences']['5']['can_bitrate']['writable'] is False


def test_failed_test_apply_rolls_back_touched_axes(backups):
    store, drives = backups
    store.capture('base'); store.capture('pre_test')
    store.set_test({'axes': {'1': {'work_current_ma': 1800}, '3': {'work_current_ma': 1200}}})
    drives.fail_write = (3, 'work_current_ma')
    result = store.apply_test()
    assert result['applied'] is False and result['axes']['3']['error']
    assert drives.values[1]['work_current_ma'] == [0x06, 0x40], 'J1 wrócił do kopii przed testami'
    assert result['rollback']['axes']['1']['written'] == ['work_current_ma']


def test_async_operation_and_status(backups):
    store, drives = backups
    store.start('capture_pre_test', {})
    with pytest.raises(Conflict):
        store.start('verify', {'kind': 'pre_test'})
    for _ in range(200):
        if store.status()['operation']['phase'] != 'running':
            break
        time.sleep(.01)
    status = store.status()
    assert status['operation']['success'] is True and status['containers']['pre_test']['valid']
    assert status['containers']['base'] is None
    assert status['containers']['pre_test']['axes']['2']['work_mode']['value'] == 'SR_vFOC'
    with pytest.raises(ValueError):
        store.start('restore', {'source': 'test'})
    store.capture('base')
    with pytest.raises(Conflict, match='nietykalna'):
        store.start('capture_base', {})


def test_sequential_candidates_need_single_pre_test_but_other_changes_block(backups):
    store, drives = backups
    store.capture('base'); store.capture('pre_test')
    for kp in (240, 260, 280):
        store.set_test({'axes': {'4': {'vfoc_kp_ki': [kp, 100]}}})
        assert store.apply_test()['applied'] is True
        assert drives.values[4]['vfoc_kp_ki'] == [0, kp >> 8, kp & 0xff, 0, 100]
    drives.values[4]['work_current_ma'] = [0x05, 0xdc]  # zmiana poza kontenerem testowym
    store.set_test({'axes': {'4': {'vfoc_kp_ki': [300, 100]}}})
    with pytest.raises(Conflict, match='różni się'):
        store.apply_test()
    drives.values[4]['work_current_ma'] = [0x06, 0x40]
    assert store.restore('pre_test', axes=[4])['remaining_differences'] == {}
    assert drives.values[4]['vfoc_kp_ki'] == [0, 4, 4, 4, 4]
