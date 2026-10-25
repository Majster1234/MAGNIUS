from copy import deepcopy
import json
import math
import xml.etree.ElementTree as ET
import pytest
from robot_configuration.schema import empty_document, validate, profiles, build_urdf, validate_transition
from robot_configuration.storage import Storage, decode
from robot_configuration.coordinator import Coordinator, Conflict


def configured(axis=1):
    document = empty_document()
    for key, value in dict(gear_ratio=50, direction=-1, zero_offset=10, lower=-180, upper=180, velocity=30, acceleration=60).items():
        document['values'][f'joint.{axis}.{key}'] = value
    document['values']['joint.1.origin_x'] = 100
    document['values']['tool.tcp_y'] = 25
    return document


def test_system_setting_is_atomic_shared_and_independent_of_robot(tmp_path):
    first = Storage(tmp_path)
    revision = first.active()['revision']
    assert first.system_settings() == {'revision': 'initial', 'ignore_missing_drivers': False, 'allow_missing_motion': False}
    saved = first.set_system_settings(True)
    assert Storage(tmp_path).system_settings() == saved and saved['ignore_missing_drivers']
    assert first.active()['revision'] == revision and not list(tmp_path.glob('.*.tmp'))
    assert first.set_system_settings(True) == saved
    with pytest.raises(ValueError):
        first.set_system_settings(1)
    first.atomic('system_settings.json', {'revision': 'bad', 'ignore_missing_drivers': 'false'})
    with pytest.raises(ValueError):
        first.system_settings()


def test_schema_roundtrip_units_partial_axes_and_zero():
    document = configured()
    assert validate(json.loads(json.dumps(document))) == document
    axis = profiles(document)[0]
    assert axis['configured'] and axis['gear_ratio'] == 50 and axis['direction'] == -1
    assert axis['zero_offset_rad'] == pytest.approx(math.pi / 18)
    assert axis['velocity_rad_s'] == pytest.approx(math.pi / 6)
    assert not profiles(document)[1]['configured']
    document['values']['joint.2.origin_x'] = 0
    assert validate(document)['values']['joint.2.origin_x'] == 0
    assert validate(document)['values']['joint.2.origin_y'] is None


@pytest.mark.parametrize('key,value', [('joint.1.gear_ratio', 0), ('joint.1.direction', 2),
    ('joint.1.velocity', -1), ('joint.1.acceleration', float('inf')), ('tool.mass', -1), ('joint.1.origin_x', True),
    ('robot.base_frame', 'link_2'), ('joint.1.upper', -190)])
def test_reject_invalid_values(key, value):
    document = configured(); document['values'][key] = value
    with pytest.raises(ValueError):
        validate(document)


def test_reject_unknown_version_fields_and_removing_active_profile():
    document = configured(); old = deepcopy(document)
    document['values']['joint.1.gear_ratio'] = None
    with pytest.raises(ValueError, match='J1'):
        validate_transition(old, validate(document))
    for change in ({'schema_version': 9}, {'units': 'm-rad-kg'}, {'unexpected': 1}):
        with pytest.raises(ValueError):
            validate({**old, **change})
    with pytest.raises(ValueError):
        decode('{"value":NaN}')


def test_urdf_uses_same_origins_rpy_tcp_and_continuous_joints():
    document = configured(); document['values']['joint.2.origin_yaw'] = 90
    root = ET.fromstring(build_urdf(document, 'revision-1'))
    assert root.attrib['configuration_revision'] == 'revision-1'
    assert root.attrib['model_kind'] == 'partial-demo'
    assert root.find("joint[@name='joint_1']/origin").attrib['xyz'] == '0.1 0 0.1'
    assert [float(x) for x in root.find("joint[@name='joint_2']/origin").attrib['rpy'].split()][2] == pytest.approx(math.pi/2)
    assert root.find("joint[@name='tool_fixed']/origin").attrib['xyz'] == '0.08 0.025 0'
    assert len([joint for joint in root.findall('joint') if joint.attrib['type'] == 'continuous']) == 6
    assert root.find('.//mesh') is None


def test_storage_pagination_restart_delete_and_invalid_files(tmp_path):
    storage = Storage(tmp_path)
    identifiers = [storage.create({**configured(), 'name': f'Wersja {i}'})['id'] for i in range(25)]
    first = storage.list(); second = storage.list(first['next_cursor'])
    assert len(first['items']) == 20 and len(second['items']) == 5
    assert len({item['id'] for item in first['items'] + second['items']}) == 25
    assert Storage(tmp_path).get(identifiers[0])['name'] == 'Wersja 0'
    storage.delete(identifiers[0])
    with pytest.raises(FileNotFoundError):
        storage.get(identifiers[0])
    for identifier in ('../../active', '/etc/passwd', '', None):
        with pytest.raises(ValueError):
            storage.get(identifier)
    (tmp_path/'active.json').write_text('{broken')
    with pytest.raises(ValueError):
        storage.active()


class Runtime:
    def __init__(self):
        self.revision = 'initial'; self.stage = 'boot'; self.locked = True; self.operation_id = ''
        self.previous = 'initial'; self.model_xml = ''; self.fail = ''; self.calls = []
        self.invalid = [False]*6; self.offsets = {}; self.resolved = None

    def control(self, phase, operation_id, state, changed, expected='', confirm=None):
        self.calls.append(phase)
        if self.fail == phase:
            raise TimeoutError('Brak odpowiedzi ' + phase)
        if phase in ('bootstrap', 'prepare'):
            self.previous = self.revision; self.operation_id = operation_id; self.locked = True
            self.stage = 'prepared'
            axes=profiles(state['document'])
            for i, offset in self.offsets.items(): axes[i]['zero_offset_rad']=offset
            invalid=list(self.invalid)
            for i, value in enumerate(confirm or [False]*6):
                if value: invalid[i]=False
            self.resolved=dict(revision=state['revision'],axes=axes,calibration_invalid=invalid)
            if phase == 'bootstrap':
                self.revision = state['revision']; self.stage = 'committed'
        if phase == 'commit':
            self.revision = state['revision']; self.stage = 'committed'
        if phase == 'rollback':
            self.revision = self.previous; self.stage = 'rolled_back'; self.operation_id = operation_id; self.locked = True
        if phase == 'finish':
            self.stage = 'finished'; self.locked = False
        return dict(revision=self.revision, stage=self.stage, operation_id=self.operation_id, locked=self.locked, resolved_configuration=self.resolved)

    def model(self, xml):
        if self.fail == 'model':
            self.fail = ''; raise RuntimeError('Odrzucono URDF')
        self.model_xml = xml


def ready(tmp_path):
    storage, transport = Storage(tmp_path), Runtime()
    coordinator = Coordinator(storage, transport); coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'idle'
    return coordinator, storage, transport


def test_apply_persists_only_confirmed_revision_and_rejects_conflicts(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    assert coordinator.operation['success'] is True
    assert storage.active()['revision'] == transport.revision == coordinator.active['revision']
    assert ET.fromstring(transport.model_xml).attrib['configuration_revision'] == transport.revision
    assert not transport.locked and storage.journal() is None
    with pytest.raises(Conflict):
        coordinator.begin(configured(), 'initial')
    assert Storage(tmp_path).active() == coordinator.active


@pytest.mark.parametrize('phase', ['idle', 'done'])
def test_reapplying_same_values_preserves_revision_model_and_files(tmp_path, phase):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    coordinator.phase(phase, 'Gotowe', True)
    previous = deepcopy(coordinator.active)
    files = {p.name: (p.read_bytes(), p.stat().st_mtime_ns) for p in tmp_path.glob('*.json')}
    calls, model = list(transport.calls), transport.model_xml
    document = deepcopy(previous['document'])
    document.update(name='Inny snapshot', created_at='2026-10-25T01:00:00Z')
    # 50.0 i 50 mają ten sam sens fizyczny; zmiana metadanych też nie zmienia rewizji.
    document['values']['joint.1.gear_ratio'] = 50.0
    for _ in range(3):
        coordinator.apply(*coordinator.begin(document, previous['revision']))
        assert coordinator.active == previous == storage.active()
        assert coordinator.operation['phase'] == 'done' and coordinator.operation['success'] is True
        assert not coordinator.running and not transport.locked
    assert transport.calls == calls and transport.model_xml == model
    assert {p.name: (p.read_bytes(), p.stat().st_mtime_ns) for p in tmp_path.glob('*.json')} == files
    assert storage.journal() is None


@pytest.mark.parametrize('key,value', [('joint.1.gear_ratio', 25), ('joint.1.zero_offset', 11),
    ('joint.1.direction', 1), ('joint.1.velocity', 31), ('joint.1.acceleration', 61),
    ('joint.1.lower', -179), ('joint.1.origin_x', 101), ('tool.tcp_y', 26),
    ('tool.mass', 1), ('robot.name', 'Inny robot')])
def test_actual_parameter_change_still_uses_new_revision_and_full_transaction(tmp_path, key, value):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    revision = coordinator.active['revision']; transport.calls.clear()
    document = deepcopy(coordinator.active['document']); document['values'][key] = value
    coordinator.apply(*coordinator.begin(document, revision))
    assert coordinator.active['revision'] != revision
    assert transport.calls == ['prepare', 'commit', 'finish']
    assert storage.active()['revision'] == transport.revision == coordinator.active['revision']


def test_no_change_still_checks_conflicts_and_serializes_requests(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    current = deepcopy(coordinator.active)
    with pytest.raises(Conflict):
        coordinator.begin(current['document'], 'initial')
    args = coordinator.begin(current['document'], current['revision'])
    with pytest.raises(Conflict):
        coordinator.begin(current['document'], current['revision'])
    coordinator.apply(*args)
    assert coordinator.active == current and not coordinator.running
    coordinator.phase('uncertain', 'Nieznany wynik')
    with pytest.raises(Conflict):
        coordinator.begin(current['document'], current['revision'])


def test_same_values_do_not_skip_calibration_confirmation_or_recovery(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    current = deepcopy(coordinator.active)
    storage.atomic('calibration.json', {'revision': current['revision'], 'invalid': [True] + [False]*5})
    transport.calls.clear()
    coordinator.apply(*coordinator.begin(current['document'], current['revision'], [1]))
    assert coordinator.active['revision'] != current['revision']
    assert transport.calls == ['prepare', 'commit', 'finish']
    assert storage.calibration(coordinator.active['revision']) == [False]*6
    current = deepcopy(coordinator.active)
    coordinator.phase('failed', 'Przywrócono poprzedni profil', False)
    transport.fail = 'prepare'
    coordinator.apply(*coordinator.begin(current['document'], current['revision']))
    assert coordinator.operation['phase'] == 'failed' and not coordinator.operation['success']


@pytest.mark.parametrize('failure', ['model', 'commit', 'disk'])
def test_failure_rolls_back_model_runtime_and_file(tmp_path, failure):
    coordinator, storage, transport = ready(tmp_path)
    args = coordinator.begin(configured(), 'initial')
    if failure == 'disk':
        atomic = storage.atomic
        def write(name, value):
            if name == 'active.json' and value['revision'] != 'initial':
                raise OSError('Dysk pełny')
            return atomic(name, value)
        storage.atomic = write
    else:
        transport.fail = failure
    coordinator.apply(*args)
    assert coordinator.operation['phase'] == 'failed'
    assert coordinator.active['revision'] == transport.revision == storage.active()['revision'] == 'initial'
    assert not transport.locked
    assert ET.fromstring(transport.model_xml).attrib['configuration_revision'] == 'initial'


def test_unknown_rollback_keeps_lock_and_can_reconcile_after_restart(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    args = coordinator.begin(configured(), 'initial')
    transport.fail = 'commit'
    original = transport.control
    def call(phase, *values):
        if phase == 'rollback':
            raise TimeoutError('Niepotwierdzone wycofanie')
        return original(phase, *values)
    transport.control = call
    coordinator.apply(*args)
    assert coordinator.operation['phase'] == 'uncertain' and transport.locked and storage.journal()
    with pytest.raises(Conflict):
        coordinator.begin(configured(), 'initial')
    transport.control = original; transport.fail = ''
    restored = Coordinator(Storage(tmp_path), transport); restored.bootstrap()
    assert restored.operation['phase'] == 'idle' and not transport.locked
    assert restored.active['revision'] == transport.revision == 'initial'


def test_bootstrap_description_failure_can_be_reconciled(tmp_path):
    storage, transport = Storage(tmp_path), Runtime()
    transport.fail = 'model'
    coordinator = Coordinator(storage, transport); coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'uncertain' and transport.locked
    coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'idle' and not transport.locked


@pytest.mark.parametrize('lost_phase', ['commit', 'finish'])
def test_lost_ack_does_not_claim_failure_or_leave_mixed_revision(tmp_path, lost_phase):
    coordinator, storage, transport = ready(tmp_path)
    original = transport.control
    lost = [False]
    def control(phase, *args):
        response = original(phase, *args)
        if phase == lost_phase and not lost[0]:
            lost[0] = True
            raise TimeoutError('Zagubiony ACK po wykonaniu operacji')
        return response
    transport.control = control
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    expected = 'failed' if lost_phase == 'commit' else 'done'
    assert coordinator.operation['phase'] == expected
    assert storage.active()['revision'] == coordinator.active['revision'] == transport.revision
    assert not transport.locked


def test_calibration_flags_survive_storage_restart_and_failed_apply(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    storage.atomic('calibration.json', {'revision':'initial', 'invalid':[True,False,False,False,False,False]})
    assert Storage(tmp_path).calibration('initial')[0]
    transport.fail = 'model'
    coordinator.apply(*coordinator.begin(configured(),'initial'))
    assert storage.calibration('initial')[0]
    assert coordinator.operation['phase'] == 'failed'


def test_reconcile_same_process_reloads_durable_revision_after_unknown_finish(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    args = coordinator.begin(configured(),'initial')
    original = transport.control
    finishing = [False]
    def control(phase, *values):
        if phase == 'finish':
            finishing[0] = True
            raise TimeoutError('Brak FINISH')
        if phase == 'status' and finishing[0]:
            raise TimeoutError('Brak STATUS')
        return original(phase, *values)
    transport.control = control; coordinator.apply(*args)
    assert coordinator.operation['phase'] == 'uncertain'
    assert storage.active()['revision'] != coordinator.active['revision']
    transport.control = original
    coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'idle'
    assert storage.active()['revision'] == coordinator.active['revision'] == transport.revision
    assert not transport.locked


def test_reconcile_rolled_back_runtime_overrides_candidate_file(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    previous = deepcopy(coordinator.active)
    operation_id, _, candidate, _ = coordinator.begin(configured(),'initial')
    storage.atomic('pending.json', {'id':operation_id,'previous':previous,'candidate':candidate,'previous_calibration':[True,False,False,False,False,False]})
    storage.atomic('active.json', candidate)
    transport.operation_id = operation_id;transport.stage = 'rolled_back';transport.locked = True
    coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'idle' and not transport.locked
    assert storage.active()['revision'] == coordinator.active['revision'] == transport.revision == 'initial'
    assert storage.calibration('initial')[0]


def test_partial_motion_settings_are_separate_durable_and_migrate(tmp_path):
    store = Storage(tmp_path)
    store.atomic('system_settings.json', {'revision': 'old', 'ignore_missing_drivers': True})
    assert store.system_settings()['allow_missing_motion'] is False
    saved = store.set_system_settings(allow_missing_motion=True)
    assert saved['ignore_missing_drivers'] and saved['allow_missing_motion']
    assert Storage(tmp_path).system_settings() == saved
    assert store.set_system_settings(False)['allow_missing_motion'] is True
    with pytest.raises(ValueError):
        store.set_system_settings(allow_missing_motion='true')


def test_resolved_offset_is_journaled_before_commit_and_persisted_for_restart(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    old = deepcopy(coordinator.active)
    document = deepcopy(old['document']); document['values']['joint.1.gear_ratio'] = 25
    transport.offsets[0] = math.radians(20)
    original = transport.control
    def control(phase, *args):
        if phase == 'commit':
            assert storage.journal()['candidate']['document']['values']['joint.1.zero_offset'] == pytest.approx(20)
            assert storage.journal()['candidate_calibration'] == [False]*6
        return original(phase, *args)
    transport.control = control
    coordinator.apply(*coordinator.begin(document, old['revision']))
    assert coordinator.operation['success']
    assert storage.active()['document']['values']['joint.1.zero_offset'] == pytest.approx(20)
    assert document['values']['joint.1.zero_offset'] == 10  # Żądanie/snapshot pozostaje bez zmian.
    restarted = Coordinator(Storage(tmp_path), transport); restarted.bootstrap()
    assert restarted.operation['phase'] == 'idle'
    assert restarted.active == coordinator.active


def test_unconfirmed_axis_persists_and_noop_cannot_clear_it(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    document = deepcopy(coordinator.active['document']); document['values']['joint.1.gear_ratio'] = 25
    transport.invalid[0] = True
    coordinator.apply(*coordinator.begin(document, coordinator.active['revision']))
    current = deepcopy(coordinator.active)
    assert storage.calibration(current['revision'])[0]
    coordinator.apply(*coordinator.begin(current['document'], current['revision']))
    assert coordinator.active == current and coordinator.snapshot()['calibration_invalid'][0]
    coordinator.apply(*coordinator.begin(current['document'], current['revision'], [1]))
    assert not any(storage.calibration(coordinator.active['revision']))


@pytest.mark.parametrize('bad', [[0], [7], [True], [1,1], '1', [1.0]])
def test_calibration_confirmation_rejects_bad_axis_lists(tmp_path, bad):
    coordinator, _, _ = ready(tmp_path)
    with pytest.raises(ValueError, match='confirm_calibration_axes'):
        coordinator.begin(configured(), 'initial', bad)


@pytest.mark.parametrize('field,value', [('zero_offset', 20), ('direction', 1), ('confirm', 1)])
def test_ratio_and_reference_changes_require_separate_writes(tmp_path, field, value):
    coordinator, _, _ = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    document = deepcopy(coordinator.active['document']); document['values']['joint.1.gear_ratio'] = 25
    if field != 'confirm': document['values']['joint.1.'+field] = value
    with pytest.raises(ValueError, match='osobnych'):
        coordinator.begin(document, coordinator.active['revision'], [1] if field == 'confirm' else [])


def test_failure_after_resolution_rolls_back_original_offset_and_calibration(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    previous = deepcopy(coordinator.active)
    document = deepcopy(previous['document']); document['values']['joint.1.gear_ratio'] = 25
    transport.offsets[0] = math.radians(20); transport.fail = 'commit'
    coordinator.apply(*coordinator.begin(document, previous['revision']))
    assert coordinator.operation['phase'] == 'failed'
    assert storage.active() == previous and not transport.locked
    assert storage.active()['document']['values']['joint.1.zero_offset'] == 10


def test_lost_finish_recovery_preserves_unconfirmed_calibration(tmp_path):
    coordinator, storage, transport = ready(tmp_path)
    transport.invalid[0] = True
    original = transport.control; finishing = [False]
    def control(phase, *args):
        if phase == 'finish':
            finishing[0] = True
            raise TimeoutError('FINISH')
        if phase == 'status' and finishing[0]: raise TimeoutError('STATUS')
        return original(phase, *args)
    transport.control = control
    coordinator.apply(*coordinator.begin(configured(), 'initial'))
    assert coordinator.operation['phase'] == 'uncertain' and storage.journal()['candidate_calibration'][0]
    transport.control = original
    coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'idle' and storage.calibration(coordinator.active['revision'])[0]
