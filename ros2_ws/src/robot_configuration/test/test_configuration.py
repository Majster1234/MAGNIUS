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

    def control(self, phase, operation_id, state, changed, expected=''):
        self.calls.append(phase)
        if self.fail == phase:
            raise TimeoutError('Brak odpowiedzi ' + phase)
        if phase in ('bootstrap', 'prepare'):
            self.previous = self.revision; self.operation_id = operation_id; self.locked = True
            self.stage = 'prepared'
            if phase == 'bootstrap':
                self.revision = state['revision']; self.stage = 'committed'
        if phase == 'commit':
            self.revision = state['revision']; self.stage = 'committed'
        if phase == 'rollback':
            self.revision = self.previous; self.stage = 'rolled_back'; self.operation_id = operation_id; self.locked = True
        if phase == 'finish':
            self.stage = 'finished'; self.locked = False
        return dict(revision=self.revision, stage=self.stage, operation_id=self.operation_id, locked=self.locked)

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
    operation_id, _, candidate = coordinator.begin(configured(),'initial')
    storage.atomic('pending.json', {'id':operation_id,'previous':previous,'candidate':candidate,'previous_calibration':[True,False,False,False,False,False]})
    storage.atomic('active.json', candidate)
    transport.operation_id = operation_id;transport.stage = 'rolled_back';transport.locked = True
    coordinator.bootstrap()
    assert coordinator.operation['phase'] == 'idle' and not transport.locked
    assert storage.active()['revision'] == coordinator.active['revision'] == transport.revision == 'initial'
    assert storage.calibration('initial')[0]
