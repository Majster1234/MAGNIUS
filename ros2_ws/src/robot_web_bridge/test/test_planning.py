import asyncio
import json
from types import SimpleNamespace
import threading
import math
import pytest
from robot_web_bridge.app import create_app
from robot_web_bridge.planning import PlanningClient, pose_values
from robot_web_bridge.status import StatusStore
from test_bridge import scope, http


def test_tcp_units_and_quaternion():
    pose = pose_values([100, -50, 0, 0, 0, 90])
    assert pose['position'] == {'x': .1, 'y': -.05, 'z': 0}
    assert pose['orientation']['z'] == pytest.approx(2**-.5)
    with pytest.raises(ValueError):
        pose_values([None] * 6)


def client():
    value = PlanningClient.__new__(PlanningClient)
    axes = [dict(axis=i+1, available=i < 2, position_known=i < 2, position_degrees=0, joint_configured=False) for i in range(6)]
    value.manual = SimpleNamespace(snapshot=lambda: {'fresh': True, 'configuration_revision': 'rev', 'axes': axes})
    value.descriptions = SimpleNamespace(snapshot=lambda: ('<robot/>', 'model'))
    async def config(operation):
        return 200, {'operation': {'phase': 'idle', 'success': True}, 'active': {'revision': 'rev', 'document': {'values': {}}}}
    value.configuration = config
    return value


def test_context_freezes_missing_axes_requires_explicit_positions():
    value = client()
    data = dict(model_id='model', configuration_revision='rev', seed_degrees=[0.] * 6,
                preview_velocity_degrees_s=10, preview_acceleration_degrees_s2=50)
    result = asyncio.run(value.context(data))
    assert list(result.locked) == [False, False, True, True, True, True]
    data['seed_degrees'][2] = None
    with pytest.raises(ValueError, match='J3'):
        asyncio.run(value.context(data))
    data['configuration_revision'] = 'old'
    with pytest.raises(ValueError, match='zmianie'):
        asyncio.run(value.context(data))


@pytest.mark.parametrize('phase,success,accepted', [
    ('idle', True, True), ('done', True, True), ('done', False, False),
    ('done', None, False), ('idle', None, False), ('failed', False, False),
    ('uncertain', None, False), ('committing', None, False)])
def test_physical_context_requires_successful_configuration(phase, success, accepted):
    value = client()
    data = dict(model_id='model', configuration_revision='rev', seed_degrees=[0.]*6,
                preview_velocity_degrees_s=10, preview_acceleration_degrees_s2=50)
    async def configuration(_):
        return 200, dict(operation=dict(phase=phase, success=success),
                         active=dict(revision='rev', document=dict(values={})))
    value.configuration = configuration
    if accepted:
        result = asyncio.run(value.context(data))
        assert result.configuration_revision == 'rev' and not result.simulation_only
    else:
        with pytest.raises(RuntimeError, match='nieuzgodniona'):
            asyncio.run(value.context(data))


def test_done_does_not_allow_physical_context_with_stale_axis_revision():
    value = client()
    async def configuration(_):
        return 200, dict(operation=dict(phase='done', success=True),
                         active=dict(revision='new', document=dict(values={})))
    value.configuration = configuration
    with pytest.raises(ValueError, match='Rewizja profilu'):
        asyncio.run(value.context(dict(model_id='model', configuration_revision='new')))


def test_preview_reference_cache_has_provenance_and_model_revision_boundary():
    value = client(); value.reference_lock = threading.Lock()
    state = value.manual.snapshot()
    seeds, sources = value.reference('m','rev',state)
    assert seeds == [0]*6 and sources == ['measured','measured','zero','zero','zero','zero']
    state['axes'][0]['position_degrees'] = 15
    value.reference('m','rev',state)
    state['axes'][0]['available'] = False
    seeds, sources = value.reference('m','rev',state)
    assert seeds[0] == 15 and sources[0] == 'last'
    seeds, sources = value.reference('another','rev',state)
    assert seeds[0] == 0 and sources[0] == 'zero'


def feed_client():
    from robot_control_interfaces.action import ComputePreviewMotion
    value = client()
    value.descriptions = SimpleNamespace(snapshot=lambda: ('<robot configuration_revision="rev"/>', 'model'))
    value.preview_type = ComputePreviewMotion
    value.preview = SimpleNamespace(server_is_ready=lambda: True)
    values = {f'joint.{i}.{k}': v for i in range(1, 7) for k, v in dict(
        gear_ratio=1, direction=1, zero_offset=0, lower=-180, upper=180, velocity=30, acceleration=100).items()}
    async def configuration(operation):
        if operation == 'system_get':
            return 200, dict(revision='settings')
        return 200, dict(operation=dict(phase='idle', success=True), active=dict(revision='rev', document=dict(values=values)))
    async def submit(goal, action, metadata):
        return goal
    value.configuration, value.submit = configuration, submit
    return value


def test_lin_tcp_units_and_profile_limits_without_joint_speed_inputs():
    data = dict(model_id='model', configuration_revision='rev', settings_revision='settings', motion_type='LIN',
                seed_degrees=[0.]*6, target_degrees=[1.]*6, simulated_start=[True]*6,
                tcp_velocity_mm_s=25, tcp_acceleration_mm_s2=200, tcp_angular_velocity_degrees_s=15)
    goal = asyncio.run(feed_client().move(data))
    assert goal.tcp_velocity == pytest.approx(.025)
    assert goal.tcp_acceleration == pytest.approx(.2)
    assert goal.tcp_angular_velocity == pytest.approx(math.radians(15))
    assert list(goal.context.max_velocity) == pytest.approx([math.radians(30)]*6)
    assert list(goal.context.max_acceleration) == pytest.approx([math.radians(100)]*6)


@pytest.mark.parametrize('field,value', [('tcp_velocity_mm_s', 0), ('tcp_velocity_mm_s', -1),
    ('tcp_velocity_mm_s', float('nan')), ('tcp_acceleration_mm_s2', 0), ('tcp_angular_velocity_degrees_s', 0)])
def test_lin_rejects_invalid_tcp_limits(field, value):
    data = dict(model_id='model', configuration_revision='rev', settings_revision='settings', motion_type='LIN',
                seed_degrees=[0.]*6, target_degrees=[1.]*6, simulated_start=[True]*6, tcp_velocity_mm_s=25)
    data[field] = value
    with pytest.raises(ValueError):
        asyncio.run(feed_client().move(data))


def test_read_and_cancel_routes_and_no_execution_api(tmp_path):
    calls = []
    class Stub:
        jobs = {'job': {'id': 'job', 'status': 'running'}}
        async def status(self):
            return {'planner_available': True}
        async def cancel(self, identifier):
            calls.append(identifier); return {'status': 'cancelled'}
        def get_plan(self, identifier):
            return {'id': identifier, 'execution_ready': False}
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, planning_client=Stub())
    assert asyncio.run(http(app, '/api/v1/planning/context'))[0] == 200
    assert asyncio.run(http(app, '/api/v1/planning/jobs/missing'))[0] == 404
    assert asyncio.run(http(app, '/api/v1/planning/execute'))[0] == 404
    async def cancel():
        output = []
        async def receive():
            return {'type': 'http.request', 'body': b'', 'more_body': False}
        async def send(message):
            output.append(message)
        request = scope('http', '/api/v1/planning/jobs/job'); request['method'] = 'DELETE'
        await app(request, receive, send)
        return output
    assert asyncio.run(cancel())[0]['status'] == 200 and calls == ['job']


def circ_client(extra_values=None):
    from robot_control_interfaces.action import ComputeMotionChain
    value = feed_client()
    value.chain_type = ComputeMotionChain
    value.chain = SimpleNamespace(server_is_ready=lambda: True)
    base = value.configuration
    async def configuration(operation):
        code, data = await base(operation)
        if operation != 'system_get' and extra_values:
            data['active']['document']['values'].update(extra_values)
        return code, data
    async def submit(goal, action, metadata):
        return goal, metadata
    value.configuration, value.submit = configuration, submit
    return value


def circ_data(**extra):
    return dict(model_id='model', configuration_revision='rev', settings_revision='settings', motion_type='CIRC',
                seed_degrees=[0.]*6, target_degrees=[2.]*6, via_degrees=[1.]*6, simulated_start=[True]*6,
                tcp_velocity_mm_s=20, tcp_acceleration_mm_s2=100, tcp_angular_velocity_degrees_s=30, **extra)


def test_circ_preview_is_single_segment_chain_with_orientation_and_request():
    from robot_control_interfaces.msg import MotionSegment
    goal, metadata = asyncio.run(circ_client().move(circ_data(circ_orientation='path')))
    assert len(goal.segments) == 1 and goal.max_duration == pytest.approx(95)
    segment = goal.segments[0]
    assert segment.motion_type == MotionSegment.CIRC and segment.circ_orientation == MotionSegment.CIRC_PATH
    assert list(segment.via_positions) == pytest.approx([math.radians(1)]*6)
    assert list(segment.target_positions) == pytest.approx([math.radians(2)]*6)
    assert segment.tcp_velocity == pytest.approx(.02) and segment.blend_radius == 0
    assert list(segment.max_joint_velocity) == pytest.approx([math.radians(30)]*6)   # profil osi (profile_limits)
    assert metadata['circ_request'] == {'via_degrees': [1.]*6, 'target_degrees': [2.]*6, 'circ_orientation': 'path',
                                        'tcp_angular_acceleration_degrees_s2': 0.}
    goal, metadata = asyncio.run(circ_client({'tcp.angular_acceleration': 100}).move(circ_data()))
    assert goal.segments[0].tcp_angular_acceleration == pytest.approx(math.radians(100))
    assert metadata['circ_request']['tcp_angular_acceleration_degrees_s2'] == 100
    goal, _ = asyncio.run(circ_client().move(circ_data()))
    assert goal.segments[0].circ_orientation == MotionSegment.CIRC_INTERPOLATED


@pytest.mark.parametrize('change,text', [(dict(via_degrees=[1.]*5), 'pośrednich'), (dict(circ_orientation='center'), 'orientacja'),
                                         (dict(tcp_velocity_mm_s=150), 'tcp.max_linear_velocity')])
def test_circ_preview_rejects_bad_input_and_speed_above_configuration(change, text):
    data = circ_data(); data.update(change)
    with pytest.raises(ValueError, match=text):
        asyncio.run(circ_client({'tcp.max_linear_velocity': 100, 'tcp.max_angular_velocity': 100}).move(data))
