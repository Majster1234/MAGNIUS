"""Kroki TCP: matematyka, walidacja i HTTP; bez napędów."""
import asyncio
from concurrent.futures import Future
from copy import deepcopy
import json
import math
from types import SimpleNamespace

from geometry_msgs.msg import Pose
import pytest
from robot_control_interfaces.srv import SolveIK
from robot_web_bridge.app import create_app
from robot_web_bridge.planning import pose_values
from robot_web_bridge.status import StatusStore
from robot_web_bridge.tcp_jog import increment_pose, multiply
from rosidl_runtime_py.set_message import set_message_fields
from test_bridge import scope
from test_planning import feed_client


def pose(values):
    value = Pose()
    set_message_fields(value, pose_values(values))
    return value


@pytest.mark.parametrize('axis', ['x', 'y', 'z', 'rx', 'ry', 'rz'])
@pytest.mark.parametrize('direction', [-1, 1])
def test_six_tcp_directions(axis, direction):
    original = pose([100, 200, 300, 0, 0, 0])
    result = increment_pose(original, 'base', axis, direction*.001)
    index = ['x', 'y', 'z', 'rx', 'ry', 'rz'].index(axis) % 3
    xyz = [result.position.x, result.position.y, result.position.z]
    if len(axis) == 1:
        expected = [.1, .2, .3]
        expected[index] += direction*.001
        assert xyz == pytest.approx(expected)
        assert result.orientation.w == 1
    else:
        assert xyz == pytest.approx([.1, .2, .3])
        assert [result.orientation.x, result.orientation.y, result.orientation.z][index] == pytest.approx(math.sin(direction*.001/2))
    assert original == pose([100, 200, 300, 0, 0, 0])


def test_tool_translation_and_noncommuting_quaternion_rotations():
    original = pose([0, 0, 0, 0, 0, 90])
    base = increment_pose(original, 'base', 'x', .01)
    tool = increment_pose(original, 'tool', 'x', .01)
    assert [base.position.x, base.position.y] == pytest.approx([.01, 0])
    assert [tool.position.x, tool.position.y] == pytest.approx([0, .01])
    base_rotation = increment_pose(original, 'base', 'rx', .1).orientation
    tool_rotation = increment_pose(original, 'tool', 'rx', .1).orientation
    assert base_rotation.y == pytest.approx(-tool_rotation.y)
    assert abs(base_rotation.y) > .01
    q = (base_rotation.x, base_rotation.y, base_rotation.z, base_rotation.w)
    assert sum(v*v for v in q) == pytest.approx(1)
    assert multiply(q, (-q[0], -q[1], -q[2], q[3])) == pytest.approx([0, 0, 0, 1])


def client():
    value = feed_client()
    value.ik_type = SolveIK
    requests = []
    response = SimpleNamespace(success=True, message='', positions=[0.]*6, tcp=pose([100, 200, 300, 0, 0, 90]))
    def call(request):
        requests.append(request)
        result = deepcopy(response)
        if not request.forward_only:
            result.positions[0] = math.radians(.1)
            result.tcp = request.target
        future = Future()
        future.set_result(result)
        return future
    value.ik = SimpleNamespace(service_is_ready=lambda: True, call_async=call, remove_pending_request=lambda _: None)
    old_config = value.configuration
    settings = dict(revision='settings', ignore_missing_drivers=True)
    async def config(operation):
        return (200, settings) if operation == 'system_get' else await old_config(operation)
    value.configuration = config
    return value, requests, response, settings


def payload(**changes):
    return dict(model_id='model', configuration_revision='rev', settings_revision='settings',
                seed_degrees=[0.]*6, simulated_start=[True]*6, frame='tool', axis='x', direction=1,
                velocity=10, duration_s=.05, **changes)


def test_jog_uses_preview_fk_then_ik_and_never_exposes_execution():
    value, requests, _, _ = client()
    result = asyncio.run(value.tcp_jog(payload()))
    assert result['simulation_only'] and result['execution_ready'] is False
    assert result['positions_degrees'][0] == pytest.approx(.1)
    assert len(requests) == 2 and requests[0].forward_only and not requests[1].forward_only
    assert all(r.context.simulation_only and not any(r.context.locked) for r in requests)
    assert requests[1].target.position.y == pytest.approx(.2005)


@pytest.mark.parametrize('changes', [dict(frame='camera'), dict(axis='bad'), dict(direction=0), dict(duration_s=1),
    dict(velocity=float('nan')), dict(velocity=101), dict(seed_degrees=[None]*6), dict(settings_revision='old'),
    dict(configuration_revision='old'), dict(model_id='old')])
def test_invalid_input(changes):
    value, _, _, _ = client()
    with pytest.raises(ValueError):
        asyncio.run(value.tcp_jog({**payload(), **changes}))


def test_missing_stale_or_uncalibrated_axes_cannot_bypass_settings():
    value, _, _, settings = client()
    settings['ignore_missing_drivers'] = False
    with pytest.raises(ValueError, match='pozycji'):
        asyncio.run(value.tcp_jog(payload()))
    state = dict(fresh=True, configuration_revision='rev', axes=[dict(axis=i+1, available=True,
                 position_known=True, position_age_seconds=0) for i in range(6)])
    value.manual.snapshot = lambda: state
    assert asyncio.run(value.tcp_jog(payload()))['simulation_only']
    for changes in (dict(available=False), dict(position_age_seconds=.25),
                    dict(joint_configured=True, joint_position_known=True, calibration_valid=False)):
        state['axes'][4] = {**state['axes'][0], 'axis': 5, **changes}
        with pytest.raises(ValueError, match='J5'):
            asyncio.run(value.tcp_jog(payload()))


def test_unreachable_and_velocity_or_position_limit_rejected():
    value, _, response, _ = client()
    response.success = False
    response.message = 'Brak rozwiązania'
    with pytest.raises(ValueError, match='Brak rozwiązania'):
        asyncio.run(value.tcp_jog(payload()))
    response.success = True
    response.positions[1] = math.radians(10)
    with pytest.raises(ValueError, match='J2'):
        asyncio.run(value.tcp_jog(payload()))
    response.positions[1] = math.radians(181)
    with pytest.raises(ValueError, match='J2'):
        asyncio.run(value.tcp_jog(payload()))


@pytest.mark.parametrize('axis,speed,steps', [('x', 100, 5), ('rx', 30, 2)])
def test_full_speed_with_small_ik_segments(axis, speed, steps):
    value, requests, _, _ = client()
    result = asyncio.run(value.tcp_jog({**payload(), 'axis': axis, 'velocity': speed}))
    assert result['simulation_only'] and not result['execution_ready']
    assert len(requests) == 1+steps  # FK jeden raz, potem IK krótkich odcinków.
    if axis == 'x':
        assert requests[-1].target.position.y == pytest.approx(.205)
        assert [r.target.position.y for r in requests[1:]] == pytest.approx([.201, .202, .203, .204, .205])
    assert requests[2].context.seed[0] == pytest.approx(math.radians(.1))


def test_tcp_http_route_without_hardware_api(tmp_path):
    value, _, _, _ = client()
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, planning_client=value)
    async def request(data):
        output = []
        async def receive():
            return dict(type='http.request', body=json.dumps(data).encode(), more_body=False)
        async def send(item):
            output.append(item)
        query = scope('http', '/api/v1/planning/tcp-jog')
        query['method'] = 'POST'
        await app(query, receive, send)
        return output[0]['status'], json.loads(output[1]['body'])
    status, result = asyncio.run(request(payload()))
    assert status == 200 and result['simulation_only'] and result['execution_ready'] is False
    assert asyncio.run(request({**payload(), 'direction': 0}))[0] == 422


@pytest.mark.parametrize('amount', [.1, 1, 5, 10, 20])
@pytest.mark.parametrize('axis', ['x', 'rz'])
def test_impulse_target_and_smooth_lin(amount, axis):
    value, requests, _, _ = client()
    goal = asyncio.run(value.tcp_step({**payload(), 'axis': axis, 'step': amount}))
    assert goal.motion_type == 1 and goal.context.simulation_only
    assert len(requests) == 2 and requests[0].forward_only
    if axis == 'x':
        assert requests[1].target.position.y == pytest.approx(.2 + amount/1000)
        assert goal.tcp_velocity == pytest.approx(.01)
    else:
        result = value.pose_json(requests[1].target)
        assert result[-1] == pytest.approx(90 + amount)
        assert goal.tcp_angular_velocity == pytest.approx(math.radians(10))
    assert goal.tcp_acceleration == pytest.approx(.1)


@pytest.mark.parametrize('changes', [dict(step=0), dict(step=2), dict(step=21), dict(step=True),
    dict(direction=0), dict(frame='bad'), dict(settings_revision='old')])
def test_impulse_validation(changes):
    value, _, _, _ = client()
    with pytest.raises(ValueError):
        asyncio.run(value.tcp_step({**payload(), 'step': 1, **changes}))
