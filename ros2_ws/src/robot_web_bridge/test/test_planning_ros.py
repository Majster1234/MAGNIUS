"""HTTP → rzeczywisty planner C++ → magazyn planów; syntetyczny ROS, bez CAN."""
import asyncio
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace
import pytest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from robot_control_interfaces.msg import ManualState, ManualAxisState, RobotConfiguration
from robot_web_bridge.app import create_app
from robot_web_bridge.planning import PlanningClient
from robot_web_bridge.status import StatusStore
from test_bridge import scope


def test_http_cpp_plan_without_execution(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '133')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    process = subprocess.Popen([str(Path(get_package_prefix('robot_motion_planner')) / 'lib/robot_motion_planner/planner')],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=os.environ.copy())
    rclpy.init(); node = Node('test_http_cpp_planning')
    executor = SingleThreadedExecutor(); executor.add_node(node)
    description = node.create_publisher(String, '/robot/preview/robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = node.create_publisher(ManualState, '/robot/manual/state', 1)
    active_publisher = node.create_publisher(RobotConfiguration, '/robot/configuration/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    xml = (Path(__file__).parents[2] / 'robot_description/urdf/preview_6dof.example.urdf').read_text()
    xml = xml.replace('<robot ', '<robot configuration_revision="http-test" ', 1)
    identifier = sha256(xml.encode()).hexdigest()
    axes = [dict(axis=i+1, available=True, position_known=True, joint_configured=False) for i in range(6)]
    descriptions = SimpleNamespace(snapshot=lambda: (xml, identifier))
    fresh = [True]
    manual = SimpleNamespace(snapshot=lambda: dict(fresh=fresh[0], configuration_revision='http-test', axes=axes))
    async def configuration(operation):
        if operation == 'system_get':
            return 200, dict(revision='settings', ignore_missing_drivers=True)
        return 200, dict(operation=dict(phase='idle'), active=dict(revision='http-test', document=dict(values={})))
    client = PlanningClient(node, descriptions, manual, configuration)
    def publish():
        publisher.publish(ManualState(monotonic_ns=time.monotonic_ns(), configuration_revision='http-test',
                          axes=[ManualAxisState(axis=i+1, available=True, position_known=True, position_age_seconds=.001) for i in range(6)]))
    manual_timer = node.create_timer(.02, publish)
    node.create_timer(.1, lambda: active_publisher.publish(RobotConfiguration(revision='http-test')))
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, planning_client=client)
    async def request(path, data=None, method=None):
        output = []
        async def receive():
            return dict(type='http.request', body=json.dumps(data).encode() if data is not None else b'', more_body=False)
        async def send(message):
            output.append(message)
        query = scope('http', '/api/v1/planning/' + path)
        if data is not None:
            query['method'] = method or 'POST'
        await app(query, receive, send)
        return output[0]['status'], json.loads(b''.join(item.get('body', b'') for item in output[1:]))
    async def scenario():
        data = dict(model_id=identifier, configuration_revision='http-test', seed_degrees=[math.degrees(q) for q in [.2,.3,-.5,.2,.2,-.1]],
                    preview_velocity_degrees_s=10, preview_acceleration_degrees_s2=50, forward_only=True)
        code, status = await request('context')
        assert code == 200 and status['planner_available'] and status['tcp_jog_segmented']
        shifted = {**data, 'seed_degrees': [data['seed_degrees'][0]+.5, *data['seed_degrees'][1:]]}
        code, fk = await request('ik', shifted)
        assert code == 200, fk
        # Rzeczywisty FK/IK C++ dla przyrostów TCP; żadnego wykonawcy CAN.
        tcp_data = {**data, 'settings_revision': 'settings', 'simulated_start': [True]*6,
                    'duration_s': .05, 'velocity': 1}
        for frame in ('base', 'tool'):
            for axis in ('x', 'y', 'z', 'rx', 'ry', 'rz'):
                for direction in (-1, 1):
                    code, step = await request('tcp-jog', {**tcp_data, 'frame': frame, 'axis': axis, 'direction': direction})
                    assert code == 200, (frame, axis, direction, step)
                    assert step['simulation_only'] and not step['execution_ready']
                    assert len(step['positions_degrees']) == 6
                    assert any(abs(a-b) > 1e-6 for a, b in zip(step['positions_degrees'], data['seed_degrees']))
        code, origin = await request('ik', data)
        assert code == 200, origin
        code, fast = await request('tcp-jog', {**tcp_data, 'frame': 'base', 'axis': 'x', 'direction': 1, 'velocity': 100})
        assert code == 200, fast
        assert fast['tcp'][0] - origin['tcp'][0] == pytest.approx(5, abs=.015)
        assert fast['tcp'][1:3] == pytest.approx(origin['tcp'][1:3], abs=.015)
        code, job = await request('jobs', {**data, 'targets': [fk['tcp']], 'blend_position_mm': 2, 'blend_orientation_degrees': .5})
        assert code == 202 and job['status'] == 'running', job
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            code, job = await request('jobs/' + job['id'])
            if job['status'] != 'running':
                break
            await asyncio.sleep(.02)
        assert job['status'] == 'succeeded', job
        code, plan = await request('plans/' + job['plan_id'])
        assert code == 200 and not plan['execution_ready'] and not plan['mks_compatible']
        points = plan['trajectory']['points']
        assert len(points) > 2 and abs(points[-1]['positions'][0]-points[0]['positions'][0]) > .001
        assert plan['configuration_revision'] == 'http-test'
        assert (await request('execute', {}))[0] == 404
        # Impulsy: rzeczywisty LIN C++, dokładny dystans także dla 0,1 mm/°.
        impulse = {**tcp_data, 'seed_degrees': [15., 20., -35., 20., 45., -10.], 'velocity': 10, 'frame': 'base', 'direction': 1}
        code, impulse_origin = await request('ik', {**data, 'seed_degrees': impulse['seed_degrees']})
        assert code == 200, impulse_origin
        for axis in ('x', 'rz'):
            for amount in (.1, 1, 5, 10, 20):
                code, step_job = await request('tcp-step', {**impulse, 'axis': axis, 'step': amount})
                assert code == 202, step_job
                for _ in range(500):
                    _, step_job = await request('jobs/'+step_job['id'])
                    if step_job['status'] != 'running':
                        break
                    await asyncio.sleep(.02)
                assert step_job['status'] == 'succeeded', (axis, amount, step_job)
                code, step_plan = await request('plans/'+step_job['plan_id'])
                assert code == 200 and step_plan['simulation_only'] and not step_plan['execution_ready']
                points = step_plan['trajectory']['points']
                assert max(abs(v) for v in points[0]['velocities']+points[-1]['velocities']) < 1e-8
                code, endpoint = await request('ik', {**data, 'seed_degrees': [math.degrees(q) for q in points[-1]['positions']]})
                assert code == 200, endpoint
                if axis == 'x':
                    assert endpoint['tcp'][0]-impulse_origin['tcp'][0] == pytest.approx(amount, abs=.002)
                    assert endpoint['tcp'][1:3] == pytest.approx(impulse_origin['tcp'][1:3], abs=.002)
                else:
                    assert endpoint['tcp'][-1]-impulse_origin['tcp'][-1] == pytest.approx(amount, abs=.002)
        manual_timer.cancel(); fresh[0] = False
        for axis in axes:
            axis['available'] = axis['position_known'] = False
        await asyncio.sleep(.3)
        target = [data['seed_degrees'][0]+.3, *data['seed_degrees'][1:]]
        move = dict(**data, settings_revision='settings', simulated_start=[True]*6, target_degrees=target)
        for motion in ('PTP','LIN'):
            code, job = await request('moves', {**move, 'motion_type': motion})
            assert code == 202 and job['status'] == 'running', job
            for _ in range(500):
                _, job = await request('jobs/'+job['id'])
                if job['status'] != 'running':
                    break
                await asyncio.sleep(.02)
            assert job['status'] == 'succeeded', job
            code, plan = await request('plans/'+job['plan_id'])
            assert code == 200 and plan['motion_type'] == motion and plan['simulation_only']
            assert not plan['execution_ready'] and plan['locked'] == [False]*6
            assert plan['simulated_start'] == [True]*6
            assert plan['target_positions'] == pytest.approx([math.radians(q) for q in target], abs=1e-8)
        feed = {k: v for k, v in move.items() if not k.startswith('preview_')}
        code, job = await request('moves', {**feed, 'motion_type': 'LIN', 'tcp_velocity_mm_s': 1,
            'tcp_acceleration_mm_s2': 100, 'tcp_angular_velocity_degrees_s': 2})
        assert code == 202 and job['status'] == 'running', job
        for _ in range(500):
            _, job = await request('jobs/'+job['id'])
            if job['status'] != 'running':
                break
            await asyncio.sleep(.02)
        assert job['status'] == 'succeeded', job
        code, plan = await request('plans/'+job['plan_id'])
        assert code == 200 and plan['motion_type'] == 'LIN' and not plan['execution_ready']
        duration = plan['trajectory']['points'][-1]['time_from_start']
        assert duration['sec'] + duration['nanosec'] * 1e-9 > 1
        long_target = [move['seed_degrees'][0]+math.degrees(1), *move['seed_degrees'][1:]]
        code, job = await request('moves', {**move, 'motion_type':'PTP', 'target_degrees':long_target, 'preview_velocity_degrees_s':.5})
        assert code == 202 and job['status'] == 'running', job
        code, cancelled = await request('jobs/'+job['id'], {}, 'DELETE')
        assert code == 200 and cancelled['status'] == 'cancelled', cancelled
    try:
        assert client.ik.wait_for_service(timeout_sec=10) and client.action.wait_for_server(timeout_sec=5)
        description.publish(String(data=xml)); time.sleep(.15)
        asyncio.run(scenario())
    finally:
        executor.shutdown(timeout_sec=2); thread.join(timeout=2); node.destroy_node(); rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
