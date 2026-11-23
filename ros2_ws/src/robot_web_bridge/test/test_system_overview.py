import asyncio
import json
import os
from types import SimpleNamespace

import psutil

from robot_web_bridge.app import create_app
from robot_web_bridge.status import StatusStore
from robot_web_bridge.system_overview import (
    ConnectionCounter, SystemOverview, executable_origin, node_name, overlay_name, parse_systemctl)


def test_node_name_prefers_ros_remap():
    assert node_name(['/opt/ros/humble/lib/controller_manager/ros2_control_node', '--ros-args',
                      '-r', '__node:=controller_manager']) == 'controller_manager'
    assert node_name(['/usr/bin/python3', '/install/lib/robot_system_monitor/monitor', '--ros-args']) == 'monitor'


def test_executable_origin_per_process():
    assert executable_origin(['/usr/bin/python3', '/home/u/P/install/stop-recovery-ros/robot_web_bridge/lib/robot_web_bridge/bridge',
                              '--ros-args']) == ('robot_web_bridge', 'install/stop-recovery-ros')
    assert executable_origin(['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher', '--ros-args']) == (
        'robot_state_publisher', '/opt/ros/humble')
    assert executable_origin(['bash']) == (None, None)


def test_overlay_name_from_ament_prefix():
    assert overlay_name('/home/u/ws/install/stop-recovery-ros/robot_web_bridge:/opt/ros/humble') == 'install/stop-recovery-ros'
    assert overlay_name('') is None


def test_parse_systemctl_blocks():
    text = ('Id=robot-web-status.service\nActiveState=active\nSubState=running\nMainPID=47062\nResult=success\n\n'
            'Id=robot-can@can0.service\nActiveState=active\nSubState=exited\nMainPID=0\nResult=success\n')
    assert parse_systemctl(text) == [
        {'unit': 'robot-web-status.service', 'active_state': 'active', 'sub_state': 'running', 'result': 'success', 'main_pid': 47062},
        {'unit': 'robot-can@can0.service', 'active_state': 'active', 'sub_state': 'exited', 'result': 'success', 'main_pid': None}]


def test_connection_counter_never_negative():
    counter = ConnectionCounter()
    counter.opened('status'); counter.opened('status'); counter.closed('status'); counter.closed('can')
    assert counter.snapshot() == {'status': 1, 'can': 0}


def own_process_only(attrs):
    proc = psutil.Process(os.getpid())
    proc.info = {'pid': proc.pid, 'cmdline': proc.cmdline()}
    return [proc]


def test_snapshot_lists_backend_and_caches():
    clock = [10.0]
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout='Id=a.service\nActiveState=active\nSubState=running\nMainPID=1\nResult=success\n')

    node = SimpleNamespace(get_node_names_and_namespaces=lambda: [('robot_web_bridge', '/'), ('planner', '/robot')])
    overview = SystemOverview(node, units=['a.service'], clock=lambda: clock[0], run=run,
                              processes=own_process_only, environ={'ROS_DISTRO': 'humble'})
    first = overview.snapshot()
    assert first['backend']['pid'] == os.getpid()
    assert first['backend']['cpu_percent'] is None  # pierwszy pomiar bez okna czasowego
    assert first['ros']['nodes'] == ['/robot/planner', '/robot_web_bridge']
    assert first['ros']['distro'] == 'humble' and first['ros']['processes'] == []
    assert first['services'][0]['unit'] == 'a.service'
    clock[0] += 0.5
    overview.snapshot()
    assert len(calls) == 1  # bufor 1 s
    clock[0] += 1.0
    second = overview.snapshot()
    assert len(calls) == 2 and second['backend']['cpu_percent'] is not None


def test_snapshot_survives_missing_systemctl_and_node():
    def run(command, **kwargs):
        raise OSError('brak systemctl')
    overview = SystemOverview(None, run=run, processes=lambda attrs: [], environ={})
    data = overview.snapshot()
    assert data['services'] is None and data['ros']['nodes'] is None and data['backend']['websocket_clients'] == {}


async def get(app, path):
    output = []
    async def send(event):
        output.append(event)
    async def receive():
        return {'type': 'http.request', 'body': b'', 'more_body': False}
    await app({'type': 'http', 'asgi': {'version': '3.0'}, 'http_version': '1.1', 'scheme': 'http', 'method': 'GET',
               'path': path, 'raw_path': path.encode(), 'query_string': b'', 'root_path': '',
               'server': ('192.168.1.69', 8080), 'client': ('192.168.1.20', 45000),
               'headers': [(b'host', b'192.168.1.69:8080')]}, receive, send)
    return output[0]['status'], json.loads(output[1]['body'])


def test_overview_endpoint(tmp_path):
    (tmp_path / 'index.html').write_text('<html></html>')
    store = StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24')
    overview = SystemOverview(None, run=lambda *a, **k: SimpleNamespace(returncode=1, stdout=''),
                              processes=lambda attrs: [], environ={})
    overview.connections.opened('status')
    code, body = asyncio.run(get(create_app(store, tmp_path, overview=overview), '/api/v1/system/overview'))
    assert code == 200 and body['schema_version'] == 1 and body['backend']['websocket_clients'] == {'status': 1}
    code, body = asyncio.run(get(create_app(store, tmp_path), '/api/v1/system/overview'))
    assert code == 503
