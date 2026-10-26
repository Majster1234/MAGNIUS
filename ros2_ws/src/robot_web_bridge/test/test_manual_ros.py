"""Typowane usługi ROS na atrapach, odrębny domain; bez SocketCAN."""
import asyncio
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_control_interfaces.srv import SetMotorEnable, SetAllMotorsEnable, ZeroMotor, JogMotor, JogJoint, JogTcp
from robot_web_bridge.manual import ManualRosClient


def test_manual_client_uses_typed_ros_and_fresh_monotonic_time(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '129')
    rclpy.init()
    caller = Node('manual_client_test')
    server = Node('manual_fake_server')
    received = []
    def reply(request, response):
        received.append(request)
        response.accepted = 0 <= time.monotonic_ns() - request.issued_monotonic_ns < 100_000_000
        response.message = 'Potwierdzenie atrapy'
        return response
    services = [server.create_service(kind, '/robot/manual/' + name, reply)
                for name, kind in (('enable', SetMotorEnable), ('enable_all', SetAllMotorsEnable), ('zero', ZeroMotor), ('jog', JogMotor), ('jog_joint', JogJoint), ('jog_tcp', JogTcp))]
    client = ManualRosClient(caller)
    executor = SingleThreadedExecutor()
    executor.add_node(caller); executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        for service in client.clients.values():
            assert service.wait_for_service(timeout_sec=5)
        async def scenario():
            assert (await client('enable', axis=4, enabled=True))[0]
            assert (await client('zero', axis=4))[0]
            assert (await client('jog', action=JogMotor.Request.START, axis=4, direction=1,
                                  rpm=30, session='test-panel', sequence=1))[0]
            assert (await client('jog', action=JogMotor.Request.STOP, axis=4, direction=0,
                                  rpm=0, session='test-panel', sequence=2))[0]
            assert (await client('jog_joint', action=JogJoint.Request.START, axis=2, direction=-1,
                                  velocity_rad_s=.2, revision='revision-1', session='test-panel', sequence=3))[0]
            assert (await client('jog_joint', action=JogJoint.Request.STOP, axis=2, direction=0,
                                  velocity_rad_s=0., revision='revision-1', session='test-panel', sequence=4))[0]
            assert client.pending == 0
        asyncio.run(scenario())
        assert received[0].axis == 4 and received[0].enabled
        assert received[-1].session == 'test-panel' and received[-1].sequence == 4
        assert received[-2].velocity_rad_s == .2 and received[-2].revision == 'revision-1'
        assert asyncio.run(client('jog_tcp', action=JogTcp.Request.STOP, emergency=True, session='tcp', sequence=5))[0]
        assert received[-1].emergency and received[-1].session == 'tcp'
        # Liczby JSON z przeglądarki: step=0 i velocity=1 nie są Python float.
        # Sprawdzamy cały adapter WS -> rzeczywisty typ ROS, nadal bez transportu CAN.
        import json
        from pathlib import Path
        from robot_web_bridge.app import create_app
        from robot_web_bridge.manual import ManualStore
        from robot_web_bridge.status import StatusStore
        from test_bridge import scope
        app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'),
                         Path('/tmp'), manual_store=ManualStore(), manual_action=client)
        first_tcp = len(received)
        async def websocket_scenario():
            incoming = asyncio.Queue()
            await incoming.put({'type': 'websocket.connect'})
            await incoming.put({'type': 'websocket.receive', 'text': json.dumps(dict(space='tcp', action='start', sequence=1,
                axis='x', direction=1, frame='base', impulse=False, velocity=1, step=0, model_id='m', revision='r'))})
            async def send(event):
                if event['type'] != 'websocket.send':
                    return
                data = json.loads(event['text'])
                if data['type'] != 'tcp_ack':
                    return
                assert data['accepted']
                if data['sequence'] < 3:
                    await incoming.put({'type': 'websocket.receive', 'text': json.dumps(dict(space='tcp',
                        action='keepalive' if data['sequence'] == 1 else 'stop', sequence=data['sequence'] + 1))})
                else:
                    await incoming.put({'type': 'websocket.disconnect', 'code': 1000})
            await asyncio.wait_for(app(scope('websocket', '/api/v1/manual/ws'), incoming.get, send), 2.)
        asyncio.run(websocket_scenario())
        assert [r.action for r in received[first_tcp:first_tcp + 3]] == [0, 1, 2]
        assert received[first_tcp].velocity == 1.0 and received[first_tcp].step == 0.0
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        for service in services:
            server.destroy_service(service)
        caller.destroy_node(); server.destroy_node()
        rclpy.shutdown()
