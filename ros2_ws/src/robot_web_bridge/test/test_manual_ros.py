"""Typowane usługi ROS na atrapach, odrębny domain; bez SocketCAN."""
import asyncio
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_control_interfaces.srv import SetMotorEnable, ZeroMotor, JogMotor, JogJoint
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
                for name, kind in (('enable', SetMotorEnable), ('zero', ZeroMotor), ('jog', JogMotor), ('jog_joint', JogJoint))]
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
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        for service in services:
            server.destroy_service(service)
        caller.destroy_node(); server.destroy_node()
        rclpy.shutdown()
