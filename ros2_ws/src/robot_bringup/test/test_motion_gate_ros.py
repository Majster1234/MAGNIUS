"""Integracja lokalnego ROS z atrapą JTC; odrębny domain, bez CAN i napędów."""
import importlib.util
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

spec = importlib.util.spec_from_file_location(
    'motion_gate_ros', Path(__file__).parents[1] / 'scripts' / 'motion_gate.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def wait(future, seconds=5):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), 'Przekroczony czas odpowiedzi lokalnego ROS'
    return future.result()


def test_proxy_action_and_jog_use_one_source(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '127')
    rclpy.init()
    gate = module.MotionGate()
    controller = Node('test_controller')
    operator = Node('test_operator')
    jog_received = []
    feedback_received = []

    def execute(handle):
        feedback = FollowJointTrajectory.Feedback(joint_names=list(module.JOINTS))
        handle.publish_feedback(feedback)
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0)

    server = ActionServer(controller, FollowJointTrajectory,
                          '/joint_trajectory_controller/follow_joint_trajectory', execute)
    controller.create_subscription(JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
                                   lambda message: jog_received.append(message), 1)
    client = ActionClient(operator, FollowJointTrajectory, '/robot/motion/follow_joint_trajectory')
    select_trajectory = operator.create_client(Trigger, '/robot/motion/select_trajectory')
    select_jog = operator.create_client(Trigger, '/robot/motion/select_jog')
    publisher = operator.create_publisher(JointTrajectory, '/robot/motion/jog_trajectory', 1)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (gate, controller, operator):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert gate.client.wait_for_server(timeout_sec=5)
        assert client.wait_for_server(timeout_sec=5)
        assert select_trajectory.wait_for_service(timeout_sec=5)
        assert wait(select_trajectory.call_async(Trigger.Request())).success
        trajectory = JointTrajectory(joint_names=list(module.JOINTS))
        point = JointTrajectoryPoint(positions=[0.] * 6)
        point.time_from_start.nanosec = 20000000
        trajectory.points = [point]
        goal = wait(client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory),
                                          feedback_callback=feedback_received.append))
        assert goal.accepted
        outcome = wait(goal.get_result_async())
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.error_code == 0
        assert feedback_received

        trajectory.header.stamp = operator.get_clock().now().to_msg()
        publisher.publish(trajectory)
        time.sleep(0.05)
        assert not jog_received, 'Jog nie może ominąć trybu trajektorii'
        assert wait(select_jog.call_async(Trigger.Request())).success
        trajectory.header.stamp.sec -= 1
        publisher.publish(trajectory)
        time.sleep(0.05)
        assert not jog_received, 'Stary jog nie może wejść do JTC'
        trajectory.header.stamp = operator.get_clock().now().to_msg()
        publisher.publish(trajectory)
        deadline = time.monotonic() + 2
        while not jog_received and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(jog_received) == 1
        time.sleep(0.25)
        with gate.lock:
            assert gate.arbiter.mode == 'none'
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        server.destroy()
        client.destroy()
        for node in (gate, controller, operator):
            node.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()
