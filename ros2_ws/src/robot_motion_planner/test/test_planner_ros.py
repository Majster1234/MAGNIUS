"""Rzeczywisty solver/action ROS z syntetycznymi pomiarami; odrębna domena, bez CAN."""
import os
from hashlib import sha256
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from robot_control_interfaces.msg import ManualState, ManualAxisState, PlanningContext, RobotConfiguration
from robot_control_interfaces.srv import SolveIK
from robot_control_interfaces.action import ComputeCartesianPath, ComputePreviewMotion, ComputePhysicalPtp


def wait(future, seconds=15):
    end = time.monotonic() + seconds
    while not future.done() and time.monotonic() < end:
        time.sleep(.01)
    assert future.done(), 'Brak odpowiedzi planera ROS'
    return future.result()


def test_ik_path_and_stale_revision(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '132')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    process = subprocess.Popen([str(Path(get_package_prefix('robot_motion_planner')) / 'lib/robot_motion_planner/planner')],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, env=os.environ.copy())
    rclpy.init(); node = Node('test_planner_without_can')
    description = node.create_publisher(String, '/robot/preview/robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = node.create_publisher(ManualState, '/robot/manual/state', 1)
    configuration = node.create_publisher(RobotConfiguration, '/robot/configuration/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    xml = (Path(__file__).parents[2] / 'robot_description/urdf/preview_6dof.example.urdf').read_text()
    context = PlanningContext(model_id=sha256(xml.encode()).hexdigest(), configuration_revision='test-plan', urdf=xml,
                              seed=[.2,.3,-.5,.2,.2,-.1], known=[True]*6, locked=[False]*6, lower=[-3.]*6, upper=[3.]*6,
                              max_velocity=[1.]*6, max_acceleration=[2.]*6)
    revision = ['test-plan']; missing = [False]*6
    def publish():
        publisher.publish(ManualState(monotonic_ns=time.monotonic_ns(), configuration_revision=revision[0],
                          axes=[ManualAxisState(axis=i+1, available=not missing[i], position_known=not missing[i], position_age_seconds=.001) for i in range(6)]))
    node.create_timer(.02, publish)
    executor = SingleThreadedExecutor(); executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    client = node.create_client(SolveIK, '/robot/planning/solve_ik')
    action = ActionClient(node, ComputeCartesianPath, '/robot/planning/compute_cartesian_path')
    try:
        assert client.wait_for_service(timeout_sec=10)
        assert action.wait_for_server(timeout_sec=5)
        description.publish(String(data=xml)); time.sleep(.15)
        answer = wait(client.call_async(SolveIK.Request(context=context, forward_only=True, position_tolerance=1e-4, orientation_tolerance=1e-3)))
        assert answer.success, answer.message
        same = wait(client.call_async(SolveIK.Request(context=context, target=answer.tcp, position_tolerance=1e-4, orientation_tolerance=1e-3)))
        assert same.success, same.message
        context.locked = [False,True,True,True,True,True]
        goal = ComputeCartesianPath.Goal(context=context, targets=[answer.tcp], blend_position_tolerance=.001,
                                         blend_orientation_tolerance=.01, velocity_scaling=.5, acceleration_scaling=.5)
        feedback = []
        handle = wait(action.send_goal_async(goal, feedback_callback=lambda value: feedback.append(value)))
        assert handle.accepted
        result = wait(handle.get_result_async()).result
        assert result.success, result.message
        assert not result.plan.execution_ready and not result.plan.mks_compatible
        assert feedback
        context.locked = [False]*6
        original = list(context.seed)
        context.seed = [original[0]+.1, *original[1:]]
        far = wait(client.call_async(SolveIK.Request(context=context, forward_only=True)))
        assert far.success
        context.seed = original
        goal.targets = [far.tcp, answer.tcp]*16
        time.sleep(.03)
        handle = wait(action.send_goal_async(goal))
        assert handle.accepted
        cancelled = wait(handle.cancel_goal_async())
        assert cancelled.goals_canceling
        assert wait(handle.get_result_async()).status == 5
        # Rzeczywista akcja podglądu: ograniczenie obu osi w zapisanej trajektorii,
        # przy zachowaniu wszystkich sześciu pozycji celu i bez wykonania sprzętu.
        preview = ActionClient(node, ComputePreviewMotion, '/robot/planning/compute_preview_motion')
        assert preview.wait_for_server(timeout_sec=5)
        configuration.publish(RobotConfiguration(revision=context.configuration_revision))
        context.simulation_only = True
        context.seed = [original[0], original[1], original[2], original[3], 0., original[5]]
        target = list(context.seed); target[3] += .3; target[5] -= .2
        time.sleep(.1)
        handle = wait(preview.send_goal_async(ComputePreviewMotion.Goal(context=context, motion_type=0, target_positions=target)))
        assert handle.accepted
        result = wait(handle.get_result_async()).result
        assert result.success, result.message
        assert not result.plan.execution_ready and result.plan.simulation_only
        for j, q in enumerate(target):
            assert abs(result.plan.trajectory.points[-1].positions[j] - q) < 1e-9
        for point in result.plan.trajectory.points:
            assert abs(point.velocities[3]) <= 5 * 3.141592653589793 / 180 + 1e-6
            assert abs(point.velocities[5]) <= 5 * 3.141592653589793 / 180 + 1e-6
        context.simulation_only = False
        physical = ActionClient(node, ComputePhysicalPtp, '/robot/planning/compute_physical_ptp')
        assert physical.wait_for_server(timeout_sec=5)
        for profile in context.profiles:
            profile.configured = True
        target = list(context.seed); target[0] += .05
        handle = wait(physical.send_goal_async(ComputePhysicalPtp.Goal(context=context, target_positions=target)))
        assert handle.accepted
        result = wait(handle.get_result_async()).result
        assert result.success and not result.plan.simulation_only, result.message
        missing[5] = True; context.allow_missing_execution = True
        context.simulated_start = [False]*5+[True]
        time.sleep(.1)
        handle = wait(physical.send_goal_async(ComputePhysicalPtp.Goal(context=context, target_positions=target)))
        assert handle.accepted
        result = wait(handle.get_result_async()).result
        assert result.success and result.plan.simulated_start[5], result.message
        missing[5] = False
        revision[0] = 'changed'; time.sleep(.1)
        stale = wait(client.call_async(SolveIK.Request(context=context, forward_only=True)))
        assert not stale.success
        assert not wait(action.send_goal_async(goal)).accepted
    finally:
        executor.shutdown(timeout_sec=2); thread.join(timeout=2); node.destroy_node(); rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
