#!/usr/bin/env python3
"""Jeden punkt wejścia trajektorii i jog; watchdog źródła niezależny od write()."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = tuple(f'joint_{index}' for index in range(1, 7))


def validate_trajectory(trajectory, *, jog=False, ros_now=0.0):
    if tuple(trajectory.joint_names) != JOINTS or not trajectory.points:
        return False
    if len(trajectory.points) > (20 if jog else 10000):
        return False
    previous = -1.0
    for point in trajectory.points:
        if point.time_from_start.nanosec >= 1000000000:
            return False
        if len(point.positions) != 6 or not all(math.isfinite(v) for v in point.positions):
            return False
        for values in (point.velocities, point.accelerations, point.effort):
            if values and (len(values) != 6 or not all(math.isfinite(v) for v in values)):
                return False
        seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        if seconds < 0 or seconds <= previous or (jog and seconds > 0.2):
            return False
        previous = seconds
    stamp = trajectory.header.stamp.sec + trajectory.header.stamp.nanosec * 1e-9
    if jog and (stamp <= 0 or ros_now - stamp > 0.2 or stamp - ros_now > 0.02):
        return False
    return True


class SourceArbiter:
    """Stały stan; zmiana źródła dozwolona tylko po zakończeniu bieżącej pracy."""

    def __init__(self):
        self.mode = 'none'
        self.busy = False
        self.last_jog = None

    def select(self, mode, now):
        if mode == 'none':
            self.mode, self.last_jog = mode, None
            return True
        if self.busy or (self.last_jog is not None and now - self.last_jog < 0.2):
            return False
        self.mode, self.last_jog = mode, None
        return True

    def claim_goal(self):
        if self.mode != 'trajectory' or self.busy:
            return False
        self.busy = True
        return True

    def accept_jog(self, now):
        if self.mode != 'jog' or self.busy:
            return False
        if self.last_jog is not None and now - self.last_jog >= 0.2:
            self.mode = 'none'
            return False
        self.last_jog = now
        return True

    def lease(self, now):
        if self.mode == 'jog' and self.last_jog is not None and now - self.last_jog >= 0.2:
            self.mode = 'none'
        return self.mode


class MotionGate(Node):
    def __init__(self):
        super().__init__('robot_motion_gate')
        self.arbiter = SourceArbiter()
        self.lock = threading.Lock()
        self.downstream_goal = None
        self.cancel_requested = False
        self.hardware_active = False
        self.last_controller_feedback = 0.0
        self.feedback_failed = False
        group = ReentrantCallbackGroup()
        controller = self.declare_parameter('controller', '/joint_trajectory_controller').value
        self.client = ActionClient(self, FollowJointTrajectory,
                                   controller + '/follow_joint_trajectory', callback_group=group)
        self.jog_publisher = self.create_publisher(JointTrajectory, controller + '/joint_trajectory', 1)
        self.lease_publisher = self.create_publisher(UInt64MultiArray, '/robot/motion/lease', 1)
        self.create_subscription(JointTrajectory, '/robot/motion/jog_trajectory', self.jog, 1)
        self.create_subscription(DiagnosticArray, '/robot/can/diagnostics', self.hardware_status, 1)
        for mode in ('trajectory', 'jog', 'none'):
            self.create_service(Trigger, '/robot/motion/select_' + mode,
                                lambda request, response, selected=mode: self.select(selected, response))
        self.create_timer(0.02, self.publish_lease)
        self.server = ActionServer(self, FollowJointTrajectory, '/robot/motion/follow_joint_trajectory',
                                   self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=group)

    def select(self, mode, response):
        with self.lock:
            response.success = ((not self.hardware_active or mode == 'none' or mode == self.arbiter.mode)
                                and self.arbiter.select(mode, time.monotonic()))
        response.message = ('Wybrano tryb: ' + mode if response.success else
                            'Zmiana źródła zablokowana: dezaktywuj hardware lub zakończ bieżący ruch')
        self.publish_lease()
        return response

    def hardware_status(self, message):
        active = any(item.name == 'can' and any(pair.key == 'active' and pair.value == 'true'
                     for pair in item.values) for item in message.status)
        with self.lock:
            self.hardware_active = active

    def publish_lease(self):
        with self.lock:
            if self.arbiter.busy and time.monotonic() - self.last_controller_feedback >= 0.5:
                self.feedback_failed = True
                self.arbiter.mode = 'none'
            mode = self.arbiter.lease(time.monotonic())
        self.lease_publisher.publish(UInt64MultiArray(
            data=[{'trajectory': 1, 'jog': 2}.get(mode, 0), time.monotonic_ns()]))

    def goal(self, request):
        if not validate_trajectory(request.trajectory) or not self.client.server_is_ready():
            return GoalResponse.REJECT
        with self.lock:
            accepted = self.arbiter.claim_goal()
            if accepted:
                self.cancel_requested = False
                self.feedback_failed = False
                self.last_controller_feedback = time.monotonic()
        return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT

    def cancel(self, handle):
        with self.lock:
            self.cancel_requested = True
            downstream = self.downstream_goal
        if downstream is not None:
            downstream.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        result = FollowJointTrajectory.Result()
        try:
            with self.lock:
                if self.arbiter.mode != 'trajectory':
                    raise RuntimeError('Źródło trajektorii zostało wycofane')
            future = self.client.send_goal_async(
                handle.request, feedback_callback=lambda feedback: self.feedback(handle, feedback))
            goal = await future
            with self.lock:
                self.downstream_goal = goal
                cancelled = self.cancel_requested
            if not goal.accepted:
                raise RuntimeError('Kontroler odrzucił trajektorię')
            if cancelled:
                goal.cancel_goal_async()
            outcome = await goal.get_result_async()
            result = outcome.result
            with self.lock:
                failed = self.feedback_failed
            if failed:
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = 'Utracono feedback kontrolera; wymagane ponowne przygotowanie ruchu'
                handle.abort()
            elif outcome.status == GoalStatus.STATUS_SUCCEEDED:
                handle.succeed()
            elif outcome.status == GoalStatus.STATUS_CANCELED:
                handle.canceled()
            else:
                with self.lock:
                    self.arbiter.mode = 'none'
                handle.abort()
        except Exception as error:
            with self.lock:
                self.arbiter.mode = 'none'
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            handle.abort()
        finally:
            with self.lock:
                self.downstream_goal = None
                self.arbiter.busy = False
            self.publish_lease()
        return result

    def feedback(self, handle, feedback):
        with self.lock:
            self.last_controller_feedback = time.monotonic()
        handle.publish_feedback(feedback.feedback)

    def jog(self, message):
        if not validate_trajectory(message, jog=True, ros_now=self.get_clock().now().nanoseconds * 1e-9):
            return
        with self.lock:
            accepted = self.arbiter.accept_jog(time.monotonic())
        if accepted:
            self.jog_publisher.publish(message)


def main():
    rclpy.init()
    node = MotionGate()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.lease_publisher.publish(UInt64MultiArray(data=[0, time.monotonic_ns()]))
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
