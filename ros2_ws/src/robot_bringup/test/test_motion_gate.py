import importlib.util
from pathlib import Path

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

spec = importlib.util.spec_from_file_location(
    'motion_gate', Path(__file__).parents[1] / 'scripts' / 'motion_gate.py')
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


def test_only_one_motion_source_and_goal():
    arbiter = gate.SourceArbiter()
    assert not arbiter.claim_goal()
    assert arbiter.select('trajectory', 0)
    assert arbiter.claim_goal()
    assert not arbiter.claim_goal()
    assert not arbiter.select('jog', 0)
    assert not arbiter.accept_jog(0)


def test_jog_lease_expires_without_being_refreshed_by_timer():
    arbiter = gate.SourceArbiter()
    assert arbiter.select('jog', 0)
    assert arbiter.accept_jog(1)
    assert not arbiter.select('trajectory', 1.1)
    assert arbiter.lease(1.19) == 'jog'
    assert arbiter.lease(1.21) == 'none'


def test_late_jog_cannot_revive_an_expired_source():
    arbiter = gate.SourceArbiter()
    assert arbiter.select('jog', 0)
    assert arbiter.accept_jog(1)
    assert not arbiter.accept_jog(1.21)
    assert arbiter.mode == 'none'


def test_source_can_be_withdrawn_during_active_goal():
    arbiter = gate.SourceArbiter()
    assert arbiter.select('trajectory', 0)
    assert arbiter.claim_goal()
    assert arbiter.select('none', 1)
    assert arbiter.lease(1) == 'none'


def trajectory():
    message = JointTrajectory(joint_names=list(gate.JOINTS))
    point = JointTrajectoryPoint(positions=[0.] * 6)
    point.time_from_start.nanosec = 20000000
    message.points = [point]
    message.header.stamp.sec = 10
    return message


def test_validates_complete_finite_commands():
    message = trajectory()
    assert gate.validate_trajectory(message)
    message.points[0].positions[0] = float('nan')
    assert not gate.validate_trajectory(message)
    message = trajectory()
    message.joint_names.reverse()
    assert not gate.validate_trajectory(message)


def test_rejects_old_or_future_jog_and_long_horizon():
    message = trajectory()
    assert gate.validate_trajectory(message, jog=True, ros_now=10.1)
    assert not gate.validate_trajectory(message, jog=True, ros_now=10.3)
    assert not gate.validate_trajectory(message, jog=True, ros_now=9.9)
    message.points[0].time_from_start.sec = 1
    assert not gate.validate_trajectory(message, jog=True, ros_now=10)
