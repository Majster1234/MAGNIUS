"""Rzeczywisty robot_state_publisher, odrębny domain, bez CAN i napędów."""
import math
import os
from pathlib import Path
import subprocess
import time
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from robot_web_bridge.description import DescriptionStore


def test_description_and_partial_joint_state_share_preview_namespace(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '128')
    monkeypatch.setenv('ROS_LOG_DIR', '/tmp/robot-preview-test-logs')
    xml = (Path(get_package_share_directory('robot_description')) / 'urdf/preview_6dof.example.urdf').read_text()
    executable = Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'
    parameters = tmp_path / 'preview.yaml'
    parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': xml, 'frame_prefix': 'preview/'}}}))
    process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/robot/preview',
        '--params-file', str(parameters),
        '-r', '/tf:=/robot/preview/tf', '-r', '/tf_static:=/robot/preview/tf_static'],
        env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = Node('test_robot_preview')
    store, transforms, global_tf = DescriptionStore(), {}, []
    node.create_subscription(String, '/robot/preview/robot_description', store.update,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def receive(message):
        transforms.update({t.child_frame_id: t for t in message.transforms})
    node.create_subscription(TFMessage, '/robot/preview/tf', receive, 10)
    node.create_subscription(TFMessage, '/tf', lambda message: global_tf.append(message), 1)
    publisher = node.create_publisher(JointState, '/robot/preview/joint_states', 1)
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and (store.snapshot() is None or not transforms):
            assert process.poll() is None, 'robot_state_publisher zakończył pracę'
            sample = JointState(name=['joint_1'], position=[-math.pi / 2])
            sample.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(sample)
            rclpy.spin_once(node, timeout_sec=.05)
        assert store.snapshot()[0] == xml
        assert set(transforms) == {'preview/link_1'}, 'Brakujące osie nie mogą otrzymać wymyślonej pozycji'
        transform = transforms['preview/link_1']
        assert transform.header.frame_id == 'preview/base_link'
        assert math.isclose(transform.transform.rotation.z, -math.sqrt(.5), abs_tol=1e-9)
        assert math.isclose(transform.transform.rotation.w, math.sqrt(.5), abs_tol=1e-9)
        assert not global_tf, 'Model przykładowy nie może publikować do globalnego /tf'
    finally:
        node.destroy_node(); rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=3)
