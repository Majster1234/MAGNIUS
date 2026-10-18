"""Jeden proces HTTP i jeden executor ROS, poprawne zamknięcie przy Ctrl+C."""

import os
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
import uvicorn

from .app import create_app
from .network import lan_address
from .status import StatusStore


def main(args=None):
    # Uvicorn zarządza SIGINT/SIGTERM; kontekst ROS zamykamy w finally.
    from rclpy.signals import SignalHandlerOptions

    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('robot_web_bridge')
    executor = SingleThreadedExecutor()
    thread = None
    try:
        for name, default in (('lan_interface', 'enp2s0'), ('lan_subnet', '192.168.1.0/24'),
                              ('port', 8080), ('static_dir', '')):
            node.declare_parameter(name, default)
        interface = node.get_parameter('lan_interface').value
        subnet = node.get_parameter('lan_subnet').value
        port = node.get_parameter('port').value
        if not 1024 <= port <= 65535:
            raise RuntimeError('Port musi należeć do zakresu 1024–65535')
        address = lan_address(interface, subnet)
        static_dir = node.get_parameter('static_dir').value or str(
            Path(get_package_share_directory('robot_web_bridge')) / 'web')
        if not (Path(static_dir) / 'index.html').is_file():
            raise RuntimeError('Brak zbudowanego panelu. Wykonaj npm ci i npm run build w web/, '
                               'a następnie colcon build --symlink-install.')
        # ament_python --symlink-install tworzy linki do plików z web/dist.
        static_dir = (Path(static_dir) / 'index.html').resolve().parent
        store = StatusStore(address, port, interface, subnet)
        node.create_subscription(DiagnosticArray, '/robot/system_status', store.update,
                                 QoSProfile(depth=1))
        executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        thread = threading.Thread(target=spin, name='ros-executor', daemon=True)
        thread.start()
        node.get_logger().info(f"Panel diagnostyczny: {store.network['url']} ({interface})")
        uvicorn.run(create_app(store, static_dir), host=address, port=port,
                    proxy_headers=False, access_log=False, ws='wsproto', lifespan='off')
    finally:
        executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
