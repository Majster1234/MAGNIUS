"""Jeden proces HTTP i jeden executor ROS, poprawne zamknięcie przy Ctrl+C."""

import os
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
import uvicorn

from .app import create_app
from .network import lan_address
from .status import StatusStore
from .status import CanStatusStore
from .manual import ManualStore, ManualRosClient
from .description import DescriptionStore
from .configuration import ConfigurationClient
from .planning import PlanningClient
from .motion import MotionClient
from .trace import trace, recording
from robot_control_interfaces.msg import ManualState
from std_srvs.srv import Trigger


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
        can_store = CanStatusStore(node.declare_parameter('can_interface', 'can0').value)
        manual_store = ManualStore()
        description_store = DescriptionStore()
        node.create_subscription(String, '/robot/preview/robot_description', description_store.update,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        manual_action = ManualRosClient(node)
        configuration_action = ConfigurationClient(node)
        planning_client = PlanningClient(node, description_store, manual_store, configuration_action)
        motion_client = MotionClient(node, planning_client, configuration_action)
        def manual_update(message):
            manual_store.update(message)
            planning_client.remember_positions()
        node.create_subscription(ManualState, '/robot/manual/state', manual_update, QoSProfile(depth=1))
        def system_update(message):
            store.update(message)
            if recording():
                trace('system_status', components=store.snapshot()['components'])

        def can_update(message):
            can_store.update(message)
            if recording():
                trace('can_summary', summary=can_store.snapshot()['summary'])

        node.create_subscription(DiagnosticArray, '/robot/system_status', system_update,
                                 QoSProfile(depth=1))
        node.create_subscription(DiagnosticArray, '/robot/can/diagnostics', can_update,
                                 QoSProfile(depth=1))
        scan_client = node.create_client(Trigger, '/robot/can/scan')
        reset_client = node.create_client(Trigger, '/robot/can/reset')

        def can_action(action):
            client = scan_client if action == 'scan' else reset_client
            if not client.service_is_ready():
                return False, 'Usługa diagnostyki CAN nie jest dostępna'
            future = client.call_async(Trigger.Request())
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(0.35):
                client.remove_pending_request(future)
                future.cancel()
                return False, 'Brak potwierdzenia przyjęcia operacji — sprawdź status CAN'
            try:
                result = future.result()
                return bool(result.success), result.message
            except Exception:
                return False, 'Nie udało się przekazać operacji do diagnostyki CAN'
        executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        thread = threading.Thread(target=spin, name='ros-executor', daemon=True)
        thread.start()
        node.get_logger().info(f"Panel diagnostyczny: {store.network['url']} ({interface})")
        uvicorn.run(create_app(store, static_dir, can_store, can_action, manual_store, manual_action, description_store, configuration_action, planning_client, motion_client), host=address, port=port,
                    proxy_headers=False, access_log=False, ws='wsproto', lifespan='off')
    finally:
        executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
