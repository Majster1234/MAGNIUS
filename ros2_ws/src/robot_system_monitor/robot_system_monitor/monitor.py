"""Węzeł zbierający diagnostykę raz na sekundę, niezależnie od odbiorców."""

from datetime import datetime, timezone
import json
import signal
import time

from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions

from .probes import can_status, host_status


def diagnostic(name, level, message, values):
    status = DiagnosticStatus(name=name, level=bytes([level]), message=message,
                              hardware_id='robot-host')
    payload = dict(values, sampled_at=datetime.now(timezone.utc).isoformat())
    status.values = [KeyValue(key=key, value=json.dumps(value, ensure_ascii=False))
                     for key, value in payload.items()]
    return status


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('robot_system_monitor')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('controller_service', '/controller_manager/list_controllers')
        self.publisher = self.create_publisher(
            DiagnosticArray, '/robot/system_status', QoSProfile(depth=1))
        self.controllers = self.create_client(
            ListControllers, self.get_parameter('controller_service').value)
        self.pending = None
        self.pending_since = 0.0
        self.controller_result = None
        self.controller_received = 0.0
        self.timer = self.create_timer(1.0, self.publish_status)

    def controller_status(self):
        now = time.monotonic()
        if self.pending is not None:
            if self.pending.done():
                try:
                    result = self.pending.result()
                    self.controller_result = [
                        {'name': item.name, 'state': item.state, 'type': item.type}
                        for item in result.controller]
                    self.controller_received = now
                except Exception:
                    self.controller_result = None
                self.pending = None
            elif now - self.pending_since >= 2.0:
                self.controllers.remove_pending_request(self.pending)
                self.pending.cancel()
                self.pending = None
                self.controller_result = None
        if not self.controllers.service_is_ready():
            self.controller_result = None
            return 1, 'Kontrolery nieuruchomione', {'configured': False, 'controllers': []}
        if self.pending is None:
            self.pending = self.controllers.call_async(ListControllers.Request())
            self.pending_since = now
        if self.controller_result is None or now - self.controller_received > 3.0:
            return 1, 'Oczekiwanie na odpowiedź kontrolerów', {'controllers': []}
        if not self.controller_result:
            return 1, 'Brak załadowanych kontrolerów', {'controllers': []}
        active = all(item['state'] == 'active' for item in self.controller_result)
        return (0 if active else 1), ('Kontrolery aktywne' if active else
                                     'Część kontrolerów jest nieaktywna'), {
                                         'controllers': self.controller_result}

    def publish_status(self):
        statuses = []
        probes = {
            'host': lambda: (0, 'Odczyt parametrów komputera', host_status()),
            'can': lambda: can_status(self.get_parameter('can_interface').value),
            'controllers': self.controller_status,
            'drives': lambda: (1, 'Brak integracji napędów', {'configured': False}),
        }
        for name, probe in probes.items():
            try:
                level, message, values = probe()
            except Exception as error:
                level, message, values = 2, f'Błąd odczytu: {error}', {}
            statuses.append(diagnostic(name, level, message, values))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = statuses
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    # Ctrl+C może trafić bezpośrednio z terminala oraz ponownie od launch.
    # Flaga pozwala dokończyć sprzątanie bez kolejnego KeyboardInterrupt.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = SystemMonitor()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
