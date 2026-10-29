"""Właściciel konfiguracji i plików. Brak socketów CAN i automatycznego ENABLE."""
import json
import fcntl
import os
from pathlib import Path
import threading
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import String
from robot_control_interfaces.msg import JointConfiguration, RobotConfiguration, ManualState, MotionSettings
from robot_control_interfaces.srv import ConfigurationCommand, ConfigureRobot
from .schema import profiles, validate, CRITICAL_FAULT_POLICY, DISPATCH_MODE
from .storage import Storage, decode
from .coordinator import Coordinator, Conflict


class RosTransport:
    def __init__(self, node, group, storage):
        self.node, self.storage = node, storage
        self.client = node.create_client(ConfigureRobot, '/robot/configuration/control', callback_group=group)
        self.parameters = node.create_client(SetParametersAtomically, '/robot/preview/state_publisher/set_parameters_atomically', callback_group=group)
        self.condition = threading.Condition()
        self.description = ''
        node.create_subscription(String, '/robot/preview/robot_description', self.update,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=group)

    def update(self, message):
        with self.condition:
            self.description = message.data
            self.condition.notify_all()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError(f'Usługa ROS niedostępna: {client.srv_name}')
        if hasattr(request, 'issued_monotonic_ns'):
            request.issued_monotonic_ns = time.monotonic_ns()
        future = client.call_async(request)
        done = threading.Event(); future.add_done_callback(lambda _: done.set())
        if not done.wait(3):
            client.remove_pending_request(future)
            raise TimeoutError(f'Brak potwierdzenia usługi {client.srv_name}; wynik wymaga uzgodnienia')
        return future.result()

    def control(self, phase, operation_id, state, changed, expected='', confirm=None):
        request = ConfigureRobot.Request()
        request.phase = {'prepare': 0, 'commit': 1, 'rollback': 2, 'finish': 3, 'status': 4, 'bootstrap': 5}[phase]
        request.operation_id = operation_id; request.expected_revision = expected
        configuration = RobotConfiguration(revision=state['revision'])
        configuration.ptp_lin_dispatch_mode = state['document']['values'].get(DISPATCH_MODE, 'streamed')
        configuration.disable_enable_on_critical_fault = state['document']['values'].get(CRITICAL_FAULT_POLICY, True)
        configuration.axes = [JointConfiguration(**profile) for profile in profiles(state['document'])]
        configuration.changed = changed or [False] * 6
        if phase == 'bootstrap':
            configuration.calibration_invalid = self.storage.calibration(state['revision'])
        request.configuration = configuration
        request.confirm_calibration = confirm or [False] * 6
        result = self.call(self.client, request)
        if not result.accepted:
            raise RuntimeError(result.message)
        response = {key: getattr(result, key) for key in ('revision', 'operation_id', 'stage', 'locked')}
        resolved = result.resolved_configuration
        response['resolved_configuration'] = dict(revision=resolved.revision, calibration_invalid=list(resolved.calibration_invalid),
            disable_enable_on_critical_fault=resolved.disable_enable_on_critical_fault,
            ptp_lin_dispatch_mode=resolved.ptp_lin_dispatch_mode,
            axes=[{key: getattr(axis, key) for key in ('axis', 'configured', 'gear_ratio', 'direction', 'zero_offset_rad',
                  'lower_rad', 'upper_rad', 'velocity_rad_s', 'acceleration_rad_s2')} for axis in resolved.axes])
        return response

    def model(self, xml):
        parameter = Parameter(name='robot_description', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=xml))
        response = self.call(self.parameters, SetParametersAtomically.Request(parameters=[parameter]))
        if not response.result.successful:
            raise RuntimeError('ROS odrzucił opis modelu: ' + response.result.reason)
        with self.condition:
            if not self.condition.wait_for(lambda: self.description == xml, timeout=3):
                raise TimeoutError('Brak publikacji nowego robot_description; konfiguracja niepotwierdzona')


class ConfigurationNode(Node):
    def __init__(self):
        super().__init__('robot_configuration')
        default = str(Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'robot/configuration')
        directory = self.declare_parameter('configuration_dir', default).value
        self.group = ReentrantCallbackGroup()
        self.storage = Storage(directory)
        self.owner = (self.storage.root / 'owner.lock').open('a')
        try:
            fcntl.flock(self.owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Inny węzeł jest już właścicielem tego katalogu konfiguracji') from None
        self.coordinator = Coordinator(self.storage, RosTransport(self, self.group, self.storage))
        self.calibration_state = None
        self.create_subscription(ManualState, '/robot/manual/state', self.calibration_update, 1, callback_group=self.group)
        self.service = self.create_service(ConfigurationCommand, '/robot/configuration/command', self.command, callback_group=self.group)
        self.publisher = self.create_publisher(String, '/robot/configuration/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.active_publisher = self.create_publisher(RobotConfiguration, '/robot/configuration/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.motion_settings_publisher = self.create_publisher(MotionSettings, '/robot/motion/settings', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(.5, self.publish)
        self.worker = None
        self.startup = self.create_timer(.2, self.bootstrap)

    def calibration_update(self, message):
        with self.coordinator.lock:
            revision = self.coordinator.active['revision']
            if message.configuration_revision != revision or self.coordinator.running:
                return
            invalid = [axis.joint_configured and not axis.calibration_valid for axis in message.axes]
            signature = (revision, tuple(invalid))
            if signature == self.calibration_state:
                return
            try:
                self.storage.atomic('calibration.json', {'revision': revision, 'invalid': invalid})
                self.calibration_state = signature
            except OSError:
                self.get_logger().error('Nie udało się utrwalić stanu kalibracji; wymagane sprawdzenie dysku')

    def bootstrap(self):
        self.startup.cancel()
        self.worker = threading.Thread(target=self.coordinator.bootstrap, name='configuration-startup', daemon=True)
        self.worker.start()

    def destroy_node(self):
        result = super().destroy_node()
        self.owner.close()
        return result

    def publish(self):
        settings = self.storage.system_settings()
        self.motion_settings_publisher.publish(MotionSettings(revision=settings['revision'], allow_missing_motion=settings['allow_missing_motion']))
        snapshot = self.coordinator.snapshot()
        self.publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False, allow_nan=False)))
        # Również przy braku CAN: opis aktywnego profilu jest źródłem symulacji.
        if snapshot['operation']['phase'] in ('idle', 'done', 'uncertain', 'failed'):
            active = snapshot['active']
            self.active_publisher.publish(RobotConfiguration(revision=active['revision'],
                ptp_lin_dispatch_mode=active['document']['values'].get(DISPATCH_MODE, 'streamed'),
                disable_enable_on_critical_fault=active['document']['values'].get(CRITICAL_FAULT_POLICY, True),
                calibration_invalid=snapshot['calibration_invalid'],
                tcp_jog_max_linear_velocity=(active['document']['values'].get('tcp.max_linear_velocity') or 0) / 1000,
                tcp_jog_max_angular_velocity=math.radians(active['document']['values'].get('tcp.max_angular_velocity') or 0),
                tcp_jog_linear_acceleration=(active['document']['values'].get('tcp.linear_acceleration') or 0) / 1000,
                tcp_jog_angular_acceleration=math.radians(active['document']['values'].get('tcp.angular_acceleration') or 0),
                axes=[JointConfiguration(**profile) for profile in profiles(active['document'])]))

    def command(self, request, response):
        try:
            data = decode(request.request_json or '{}')
            if not isinstance(data, dict):
                raise ValueError('Wymagany obiekt JSON')
            operation = request.operation
            if operation == 'status':
                result = self.coordinator.snapshot(); code = 200
            elif operation == 'system_get':
                result = self.storage.system_settings(); code = 200
            elif operation == 'system_set':
                if any(type(value) is not bool for key, value in data.items() if key in ('ignore_missing_drivers', 'allow_missing_motion')):
                    raise ValueError('Ustawienia wymagają wartości logicznych')
                result = self.storage.set_system_settings(data.get('ignore_missing_drivers'), allow_missing_motion=data.get('allow_missing_motion')); code = 200
                self.motion_settings_publisher.publish(MotionSettings(revision=result['revision'], allow_missing_motion=result['allow_missing_motion']))
            elif operation == 'list':
                result = self.storage.list(data.get('cursor', '')); code = 200
            elif operation == 'get':
                result = self.storage.get(data.get('id')); code = 200
            elif operation == 'create':
                result = self.storage.create(data.get('document')); code = 201
            elif operation == 'delete':
                self.storage.delete(data.get('id')); result = {'deleted': True}; code = 200
            elif operation == 'validate':
                result = validate(data.get('document')); code = 200
            elif operation == 'apply':
                args = self.coordinator.begin(data.get('document'), data.get('expected_revision'), data.get('confirm_calibration_axes'))
                self.worker = threading.Thread(target=self.coordinator.apply, args=args, name='configuration-apply', daemon=True)
                self.worker.start()
                result = self.coordinator.snapshot(); code = 202
            elif operation == 'reconcile':
                with self.coordinator.lock:
                    if self.coordinator.running:
                        raise Conflict('Operacja jeszcze trwa')
                    self.coordinator.running = True
                    self.coordinator.phase('starting', 'Uzgadnianie stanu konfiguracji ROS')
                self.worker = threading.Thread(target=self.coordinator.bootstrap, name='configuration-reconcile', daemon=True)
                self.worker.start()
                result = self.coordinator.snapshot(); code = 202
            else:
                raise ValueError('Nieznana operacja konfiguracji')
        except Conflict as error:
            code, result = 409, {'detail': str(error)}
        except FileNotFoundError:
            code, result = 404, {'detail': 'Snapshot nie istnieje'}
        except (ValueError, KeyError, TypeError) as error:
            code, result = 422, {'detail': str(error)}
        except OSError as error:
            code, result = 503, {'detail': f'Nie udało się odczytać lub zapisać pliku konfiguracji: {error.strerror}'}
        response.status_code = code
        response.response_json = json.dumps(result, ensure_ascii=False, allow_nan=False)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurationNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.worker:
            node.worker.join(timeout=6)
        executor.shutdown(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
