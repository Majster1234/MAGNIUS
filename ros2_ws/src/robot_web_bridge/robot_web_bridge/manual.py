"""Ostatni stan i ograniczone wywołania ROS; logika ruchu pozostaje w C++."""
import asyncio
from copy import deepcopy
import threading
import time
from .trace import trace, recording, freeze_after_fault


class ManualStore:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.state = None
        self.stamp = None
        self.stamp_ns = None

    def update(self, message):
        state = {'busy': message.busy, 'fault': message.fault,
                 'active_axis': message.active_axis, 'max_rpm': message.max_rpm,
                 'operation': message.operation, 'message': message.message, 'axes': []}
        state['configuration_revision'] = getattr(message, 'configuration_revision', 'initial')
        fields = ('axis', 'available', 'prepared', 'can_jog', 'can_zero', 'motion',
                  'enable_known', 'enabled', 'position_known', 'position_degrees',
                  'position_age_seconds', 'speed_known', 'speed_rpm',
                  'speed_age_seconds', 'enable_age_seconds')
        state['axes'] = [{key: getattr(axis, key) for key in fields} for axis in message.axes]
        for source, axis in zip(message.axes, state['axes']):
            for key in ('joint_configured', 'calibration_valid', 'joint_position_known'):
                axis[key] = getattr(source, key, False)
            axis['configuration_message'] = getattr(source, 'configuration_message', '')
            import math
            for source_key, target_key in (('joint_position_rad', 'joint_position_degrees'),
                    ('joint_lower_rad', 'joint_lower_degrees'), ('joint_upper_rad', 'joint_upper_degrees'),
                    ('jog_min_velocity_rad_s', 'jog_min_velocity_degrees_s'), ('jog_max_velocity_rad_s', 'jog_max_velocity_degrees_s')):
                axis[target_key] = math.degrees(getattr(source, source_key, 0.0))
        stamp = message.monotonic_ns / 1e9
        with self.lock:
            if self.stamp is not None and stamp <= self.stamp:
                return
            self.state, self.stamp = state, stamp
            self.stamp_ns = message.monotonic_ns
        trace('ros_state_received', state_monotonic_ns=message.monotonic_ns,
              transport_age_ns=time.monotonic_ns() - message.monotonic_ns, state=state)
        if message.fault:
            freeze_after_fault()

    def snapshot(self):
        with self.lock:
            age = None if self.stamp is None else self.clock() - self.stamp
            state = deepcopy(self.state)
            stamp_ns = self.stamp_ns
        # Dwa asynchroniczne cykle 20 Hz: opóźnienie publikacji nie jest utratą CAN.
        # Terminy ruchu nadal sprawdza C++ bezpośrednio na pomiarach CAN.
        fresh = age is not None and 0 <= age < 0.5
        control_fresh = age is not None and 0 <= age < 0.15
        if state is None:
            state = {'busy': False, 'fault': False, 'active_axis': 0, 'max_rpm': 30,
                     'operation': 'idle', 'message': 'Oczekiwanie na moduł CAN',
                     'axes': [{'axis': axis} for axis in range(1, 7)]}
        state.update(fresh=fresh, age_seconds=age, recording=recording(),
                     state_monotonic_ns=None if stamp_ns is None else str(stamp_ns),
                     server_monotonic_ns=str(time.monotonic_ns()))
        for axis in state['axes']:
            elapsed = max(0, age or 0)
            for field, known, limit in (('position_age_seconds', 'position_known', .25),
                                        ('speed_age_seconds', 'speed_known', .25),
                                        ('enable_age_seconds', 'enable_known', .5)):
                measured = axis.get(field, -1)
                axis[field] = measured + elapsed if measured >= 0 else None
                axis[known] = fresh and axis.get(known, False) and measured >= 0 and measured + elapsed < limit
            axis['available'] = fresh and axis.get('available', False) and axis['position_known']
            axis['can_jog'] = control_fresh and axis.get('can_jog', False) and axis['available'] and axis['enable_known'] and axis['speed_known']
            axis['can_zero'] = control_fresh and axis.get('can_zero', False) and axis['available'] and axis['speed_known']
            axis['joint_position_known'] = axis.get('joint_position_known', False) and axis['position_known'] and axis.get('calibration_valid', False)
            if not axis['speed_known']:
                axis['motion'] = 'stan nieznany'
        return state


class ManualRosClient:
    def __init__(self, node):
        from robot_control_interfaces.srv import JogMotor, JogJoint, SetMotorEnable, ZeroMotor
        self.types = {'enable': SetMotorEnable, 'zero': ZeroMotor, 'jog': JogMotor, 'jog_joint': JogJoint}
        self.clients = {name: node.create_client(kind, '/robot/manual/' + name)
                        for name, kind in self.types.items()}
        self.pending = 0

    async def __call__(self, operation, **values):
        if self.pending >= 8:
            return False, 'Moduł sterowania jest zajęty'
        client = self.clients[operation]
        if not client.service_is_ready():
            return False, 'Moduł ręcznego sterowania CAN jest niedostępny'
        request = self.types[operation].Request()
        for key, value in values.items():
            setattr(request, key, value)
        request.issued_monotonic_ns = time.monotonic_ns()
        started = request.issued_monotonic_ns
        trace('ros_service_request', operation=operation, issued_ns=started,
              **{key: value for key, value in values.items() if key != 'session'})
        self.pending += 1
        future = None
        try:
            future = client.call_async(request)
            loop = asyncio.get_running_loop()
            result = loop.create_future()

            def deliver(done):
                def finish():
                    if result.done():
                        return
                    try:
                        result.set_result(done.result())
                    except Exception as error:
                        result.set_exception(error)
                loop.call_soon_threadsafe(finish)

            future.add_done_callback(deliver)
            response = await asyncio.wait_for(result, timeout=.08)
            trace('ros_service_result', operation=operation, issued_ns=started,
                  duration_ns=time.monotonic_ns() - started, accepted=bool(response.accepted), message=response.message)
            return bool(response.accepted), response.message
        except asyncio.CancelledError:
            raise
        except Exception:
            trace('ros_service_timeout_or_error', operation=operation, issued_ns=started,
                  duration_ns=time.monotonic_ns() - started)
            return False, 'Brak potwierdzenia przyjęcia komendy; sprawdź stan osi'
        finally:
            if future is not None:
                client.remove_pending_request(future)
                if not future.done():
                    future.cancel()
            self.pending -= 1
