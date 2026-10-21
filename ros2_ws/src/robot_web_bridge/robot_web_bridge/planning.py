"""Obliczenia i ograniczony magazyn planów. Ten moduł nie posiada API wykonania ruchu."""
import asyncio
from collections import OrderedDict
import json
import math
import uuid
import threading
from copy import deepcopy
import xml.etree.ElementTree as ET


def number(value, label, lower=None, upper=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{label}: wymagana skończona liczba')
    if lower is not None and value < lower or upper is not None and value > upper:
        raise ValueError(f'{label}: liczba poza zakresem')
    return float(value)


def pose_values(values):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError('TCP wymaga XYZ w mm oraz Roll/Pitch/Yaw w stopniach')
    x, y, z, roll, pitch, yaw = [number(value, 'TCP') for value in values]
    r, p, y_angle = [math.radians(angle) / 2 for angle in (roll, pitch, yaw)]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y_angle), math.sin(y_angle)
    return {'position': {'x': x / 1000, 'y': y / 1000, 'z': z / 1000}, 'orientation': {
        'x': sr * cp * cy - cr * sp * sy, 'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy, 'w': cr * cp * cy + sr * sp * sy}}


async def ros_result(future, timeout=3):
    loop = asyncio.get_running_loop()
    result = loop.create_future()
    def done(completed):
        def deliver():
            if result.done():
                return
            try:
                result.set_result(completed.result())
            except Exception as error:
                result.set_exception(error)
        if not loop.is_closed():
            loop.call_soon_threadsafe(deliver)
    future.add_done_callback(done)
    return await asyncio.wait_for(result, timeout)


class PlanningClient:
    def __init__(self, node, descriptions, manual, configuration):
        from rclpy.action import ActionClient
        from robot_control_interfaces.action import ComputeCartesianPath, ComputePreviewMotion
        from robot_control_interfaces.srv import SolveIK
        self.action_type, self.ik_type = ComputeCartesianPath, SolveIK
        self.action = ActionClient(node, ComputeCartesianPath, '/robot/planning/compute_cartesian_path')
        self.preview_type = ComputePreviewMotion
        self.preview = ActionClient(node, ComputePreviewMotion, '/robot/planning/compute_preview_motion')
        self.ik = node.create_client(SolveIK, '/robot/planning/solve_ik')
        self.descriptions, self.manual, self.configuration = descriptions, manual, configuration
        self.jobs, self.plans = OrderedDict(), OrderedDict()
        self.handles, self.tasks = {}, set()
        self.job_tasks = {}
        self.reference_key, self.last_positions = None, [None]*6
        self.reference_lock = threading.Lock()

    def current(self, preview=False):
        model, state = self.descriptions.snapshot(), self.manual.snapshot()
        if model is None or (not preview and not state.get('fresh')):
            raise RuntimeError('Brak aktualnego modelu lub stanu robota')
        return model, state

    def reference(self, model, revision, state):
        with self.reference_lock:
            return self._reference(model, revision, state)

    def _reference(self, model, revision, state):
        key = (model, revision)
        if getattr(self, 'reference_key', None) != key:
            self.reference_key, self.last_positions = key, [None]*6
        seeds, sources = [], []
        for i, axis in enumerate(state['axes']):
            value = axis.get('joint_position_degrees') if axis.get('joint_configured') else axis.get('position_degrees')
            measured = (state.get('fresh') and state.get('configuration_revision') == revision and axis.get('available') and
                        axis.get('position_known') and (not axis.get('joint_configured') or axis.get('joint_position_known')) and
                        type(value) in (int, float) and math.isfinite(value))
            if measured:
                self.last_positions[i] = value
            seeds.append(value if measured else self.last_positions[i] if self.last_positions[i] is not None else 0.)
            sources.append('measured' if measured else 'last' if self.last_positions[i] is not None else 'zero')
        return seeds, sources

    def remember_positions(self):
        model, state = self.descriptions.snapshot(), self.manual.snapshot()
        if model and state.get('configuration_revision'):
            self.reference(model[1], state['configuration_revision'], state)

    async def context(self, data, preview=False, configuration_snapshot=None):
        from robot_control_interfaces.msg import PlanningContext, JointConfiguration
        (xml, identifier), state = self.current(preview)
        code, snapshot = configuration_snapshot if configuration_snapshot is not None else await self.configuration('status')
        phases = ('idle', 'done', 'uncertain', 'failed') if preview else ('idle',)
        if code != 200 or snapshot.get('operation', {}).get('phase') not in phases:
            raise RuntimeError('Konfiguracja ROS jest niedostępna lub nieuzgodniona')
        active = snapshot['active']
        if preview and active['revision'] != 'initial' and ET.fromstring(xml).get('configuration_revision') != active['revision']:
            raise ValueError('Opis modelu nie odpowiada aktywnej konfiguracji')
        if data.get('model_id') != identifier or data.get('configuration_revision') != active['revision']:
            raise ValueError('Model lub konfiguracja uległy zmianie; otwórz ponownie planowanie TCP')
        if not preview and active['revision'] != state.get('configuration_revision'):
            raise ValueError('Rewizja profilu różni się od stanu osi')
        values = active['document']['values']
        seed = data.get('seed_degrees')
        if not isinstance(seed, list) or len(seed) != 6:
            raise ValueError('Wymagane sześć jawnie znanych pozycji startowych')
        tcp_feed = preview and data.get('motion_type') == 'LIN' and 'tcp_velocity_mm_s' in data
        preview_v = math.radians(180 if tcp_feed else number(data.get('preview_velocity_degrees_s'), 'Prędkość podglądu', .01, 360))
        preview_a = math.radians(100 if tcp_feed else number(data.get('preview_acceleration_degrees_s2'), 'Przyspieszenie podglądu', .01, 3600))
        context = PlanningContext(model_id=identifier, configuration_revision=active['revision'], urdf=xml)
        profiles, lower, upper, vmax, amax, locked = [], [], [], [], [], []
        for i, axis in enumerate(state['axes']):
            prefix = f'joint.{i + 1}.'
            complete = all(values.get(prefix + key) is not None for key in ('gear_ratio','direction','zero_offset','lower','upper','velocity','acceleration'))
            profile = JointConfiguration(axis=i + 1, configured=complete if preview else bool(axis.get('joint_configured')))
            if profile.configured:
                for source, target in [('gear_ratio', 'gear_ratio'), ('direction', 'direction'), ('zero_offset', 'zero_offset_rad'),
                                       ('lower', 'lower_rad'), ('upper', 'upper_rad'), ('velocity', 'velocity_rad_s'), ('acceleration', 'acceleration_rad_s2')]:
                    value = number(values.get(prefix + source), f'J{i + 1} {source}')
                    setattr(profile, target, int(value) if source == 'direction' else value if source == 'gear_ratio' else math.radians(value))
                if profile.gear_ratio <= 0 or profile.direction not in (-1, 1):
                    raise ValueError('Nieprawidłowy profil przegubu')
                lo, hi = profile.lower_rad, profile.upper_rad
                velocity, acceleration = (profile.velocity_rad_s, profile.acceleration_rad_s2) if tcp_feed else (
                    min(preview_v, profile.velocity_rad_s), min(preview_a, profile.acceleration_rad_s2))
            else:
                lo, hi, velocity, acceleration = -20 * math.pi, 20 * math.pi, preview_v, preview_a
            q = math.radians(number(seed[i], f'Pozycja początkowa J{i + 1}'))
            if q < lo or q > hi or velocity <= 0 or acceleration <= 0:
                raise ValueError(f'J{i + 1}: pozycja lub profil poza limitami')
            lower.append(lo); upper.append(hi); vmax.append(velocity); amax.append(acceleration); profiles.append(profile)
            locked.append(False if preview else not (axis.get('available') and axis.get('position_known') and (not profile.configured or axis.get('joint_position_known'))))
        context.seed = [math.radians(float(value)) for value in seed]
        context.known = [True] * 6; context.locked = locked; context.lower = lower; context.upper = upper
        context.max_velocity = vmax; context.max_acceleration = amax; context.profiles = profiles
        context.simulation_only = preview
        if preview:
            flags = data.get('simulated_start')
            if not isinstance(flags, list) or len(flags) != 6 or any(type(flag) is not bool for flag in flags):
                raise ValueError('Wymagane oznaczenia sześciu pozycji startowych symulacji')
            context.simulated_start = flags
        return context

    async def status(self):
        (xml, identifier), state = self.current(True)
        code, snapshot = await self.configuration('status')
        if code != 200:
            raise RuntimeError('Brak aktywnej konfiguracji')
        active = snapshot['active']
        seed, sources = self.reference(identifier, active['revision'], state)
        code, settings = await self.configuration('system_get')
        if code != 200:
            raise RuntimeError('Brak ustawień systemowych')
        # Te same profile i limity co w obliczeniach, bez wymagania pozycji w limitach startu.
        values = active['document']['values']; axes = []
        for i in range(6):
            prefix = f'joint.{i+1}.'
            configured = all(values.get(prefix+key) is not None for key in ('gear_ratio','direction','zero_offset','lower','upper','velocity','acceleration'))
            axes.append({'configured': configured, 'lower': values[prefix+'lower'] if configured else -3600,
                         'upper': values[prefix+'upper'] if configured else 3600,
                         'velocity': values[prefix+'velocity'] if configured else 180})
        return {'model_id': identifier, 'configuration_revision': active['revision'],
                'tcp_jog_segmented': True,
                'tcp_step_available': True,
                'planner_available': self.preview.server_is_ready(), 'profiles': axes,
                'settings_revision': settings.get('revision'), 'ignore_missing_drivers': settings.get('ignore_missing_drivers', False),
                'preview_seed_degrees': seed, 'seed_sources': sources,
                'available': [bool(axis.get('available') and axis.get('position_known') and (not axis.get('joint_configured') or axis.get('joint_position_known'))) for axis in state['axes']],
                'seed_degrees': [(axis.get('joint_position_degrees') if axis.get('joint_configured') else axis.get('position_degrees'))
                                 if axis.get('available') and axis.get('position_known') else None for axis in state['axes']]}

    async def solve(self, data):
        from rosidl_runtime_py.set_message import set_message_fields
        if not self.ik.service_is_ready():
            raise RuntimeError('Solver IK jest niedostępny')
        request = self.ik_type.Request(context=await self.context(data), position_tolerance=1e-4, orientation_tolerance=1e-3,
                                       forward_only=data.get('forward_only') is True)
        if not request.forward_only:
            set_message_fields(request.target, pose_values(data.get('target')))
        future = self.ik.call_async(request)
        try:
            answer = await ros_result(future)
            if not answer.success:
                raise ValueError(answer.message)
            return {'positions_degrees': [math.degrees(q) for q in answer.positions], 'tcp': self.pose_json(answer.tcp),
                    'position_error_mm': answer.position_error * 1000, 'orientation_error_degrees': math.degrees(answer.orientation_error)}
        finally:
            self.ik.remove_pending_request(future)

    async def tcp_step(self, data):
        from .tcp_jog import increment_pose
        axis, frame = data.get('axis'), data.get('frame')
        if axis not in ('x', 'y', 'z', 'rx', 'ry', 'rz') or frame not in ('base', 'tool'):
            raise ValueError('Wymagany kierunek TCP i układ base/tool')
        amount = number(data.get('step'), 'Impuls TCP')
        if amount not in (.1, 1, 5, 10, 20):
            raise ValueError('Impuls TCP: wybierz 0.1, 1, 5, 10 lub 20')
        direction = number(data.get('direction'), 'Kierunek')
        if direction not in (-1, 1):
            raise ValueError('Kierunek musi wynosić −1 lub 1')
        linear = axis in ('x', 'y', 'z')
        velocity = number(data.get('velocity'), 'Prędkość TCP', .01, 100 if linear else 30)
        settings, configuration = await asyncio.gather(self.configuration('system_get'), self.configuration('status'))
        await self.tcp_settings_guard(data, settings)
        parameters = {**data, 'motion_type': 'LIN', 'tcp_velocity_mm_s': velocity if linear else 10,
                      'tcp_angular_velocity_degrees_s': 5 if linear else velocity, 'tcp_acceleration_mm_s2': 100}
        context = await self.context(parameters, True, configuration)
        if not self.ik.service_is_ready():
            raise RuntimeError('Solver IK jest niedostępny')
        async def compute(request):
            future = self.ik.call_async(request)
            try:
                answer = await ros_result(future)
                if not answer.success:
                    raise ValueError(answer.message)
                return answer
            finally:
                self.ik.remove_pending_request(future)
        fk = await compute(self.ik_type.Request(context=context, forward_only=True))
        target = increment_pose(fk.tcp, frame, axis, direction*(amount/1000 if linear else math.radians(amount)))
        answer = await compute(self.ik_type.Request(context=context, target=target,
                               position_tolerance=1e-6, orientation_tolerance=1e-5))
        await self.tcp_settings_guard(data, await self.configuration('system_get'))
        # LIN sprawdza całą drogę, gałąź IK, q/v/a i posuw. Wynik nie jest wykonywany.
        return await self.move({**parameters, 'target_degrees': [math.degrees(q) for q in answer.positions]})

    async def tcp_settings_guard(self, data, settings_snapshot):
        code, settings = settings_snapshot
        if code != 200 or settings.get('revision') != data.get('settings_revision'):
            raise ValueError('Ustawienia symulacji zmieniły się; ponów planowanie TCP')
        if not settings.get('ignore_missing_drivers', False):
            state = self.manual.snapshot()
            if not state.get('fresh') or state.get('configuration_revision') != data.get('configuration_revision') or len(state['axes']) != 6:
                raise ValueError('TCP wymaga aktualnych pozycji sześciu osi')
            for i, item in enumerate(state['axes']):
                age = item.get('position_age_seconds')
                if (item.get('axis') != i+1 or not item.get('available') or not item.get('position_known') or
                        type(age) not in (int, float) or not 0 <= age < .25 or
                        (item.get('joint_configured') and not (item.get('joint_position_known') and item.get('calibration_valid')))):
                    raise ValueError(f'TCP wymaga aktualnej pozycji J{i+1}')

    async def tcp_jog(self, data):
        from .tcp_jog import increment_pose
        frame, axis = data.get('frame'), data.get('axis')
        if frame not in ('base', 'tool') or axis not in ('x', 'y', 'z', 'rx', 'ry', 'rz'):
            raise ValueError('Wymagany układ base/tool i kierunek x/y/z/rx/ry/rz')
        direction = number(data.get('direction'), 'Kierunek')
        if direction not in (-1, 1):
            raise ValueError('Kierunek musi wynosić −1 lub 1')
        duration = number(data.get('duration_s'), 'Czas kroku', .01, .1)
        linear = axis in ('x', 'y', 'z')
        speed = number(data.get('velocity'), 'Prędkość TCP', .01, 100 if linear else 30)
        distance = speed*duration/1000 if linear else math.radians(speed*duration)
        # Niezależne odczyty równolegle; rewizje nadal sprawdzane przed i po IK.
        settings, configuration = await asyncio.gather(self.configuration('system_get'), self.configuration('status'))
        await self.tcp_settings_guard(data, settings)
        context_data = {**data, 'preview_velocity_degrees_s': 180, 'preview_acceleration_degrees_s2': 100}
        context = await self.context(context_data, True, configuration)
        if not self.ik.service_is_ready():
            raise RuntimeError('Solver IK jest niedostępny')

        async def compute(request):
            future = self.ik.call_async(request)
            try:
                answer = await ros_result(future)
                if not answer.success:
                    raise ValueError(answer.message)
                return answer
            finally:
                self.ik.remove_pending_request(future)

        fk = await compute(self.ik_type.Request(context=context, forward_only=True))
        # Pełny posuw przy 20 Hz, ale IK i kontrola limitów co najwyżej 1 mm / 1°.
        steps = max(1, math.ceil(abs(distance)/(.001 if linear else math.radians(1)) - 1e-12))
        previous = list(context.seed)
        for step in range(1, steps+1):
            segment = deepcopy(context)
            segment.seed = previous
            target = increment_pose(fk.tcp, frame, axis, direction*distance*step/steps)
            answer = await compute(self.ik_type.Request(context=segment, target=target,
                                   position_tolerance=1e-5, orientation_tolerance=1e-4))
            for i, (q, start, lower, upper, velocity) in enumerate(zip(answer.positions, previous,
                    context.lower, context.upper, context.max_velocity)):
                if not math.isfinite(q) or q < lower or q > upper or abs(q-start) > min(.25, velocity*duration/steps) + 1e-9:
                    raise ValueError(f'Krok TCP przekracza limit pozycji/prędkości lub ciągłości J{i+1}')
            previous = list(answer.positions)
        for i, (q, start, lower, upper, velocity) in enumerate(zip(answer.positions, context.seed,
                context.lower, context.upper, context.max_velocity)):
            if not math.isfinite(q) or q < lower or q > upper or abs(q-start) > min(.25, velocity*duration) + 1e-9:
                raise ValueError(f'Krok TCP przekracza limit pozycji/prędkości lub ciągłości J{i+1}')
        settings, configuration = await asyncio.gather(self.configuration('system_get'), self.configuration('status'))
        await self.tcp_settings_guard(data, settings)
        # Ponowna kontrola rewizji przed przekazaniem wyniku klientowi.
        await self.context(context_data, True, configuration)
        return {'positions_degrees': [math.degrees(q) for q in answer.positions], 'tcp': self.pose_json(answer.tcp),
                'model_id': data['model_id'], 'configuration_revision': data['configuration_revision'],
                'settings_revision': data['settings_revision'], 'simulation_only': True, 'execution_ready': False}

    @staticmethod
    def pose_json(pose):
        x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        return [pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000,
                math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2*(x*x + y*y))),
                math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x))))),
                math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2*(y*y + z*z)))]

    async def start(self, data):
        from geometry_msgs.msg import Pose
        from rosidl_runtime_py.set_message import set_message_fields
        if not self.action.server_is_ready():
            raise RuntimeError('Planner C++ jest niedostępny')
        if any(job['status'] == 'running' for job in self.jobs.values()):
            raise ValueError('Trwa obliczanie innej drogi; anuluj je przed kolejnym żądaniem')
        targets = data.get('targets')
        if not isinstance(targets, list) or not 1 <= len(targets) <= 32:
            raise ValueError('Wymagane od 1 do 32 celów TCP')
        goal = self.action_type.Goal(context=await self.context(data))
        for target in targets:
            pose = Pose(); set_message_fields(pose, pose_values(target)); goal.targets.append(pose)
        goal.blend_position_tolerance = number(data.get('blend_position_mm'), 'Tolerancja pozycji', .01, 50) / 1000
        goal.blend_orientation_tolerance = math.radians(number(data.get('blend_orientation_degrees'), 'Tolerancja orientacji', .01, 28))
        goal.velocity_scaling = number(data.get('velocity_scaling', 1), 'Skala prędkości', .01, 1)
        goal.acceleration_scaling = number(data.get('acceleration_scaling', 1), 'Skala przyspieszenia', .01, 1)
        return await self.submit(goal, self.action)

    async def move(self, data):
        if not self.preview.server_is_ready():
            raise RuntimeError('Planner podglądu jest niedostępny')
        code, settings = await self.configuration('system_get')
        if code != 200 or settings.get('revision') != data.get('settings_revision'):
            raise ValueError('Ustawienia symulacji zmieniły się; wybierz ruch ponownie')
        target = data.get('target_degrees')
        if not isinstance(target, list) or len(target) != 6 or data.get('motion_type') not in ('PTP','LIN'):
            raise ValueError('Wymagany typ PTP/LIN i sześć pozycji docelowych')
        context = await self.context(data, True)
        goal = self.preview_type.Goal(context=context, motion_type=0 if data['motion_type']=='PTP' else 1,
                                      target_positions=[math.radians(number(q, 'Cel osi')) for q in target])
        if data['motion_type'] == 'LIN' and 'tcp_velocity_mm_s' in data:
            goal.tcp_velocity = number(data['tcp_velocity_mm_s'], 'Prędkość TCP mm/s', .01, 10000) / 1000
            goal.tcp_acceleration = number(data.get('tcp_acceleration_mm_s2', 100), 'Przyspieszenie TCP mm/s²', .01, 100000) / 1000
            goal.tcp_angular_velocity = math.radians(number(data.get('tcp_angular_velocity_degrees_s', 10), 'Obrót TCP °/s', .01, 360))
        return await self.submit(goal, self.preview, {'settings_revision': settings['revision'], 'execution_limits': {key: data[key] for key in ('preview_velocity_degrees_s', 'preview_acceleration_degrees_s2') if key in data}})

    async def submit(self, goal, action, metadata=None):
        if any(job['status'] == 'running' for job in self.jobs.values()):
            raise ValueError('Trwają inne obliczenia; poczekaj na zakończenie anulowania')
        identifier = uuid.uuid4().hex
        while len(self.jobs) >= 8:
            self.jobs.popitem(last=False)
        job = {'id': identifier, 'status': 'running', 'progress': 0, 'phase': 'Oczekiwanie na planner', 'plan_id': None, 'message': ''}
        job.update(metadata or {})
        self.jobs[identifier] = job
        loop = asyncio.get_running_loop()
        def feedback(value):
            if not loop.is_closed():
                loop.call_soon_threadsafe(job.update, {'progress': value.feedback.progress, 'phase': value.feedback.phase})
        pending = action.send_goal_async(goal, feedback_callback=feedback)
        try:
            handle = await ros_result(pending)
        except Exception:
            def cancel_late(completed):
                try:
                    late = completed.result()
                    if late and late.accepted:
                        late.cancel_goal_async()
                except Exception:
                    pass
            pending.add_done_callback(cancel_late)
            job.update(status='failed', message='Brak potwierdzenia planera ROS')
            raise RuntimeError('Nie potwierdzono rozpoczęcia obliczeń') from None
        if not handle.accepted:
            job.update(status='failed', message='Planner odrzucił nieaktualny kontekst lub jest zajęty')
            return job.copy()
        self.handles[identifier] = handle
        task = asyncio.create_task(self.watch(identifier, handle))
        self.job_tasks[identifier] = task
        self.tasks.add(task); task.add_done_callback(self.tasks.discard)
        task.add_done_callback(lambda _: self.job_tasks.pop(identifier, None))
        return job.copy()

    async def watch(self, identifier, handle):
        from rosidl_runtime_py.convert import message_to_ordereddict
        job = self.jobs[identifier]
        try:
            result = (await ros_result(handle.get_result_async(), 60)).result
            if job.get('cancel_requested'):
                job.update(status='cancelled', message='Podgląd anulowany'); return
            if not result.success:
                job.update(status='cancelled' if job.get('cancel_requested') else 'failed', message=result.message)
                return
            plan = message_to_ordereddict(result.plan)
            if plan.get('simulation_only'):
                plan['settings_revision'] = job['settings_revision']
                plan['execution_limits'] = job.get('execution_limits', {})
            if len(json.dumps(plan)) > 8 * 1024 * 1024:
                raise ValueError('Plan przekracza limit podglądu 8 MiB')
            while len(self.plans) >= 4:
                self.plans.popitem(last=False)
            self.plans[plan['id']] = plan
            job.update(status='succeeded', progress=1, phase='Gotowy podgląd', plan_id=plan['id'], message=result.message)
        except Exception as error:
            handle.cancel_goal_async(); job.update(status='failed', message=str(error) or 'Przekroczono czas obliczeń')
        finally:
            self.handles.pop(identifier, None)

    async def cancel(self, identifier):
        job = self.jobs.get(identifier)
        if job is None:
            raise KeyError('Nie znaleziono obliczeń')
        job['cancel_requested'] = True
        handle = self.handles.get(identifier)
        if handle is not None:
            await ros_result(handle.cancel_goal_async())
        task = self.job_tasks.get(identifier)
        if task is not None:
            await asyncio.wait_for(asyncio.shield(task), 3)
        return job.copy()

    async def get_plan(self, identifier):
        plan = self.plans.get(identifier)
        if plan is None:
            raise KeyError('Plan nie istnieje lub został usunięty z pamięci')
        if plan.get('simulation_only'):
            (_, current_model), _ = self.current(True)
            code, snapshot = await self.configuration('status')
            settings_code, settings = await self.configuration('system_get')
            if (code != 200 or settings_code != 200 or current_model != plan['model_id'] or
                    snapshot['active']['revision'] != plan['configuration_revision'] or settings['revision'] != plan['settings_revision']):
                raise ValueError('Kontekst symulacji zmienił się; wybierz ruch ponownie')
            return plan
        (_, current_model), state = self.current()
        if current_model != plan['model_id'] or state.get('configuration_revision') != plan['configuration_revision']:
            raise ValueError('Plan dotyczy innego modelu lub rewizji')
        if any(not frozen and not (axis.get('available') and axis.get('position_known') and
               (not axis.get('joint_configured') or axis.get('joint_position_known')))
               for frozen, axis in zip(plan['locked'], state['axes'])):
            raise ValueError('Zmieniła się dostępność osi planu; oblicz drogę ponownie')
        return plan


def add_planning_routes(app, client):
    from fastapi import HTTPException, Request
    from starlette.responses import JSONResponse
    async def body(request):
        raw = bytearray()
        async for chunk in request.stream():
            raw.extend(chunk)
            if len(raw) > 65536:
                raise HTTPException(413, 'Żądanie planowania przekracza 64 KiB')
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError()
            return data
        except (ValueError, UnicodeDecodeError):
            raise HTTPException(422, 'Nieprawidłowy JSON planowania') from None
    async def call(operation, *args):
        if client is None:
            raise HTTPException(503, 'Planner jest niedostępny')
        try:
            result = operation(*args)
            if asyncio.iscoroutine(result):
                result = await result
            return JSONResponse(result, headers={'Cache-Control': 'no-store'})
        except KeyError as error:
            raise HTTPException(404, str(error)) from None
        except (ValueError, TypeError) as error:
            raise HTTPException(422, str(error)) from None
        except (RuntimeError, TimeoutError) as error:
            raise HTTPException(503, str(error)) from None
    @app.get('/api/v1/planning/context')
    async def context():
        return await call(client.status if client else None)
    @app.post('/api/v1/planning/ik')
    async def ik(request: Request):
        return await call(client.solve if client else None, await body(request))
    @app.post('/api/v1/planning/tcp-jog')
    async def tcp_jog(request: Request):
        return await call(client.tcp_jog if client else None, await body(request))
    @app.post('/api/v1/planning/tcp-step', status_code=202)
    async def tcp_step(request: Request):
        result = await call(client.tcp_step if client else None, await body(request)); result.status_code = 202; return result
    @app.post('/api/v1/planning/jobs', status_code=202)
    async def start(request: Request):
        result = await call(client.start if client else None, await body(request)); result.status_code = 202; return result
    @app.post('/api/v1/planning/moves', status_code=202)
    async def move(request: Request):
        result = await call(client.move if client else None, await body(request)); result.status_code = 202; return result
    @app.get('/api/v1/planning/jobs/{identifier}')
    async def job(identifier: str):
        def get():
            if identifier not in client.jobs:
                raise KeyError('Nie znaleziono obliczeń')
            return client.jobs[identifier].copy()
        return await call(get)
    @app.delete('/api/v1/planning/jobs/{identifier}')
    async def cancel(identifier: str):
        return await call(client.cancel if client else None, identifier)
    @app.get('/api/v1/planning/plans/{identifier}')
    async def plan(identifier: str):
        return await call(client.get_plan if client else None, identifier)
