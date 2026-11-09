"""Przygotowanie i zatwierdzanie lokalnych programów. Brak zegara sterowania w HTTP."""
import asyncio
import math
import time
import uuid
from .planning import configuration_ready, lin_limits, ros_result


class MotionConflict(ValueError):
    pass


class MotionClient:
    def __init__(self, node, planning, configuration):
        from rclpy.action import ActionClient
        from robot_control_interfaces.action import ComputePhysicalMotion, ExecuteMotionProgram
        from robot_control_interfaces.srv import PrepareMotionProgram, GetMotionProgram
        from std_srvs.srv import Trigger
        from controller_manager_msgs.srv import ListControllers, SwitchController
        self.planning, self.configuration = planning, configuration
        self.physical_type, self.execute_type = ComputePhysicalMotion, ExecuteMotionProgram
        self.prepare_type, self.status_type, self.stop_type = PrepareMotionProgram, GetMotionProgram, Trigger
        self.physical = ActionClient(node, ComputePhysicalMotion, '/robot/planning/compute_physical_motion')
        self.executor = ActionClient(node, ExecuteMotionProgram, '/robot/motion/execute_program')
        self.prepare_client = node.create_client(PrepareMotionProgram, '/robot/motion/prepare_program')
        self.status_client = node.create_client(GetMotionProgram, '/robot/motion/program_status')
        self.stop_client = node.create_client(Trigger, '/robot/motion/stop')
        self.list_type, self.switch_type = ListControllers, SwitchController
        self.controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
        self.controller = node.declare_parameter('execution_controller', 'joint_trajectory_controller').value
        self.preparation = {'state': 'idle'}
        # Nastawy regulatora wykonania (testy jakości ruchu): publikuje je robot_can_diagnostics, zmiana parametrami ROS.
        import json
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from rcl_interfaces.srv import SetParameters
        self.execution_tuning = None
        def tuning_update(message):
            try:
                self.execution_tuning = json.loads(message.data)
            except ValueError:
                self.execution_tuning = None
        node.create_subscription(String, '/robot/motion/execution_tuning', tuning_update,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.parameter_type = SetParameters
        self.parameter_client = node.create_client(SetParameters, '/robot_can_diagnostics/set_parameters')
        self.task = None
        self.planner_handle = None
        self.lock = asyncio.Lock()
        self.owner = None   # token wykonawcy programu: ręczne prepare/execute przez HTTP są wtedy blokowane

    async def call(self, client, request, timeout=3):
        if not client.service_is_ready():
            raise RuntimeError('Usługa wykonawcy ROS jest niedostępna')
        return await ros_result(client.call_async(request), timeout)

    async def status(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        response = await self.call(self.status_client, self.status_type.Request())
        return {**message_to_ordereddict(response.status), 'preparation': dict(self.preparation),
                'motion_type': self.program_motion_type(response.status.program_id),
                'execution_tuning': getattr(self, 'execution_tuning', None)}

    TUNING_PARAMETERS = {'lookahead_time_sec': 'execution_lookahead_time_sec', 'velocity_ff_scale': 'execution_velocity_ff_scale',
                         'position_kp': 'execution_position_kp', 'integral_gain': 'execution_position_integral_gain',
                         'integral_limit_rad': 'execution_position_integral_limit_rad',
                         'acceleration_code': 'execution_acceleration_override'}   # test: 0–255, -1 = konfiguracja/plik

    async def tuning(self):
        if self.execution_tuning is None:
            raise RuntimeError('Brak informacji o nastawach regulatora z wykonawcy ROS')
        return dict(self.execution_tuning)

    async def set_tuning(self, data):
        """Zmiana nastaw regulatora wykonania poza konfiguracją robota; {"reset": true} przywraca wartości z pliku."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        if not isinstance(data, dict):
            raise ValueError('Wymagany obiekt JSON z nastawami')
        if data.get('reset') is True:
            if self.execution_tuning is None:
                raise RuntimeError('Nieznane nastawy bazowe wykonawcy')
            values = dict(self.execution_tuning['baseline'])
        else:
            unknown = set(data) - set(self.TUNING_PARAMETERS)
            if unknown or not data:
                raise ValueError('Dozwolone nastawy: ' + ', '.join(self.TUNING_PARAMETERS))
            values = data
        request = self.parameter_type.Request()
        for key, value in values.items():
            if key not in self.TUNING_PARAMETERS:
                continue
            if key == 'acceleration_code':
                if type(value) not in (int, float) or value != int(value) or not -1 <= value <= 255:
                    raise ValueError('acceleration_code: liczba całkowita 0–255 albo -1')
                request.parameters.append(Parameter(name=self.TUNING_PARAMETERS[key], value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))))
                continue
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'{key}: wymagana liczba')
            request.parameters.append(Parameter(name=self.TUNING_PARAMETERS[key], value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))))
        async with self.lock:
            if (await self.status())['busy']:
                raise MotionConflict('Zmiana nastaw regulatora wymaga postoju')
            response = await self.call(self.parameter_client, request)
        failed = [result.reason for result in response.results if not result.successful]
        if failed:
            raise MotionConflict('; '.join(failed))
        for _ in range(20):   # temat transient-local dociera asynchronicznie
            current = self.execution_tuning or {}
            if all(abs(current.get('active', {}).get(key, math.nan) - float(value)) < 1e-9
                   for key, value in values.items() if key in self.TUNING_PARAMETERS):
                break
            await asyncio.sleep(0.05)
        return await self.tuning()

    def program_motion_type(self, program_id):
        plan = getattr(self, 'physical_plan', None)
        if program_id and program_id == getattr(self, 'prepared_program_id', None) and plan is not None:
            return plan.motion_type if plan.motion_type in ('PTP', 'LIN', 'CHAIN') else None
        return None

    # ----- wykonawca programów (program_runner.py) -----
    def claim(self, token):
        if getattr(self, 'owner', None) is not None and self.owner != token:
            raise MotionConflict('Ruchem steruje inny przebieg programu')
        self.owner = token

    def release(self, token):
        if getattr(self, 'owner', None) == token:
            self.owner = None

    def guard_owner(self, token=None):
        owner = getattr(self, 'owner', None)
        if owner is not None and owner != token:
            raise MotionConflict('Ruchem steruje wykonawca programu; zatrzymaj program (STOP) przed ruchem ręcznym')

    def measured_degrees(self):
        (_, model), state = self.planning.current(False)
        seeds, sources = self.planning.reference(model, state.get('configuration_revision'), state)
        if any(source != 'measured' for source in sources):
            raise ValueError('Wykonanie programu wymaga aktualnych pozycji pomiarowych wszystkich osi')
        return seeds

    async def prepare_chain(self, physical, token):
        """Przygotowanie fizycznego ciągu ruchów (motion_type CHAIN) obliczonego przez wykonawcę programów."""
        self.guard_owner(token)
        async with self.lock:
            if self.task and not self.task.done():
                raise MotionConflict('Trwa przygotowanie programu')
            if (await self.status())['busy']:
                raise MotionConflict('Trwa wykonanie programu')
            code, settings = await self.configuration('system_get')
            if code != 200:
                raise RuntimeError('Brak potwierdzonych ustawień ruchu')
            (_, model), state = self.planning.current(False)
            code, snapshot = await self.configuration('status')
            if code != 200 or not configuration_ready(snapshot):
                raise RuntimeError('Konfiguracja fizycznego robota nie jest uzgodniona')
            if (physical.simulation_only or physical.motion_type != 'CHAIN' or physical.model_id != model or
                    state.get('configuration_revision') != physical.configuration_revision or
                    snapshot['active']['revision'] != physical.configuration_revision):
                raise ValueError('Ciąg ruchów dotyczy innego modelu lub konfiguracji; uruchom program ponownie')
            await self.release_controller()
            request = self.prepare_type.Request(plan=physical, settings_revision=settings['revision'],
                                                issued_monotonic_ns=time.monotonic_ns(), participation_mask=0)
            prepared = await self.call(self.prepare_client, request, 10)
            if not prepared.accepted:
                raise ValueError(prepared.message)
            self.physical_plan = physical
            self.prepared_program_id = prepared.status.program_id
            self.prepared_settings_revision = settings['revision']
            self.prepared_configuration_revision = physical.configuration_revision
            self.prepared_model_id = physical.model_id
            self.preparation = {'id': uuid.uuid4().hex, 'state': 'ready', 'motion_type': 'CHAIN',
                                'program_id': prepared.status.program_id, 'message': prepared.message}
            return prepared.status.program_id

    async def run_execute(self, program_id, key, token):
        self.guard_owner(token)
        return await self.execute({'program_id': program_id, 'execution_key': key}, token)

    async def prepare(self, data):
        self.guard_owner()
        async with self.lock:
            if self.task and not self.task.done():
                raise MotionConflict('Trwa przygotowanie programu')
            status = await self.status()
            if status['busy']:
                raise MotionConflict('Trwa wykonanie programu')
            plan = await self.planning.get_plan(data.get('plan_id'))
            if plan.get('motion_type') not in ('PTP', 'LIN') or not plan.get('simulation_only'):
                raise ValueError('Wybierz zapisany podgląd PTP lub LIN')
            if plan['motion_type'] == 'LIN':
                lin_limits(plan.get('execution_limits', {}), require_all=True)
            # Ruch izolowany jednej/kilku osi (np. strojenie): pozostałe osie nie są komendowane ani sprawdzane,
            # więc ich cel musi być równy pozycji startowej planu. Tylko PTP (LIN wymaga wszystkich osi TCP).
            mask = 0
            if data.get('participating_axes') is not None:
                axes = data['participating_axes']
                if (not isinstance(axes, list) or not axes or len(set(axes)) != len(axes)
                        or any(type(a) is not int or not 1 <= a <= 6 for a in axes)):
                    raise ValueError('participating_axes: lista unikalnych osi 1–6')
                if plan['motion_type'] != 'PTP':
                    raise ValueError('Ruch osi wybranych wymaga PTP')
                for j in range(6):
                    if j + 1 not in axes and abs(plan['target_positions'][j] - plan['start_positions'][j]) > 1e-9:
                        raise ValueError(f'J{j + 1} spoza osi ruchu ma inny cel niż pozycja startowa')
                mask = sum(1 << (a - 1) for a in axes)
            code, settings = await self.configuration('system_get')
            if code != 200:
                raise RuntimeError('Brak potwierdzonych ustawień ruchu')
            # Właściciel CAN potwierdza tę politykę ponownie ze swojego abonamentu ROS.
            allow_missing = settings.get('allow_missing_motion') is True
            (_, model), state = self.planning.current(False)
            code, snapshot = await self.configuration('status')
            if code != 200 or not configuration_ready(snapshot):
                raise RuntimeError('Konfiguracja fizycznego robota nie jest uzgodniona')
            if (model != plan['model_id'] or state.get('configuration_revision') != plan['configuration_revision']
                    or snapshot['active']['revision'] != plan['configuration_revision']):
                raise ValueError('Zmienił się model lub konfiguracja; przelicz podgląd')
            seeds, sources = self.planning.reference(model, plan['configuration_revision'], state)
            if not allow_missing and any(source != 'measured' for source in sources):
                raise ValueError('Wymagane sześć pozycji pomiarowych albo jawny tryb niepełnego zestawu')
            for j, source in enumerate(sources):
                if source != 'measured':
                    seeds[j] = math.degrees(plan['start_positions'][j])
            identifier = uuid.uuid4().hex
            self.preparation = {'id': identifier, 'state': 'planning', 'preview_plan_id': plan['id'],
                                'motion_type': plan['motion_type'], 'message': f"Obliczanie fizycznego {plan['motion_type']}"}
            self.task = asyncio.create_task(self.prepare_program(plan, settings, seeds, sources, snapshot, identifier, mask))
            return dict(self.preparation)

    async def release_controller(self):
        if not self.controllers.service_is_ready():
            return  # Tryb standalone: jedyny właściciel CAN to diagnostyka C++.
        listing = await self.call(self.controllers, self.list_type.Request())
        active = [item for item in listing.controller if item.state == 'active' and
                  any(name.startswith('joint_') and name.endswith(('/position', '/velocity')) for name in item.claimed_interfaces)]
        if any(item.name != self.controller for item in active):
            raise MotionConflict('Inny kontroler posiada osie; zwolnij go przed wykonaniem ruchu')
        if active:
            request = self.switch_type.Request(deactivate_controllers=[self.controller], strictness=self.switch_type.Request.STRICT)
            request.timeout.sec = 2
            result = await self.call(self.switch, request)
            if not result.ok:
                raise RuntimeError('Nie potwierdzono dezaktywacji JTC')

    async def prepare_program(self, plan, settings, seeds, sources, snapshot, identifier, mask=0):
        try:
            payload = {'model_id': plan['model_id'], 'configuration_revision': plan['configuration_revision'],
                       'motion_type': plan['motion_type'],
                       'seed_degrees': seeds, 'simulated_start': [source != 'measured' for source in sources],
                       **plan.get('execution_limits', {})}
            # Limity z podglądu są zachowane: nie zwiększamy zatwierdzanego posuwu.
            context = await self.planning.context(payload, True, (200, snapshot))
            context.simulation_only = False
            context.allow_missing_execution = settings.get('allow_missing_motion') is True
            context.simulated_start = payload['simulated_start']
            context.locked = [False] * 6
            if not self.physical.server_is_ready():
                raise RuntimeError('Planner fizycznego PTP/LIN jest niedostępny')
            goal = self.physical_type.Goal(context=context, motion_type=0 if plan['motion_type'] == 'PTP' else 1,
                                          target_positions=plan['target_positions'])
            if plan['motion_type'] == 'LIN':
                limits = lin_limits(plan['execution_limits'], require_all=True)
                goal.tcp_velocity = limits['tcp_velocity_mm_s'] / 1000
                goal.tcp_acceleration = limits['tcp_acceleration_mm_s2'] / 1000
                goal.tcp_angular_velocity = math.radians(limits['tcp_angular_velocity_degrees_s'])
            handle = await ros_result(self.physical.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Planner odrzucił fizyczny kontekst ruchu')
            self.planner_handle = handle
            result = (await ros_result(handle.get_result_async(), 60)).result
            if not result.success:
                raise ValueError(result.message)
            physical = result.plan
            if physical.simulation_only or physical.motion_type != plan['motion_type'] or physical.model_id != plan['model_id'] or physical.configuration_revision != plan['configuration_revision']:
                raise ValueError('Planner zwrócił niezgodny program')
            code, current = await self.configuration('system_get')
            if code != 200 or current.get('revision') != settings['revision']:
                raise ValueError('Ustawienia zmieniły się podczas przygotowania')
            await self.release_controller()
            request = self.prepare_type.Request(plan=physical, settings_revision=settings['revision'], issued_monotonic_ns=time.monotonic_ns(),
                                                participation_mask=mask)
            prepared = await self.call(self.prepare_client, request, 10)
            if not prepared.accepted:
                raise ValueError(prepared.message)
            self.physical_plan = physical
            self.prepared_program_id = prepared.status.program_id
            self.prepared_settings_revision = settings['revision']
            self.prepared_configuration_revision = plan['configuration_revision']
            self.prepared_model_id = plan['model_id']
            self.preparation.update(state='ready', program_id=prepared.status.program_id, message=prepared.message)
        except asyncio.CancelledError:
            self.preparation.update(state='cancelled', message='Przygotowanie anulowane')
            raise
        except Exception as error:
            self.preparation.update(state='failed', message=str(error))
        finally:
            if self.planner_handle is not None:
                self.planner_handle.cancel_goal_async()
                self.planner_handle = None

    async def prepared(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        status = await self.status()
        if not getattr(self, 'physical_plan', None) or status['program_id'] != getattr(self, 'prepared_program_id', None):
            raise ValueError('Brak aktualnego podglądu fizycznego programu; przygotuj go ponownie')
        plan = message_to_ordereddict(self.physical_plan)
        # Osiom pominiętym nie przypisujemy w podglądzie ruchu, którego nie wykonają.
        for j, participates in enumerate(status['participating']):
            if not participates:
                for point in plan['trajectory']['points']:
                    point['positions'][j] = plan['start_positions'][j]
                    point['velocities'][j] = point['accelerations'][j] = 0.
                plan['target_positions'][j] = plan['start_positions'][j]
        return {'program_id': status['program_id'], 'plan': plan}

    async def execute(self, data, token=None):
        self.guard_owner(token)
        async with self.lock:
            identifier, key = data.get('program_id'), data.get('execution_key')
            if not isinstance(identifier, str) or not isinstance(key, str) or not 1 <= len(key) <= 64:
                raise ValueError('Wymagany program_id i klucz idempotencji')
            status = await self.status()
            if status['program_id'] == identifier and status['execution_key'] == key:
                return status
            if status['busy']:
                raise MotionConflict('Trwa inny program')
            if status['state'] != 'ready' or status['program_id'] != identifier or self.preparation.get('state') == 'planning':
                raise ValueError('Program nie jest gotowy; przygotuj go ponownie')
            if getattr(self, 'prepared_program_id', None) != identifier:
                raise ValueError('Przygotuj program ponownie przed pierwszym wykonaniem')
            code, settings = await self.configuration('system_get')
            if code != 200:
                raise RuntimeError('Ustawienia ROS są niedostępne')
            if settings.get('revision') != self.prepared_settings_revision:
                raise ValueError('Ustawienia zmieniły się; przygotuj program ponownie')
            code, snapshot = await self.configuration('status')
            if code != 200:
                raise RuntimeError('Konfiguracja ROS jest niedostępna')
            if not configuration_ready(snapshot, self.prepared_configuration_revision):
                raise ValueError('Konfiguracja nie jest aktualna lub uzgodniona')
            (_, model), measured = self.planning.current(False)
            if model != self.prepared_model_id or measured.get('configuration_revision') != self.prepared_configuration_revision:
                raise ValueError('Model lub stan robota zmienił się; przygotuj program ponownie')
            if not self.executor.server_is_ready():
                raise RuntimeError('Wykonawca C++ jest niedostępny')
            handle = await ros_result(self.executor.send_goal_async(self.execute_type.Goal(program_id=identifier, execution_key=key)))
            if not handle.accepted:
                raise ValueError('Start odrzucony: zmieniona pozycja, ustawienia lub stan napędów')
            return await self.status()

    async def stop(self):
        if self.task and not self.task.done():
            self.task.cancel()
        response = await self.call(self.stop_client, self.stop_type.Request())
        if not response.success:
            raise RuntimeError(response.message)
        return {'stopped': True, 'message': response.message}


def add_motion_routes(app, client):
    from fastapi import HTTPException, Request
    from starlette.responses import JSONResponse
    async def body(request):
        raw = bytearray()
        async for chunk in request.stream():
            raw.extend(chunk)
            if len(raw) > 65536:
                raise HTTPException(413, 'Żądanie wykonania przekracza 64 KiB')
        import json
        try:
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError()
            return value
        except (ValueError, UnicodeDecodeError):
            raise HTTPException(422, 'Wymagany obiekt JSON') from None
    async def call(operation, *args):
        if client is None:
            raise HTTPException(503, 'Wykonawca ruchu jest niedostępny')
        try:
            return JSONResponse(await getattr(client, operation)(*args), headers={'Cache-Control': 'no-store'})
        except MotionConflict as error:
            raise HTTPException(409, str(error)) from None
        except (ValueError, KeyError, TypeError) as error:
            raise HTTPException(422, str(error)) from None
        except (RuntimeError, TimeoutError) as error:
            raise HTTPException(503, str(error)) from None
    @app.post('/api/v1/motion/prepare', status_code=202)
    async def prepare(request: Request):
        result = await call('prepare', await body(request));result.status_code = 202;return result
    @app.post('/api/v1/motion/execute', status_code=202)
    async def execute(request: Request):
        result = await call('execute', await body(request));result.status_code = 202;return result
    @app.get('/api/v1/motion/status')
    async def status():
        return await call('status')
    @app.get('/api/v1/motion/prepared')
    async def prepared():
        return await call('prepared')
    @app.post('/api/v1/motion/stop')
    async def stop():
        return await call('stop')
    @app.get('/api/v1/motion/execution-tuning')
    async def tuning():
        return await call('tuning')
    @app.post('/api/v1/motion/execution-tuning')
    async def set_tuning(request: Request):
        return await call('set_tuning', await body(request))
