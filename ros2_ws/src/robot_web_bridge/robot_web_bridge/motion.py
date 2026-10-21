"""Przygotowanie i zatwierdzanie lokalnych programów. Brak zegara sterowania w HTTP."""
import asyncio
import math
import time
import uuid
from .planning import ros_result


class MotionConflict(ValueError):
    pass


class MotionClient:
    def __init__(self, node, planning, configuration):
        from rclpy.action import ActionClient
        from robot_control_interfaces.action import ComputePhysicalPtp, ExecuteMotionProgram
        from robot_control_interfaces.srv import PrepareMotionProgram, GetMotionProgram
        from std_srvs.srv import Trigger
        from controller_manager_msgs.srv import ListControllers, SwitchController
        self.planning, self.configuration = planning, configuration
        self.physical_type, self.execute_type = ComputePhysicalPtp, ExecuteMotionProgram
        self.prepare_type, self.status_type, self.stop_type = PrepareMotionProgram, GetMotionProgram, Trigger
        self.physical = ActionClient(node, ComputePhysicalPtp, '/robot/planning/compute_physical_ptp')
        self.executor = ActionClient(node, ExecuteMotionProgram, '/robot/motion/execute_program')
        self.prepare_client = node.create_client(PrepareMotionProgram, '/robot/motion/prepare_program')
        self.status_client = node.create_client(GetMotionProgram, '/robot/motion/program_status')
        self.stop_client = node.create_client(Trigger, '/robot/motion/stop')
        self.list_type, self.switch_type = ListControllers, SwitchController
        self.controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
        self.controller = node.declare_parameter('execution_controller', 'joint_trajectory_controller').value
        self.preparation = {'state': 'idle'}
        self.task = None
        self.planner_handle = None
        self.lock = asyncio.Lock()

    async def call(self, client, request, timeout=3):
        if not client.service_is_ready():
            raise RuntimeError('Usługa wykonawcy ROS jest niedostępna')
        return await ros_result(client.call_async(request), timeout)

    async def status(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        response = await self.call(self.status_client, self.status_type.Request())
        return {**message_to_ordereddict(response.status), 'preparation': dict(self.preparation)}

    async def prepare(self, data):
        async with self.lock:
            if self.task and not self.task.done():
                raise MotionConflict('Trwa przygotowanie programu')
            status = await self.status()
            if status['busy']:
                raise MotionConflict('Trwa wykonanie programu')
            plan = await self.planning.get_plan(data.get('plan_id'))
            if plan.get('motion_type') != 'PTP' or not plan.get('simulation_only'):
                raise ValueError('Wybierz zapisany podgląd PTP; fizyczny LIN nie jest jeszcze dostępny')
            code, settings = await self.configuration('system_get')
            if code != 200:
                raise RuntimeError('Brak potwierdzonych ustawień ruchu')
            # Właściciel CAN potwierdza tę politykę ponownie ze swojego abonamentu ROS.
            allow_missing = settings.get('allow_missing_motion') is True
            (_, model), state = self.planning.current(False)
            code, snapshot = await self.configuration('status')
            if code != 200 or snapshot.get('operation', {}).get('phase') != 'idle':
                raise RuntimeError('Konfiguracja fizycznego robota nie jest uzgodniona')
            if model != plan['model_id'] or state.get('configuration_revision') != plan['configuration_revision']:
                raise ValueError('Zmienił się model lub konfiguracja; przelicz podgląd')
            seeds, sources = self.planning.reference(model, plan['configuration_revision'], state)
            if not allow_missing and any(source != 'measured' for source in sources):
                raise ValueError('Wymagane sześć pozycji pomiarowych albo jawny tryb niepełnego zestawu')
            for j, source in enumerate(sources):
                if source != 'measured':
                    seeds[j] = math.degrees(plan['start_positions'][j])
            identifier = uuid.uuid4().hex
            self.preparation = {'id': identifier, 'state': 'planning', 'preview_plan_id': plan['id'], 'message': 'Obliczanie fizycznego PTP'}
            self.task = asyncio.create_task(self.prepare_program(plan, settings, seeds, sources, snapshot, identifier))
            return dict(self.preparation)

    async def release_controller(self):
        if not self.controllers.service_is_ready():
            return  # Tryb standalone: jedyny właściciel CAN to diagnostyka C++.
        listing = await self.call(self.controllers, self.list_type.Request())
        active = [item for item in listing.controller if item.state == 'active' and
                  any(name.startswith('joint_') and name.endswith(('/position', '/velocity')) for name in item.claimed_interfaces)]
        if any(item.name != self.controller for item in active):
            raise MotionConflict('Inny kontroler posiada osie; zwolnij go przed wykonaniem PTP')
        if active:
            request = self.switch_type.Request(deactivate_controllers=[self.controller], strictness=self.switch_type.Request.STRICT)
            request.timeout.sec = 2
            result = await self.call(self.switch, request)
            if not result.ok:
                raise RuntimeError('Nie potwierdzono dezaktywacji JTC')

    async def prepare_program(self, plan, settings, seeds, sources, snapshot, identifier):
        try:
            payload = {'model_id': plan['model_id'], 'configuration_revision': plan['configuration_revision'],
                       'seed_degrees': seeds, 'simulated_start': [source != 'measured' for source in sources],
                       **plan.get('execution_limits', {})}
            # Limity z podglądu są zachowane: nie zwiększamy zatwierdzanego posuwu.
            context = await self.planning.context(payload, True, (200, snapshot))
            context.simulation_only = False
            context.allow_missing_execution = settings.get('allow_missing_motion') is True
            context.simulated_start = payload['simulated_start']
            context.locked = [False] * 6
            if not self.physical.server_is_ready():
                raise RuntimeError('Planner fizycznego PTP jest niedostępny')
            goal = self.physical_type.Goal(context=context, target_positions=plan['target_positions'])
            handle = await ros_result(self.physical.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Planner odrzucił fizyczny kontekst ruchu')
            self.planner_handle = handle
            result = (await ros_result(handle.get_result_async(), 60)).result
            if not result.success:
                raise ValueError(result.message)
            physical = result.plan
            if physical.simulation_only or physical.motion_type != 'PTP' or physical.model_id != plan['model_id'] or physical.configuration_revision != plan['configuration_revision']:
                raise ValueError('Planner zwrócił niezgodny program')
            code, current = await self.configuration('system_get')
            if code != 200 or current.get('revision') != settings['revision']:
                raise ValueError('Ustawienia zmieniły się podczas przygotowania')
            await self.release_controller()
            request = self.prepare_type.Request(plan=physical, settings_revision=settings['revision'], issued_monotonic_ns=time.monotonic_ns())
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

    async def execute(self, data):
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
            if snapshot.get('operation', {}).get('phase') != 'idle' or snapshot.get('active', {}).get('revision') != self.prepared_configuration_revision:
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
