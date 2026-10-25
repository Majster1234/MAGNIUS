import asyncio
from concurrent.futures import Future
from types import SimpleNamespace as NS
import pytest
from robot_web_bridge.motion import MotionClient, MotionConflict
from robot_web_bridge.app import create_app
from robot_web_bridge.status import StatusStore
from test_configuration_api import request


def future(value):
    result = Future(); result.set_result(value); return result


def motion_client(allow=False, phase='idle', success=True, revision='rev'):
    c = MotionClient.__new__(MotionClient)
    c.lock = asyncio.Lock(); c.task = None; c.preparation = {'state': 'idle'}; c.planner_handle = None
    plan = dict(id='preview', model_id='model', configuration_revision='rev', simulation_only=True,
                motion_type='PTP', start_positions=[.1]*6, target_positions=[.2]*6,
                execution_limits=dict(preview_velocity_degrees_s=10, preview_acceleration_degrees_s2=50))
    async def configuration(operation):
        if operation == 'system_get':
            return 200, dict(revision='settings', allow_missing_motion=allow)
        return 200, dict(operation=dict(phase=phase, success=success), active=dict(revision=revision))
    async def get_plan(identifier):
        assert identifier == 'preview'; return plan
    async def context(payload, *args):
        c.payload = payload
        return NS(simulation_only=True)
    c.planning = NS(get_plan=get_plan, context=context, current=lambda _: (('xml', 'model'), {'configuration_revision': 'rev'}),
                    reference=lambda *args: ([0.]*6, ['measured']*2+['zero']*4))
    c.configuration = configuration
    c.current_status = dict(program_id='', state='idle', execution_key='', busy=False)
    async def status(): return {**c.current_status, 'preparation': c.preparation}
    c.status = status
    physical = NS(simulation_only=False, motion_type='PTP', model_id='model', configuration_revision='rev')
    c.handle = NS(accepted=True, get_result_async=lambda: future(NS(result=NS(success=True, plan=physical))), cancel_goal_async=lambda: None)
    c.physical = NS(server_is_ready=lambda: True, send_goal_async=lambda goal: future(c.handle))
    c.physical_type = NS(Goal=lambda **kwargs: NS(**kwargs))
    c.prepare_type = NS(Request=lambda **kwargs: NS(**kwargs)); c.prepare_client = object()
    async def call(client, req, timeout):
        assert req.plan is physical and req.settings_revision == 'settings'
        c.current_status.update(program_id='program', state='ready')
        return NS(accepted=True, message='gotowy', status=NS(program_id='program'))
    c.call = call
    async def release(): c.controller_released = True
    c.release_controller = release
    return c


def test_prepare_rejects_missing_axes_until_separate_setting_enabled():
    async def scenario():
        c = motion_client()
        with pytest.raises(ValueError, match='sześć pozycji'):
            await c.prepare({'plan_id': 'preview'})
        assert c.task is None
        c = motion_client(True)
        result = await c.prepare({'plan_id': 'preview'})
        assert result['state'] == 'planning'
        await c.task
        assert c.preparation['state'] == 'ready', c.preparation
        assert c.payload['seed_degrees'][:2] == [0, 0]
        assert c.payload['seed_degrees'][2] == pytest.approx(.1*180/3.141592653589793)
        assert c.payload['preview_velocity_degrees_s'] == 10
        assert c.controller_released
    asyncio.run(scenario())


def test_execute_idempotency_and_no_implicit_queue():
    async def scenario():
        c = motion_client(True)
        c.current_status.update(program_id='program', state='running', execution_key='once', busy=True)
        c.executor = NS(server_is_ready=lambda: (_ for _ in ()).throw(AssertionError('ponowny start')))
        result = await c.execute({'program_id': 'program', 'execution_key': 'once'})
        assert result['busy']
        with pytest.raises(MotionConflict):
            await c.execute({'program_id': 'program', 'execution_key': 'other'})
    asyncio.run(scenario())


def test_motion_routes_preserve_access_control_and_status_codes(tmp_path):
    class Client:
        async def prepare(self, body):
            if body.get('busy'): raise MotionConflict('zajęty')
            return {'state': 'planning'}
        async def execute(self, body): return {'state': 'arming'}
        async def status(self): return {'state': 'running'}
        async def stop(self): return {'stopped': True}
    app = create_app(StatusStore('192.168.1.69',8080,'enp2s0','192.168.1.0/24'), tmp_path, motion_client=Client())
    for method, path in [('POST','prepare'),('POST','execute'),('GET','status'),('POST','stop')]:
        assert asyncio.run(request(app,method,f'/api/v1/motion/{path}',{}))[0] in (200,202)
        assert asyncio.run(request(app,method,f'/api/v1/motion/{path}',{}, client='100.64.0.1'))[0] == 403
    assert asyncio.run(request(app,'POST','/api/v1/motion/prepare',{'busy':True}))[0] == 409
    assert asyncio.run(request(app,'POST','/api/v1/motion/execute',[]))[0] == 422


def test_missing_motion_setting_forwarded_without_changing_preview_setting(tmp_path):
    calls = []
    async def configuration(operation, **values):
        calls.append((operation,values)); return 200, values
    app = create_app(StatusStore('192.168.1.69',8080,'enp2s0','192.168.1.0/24'),tmp_path, configuration_action=configuration)
    assert asyncio.run(request(app,'PUT','/api/v1/configuration/system',{'allow_missing_motion':True}))[0] == 200
    assert calls == [('system_set',{'allow_missing_motion':True})]

def test_prepared_preview_never_animates_skipped_axes(monkeypatch):
    from copy import deepcopy
    import rosidl_runtime_py.convert
    c = motion_client(True); c.physical_plan = object(); c.prepared_program_id = 'program'
    c.current_status.update(program_id='program', participating=[True]+[False]*5)
    points = [dict(positions=[.1]*6, velocities=[.2]*6, accelerations=[.3]*6),
              dict(positions=[.5]*6, velocities=[.2]*6, accelerations=[.3]*6)]
    original = dict(start_positions=[.1]*6, target_positions=[.5]*6, trajectory=dict(points=points))
    monkeypatch.setattr(rosidl_runtime_py.convert, 'message_to_ordereddict', lambda _: deepcopy(original))
    value = asyncio.run(c.prepared())['plan']
    assert value['trajectory']['points'][-1]['positions'][0] == .5
    assert value['trajectory']['points'][-1]['positions'][1:] == [.1]*5
    assert value['trajectory']['points'][-1]['velocities'][1:] == [0]*5
    assert original['trajectory']['points'][-1]['positions'] == [.5]*6


def test_first_execute_rechecks_persistent_revision_before_ros_goal():
    async def scenario():
        c = motion_client(True)
        await c.prepare({'plan_id': 'preview'}); await c.task
        c.prepared_settings_revision = 'old'
        with pytest.raises(ValueError, match='Ustawienia'):
            await c.execute({'program_id': 'program', 'execution_key': 'new'})
        c.prepared_settings_revision = 'settings'
        calls = []
        def send(goal):
            calls.append(goal)
            c.current_status.update(state='arming', busy=True, execution_key=goal.execution_key)
            return future(NS(accepted=True))
        c.executor = NS(server_is_ready=lambda: True, send_goal_async=send)
        c.execute_type = NS(Goal=lambda **kwargs: NS(**kwargs))
        result = await c.execute({'program_id': 'program', 'execution_key': 'new'})
        assert result['busy'] and len(calls) == 1
    asyncio.run(scenario())


@pytest.mark.parametrize('phase', ['idle', 'done'])
def test_confirmed_configuration_allows_prepare_and_first_execute(phase):
    async def scenario():
        c = motion_client(True, phase)
        await c.prepare({'plan_id': 'preview'})
        await c.task
        assert c.preparation['state'] == 'ready'
        goals = []
        c.executor = NS(server_is_ready=lambda: True,
                        send_goal_async=lambda goal: (goals.append(goal) or future(NS(accepted=True))))
        c.execute_type = NS(Goal=lambda **kwargs: NS(**kwargs))
        await c.execute({'program_id': 'program', 'execution_key': 'new'})
        assert len(goals) == 1
    asyncio.run(scenario())


@pytest.mark.parametrize('phase,success', [
    ('starting', None), ('preparing', None), ('model', None), ('committing', None),
    ('rolling_back', None), ('uncertain', None), ('failed', False),
    ('done', False), ('done', None), ('idle', False), ('idle', None)])
def test_unconfirmed_configuration_never_reaches_prepare_or_execute(phase, success):
    async def scenario():
        c = motion_client(True, phase, success)
        with pytest.raises(RuntimeError, match='nie jest uzgodniona'):
            await c.prepare({'plan_id': 'preview'})
        assert c.task is None
        c.current_status.update(program_id='program', state='ready')
        c.prepared_program_id, c.prepared_settings_revision = 'program', 'settings'
        c.prepared_configuration_revision = 'rev'
        with pytest.raises(ValueError, match='uzgodniona'):
            await c.execute({'program_id': 'program', 'execution_key': 'new'})
    asyncio.run(scenario())


@pytest.mark.parametrize('revision', ['changed', '', None])
def test_active_revision_must_match_preview_and_prepared_program(revision):
    async def scenario():
        c = motion_client(True, 'done', True, revision)
        with pytest.raises((RuntimeError, ValueError)):
            await c.prepare({'plan_id': 'preview'})
        assert c.task is None
        c.current_status.update(program_id='program', state='ready')
        c.prepared_program_id, c.prepared_settings_revision = 'program', 'settings'
        c.prepared_configuration_revision = 'rev'
        with pytest.raises(ValueError, match='aktualna lub uzgodniona'):
            await c.execute({'program_id': 'program', 'execution_key': 'new'})
    asyncio.run(scenario())
