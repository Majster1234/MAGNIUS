"""Przygotowanie LIN i kontrakt statusu; atrapy ROS, bez CAN."""
import asyncio
import math
from types import SimpleNamespace as NS
import pytest
from robot_web_bridge.motion import MotionClient
from test_motion import motion_client, future
from test_planning import feed_client


async def lin_client(allow=True):
    c=motion_client(allow)
    plan=await c.planning.get_plan('preview')
    plan.update(motion_type='LIN',execution_limits=dict(tcp_velocity_mm_s=25.,tcp_acceleration_mm_s2=120.,
                                                     tcp_angular_velocity_degrees_s=7.))
    physical=c.handle.get_result_async().result().result.plan;physical.motion_type='LIN'
    c.goals=[]
    c.physical.send_goal_async=lambda goal: (c.goals.append(goal) or future(c.handle))
    return c,plan,physical


def test_lin_replans_measured_start_preserves_feed_and_never_executes_implicitly():
    async def run():
        c,_,_=await lin_client()
        c.executor=NS(server_is_ready=lambda: (_ for _ in ()).throw(AssertionError('Samoczynny start')))
        result=await c.prepare({'plan_id':'preview'});assert result['motion_type']=='LIN'
        await c.task
        assert c.preparation['state']=='ready',c.preparation
        assert c.payload['motion_type']=='LIN' and c.payload['seed_degrees'][:2]==[0,0]
        assert c.payload['seed_degrees'][2:]==pytest.approx([math.degrees(.1)]*4)
        assert c.goals[0].motion_type==1
        assert c.goals[0].tcp_velocity==pytest.approx(.025)
        assert c.goals[0].tcp_acceleration==pytest.approx(.12)
        assert c.goals[0].tcp_angular_velocity==pytest.approx(math.radians(7))
        assert c.goals[0].context.allow_missing_execution is True
        assert c.goals[0].context.simulation_only is False
        assert c.controller_released
    asyncio.run(run())


@pytest.mark.parametrize('field,value',[('tcp_velocity_mm_s',None),('tcp_acceleration_mm_s2',0),
                                      ('tcp_angular_velocity_degrees_s',float('nan'))])
def test_lin_rejects_missing_or_invalid_saved_limits_before_planning(field,value):
    async def run():
        c,plan,_=await lin_client()
        if value is None: plan['execution_limits'].pop(field)
        else: plan['execution_limits'][field]=value
        with pytest.raises(ValueError): await c.prepare({'plan_id':'preview'})
        assert not c.goals and c.task is None
    asyncio.run(run())


def test_lin_missing_axes_requires_existing_setting_and_allows_measured_full_set():
    async def run():
        c,_,_=await lin_client(False)
        with pytest.raises(ValueError,match='sześć pozycji'): await c.prepare({'plan_id':'preview'})
        c.planning.reference=lambda *args: ([1.]*6,['measured']*6)
        await c.prepare({'plan_id':'preview'});await c.task
        assert c.preparation['state']=='ready' and not any(c.goals[0].context.simulated_start)
    asyncio.run(run())


@pytest.mark.parametrize('field,value',[('motion_type','PTP'),('simulation_only',True),
                                      ('configuration_revision','old'),('model_id','old')])
def test_lin_rejects_mismatched_physical_result_without_preparing_executor(field,value):
    async def run():
        c,_,physical=await lin_client();setattr(physical,field,value)
        await c.prepare({'plan_id':'preview'});await c.task
        assert c.preparation['state']=='failed'
        assert not hasattr(c,'controller_released')
    asyncio.run(run())


def test_http_status_type_belongs_to_matching_prepared_program(monkeypatch):
    import rosidl_runtime_py.convert
    async def run():
        c,_,physical=await lin_client()
        c.status_type=NS(Request=lambda: object());c.status_client=object()
        async def call(*args,**kwargs): return NS(status=NS(program_id='current'))
        c.call=call
        monkeypatch.setattr(rosidl_runtime_py.convert,'message_to_ordereddict',lambda value: vars(value))
        assert (await MotionClient.status(c))['motion_type'] is None
        c.physical_plan=physical;c.prepared_program_id='current'
        assert (await MotionClient.status(c))['motion_type']=='LIN'
        c.prepared_program_id='other'
        assert (await MotionClient.status(c))['motion_type'] is None
    asyncio.run(run())


def test_preview_lin_stores_normalized_limits_including_defaults():
    async def run():
        c=feed_client();captured={}
        async def submit(goal,action,metadata): captured.update(metadata);return goal
        c.submit=submit
        await c.move(dict(model_id='model',configuration_revision='rev',settings_revision='settings',motion_type='LIN',
                          seed_degrees=[0.]*6,target_degrees=[1.]*6,simulated_start=[True]*6,tcp_velocity_mm_s=25))
        assert captured['execution_limits']==dict(tcp_velocity_mm_s=25.,tcp_acceleration_mm_s2=100.,tcp_angular_velocity_degrees_s=10.)
    asyncio.run(run())


def test_physical_lin_uses_real_ros_goal_and_configured_joint_limits():
    from robot_control_interfaces.action import ComputePhysicalMotion
    from rclpy.serialization import serialize_message
    async def run():
        c,plan,_=await lin_client()
        planning=feed_client()
        async def get_plan(_): return plan
        planning.get_plan=get_plan
        planning.reference=c.planning.reference
        original_configuration=c.configuration
        async def configuration(operation):
            return await (planning.configuration(operation) if operation=='status' else original_configuration(operation))
        c.configuration=configuration
        c.planning=planning;c.physical_type=ComputePhysicalMotion
        await c.prepare({'plan_id':'preview'});await c.task
        assert c.preparation['state']=='ready',c.preparation
        goal=c.goals[0]
        assert serialize_message(goal)
        assert list(goal.context.max_velocity)==pytest.approx([math.radians(30)]*6)
        assert list(goal.context.max_acceleration)==pytest.approx([math.radians(100)]*6)
        assert goal.motion_type==ComputePhysicalMotion.Goal.LIN
        assert not goal.context.simulation_only and goal.context.allow_missing_execution
    asyncio.run(run())
