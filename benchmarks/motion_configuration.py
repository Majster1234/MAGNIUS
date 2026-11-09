"""Bramki PTP na atrapach usług; bez ROS node, HTTP i CAN."""
import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from benchmarks.harness import main

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_web_bridge'))
from robot_web_bridge.motion import MotionClient
from robot_web_bridge.planning import PlanningClient


class Cycle:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.results = []
        self.motion = MotionClient.__new__(MotionClient)
        c = self.motion
        c.lock, c.task = asyncio.Lock(), None
        self.snapshot = dict(operation=dict(phase='idle', success=True),
                             active=dict(revision='rev', document=dict(values={})))
        async def configuration(operation):
            if operation == 'system_get':
                return 200, dict(revision='settings', allow_missing_motion=True)
            return 200, self.snapshot
        self.planning = PlanningClient.__new__(PlanningClient)
        p = self.planning
        state = dict(fresh=True, configuration_revision='rev', axes=[dict(
            available=True, position_known=True, joint_configured=False) for _ in range(6)])
        p.descriptions = NS(snapshot=lambda: ('<robot/>', 'model'))
        p.manual, p.configuration = NS(snapshot=lambda: state), configuration
        self.data = dict(model_id='model', configuration_revision='rev', seed_degrees=[0.]*6,
                         preview_velocity_degrees_s=10., preview_acceleration_degrees_s2=50.)
        async def plan(_):
            return dict(id='preview', model_id='model', configuration_revision='rev', simulation_only=True,
                        motion_type='PTP', start_positions=[0.]*6)
        p.get_plan, p.reference = plan, lambda *args: ([0.]*6, ['measured']*6)
        c.planning, c.configuration = p, configuration
        async def status():
            return dict(busy=False, state='ready', program_id='program', execution_key='')
        async def prepare(*args):
            pass
        c.status, c.prepare_program = status, prepare
        c.prepared_program_id, c.prepared_settings_revision = 'program', 'settings'
        c.prepared_configuration_revision, c.prepared_model_id = 'rev', 'model'
        # Wykonawca zawsze niedostępny: sprawdzamy tylko bramkę przed wysłaniem celu.
        c.executor = NS(server_is_ready=lambda: False)

    async def run(self):
        results = []
        for phase in ('idle', 'done', 'committing'):
            self.snapshot['operation']['phase'] = phase
            try:
                await self.planning.context(self.data)
                results.append('context-ok')
            except RuntimeError:
                results.append('context-blocked')
            self.motion.preparation = dict(state='idle')
            try:
                await self.motion.prepare(dict(plan_id='preview'))
                await self.motion.task
                results.append('prepare-ok')
            except RuntimeError:
                results.append('prepare-blocked')
            try:
                await self.motion.execute(dict(program_id='program', execution_key='test'))
            except (ValueError, RuntimeError) as error:
                results.append(str(error))
        return results

    def cycle(self, index):
        self.results = self.loop.run_until_complete(self.run())

    def verify(self):
        self.loop.close()
        return (len(self.results) == 9 and self.results[0:2] == ['context-ok', 'prepare-ok']
                and self.results[6:8] == ['context-blocked', 'prepare-blocked'])


if __name__ == '__main__':
    main(Cycle)
