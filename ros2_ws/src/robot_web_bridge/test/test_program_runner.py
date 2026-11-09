"""Wykonawca programów: interpreter, ciągi ruchów (promień pominięcia), kursory i przebieg z atrapami planera/wykonawcy."""
import asyncio
import json
import math
import pytest
from robot_web_bridge.app import create_app
from robot_web_bridge.motion import MotionConflict
from robot_web_bridge.motion_chain import ChainBuilder, MotionLimits, robot_cursor
from robot_web_bridge.program_interpreter import Interpreter, Move, ProgramFailure, Sync, evaluate, moves_of
from robot_web_bridge.program_runner import PlannerBusy, ProgramRunner
from robot_web_bridge.programs import DEFAULT_COMMANDS, ProgramStore
from robot_web_bridge.status import StatusStore
from test_configuration_api import request

VALUES = {**{f'joint.{j}.velocity': 60. for j in range(1, 7)}, **{f'joint.{j}.acceleration': 120. for j in range(1, 7)},
          'joint.3.velocity': 30., 'tcp.max_linear_velocity': 100., 'tcp.linear_acceleration': 500., 'tcp.max_angular_velocity': 90.}
LIMITS = MotionLimits.from_configuration(VALUES)


def program(blocks=None, **extra):
    return {'name': 'Ciąg testowy', 'variables': [{'name': 'v', 'type': 'REAL', 'init': 40}, {'name': 'cel', 'type': 'POZYCJA', 'init': 'B'}],
            'positions': [{'name': 'A', 'joints_deg': [0, 0, 0, 0, 45, 0]}, {'name': 'B', 'joints_deg': [20, 10, 0, 0, 45, 0]},
                          {'name': 'C', 'joints_deg': [20, 30, -10, 0, 45, 0]}],
            'blocks': blocks if blocks is not None else [
                {'id': 'p1', 'command': 'move_ptp', 'params': {'position': {'position': 'A'}, 'speed': 50, 'blend_mm': 0}},
                {'id': 'l1', 'command': 'move_lin', 'params': {'position': {'var': 'cel'}, 'speed': {'var': 'v'}, 'blend_mm': 10}},
                {'id': 'l2', 'command': 'move_lin', 'params': {'position': {'position': 'C'}, 'speed': 20, 'blend_mm': 5}}], **extra}


# ----- interpreter -----
def test_interpreter_resolves_positions_variables_and_skips_disabled():
    document = program()
    document['blocks'].insert(1, {'id': 'off', 'command': 'move_ptp', 'disabled': True, 'params': {'position': {'position': 'C'}, 'speed': 10, 'blend_mm': 0}})
    moves = moves_of(document)
    assert [(m.block_id, m.kind, m.position, m.speed, m.blend_mm) for m in moves] == [
        ('p1', 'PTP', 'A', 50, 0), ('l1', 'LIN', 'B', 40, 10), ('l2', 'LIN', 'C', 20, 5)]
    assert moves[1].joints_deg == [20, 10, 0, 0, 45, 0]
    with pytest.raises(ProgramFailure) as error:
        moves_of(program([{'id': 'x', 'command': 'move_ptp', 'params': {'position': {'position': 'Brak'}, 'speed': 10}}]))
    assert error.value.block_id == 'x'
    with pytest.raises(ProgramFailure):
        moves_of(program([{'id': 'y', 'command': 'nieznany'}]))
    assert evaluate('(2 + 3) * x >= 10 AND NOT FALSE', {'x': 2}) is True
    with pytest.raises(ProgramFailure):
        evaluate('__import__("os")', {})


# ----- limity i ciągi -----
def test_segment_mapping_uses_configuration_limits():
    ptp = LIMITS.segment(Move('a', 'PTP', 'A', [0] * 6, 50, 0))
    assert ptp['max_joint_velocity'][0] == pytest.approx(math.radians(15)) and ptp['tcp_velocity'] == 0   # 50% najwolniejszej (30°/s)
    lin = LIMITS.segment(Move('b', 'LIN', 'B', [0] * 6, 40, 12))
    assert lin['tcp_velocity'] == pytest.approx(.04) and lin['blend_radius'] == pytest.approx(.012)
    assert lin['max_joint_velocity'][2] == pytest.approx(math.radians(30))
    for bad in (Move('c', 'PTP', 'A', [0] * 6, 120, 0), Move('d', 'LIN', 'A', [0] * 6, 150, 0), Move('e', 'LIN', 'A', [0] * 6, 10, 600)):
        with pytest.raises(ProgramFailure) as error:
            LIMITS.segment(bad)
        assert error.value.block_id == bad.block_id


def test_chain_builder_breaks_on_end_sync_and_limits_with_stop():
    moves = [Move(f'm{k}', 'PTP', 'A', [k * 10] + [0] * 5, 100, 5) for k in range(3)]
    chains = ChainBuilder(iter(moves), LIMITS)
    chain, warnings = chains.next_chain([0] * 6)
    assert [m.block_id for m in chain] == ['m0', 'm1', 'm2'] and chain[-1].blend_mm == 0 and chain[0].blend_mm == 5
    assert warnings == [{'block_id': 'm2', 'code': 'chain_end', 'text': 'Ostatni ruch programu — postój w punkcie'}]
    assert moves[2].blend_mm == 5   # zdarzenia interpretera nie są modyfikowane
    chains = ChainBuilder(iter([moves[0], Sync('s', 'czekaj'), moves[1]]), LIMITS)
    first, warnings = chains.next_chain()
    assert [m.block_id for m in first] == ['m0'] and warnings[0]['code'] == 'blend_not_possible'
    second, warnings = chains.next_chain()
    assert [m.block_id for m in second] == ['m1'] and warnings[0]['code'] == 'sync'
    many = ChainBuilder(iter([Move(f'n{k}', 'PTP', 'A', [0] * 6, 50, 3) for k in range(70)]), LIMITS)
    first, warnings = many.next_chain([0] * 6)
    assert len(first) == 64 and warnings[-1]['code'] == 'chain_split'
    many.push_front(first[60:])
    assert [m.block_id for m in many.next_chain()[0]][:5] == ['n60', 'n61', 'n62', 'n63', 'n64']


def test_robot_cursor_moving_blending_stop_and_states():
    timings = [{'tag': 'a', 'start_time': 0, 'end_time': 1, 'transition_end': 1.4, 'stop': False},
               {'tag': 'b', 'start_time': 1.4, 'end_time': 2, 'transition_end': 2.01, 'stop': True},
               {'tag': 'c', 'start_time': 2.01, 'end_time': 3, 'transition_end': 3, 'stop': True}]
    assert robot_cursor(timings, .5)['block_id'] == 'a'
    blend = robot_cursor(timings, 1.1)
    assert blend['phase'] == 'blending' and blend['blend'] == {'from': 'a', 'to': 'b', 'progress': .25} and blend['block_id'] == 'a'
    assert robot_cursor(timings, 1.3)['block_id'] == 'b'
    assert robot_cursor(timings, 2.005) == {'block_id': 'b', 'phase': 'stopped', 'blend': None}
    assert robot_cursor(timings, 2.5)['block_id'] == 'c' and robot_cursor(timings, 9)['phase'] == 'stopped'
    assert robot_cursor(timings, 0, 'arming')['phase'] == 'arming'
    assert robot_cursor(timings, 3, 'settling') == {'block_id': 'c', 'phase': 'settling', 'blend': None}


# ----- atrapy -----
class FakePlanning:
    def __init__(self, truncate=None):
        self.calls, self.truncate, self.busy = [], truncate, 0
        self.events = []

    async def program_start(self):
        return {'model_id': 'm', 'configuration_revision': 'r1', 'snapshot': (200, {}), 'values': VALUES,
                'seeds': [0.] * 6, 'sources': ['measured'] * 6}

    async def chain_context(self, seeds, physical, start):
        return {'seeds': list(seeds), 'physical': physical}

    async def plan_chain(self, context, segments, max_duration, feedback=None):
        if self.busy:
            self.busy -= 1
            raise PlannerBusy('zajęty')
        self.calls.append((context, [dict(s) for s in segments]))
        self.events.append(('plan', context['physical'], [s['tag'] for s in segments]))
        if feedback:
            feedback(.5, 'Czas ruchów')
        count = len(segments) if self.truncate is None else min(self.truncate, len(segments))
        timings, t = [], 0.
        for k, segment in enumerate(segments[:count]):
            blend = segment['blend_radius'] > 0 and k + 1 < count
            timings.append({'segment': k, 'tag': segment['tag'], 'start_time': t, 'end_time': t + 1,
                            'transition_end': t + (1.2 if blend else 1.01), 'stop': not blend,
                            'requested_blend_radius': segment['blend_radius'], 'applied_blend_radius': segment['blend_radius'] if blend else 0})
            t = timings[-1]['transition_end']
        target = [math.degrees(q) for q in segments[count - 1]['target_positions']]
        return {'success': True, 'message': 'ok', 'planned_segments': count, 'warnings': ['0|blend_reduced|Promień ograniczony'],
                'failed_segment': -1, 'plan': {'tags': [s['tag'] for s in segments[:count]], 'target': target}, 'duration': t, 'target_degrees': target, 'timings': timings}


class FakeMotion:
    def __init__(self, planning, fail=False):
        self.planning, self.fail = planning, fail
        self.owner, self.program, self.ticks, self.stopped, self.prepared = None, None, 0, 0, []
        self.position, self.drift = [0.] * 6, 0.

    def claim(self, token):
        if self.owner and self.owner != token:
            raise MotionConflict('zajęte')
        self.owner = token

    def release(self, token):
        if self.owner == token:
            self.owner = None

    def measured_degrees(self):
        return list(self.position)

    async def prepare_chain(self, plan, token):
        assert token == self.owner
        self.prepared.append(plan['tags'])
        self.program = {'id': f'prog{len(self.prepared)}', 'plan': plan}
        return self.program['id']

    async def run_execute(self, program_id, key, token):
        assert token == self.owner and program_id == self.program['id']
        self.ticks = 0
        self.planning.events.append(('execute', program_id))

    async def status(self):
        if self.program is None:
            return {'busy': False, 'state': 'idle', 'program_id': ''}
        self.ticks += 1
        if self.fail and self.ticks > 2:
            return {'busy': False, 'state': 'failed', 'program_id': self.program['id'], 'message': 'awaria', 'elapsed': .3}
        if self.ticks < 2:
            return {'busy': True, 'state': 'arming', 'program_id': self.program['id'], 'elapsed': 0}
        if self.ticks < 5:
            return {'busy': True, 'state': 'running', 'program_id': self.program['id'], 'elapsed': (self.ticks - 1) * .4}
        self.position = [q + self.drift for q in self.program['plan']['target']]
        return {'busy': False, 'state': 'succeeded', 'program_id': self.program['id'], 'elapsed': 2}

    async def stop(self):
        self.stopped += 1


@pytest.fixture
def store(tmp_path):
    return ProgramStore(tmp_path)


def run(coroutine):
    return asyncio.run(coroutine)


async def finish(runner):
    while runner.active():
        await asyncio.sleep(.01)
    return runner.snapshot()


def approved(store, document):
    created = store.create(document)['program']
    return store.approve(created['id'], created['revision'])['program']


def test_default_library_has_ptp_and_lin_with_blend_radius():
    commands = {c['id']: c for c in json.loads(DEFAULT_COMMANDS.read_text())['commands']}
    assert set(commands) == {'move_ptp', 'move_lin'}
    for command in commands.values():
        blend = next(p for p in command['params'] if p['name'] == 'blend_mm')
        assert blend['min'] == 0 and blend['default'] == 0 and blend['unit'] == 'mm'


def test_range_problems_block_approval(store):
    bad = program([{'id': 'b', 'command': 'move_ptp', 'params': {'position': {'position': 'A'}, 'speed': 150, 'blend_mm': -1}},
                   {'id': 'c', 'command': 'move_lin', 'params': {'position': None, 'speed': 10, 'blend_mm': 0}}])
    created = store.create(bad)
    assert any('150' in p and 'maksimum' in p for p in created['problems'])
    assert any('pole nieuzupełnione' in p for p in created['problems'])
    with pytest.raises(Exception):
        store.approve(created['program']['id'], created['program']['revision'])


def test_check_builds_timeline_with_applied_radius(store):
    planning = FakePlanning(); runner = ProgramRunner(store, planning, FakeMotion(planning))
    document = store.create(program())['program']
    async def scenario():
        await runner.start(document['id'], 'check', document['revision'])
        return await finish(runner)
    status = run(scenario())
    assert status['state'] == 'succeeded', status
    assert [(e['block_id'], e['stop'], e['blend_mm_applied']) for e in status['timeline']] == [('p1', True, 0), ('l1', False, 10), ('l2', True, 0)]
    assert {'block_id': 'l2', 'code': 'chain_end', 'text': 'Ostatni ruch programu — postój w punkcie'} in status['warnings']
    assert any(w['code'] == 'blend_reduced' and w['block_id'] == 'p1' for w in status['warnings'])
    assert planning.calls[0][0]['physical'] is False and planning.calls[0][1][-1]['blend_radius'] == 0
    assert status['program_cursor']['state'] == 'done' and runner.last_check is not None


def test_execute_requires_operator_approval_and_matching_revision(store):
    planning = FakePlanning(); runner = ProgramRunner(store, planning, FakeMotion(planning))
    document = store.create(program())['program']
    with pytest.raises(ValueError, match='NIE jest zatwierdzona'):
        run(runner.start(document['id'], 'execute', document['revision']))
    approved_document = store.approve(document['id'], document['revision'])['program']
    with pytest.raises(ValueError):
        run(runner.start(document['id'], 'execute', 'inna-rewizja'))
    assert not planning.calls and approved_document['approval']['revision'] == document['revision']


def test_execute_plans_next_chain_before_current_finishes(store, monkeypatch):
    import robot_web_bridge.motion_chain as chain
    monkeypatch.setattr(chain, 'MAX_CHAIN_SEGMENTS', 2)   # 3 ruchy → dwa ciągi
    planning = FakePlanning(); motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    document = approved(store, program())
    async def scenario():
        await runner.start(document['id'], 'execute', document['revision'])
        return await finish(runner)
    status = run(scenario())
    assert status['state'] == 'succeeded', status
    physical = [e for e in planning.events if e[0] in ('plan', 'execute') and (e[0] == 'execute' or e[1])]
    # ciąg 0 zaplanowany → wykonanie → ciąg 1 planowany w trakcie (przed drugim execute)
    assert [e[0] for e in physical] == ['plan', 'execute', 'plan', 'execute']
    assert motion.prepared == [['p1', 'l1'], ['l2']] and motion.owner is None
    assert any(w['code'] == 'chain_split' and w['block_id'] == 'l1' for w in status['warnings'])
    assert status['program_cursor']['state'] == 'done' and status['chains'] == 2


def test_truncated_plan_pushes_rest_and_replans_on_position_drift(store):
    planning = FakePlanning(truncate=1); motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    document = approved(store, program())
    async def scenario():
        await runner.start(document['id'], 'execute', document['revision'])
        await asyncio.sleep(0)
        motion.drift = 5.     # robot zatrzymał się dalej niż przewidywał plan
        return await finish(runner)
    status = run(scenario())
    assert status['state'] == 'succeeded', status
    assert motion.prepared == [['p1'], ['l1'], ['l2']]
    assert any(w['code'] == 'replanned' for w in status['warnings'])
    assert sum(1 for call in planning.calls if call[0]['physical']) == 5   # 3 ciągi + 2 przeliczenia od pomiaru


def test_executor_failure_and_stop_call_motion_stop(store):
    planning = FakePlanning(); motion = FakeMotion(planning, fail=True); runner = ProgramRunner(store, planning, motion)
    document = approved(store, program())
    async def failing():
        await runner.start(document['id'], 'execute', document['revision'])
        return await finish(runner)
    status = run(failing())
    assert status['state'] == 'failed' and motion.stopped == 1 and status['error']['block_id'] in ('p1', 'l1')
    motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    async def stopping():
        await runner.start(document['id'], 'execute', document['revision'])
        await asyncio.sleep(.05)
        await runner.stop()
        return runner.snapshot()
    status = run(stopping())
    assert status['state'] == 'stopped' and motion.stopped >= 1 and motion.owner is None


def test_revision_change_at_chain_boundary_stops(store, monkeypatch):
    import robot_web_bridge.motion_chain as chain
    monkeypatch.setattr(chain, 'MAX_CHAIN_SEGMENTS', 1)
    planning = FakePlanning(); motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    document = approved(store, program())
    async def scenario():
        await runner.start(document['id'], 'execute', document['revision'])
        await asyncio.sleep(.02)
        store.revoke(document['id'])
        return await finish(runner)
    status = run(scenario())
    assert status['state'] == 'failed' and 'zatwierdzenie' in status['message'] and len(motion.prepared) == 1


def test_planner_busy_is_retried(store):
    planning = FakePlanning(); planning.busy = 2; runner = ProgramRunner(store, planning, FakeMotion(planning))
    document = store.create(program())['program']
    async def scenario():
        await runner.start(document['id'], 'check', document['revision'])
        return await finish(runner)
    assert run(scenario())['state'] == 'succeeded'


def test_routes_and_manual_motion_guard(store, tmp_path):
    planning = FakePlanning(); motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, program_store=store, program_runner=runner)
    document = store.create(program())['program']
    async def scenario():
        code, data = await request(app, 'POST', f'/api/v1/programs/{document["id"]}/run', {'mode': 'execute', 'revision': document['revision']})
        assert code == 422 and 'zatwierdzona' in data['detail']
        code, data = await request(app, 'POST', f'/api/v1/programs/{document["id"]}/run', {'mode': 'check', 'revision': document['revision']})
        assert code == 202 and data['mode'] == 'check'
        await finish(runner)
        code, data = await request(app, 'GET', '/api/v1/program-run')
        assert code == 200 and data['state'] == 'succeeded' and len(data['timeline']) == 3
        code, data = await request(app, 'POST', '/api/v1/program-run/check-document', {'program': program()})
        assert code == 202
        await finish(runner)
        code, _ = await request(app, 'POST', '/api/v1/program-run/stop')
        assert code == 200
    run(scenario())


def test_motion_client_owner_guard():
    from robot_web_bridge.motion import MotionClient
    client = MotionClient.__new__(MotionClient)
    client.owner = None
    client.claim('run')
    with pytest.raises(MotionConflict):
        client.guard_owner()
    with pytest.raises(MotionConflict):
        client.claim('inny')
    client.guard_owner('run'); client.release('run'); client.guard_owner()
