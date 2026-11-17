"""Logika programów: wyrażenia, warunki, wybór, pętle, skoki, zmienne, czekanie i uwagi statyczne."""
import asyncio
import math
import pytest
from robot_web_bridge.motion_chain import ChainBuilder
from robot_web_bridge.program_interpreter import Interpreter, Move, ProgramFailure, Sync, evaluate, flow_problems, moves_of
from robot_web_bridge.program_runner import ProgramRunner
from robot_web_bridge.programs import ProgramError, ProgramStore
from test_program_runner import LIMITS, FakeMotion, FakePlanning, approved, finish, run


def doc(blocks, variables=None):
    return {'name': 'Logika', 'positions': [{'name': n, 'joints_deg': [k * 10, 0, 0, 0, 45, 0]} for k, n in enumerate('ABCD')],
            'variables': variables if variables is not None else [
                {'name': 'i', 'type': 'INT', 'init': 0}, {'name': 'x', 'type': 'REAL', 'init': 0},
                {'name': 'ok', 'type': 'BOOL', 'init': True}, {'name': 'p', 'type': 'POZYCJA', 'init': 'A'},
                {'name': 't', 'type': 'TEKST', 'init': ''}],
            'blocks': blocks}


def ptp(block_id, position, blend=0):
    ref = {'var': position} if position == 'p' else {'position': position}
    return {'id': block_id, 'command': 'move_ptp', 'params': {'position': ref, 'speed': 50, 'blend_mm': blend}}


def tags(blocks, variables=None):
    return [(m.block_id, m.position) for m in moves_of(doc(blocks, variables))]


def test_expressions_precedence_functions_and_errors():
    assert evaluate('(2 + 3) * x >= 10 AND NOT FALSE', {'x': 2}) is True
    assert evaluate('x MOD 3 <> 0 XOR ok', {'x': 4, 'ok': True}) is False
    assert evaluate('-2^2 + SQRT(16) + MAX(1, 7, 3)', {}) == 7
    assert evaluate('not x > 5 or false', {'x': 3}) is True
    assert evaluate("t + 'b' = 'ab'", {'t': 'a'}) is True
    assert evaluate('ROUND(2.5) + LIMIT(0, 120, 100) + SIN(90)', {}) == 104
    for bad in ('__import__("os")', '1 +', 'y + 1', 'x.y', '1/0', '1 < x < 3', 'SQRT(1, 2)', 'NOT 1', "'a' < 1", '', '2 ^ 1000'):
        with pytest.raises(ProgramFailure):
            evaluate(bad, {'x': 1})


def test_if_switch_and_variables():
    blocks = [{'id': 'a', 'command': 'assign', 'params': {'target': {'var': 'i'}, 'expression': '2 * 3 - 4'}},
              {'id': 'if', 'command': 'if', 'params': {'condition': 'i = 2 AND ok'}, 'body': [ptp('t1', 'B')], 'else': [ptp('e1', 'C')]},
              {'id': 's', 'command': 'switch', 'params': {'value': {'var': 'i'}}, 'body': [
                  {'id': 'c1', 'command': 'case', 'params': {'match': 1}, 'body': [ptp('m1', 'A')]},
                  {'id': 'c2', 'command': 'case', 'params': {'match': 2}, 'body': [ptp('m2', 'D')]}], 'else': [ptp('m0', 'C')]},
              {'id': 'tg', 'command': 'logic_toggle', 'params': {'target': {'var': 'ok'}}},
              {'id': 'if2', 'command': 'if', 'params': {'condition': {'var': 'ok'}}, 'body': [ptp('t2', 'B')], 'else': [ptp('e2', 'A')]},
              {'id': 'pv', 'command': 'assign', 'params': {'target': {'var': 'p'}, 'expression': {'position': 'D'}}},
              ptp('pp', 'p')]
    assert tags(blocks) == [('t1', 'B'), ('m2', 'D'), ('e2', 'A'), ('pp', 'D')]
    interpreter = Interpreter(doc(blocks)); list(interpreter.events())
    assert interpreter.env['i'] == 2 and interpreter.env['ok'] is False


def test_math_blocks_types_and_variable_range():
    blocks = [{'id': 'a', 'command': 'assign', 'params': {'target': {'var': 'i'}, 'expression': '7 / 2'}},
              {'id': 'b', 'command': 'increment', 'params': {'target': {'var': 'x'}, 'step': 2.5}},
              {'id': 'c', 'command': 'calc', 'params': {'target': {'var': 'x'}, 'a': {'var': 'x'}, 'op': '^', 'b': 2}},
              {'id': 'd', 'command': 'math_func', 'params': {'target': {'var': 'x'}, 'func': 'SQRT', 'arg': {'var': 'x'}}},
              {'id': 'e', 'command': 'limit', 'params': {'target': {'var': 'i'}, 'min': 0, 'max': 3}},
              {'id': 'f', 'command': 'assign', 'params': {'target': {'var': 't'}, 'expression': 'x * 2'}}]
    interpreter = Interpreter(doc(blocks)); list(interpreter.events())
    assert interpreter.env == {**interpreter.env, 'i': 3, 'x': 2.5, 't': '5'}
    variables = [{'name': 'i', 'type': 'INT', 'init': 0, 'max': 2}]
    with pytest.raises(ProgramFailure) as error:
        moves_of(doc([{'id': 'g', 'command': 'increment', 'params': {'target': {'var': 'i'}, 'step': 3}}], variables))
    assert error.value.block_id == 'g' and 'poza zakresem' in str(error.value)
    with pytest.raises(ProgramFailure) as error:
        moves_of(doc([{'id': 'h', 'command': 'calc', 'params': {'target': {'var': 'x'}, 'a': 1, 'op': '/', 'b': 0}}]))
    assert error.value.block_id == 'h'
    with pytest.raises(ProgramFailure, match='BOOL'):
        moves_of(doc([{'id': 'k', 'command': 'if', 'params': {'condition': 'i + 1'}, 'body': [], 'else': []}]))


def test_loops_break_continue_and_until():
    blocks = [{'id': 'f', 'command': 'for', 'params': {'counter': {'var': 'i'}, 'from': 1, 'to': 5, 'step': 1}, 'body': [
                  {'id': 'sk', 'command': 'if', 'params': {'condition': 'i = 2'}, 'body': [{'id': 'cn', 'command': 'continue', 'params': {}}], 'else': []},
                  {'id': 'br', 'command': 'if', 'params': {'condition': 'i = 4'}, 'body': [{'id': 'bk', 'command': 'break', 'params': {}}], 'else': []},
                  ptp('m', 'B')]},
              {'id': 'r', 'command': 'repeat', 'params': {'count': 2}, 'body': [ptp('rr', 'C')]},
              {'id': 'w', 'command': 'while', 'params': {'condition': 'x < 2'}, 'body': [
                  {'id': 'wi', 'command': 'increment', 'params': {'target': {'var': 'x'}, 'step': 1}}, ptp('ww', 'A')]},
              {'id': 'u', 'command': 'repeat_until', 'params': {'condition': 'x >= 3'}, 'body': [
                  {'id': 'ui', 'command': 'increment', 'params': {'target': {'var': 'x'}, 'step': 1}}, ptp('uu', 'D')]},
              {'id': 'd', 'command': 'for', 'params': {'counter': {'var': 'i'}, 'from': 3, 'to': 1, 'step': -2}, 'body': [ptp('dd', 'A')]}]
    assert [t[0] for t in tags(blocks)] == ['m', 'm', 'rr', 'rr', 'ww', 'ww', 'uu', 'dd', 'dd']
    with pytest.raises(ProgramFailure, match='kroków logiki'):
        moves_of(doc([{'id': 'inf', 'command': 'while', 'params': {'condition': 'TRUE'}, 'body': []}]))
    with pytest.raises(ProgramFailure, match='poza pętlą'):
        moves_of(doc([{'id': 'b', 'command': 'break', 'params': {}}]))


def test_goto_labels_and_stop():
    blocks = [{'id': 'L', 'command': 'label', 'params': {'name': 'Start'}},
              {'id': 'inc', 'command': 'increment', 'params': {'target': {'var': 'i'}, 'step': 1}},
              ptp('m', 'B'),
              {'id': 'loop', 'command': 'repeat', 'params': {'count': 10}, 'body': [
                  {'id': 'g', 'command': 'if', 'params': {'condition': 'i < 3'}, 'body': [{'id': 'j', 'command': 'goto', 'params': {'target': 'Start'}}],
                   'else': [{'id': 'x', 'command': 'stop_program', 'params': {}}]}]},
              ptp('never', 'C')]
    assert [t[0] for t in tags(blocks)] == ['m', 'm', 'm']
    with pytest.raises(ProgramFailure, match='niedostępnej'):
        moves_of(doc([{'id': 'j', 'command': 'goto', 'params': {'target': 'Wnętrze'}},
                      {'id': 'r', 'command': 'repeat', 'params': {'count': 1}, 'body': [{'id': 'L', 'command': 'label', 'params': {'name': 'Wnętrze'}}]}]))


def test_wait_events_split_chains():
    blocks = [ptp('a', 'B', blend=5), {'id': 'w', 'command': 'wait_time', 'params': {'seconds': 2}}, ptp('b', 'C'),
              {'id': 'op', 'command': 'wait_operator', 'params': {'message': 'Sprawdź chwytak'}}]
    events = list(Interpreter(doc(blocks)).events())
    assert [type(e).__name__ for e in events] == ['Move', 'Sync', 'Move', 'Sync'] and events[1].seconds == 2 and events[3].kind == 'operator'
    builder = ChainBuilder(iter(events), LIMITS)
    first, warnings = builder.next_chain()
    assert [m.block_id for m in first] == ['a'] and first[0].blend_mm == 0 and warnings[0]['text'].startswith('Czekanie po ruchu')
    second, _ = builder.next_chain()
    assert [m.block_id for m in second] == ['b'] and [e.block_id for e in builder.leading] == ['w']
    third, _ = builder.next_chain()
    assert third == [] and [e.kind for e in builder.leading] == ['operator'] and not builder.more()


def test_flow_problems_and_switch_structure(tmp_path):
    store = ProgramStore(tmp_path)
    commands = {c['id']: c for c in store.commands()[0]['commands']}
    document = doc([{'id': 'l1', 'command': 'label', 'params': {'name': 'A'}}, {'id': 'l2', 'command': 'label', 'params': {'name': 'A'}},
                    {'id': 'g', 'command': 'goto', 'params': {'target': 'Brak'}}, {'id': 'b', 'command': 'break', 'params': {}},
                    {'id': 'e', 'command': 'if', 'params': {'condition': 'nieznana > 1'}, 'body': [], 'else': []},
                    {'id': 'f', 'command': 'for', 'params': {'counter': {'var': 'x'}, 'from': 1, 'to': 2, 'step': 0}, 'body': []},
                    {'id': 'c', 'command': 'calc', 'params': {'target': {'var': 'x'}, 'a': 1, 'op': '%', 'b': 2}}])
    problems = '\n'.join(flow_problems(document, commands))
    for expected in ('Etykieta „A” występuje 2×', 'nieistniejącej etykiety „Brak”', 'poza pętlą', 'Nieznana zmienna w wyrażeniu: nieznana',
                     'krok pętli nie może być 0', 'wymagany typ INT', '„%” spoza listy'):
        assert expected in problems, expected
    with pytest.raises(ProgramError, match='dozwolone tylko case'):
        store.validate(doc([{'id': 's', 'command': 'switch', 'params': {'value': 1}, 'body': [ptp('m', 'A')], 'else': []}]))
    with pytest.raises(ProgramError, match='tylko wewnątrz switch'):
        store.validate(doc([{'id': 'c', 'command': 'case', 'params': {'match': 1}, 'body': []}]))


def test_runner_check_timeline_and_operator_resume(tmp_path, monkeypatch):
    import robot_web_bridge.program_runner as runner_module
    monkeypatch.setattr(runner_module, 'CHAIN_GAP_S', 2.0)
    store = ProgramStore(tmp_path)
    blocks = [ptp('a', 'B'), {'id': 'w', 'command': 'wait_time', 'params': {'seconds': 0.05}}, ptp('b', 'C'),
              {'id': 'op', 'command': 'wait_operator', 'params': {'message': 'Dalej?'}}, ptp('c', 'D')]
    planning = FakePlanning(); motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    document = approved(store, doc(blocks))

    async def check():
        await runner.start(document['id'], 'check', document['revision'])
        return await finish(runner)
    status = run(check())
    assert status['state'] == 'succeeded', status
    assert [(e['block_id'], e.get('kind')) for e in status['timeline']] == [('a', None), ('w', 'wait'), ('b', None), ('op', 'operator'), ('c', None)]
    wait = status['timeline'][1]
    assert wait['end_s'] - wait['start_s'] == pytest.approx(.05) and status['timeline'][2]['start_s'] == pytest.approx(wait['end_s'] + 2.0)

    async def execute():
        await runner.start(document['id'], 'execute', document['revision'])
        for _ in range(500):
            if runner.status['state'] == 'waiting_operator':
                break
            await asyncio.sleep(.01)
        paused = runner.snapshot()
        await runner.resume()
        return paused, await finish(runner)
    paused, status = run(execute())
    assert paused['state'] == 'waiting_operator' and paused['wait']['block_id'] == 'op' and paused['robot_cursor']['phase'] == 'operator'
    assert status['state'] == 'succeeded', status
    assert motion.prepared == [['a'], ['b'], ['c']]


def rel(block_id, rotation=False, frame='baza', values=(0, 0, 10), speed=20, blend=0):
    names = ('rx', 'ry', 'rz') if rotation else ('dx', 'dy', 'dz')
    return {'id': block_id, 'command': 'rotate_rel' if rotation else 'move_rel',
            'params': {**dict(zip(names, values)), 'frame': frame, 'speed': speed, 'blend_mm': blend}}


def test_relative_moves_events_estimate_and_validation():
    events = list(Interpreter(doc([ptp('a', 'B'), rel('r1', values=(5, 0, -20)), rel('r2', True, 'narzędzie', (0, 0, 90), speed=30)])).events())
    assert [e.joints_deg is None for e in events] == [False, True, True]
    assert events[1].relative == {'frame': 'base', 'translation_mm': [5, 0, -20], 'rotation_deg': [0., 0., 0.]} and events[1].speed == 20
    assert events[2].relative['frame'] == 'tool' and events[2].angular_speed == 30 and events[2].kind == 'LIN'
    assert LIMITS.estimate([0] * 6, events[2]) == pytest.approx(4)
    with pytest.raises(ProgramFailure, match='zerowe'):
        moves_of(doc([rel('z', values=(0, 0, 0))]))
    with pytest.raises(ProgramFailure, match='Układ'):
        moves_of(doc([rel('f', frame='świat')]))
    resolved = Move('r', 'LIN', 'Δ', [0] * 6, 0., 0, [], events[2].relative, 30.)
    segment = LIMITS.segment(resolved)
    assert segment['tcp_angular_velocity'] == pytest.approx(math.radians(30)) and segment['tcp_velocity'] == pytest.approx(.1)
    with pytest.raises(ProgramFailure, match='Obrót'):
        LIMITS.segment(Move('r', 'LIN', 'Δ', [0] * 6, 0., 0, [], events[2].relative, 200.))   # konfiguracja: 90°/s


def test_runner_resolves_relative_targets_from_previous_target(tmp_path):
    store = ProgramStore(tmp_path)
    blocks = [ptp('a', 'B'), rel('r1', values=(10, 0, 20), blend=5), rel('r2', True, 'narzędzie', (0, 0, 45), speed=30),
              {'id': 'w', 'command': 'wait_time', 'params': {'seconds': 0}}, rel('r3', values=(-10, 0, 0))]
    planning = FakePlanning(); runner = ProgramRunner(store, planning, FakeMotion(planning))
    document = approved(store, doc(blocks))

    async def check():
        await runner.start(document['id'], 'check', document['revision'])
        return await finish(runner)
    status = run(check())
    assert status['state'] == 'succeeded', status
    relative = [e for e in planning.events if e[0] == 'relative']
    assert relative == [('relative', [10, 0, 0, 0, 45, 0], 'base'), ('relative', [11, 0, 2, 0, 45, 0], 'tool'),
                        ('relative', [11, 0, 2, 0, 45, 45], 'base')]
    targets = [[round(math.degrees(q), 6) for q in s['target_positions']] for call in planning.calls for s in call[1]]
    assert targets == [[10, 0, 0, 0, 45, 0], [11, 0, 2, 0, 45, 0], [11, 0, 2, 0, 45, 45], [10, 0, 2, 0, 45, 45]]
    assert planning.calls[0][1][2]['tcp_angular_velocity'] == pytest.approx(math.radians(30))
    failing = approved(store, doc([rel('far', frame='narzędzie', values=(2000, 0, 0))]))

    async def check_failing():
        await runner.start(failing['id'], 'check', failing['revision'])
        return await finish(runner)
    status = run(check_failing())
    assert status['state'] == 'failed' and status['error']['block_id'] == 'far' and 'zasięgiem' in status['message']


def test_ik_failure_text_names_joint_at_limit():
    import math
    from robot_web_bridge.planning import ik_failure_text
    lower, upper = [math.radians(v) for v in (-171, -200, -196, -180, -100, -180)], [math.radians(v) for v in (171, 30, 57, 180, 100, 180)]
    stalled = [math.radians(v) for v in (46.6, 30, 9.98, -0.2, 45, 1.7)]
    stalled[1] = upper[1]
    text = ik_failure_text(stalled, lower, upper, [45, 30, 10, 0, 45, 0], 9.1e-5, 2e-4)
    assert 'J2 na górnej granicy 30°' in text and '0.09 mm' in text and 'start J = [45, 30, 10, 0, 45, 0]' in text
    assert 'osobliwość' in ik_failure_text([0.1] * 6, lower, upper, [0] * 6, .01, 0., 'Brak rozwiązania')


RANGES = {**{f'joint.{j}.lower': -170. for j in range(1, 7)}, **{f'joint.{j}.upper': 170. for j in range(1, 7)}, 'joint.2.upper': 30.}


def test_preflight_reports_limits_robot_state_and_drives(tmp_path, monkeypatch):
    from test_program_runner import VALUES
    store = ProgramStore(tmp_path)
    planning = FakePlanning(); motion = FakeMotion(planning); runner = ProgramRunner(store, planning, motion)
    start = planning.program_start

    async def ranged_start():
        data = await start(); data['values'] = {**VALUES, **RANGES}; return data
    planning.program_start = ranged_start
    document = doc([ptp('a', 'B'), ptp('b', 'C')])
    document['positions'][1]['joints_deg'] = [10, 35, 0, 0, 45, 0]      # B: J2 > 30 (używana)
    document['positions'][3]['joints_deg'] = [0, 0, 0, 0, 200, 0]       # D: J5 > 170 (nieużywana)
    created = approved(store, document)
    result = run(runner.preflight(created['id']))
    steps = {step['id']: step for step in result['steps']}
    assert not result['ok'] and steps['program']['state'] == 'ok'
    assert steps['limits']['state'] == 'error' and 'B: J2 = 35° (zakres -170…30°)' in steps['limits']['detail'] and 'D (nieużywana)' in steps['limits']['detail']
    assert [i['name'] for i in steps['limits']['items'] if i['used']] == ['B'] and steps['limits']['items'][0]['block_id'] == 'a'
    assert steps['robot']['state'] == 'ok' and steps['drives']['state'] == 'ok'
    with pytest.raises(ValueError, match='poza zakresem osi'):
        run(runner.start(created['id'], 'execute', created['revision']))

    planning.state = planning.robot_state()
    planning.state['axes'][2]['enabled'] = False
    planning.state['axes'][4]['joint_position_degrees'] = 180.
    planning.state['fault'], planning.state['message'] = True, 'J3 przeciążenie'
    document['positions'][1]['joints_deg'] = [10, 25, 0, 0, 45, 0]
    fixed = approved(store, {**document, 'name': 'Poprawiony'})
    steps = {step['id']: step for step in run(runner.preflight(fixed['id']))['steps']}
    assert steps['limits']['state'] == 'warn'
    assert 'awaria: J3 przeciążenie' in steps['robot']['detail'] and 'J5: bieżąca pozycja 180.00° poza zakresem' in steps['robot']['detail']
    assert steps['drives'] == {**steps['drives'], 'state': 'action', 'disabled': [3]}


def test_check_fails_on_target_outside_joint_range():
    from robot_web_bridge.motion_chain import MotionLimits
    from test_program_runner import VALUES
    limits = MotionLimits.from_configuration({**VALUES, **RANGES})
    with pytest.raises(ProgramFailure, match=r'J2 = 31.00° poza zakresem osi \[-170, 30\]') as error:
        limits.segment(Move('rel', 'LIN', 'Δ(0, 0, 10) mm baza', [0, 31, 0, 0, 0, 0], 20, 0))
    assert error.value.block_id == 'rel'
    assert limits.segment(Move('ok', 'PTP', 'A', [0, 30, 0, 0, 0, 0], 20, 0))['tag'] == 'ok'
