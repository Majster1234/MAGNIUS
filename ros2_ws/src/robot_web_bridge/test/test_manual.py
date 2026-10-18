import asyncio
import json
from types import SimpleNamespace
import pytest
from robot_web_bridge.app import create_app, JogRateLimit
from robot_web_bridge.manual import ManualStore
from robot_web_bridge.status import StatusStore
from test_bridge import scope, http


def state(stamp=10.0):
    axes = [SimpleNamespace(axis=axis, available=True, prepared=True, can_jog=True, can_zero=True,
                motion='zatrzymany', enable_known=True, enabled=True, position_known=True,
                position_degrees=20.0, position_age_seconds=.001, speed_known=True,
                speed_rpm=0., speed_age_seconds=.001, enable_age_seconds=.01) for axis in range(1, 7)]
    return SimpleNamespace(monotonic_ns=int(stamp * 1e9), busy=False, fault=False, active_axis=0,
                           max_rpm=30, operation='idle', message='Gotowy', axes=axes)


def test_manual_freshness_does_not_rejuvenate_measurements():
    now = [10.0]
    store = ManualStore(lambda: now[0]); store.update(state())
    assert store.snapshot()['axes'][0]['can_jog']
    now[0] += .061
    assert store.snapshot()['fresh']
    assert store.snapshot()['axes'][0]['position_known']
    assert store.snapshot()['axes'][0]['can_jog']
    assert store.snapshot()['axes'][0]['position_age_seconds'] == pytest.approx(.062)
    now[0] += .1
    assert store.snapshot()['fresh']
    assert not store.snapshot()['axes'][0]['can_jog']
    now[0] += .1
    assert not store.snapshot()['axes'][0]['position_known']
    assert not store.snapshot()['axes'][0]['available']
    now[0] += .24
    assert not store.snapshot()['fresh']
    assert not store.snapshot()['axes'][0]['enable_known']
    assert store.snapshot()['axes'][0]['motion'] == 'stan nieznany'
    store.update(state(9.0)); assert not store.snapshot()['fresh']


def test_manual_display_never_overrides_controller_rejection_or_fault():
    store = ManualStore(lambda: 10.07)
    message = state(); message.axes[0].can_jog = False
    message.axes[1].available = False; message.axes[1].position_known = False
    store.update(message)
    result = store.snapshot()
    assert result['axes'][0]['available']
    assert not result['axes'][0]['can_jog']
    assert not result['axes'][1]['available']


@pytest.fixture
def manual_app(tmp_path):
    store = StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24')
    monitor = ManualStore(lambda: 10.0); monitor.update(state())
    calls = []
    async def action(operation, **values):
        calls.append((operation, values))
        return True, 'Przyjęte'
    return create_app(store, tmp_path, manual_store=monitor, manual_action=action), calls


async def post(app, path, body):
    output = []
    request = scope('http', path); request['method'] = 'POST'
    request['headers'].append((b'content-type', b'application/json'))
    async def send(event): output.append(event)
    async def receive(): return {'type': 'http.request', 'body': json.dumps(body).encode(), 'more_body': False}
    await app(request, receive, send)
    return output[0]['status']


def test_manual_http_validates_axis_and_enable_type(manual_app):
    app, calls = manual_app
    assert asyncio.run(http(app, '/api/v1/manual/status'))[0] == 200
    assert asyncio.run(post(app, '/api/v1/manual/axes/4/enable', {'enabled': True})) == 202
    assert asyncio.run(post(app, '/api/v1/manual/axes/4/enable', {'enabled': False})) == 202
    assert calls[-1] == ('enable', {'axis': 4, 'enabled': False})
    assert asyncio.run(post(app, '/api/v1/manual/axes/0/enable', {'enabled': True})) == 422
    assert asyncio.run(post(app, '/api/v1/manual/axes/1/enable', {'enabled': 'true'})) == 422
    assert asyncio.run(post(app, '/api/v1/manual/axes/2/zero', {})) == 202


@pytest.mark.parametrize('path', ['/api/v1/manual/status', '/api/v1/manual/axes/1/enable'])
def test_manual_preserves_lan_policy(manual_app, path):
    app, calls = manual_app
    assert asyncio.run(http(app, path, '100.64.0.1'))[0] == 403
    assert asyncio.run(http(app, path, headers=[(b'origin', b'http://foreign.example')]))[0] == 403
    assert calls == []


def test_manual_ws_disconnect_stops_own_session_and_publishes_20hz(manual_app):
    app, calls = manual_app
    async def scenario():
        events = []
        incoming = asyncio.Queue()
        await incoming.put({'type': 'websocket.connect'})
        await incoming.put({'type': 'websocket.receive', 'text': json.dumps({'type': 'trace', 'event': 'state_received', 'client_perf_ms': 1})})
        await incoming.put({'type': 'websocket.receive', 'text': json.dumps({'action': 'start', 'axis': 4, 'direction': 1, 'rpm': 30, 'sequence': 1})})
        async def send(event):
            events.append(event)
            if event['type'] == 'websocket.send':
                data = json.loads(event['text'])
                if data['type'] == 'ack':
                    await asyncio.sleep(.12)
                    await incoming.put({'type': 'websocket.disconnect', 'code': 1000})
        await asyncio.wait_for(app(scope('websocket', '/api/v1/manual/ws'), incoming.get, send), 1.)
        states = [json.loads(event['text']) for event in events if event['type'] == 'websocket.send' and json.loads(event['text'])['type'] == 'state']
        assert len(states) >= 2
    asyncio.run(scenario())
    assert calls[0][1]['action'] == 0
    assert calls[-1][1]['action'] == 2
    assert calls[-1][1]['session'] == calls[0][1]['session']
    assert calls[-1][1]['sequence'] > calls[0][1]['sequence']


def test_manual_ws_rejects_outside_subnet(manual_app):
    app, calls = manual_app
    async def scenario():
        events = []
        async def send(event): events.append(event)
        async def receive(): return {'type': 'websocket.connect'}
        await app(scope('websocket', '/api/v1/manual/ws', '10.0.0.1'), receive, send)
        assert events == [{'type': 'websocket.close', 'code': 1008}]
    asyncio.run(scenario()); assert calls == []


def test_manual_ws_accepts_heartbeat_burst_after_network_jitter(manual_app):
    app, calls = manual_app
    async def scenario():
        incoming = asyncio.Queue(); events = []
        await incoming.put({'type': 'websocket.connect'})
        for sequence, action in enumerate(('start', 'keepalive', 'keepalive', 'stop'), 1):
            await incoming.put({'type': 'websocket.receive', 'text': json.dumps(
                dict(action=action, axis=1, sequence=sequence, direction=1, rpm=30))})
        await incoming.put({'type': 'websocket.disconnect', 'code': 1000})
        async def send(event): events.append(event)
        await asyncio.wait_for(app(scope('websocket', '/api/v1/manual/ws'), incoming.get, send), 1.)
        acks = [json.loads(e['text']) for e in events if e['type'] == 'websocket.send' and json.loads(e['text'])['type'] == 'ack']
        assert [ack['sequence'] for ack in acks] == [1, 2, 3, 4]
        assert all(ack['accepted'] for ack in acks)
    asyncio.run(scenario())
    assert [values['action'] for operation, values in calls] == [0, 1, 1, 2]


def test_jog_rate_limit_bounds_burst_and_sustained_rate():
    limit = JogRateLimit(); now = limit.previous
    assert all(limit.allow(now) for _ in range(4))
    assert not limit.allow(now)
    assert not limit.allow(now + .04)
    assert limit.allow(now + .05)
    for i in range(2, 200):
        assert limit.allow(now + i * .05)


def test_joint_jog_converts_degrees_to_si_and_disconnect_uses_joint_stop(manual_app):
    app, calls = manual_app
    async def scenario():
        incoming = asyncio.Queue()
        await incoming.put({'type': 'websocket.connect'})
        await incoming.put({'type': 'websocket.receive', 'text': json.dumps(dict(
            action='start', axis=2, sequence=1, direction=-1, space='joint', velocity_degrees_s=30., revision='revision-1'))})
        async def send(event):
            if event['type'] == 'websocket.send' and json.loads(event['text'])['type'] == 'ack':
                await incoming.put({'type': 'websocket.disconnect', 'code': 1000})
        await asyncio.wait_for(app(scope('websocket', '/api/v1/manual/ws'), incoming.get, send), 1.)
    asyncio.run(scenario())
    import math
    assert calls[0][0] == calls[-1][0] == 'jog_joint'
    assert calls[0][1]['velocity_rad_s'] == pytest.approx(math.pi/6)
    assert calls[0][1]['revision'] == 'revision-1'
    assert calls[-1][1]['action'] == 2 and calls[-1][1]['velocity_rad_s'] == 0


def test_joint_position_freshness_and_units():
    message = state(); message.configuration_revision = 'revision-1'
    axis = message.axes[0]
    axis.joint_configured = axis.calibration_valid = axis.joint_position_known = True
    axis.joint_position_rad = -.5; axis.joint_lower_rad = -1.; axis.joint_upper_rad = 1.
    axis.jog_min_velocity_rad_s = .01; axis.jog_max_velocity_rad_s = .1
    now = [10.0]; store = ManualStore(lambda: now[0]); store.update(message)
    result = store.snapshot()
    assert result['configuration_revision'] == 'revision-1'
    import math
    assert result['axes'][0]['joint_position_degrees'] == pytest.approx(math.degrees(-.5))
    now[0] += .3
    assert not store.snapshot()['axes'][0]['joint_position_known']


def test_manual_ws_flood_still_stops_owner(manual_app):
    app, calls = manual_app
    async def scenario():
        incoming = asyncio.Queue()
        await incoming.put({'type': 'websocket.connect'})
        for sequence in range(1, 20):
            await incoming.put({'type': 'websocket.receive', 'text': json.dumps(
                dict(action='start' if sequence == 1 else 'keepalive', axis=1, sequence=sequence, direction=1, rpm=30))})
        async def send(event): pass
        await asyncio.wait_for(app(scope('websocket', '/api/v1/manual/ws'), incoming.get, send), 1.)
    asyncio.run(scenario())
    assert [values['action'] for operation, values in calls] == [0, 1, 1, 1, 2]
