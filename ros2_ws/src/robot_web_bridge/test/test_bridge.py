import asyncio
import json
import socket
import threading
import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest
from starlette.websockets import WebSocketDisconnect

from robot_web_bridge.app import create_app
from robot_web_bridge.network import lan_address
from robot_web_bridge.status import CanStatusStore, StatusStore


@pytest.fixture
def store():
    return StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24')


@pytest.fixture
def app(store, tmp_path):
    (tmp_path / 'index.html').write_text('<html lang="pl">Panel robota</html>')
    return create_app(store, tmp_path)


def message():
    return DiagnosticArray(status=[DiagnosticStatus(
        name='can', level=bytes([0]), message='CAN aktywny', values=[
            KeyValue(key='tx_packets', value='0'),
            KeyValue(key='sampled_at', value='"2026-10-18T12:00:00+00:00"')])])


def scope(kind, path, client='192.168.1.20', headers=()):
    return {'type': kind, 'asgi': {'version': '3.0'}, 'http_version': '1.1',
            'scheme': 'http' if kind == 'http' else 'ws', 'method': 'GET',
            'path': path, 'raw_path': path.encode(), 'query_string': b'', 'root_path': '',
            'server': ('192.168.1.69', 8080), 'client': (client, 45000),
            'headers': [(b'host', b'192.168.1.69:8080'), *headers],
            'subprotocols': []}


def open_socket():
    connected = False
    async def receive():
        nonlocal connected
        if not connected:
            connected = True
            return {'type': 'websocket.connect'}
        await asyncio.Future()
    return receive


async def http(app, path, client='192.168.1.20', headers=()):
    output = []
    async def send(event):
        output.append(event)
    async def receive():
        return {'type': 'http.request', 'body': b'', 'more_body': False}
    await app(scope('http', path, client, headers), receive, send)
    status = next(event['status'] for event in output if event['type'] == 'http.response.start')
    body = b''.join(event.get('body', b'') for event in output)
    return status, body


def test_liveness_without_ros(app):
    assert asyncio.run(http(app, '/healthz')) == (200, b'{"status":"ok"}')
    status, body = asyncio.run(http(app, '/api/v1/status'))
    assert status == 200
    assert json.loads(body)['components']['ros']['state'] == 'stale'


@pytest.mark.parametrize('client', ['100.64.0.10', '10.0.0.1', '192.168.2.3', '::1',
                                   '::ffff:192.168.1.20'])
@pytest.mark.parametrize('path', ['/healthz', '/api/v1/status', '/'])
def test_http_rejects_outside_lan(app, client, path):
    assert asyncio.run(http(app, path, client, [(b'x-forwarded-for', b'192.168.1.20')]))[0] == 403


def test_rejects_cross_origin(app):
    assert asyncio.run(http(app, '/', headers=[(b'origin', b'http://foreign.example')]))[0] == 403
    assert asyncio.run(http(app, '/', headers=[(b'host', b'foreign.example')]))[0] == 403


def test_serves_local_panel_and_read_only_api(app):
    assert b'Panel robota' in asyncio.run(http(app, '/'))[1]
    assert asyncio.run(http(app, '/api/v1/drive'))[0] == 404


def test_static_files_cannot_escape_directory(app, tmp_path):
    sibling = tmp_path.parent / (tmp_path.name + '-outside')
    sibling.mkdir()
    (sibling / 'secret.txt').write_text('outside')
    path = f'/../{sibling.name}/secret.txt'
    assert asyncio.run(http(app, path))[0] == 404


def test_monitor_loss_and_recovery(store):
    clock = [10.0]
    store.clock = lambda: clock[0]
    store.update(message())
    assert store.snapshot()['fresh']
    clock[0] += 2.999
    assert store.snapshot()['components']['can']['state'] == 'ok'
    clock[0] += 0.001
    stale = store.snapshot()
    assert not stale['fresh']
    assert stale['components']['can']['state'] == 'stale'
    assert stale['components']['can']['values']['tx_packets'] == 0
    store.update(message())
    assert store.snapshot()['components']['ros']['state'] == 'ok'


def test_snapshot_does_not_mutate_shared_values(store):
    store.update(message())
    store.snapshot()['components']['can']['values']['tx_packets'] = 100
    assert store.snapshot()['components']['can']['values']['tx_packets'] == 0


def test_can_snapshot_keeps_only_detected_devices_and_stales_independently():
    can_store = CanStatusStore('can0')
    data = DiagnosticArray(status=[
        DiagnosticStatus(name='can', level=bytes([0]), message='AKTYWNA', values=[
            KeyValue(key='bitrate', value='500000'), KeyValue(key='device_count', value='1')]),
        DiagnosticStatus(name='scan', level=bytes([0]), message='Skanowanie', values=[
            KeyValue(key='scanning', value='true'), KeyValue(key='checked', value='3')]),
        DiagnosticStatus(name='device/3', level=bytes([0]), message='Odpowiada', values=[
            KeyValue(key='address', value='3'), KeyValue(key='enable', value='true'),
            KeyValue(key='model', value='"MKS SERVO42D CAN"')]),
    ])
    can_store.update(data)
    snapshot = can_store.snapshot()
    assert snapshot['fresh'] and snapshot['devices'][0]['id'] == 3
    assert snapshot['devices'][0]['values']['enable'] is True
    can_store.received -= 3
    assert can_store.snapshot()['summary']['state'] == 'stale'


def test_can_routes_are_lan_protected(app):
    assert asyncio.run(http(app, '/api/v1/can/status'))[0] == 503
    assert asyncio.run(http(app, '/api/v1/can/status', '100.64.0.10'))[0] == 403


@pytest.mark.parametrize('client,headers', [
    ('100.64.0.10', []),
    ('192.168.1.20', [(b'origin', b'http://foreign.example')]),
])
def test_websocket_denied_before_accept(app, client, headers):
    output = []
    async def send(event):
        output.append(event)
    async def receive():
        raise AssertionError('Niedozwolony klient nie powinien rozpocząć sesji')
    asyncio.run(app(scope('websocket', '/api/v1/ws', client, headers), receive, send))
    assert output == [{'type': 'websocket.close', 'code': 1008}]


def test_five_websockets_share_latest_snapshot(app, store):
    store.update(message())
    received = []
    async def client():
        async def send(event):
            if event['type'] == 'websocket.send':
                received.append(json.loads(event['text']))
                raise WebSocketDisconnect()
        await app(scope('websocket', '/api/v1/ws'), open_socket(), send)
    async def run():
        await asyncio.gather(*(client() for _ in range(5)))
    asyncio.run(run())
    assert len(received) == 5
    assert {item['updated_at'] for item in received} == {store.updated_at}


def test_slow_websocket_is_closed(app):
    output = []
    async def send(event):
        if event['type'] == 'websocket.send':
            await asyncio.sleep(2)
        else:
            output.append(event)
    started = time.monotonic()
    asyncio.run(app(scope('websocket', '/api/v1/ws'), open_socket(), send))
    assert time.monotonic() - started >= 1.0
    assert output[-1]['type'] == 'websocket.close'


def test_network_discovery_fails_closed():
    addrs = {'enp2s0': [SimpleNamespace(family=socket.AF_INET, address='192.168.1.69')],
             'tailscale0': [SimpleNamespace(family=socket.AF_INET, address='100.64.0.10')]}
    assert lan_address('enp2s0', '192.168.1.0/24', addrs) == '192.168.1.69'
    with pytest.raises(RuntimeError):
        lan_address('missing', '192.168.1.0/24', addrs)
    with pytest.raises(RuntimeError):
        lan_address('tailscale0', '192.168.1.0/24', addrs)


def test_real_uvicorn_websocket_on_ubuntu(store, tmp_path):
    """Regresja: websockets 9 z Jammy nie obsługuje asyncio w Pythonie 3.10."""
    import uvicorn
    import wsproto
    from wsproto import events

    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    port = listener.getsockname()[1]
    local = StatusStore('127.0.0.1', port, 'lo', '127.0.0.0/8')
    local.update(message())
    server = uvicorn.Server(uvicorn.Config(
        create_app(local, tmp_path), ws='wsproto', lifespan='off', log_level='error',
        proxy_headers=False))
    thread = threading.Thread(target=lambda: server.run(sockets=[listener]), daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.01)
        assert server.started
        with socket.create_connection(('127.0.0.1', port), timeout=3) as client:
            protocol = wsproto.WSConnection(wsproto.ConnectionType.CLIENT)
            client.sendall(protocol.send(events.Request(
                host=f'127.0.0.1:{port}', target='/api/v1/ws')))
            received = []
            while not received:
                protocol.receive_data(client.recv(65536))
                for event in protocol.events():
                    if isinstance(event, events.TextMessage):
                        received.append(json.loads(event.data))
            assert received[0]['components']['ros']['state'] == 'ok'
            assert received[0]['components']['can']['values']['tx_packets'] == 0
            client.sendall(protocol.send(events.CloseConnection(code=1000)))
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        listener.close()
        assert not thread.is_alive()
