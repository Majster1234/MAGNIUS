"""API konfiguracji: wspólny właściciel ROS i ograniczenia dostępu LAN."""
import asyncio
import json
from robot_web_bridge.app import create_app
from robot_web_bridge.status import StatusStore
from test_bridge import scope


async def request(app, method, path, body=None, client='192.168.1.10', origin=None):
    output = []
    spec = scope('http', path, client)
    spec['method'] = method
    if origin:
        spec['headers'].append((b'origin', origin.encode()))
    content = json.dumps(body).encode() if body is not None else b''
    async def receive():
        return {'type': 'http.request', 'body': content, 'more_body': False}
    async def send(event):
        output.append(event)
    await app(spec, receive, send)
    return output[0]['status'], json.loads(b''.join(item.get('body', b'') for item in output))


def test_configuration_endpoints_forward_to_one_ros_owner_and_preserve_policy(tmp_path):
    calls = []
    async def action(operation, **values):
        calls.append((operation, values))
        return (202 if operation == 'apply' else 200), {'operation': operation}
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, configuration_action=action)
    endpoints = [('GET', '/api/v1/configuration', None, 'status'),
        ('GET', '/api/v1/configuration/snapshots', None, 'list'),
        ('GET', '/api/v1/configuration/snapshots/test', None, 'get'),
        ('DELETE', '/api/v1/configuration/snapshots/test', None, 'delete'),
        ('POST', '/api/v1/configuration/snapshots', {'document': {'test': 1}}, 'create'),
        ('POST', '/api/v1/configuration/validate', {'document': {'test': 1}}, 'validate'),
        ('POST', '/api/v1/configuration/apply', {'document': {'test': 1}, 'expected_revision': 'revision-1'}, 'apply'),
        ('POST', '/api/v1/configuration/reconcile', {}, 'reconcile')]
    for method, path, body, operation in endpoints:
        code, response = asyncio.run(request(app, method, path, body))
        assert code in (200, 202) and response['operation'] == operation
        previous = len(calls)
        assert asyncio.run(request(app, method, path, body, client='100.64.0.1'))[0] == 403
        assert asyncio.run(request(app, method, path, body, origin='http://foreign.example'))[0] == 403
        assert len(calls) == previous
    assert calls[6] == ('apply', {'document': {'test': 1}, 'expected_revision': 'revision-1'})
    assert asyncio.run(request(app, 'POST', '/api/v1/configuration/apply', {'document': 'x' * 65536}))[0] == 413
    assert asyncio.run(request(app, 'POST', '/api/v1/configuration/apply', []))[0] == 422


def test_configuration_unavailable_is_explicit(tmp_path):
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path)
    assert asyncio.run(request(app, 'GET', '/api/v1/configuration'))[0] == 503
