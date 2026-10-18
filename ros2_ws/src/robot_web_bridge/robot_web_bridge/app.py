"""Diagnostyka i żądania ręcznego sterowania, jeden origin strony/API/WS."""

import asyncio
import itertools
from ipaddress import ip_address, ip_network
import mimetypes
import os
import time
import uuid
from pathlib import Path
from urllib.parse import unquote

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Request
from pydantic import BaseModel, StrictBool
from starlette.responses import JSONResponse, Response
from starlette.websockets import WebSocketState
from wsproto.utilities import LocalProtocolError
from .trace import trace, recording

_connections = itertools.count(1)


class PanelFiles:
    """Mały serwer statyczny bez puli wątków starszego Starlette z Jammy."""

    def __init__(self, directory, html=True):
        self.root = Path(directory).resolve()
        self.html = html

    async def __call__(self, scope, receive, send):
        if scope['type'] != 'http' or scope['method'] not in ('GET', 'HEAD'):
            return await Response(status_code=404)(scope, receive, send)
        requested = unquote(scope['path']).lstrip('/')
        target = (self.root / (requested or 'index.html')).resolve()
        try:
            target.relative_to(self.root)
        except ValueError:
            return await Response(status_code=404)(scope, receive, send)
        if target.is_dir() and self.html:
            target = target / 'index.html'
        if not target.is_file():
            return await Response(status_code=404)(scope, receive, send)
        content = b'' if scope['method'] == 'HEAD' else target.read_bytes()
        media_type = mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
        return await Response(content, media_type=media_type)(scope, receive, send)


class LanOnly:
    """Polityka działa przed routingiem, także dla plików strony i WebSocket."""

    def __init__(self, app, subnet, authority):
        self.app = app
        self.network = ip_network(subnet)
        self.authority = authority

    async def __call__(self, scope, receive, send):
        if scope['type'] not in ('http', 'websocket'):
            return await self.app(scope, receive, send)
        headers = dict(scope.get('headers', []))
        try:
            client_ok = ip_address(scope['client'][0]) in self.network
        except (ValueError, TypeError, KeyError):
            client_ok = False
        host_ok = headers.get(b'host', b'').decode('latin1') == self.authority
        origin = headers.get(b'origin')
        origin_ok = origin is None or origin.decode('latin1') == f'http://{self.authority}'
        if not (client_ok and host_ok and origin_ok):
            if scope['type'] == 'websocket':
                await send({'type': 'websocket.close', 'code': 1008})
            else:
                response = JSONResponse({'detail': 'Dostęp wyłącznie z dozwolonej sieci LAN'},
                                        status_code=403)
                await response(scope, receive, send)
            return
        await self.app(scope, receive, send)


class EnableBody(BaseModel):
    enabled: StrictBool


class JogRateLimit:
    """20 komend/s z małą tolerancją zgrupowania pakietów przez sieć."""

    def __init__(self):
        self.tokens = 4.0
        self.previous = time.monotonic()

    def allow(self, now):
        self.tokens = min(4.0, self.tokens + max(0., now - self.previous) * 20.)
        self.previous = now
        if self.tokens + 1e-9 < 1.:
            return False
        self.tokens = max(0., self.tokens - 1.)
        return True


def create_app(store, static_dir, can_store=None, can_action=None, manual_store=None, manual_action=None, description_store=None, configuration_action=None):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LanOnly, subnet=store.network['subnet'],
                       authority=f"{store.network['address']}:{store.network['port']}")

    @app.get('/healthz')
    async def health():
        return JSONResponse({'status': 'ok'}, headers={'Cache-Control': 'no-store'})

    @app.get('/api/v1/status')
    async def status():
        return JSONResponse(store.snapshot(), headers={'Cache-Control': 'no-store'})

    async def configuration_call(operation, **values):
        if configuration_action is None:
            raise HTTPException(503, 'Węzeł konfiguracji ROS jest niedostępny')
        code, data = await configuration_action(operation, **values)
        return JSONResponse(data, status_code=code, headers={'Cache-Control': 'no-store'})

    async def configuration_body(request):
        import json
        content = bytearray()
        async for chunk in request.stream():
            if len(content) + len(chunk) > 65536:
                raise HTTPException(413, 'Konfiguracja przekracza 64 KiB')
            content.extend(chunk)
        try:
            data = json.loads(content)
            if not isinstance(data, dict):
                raise ValueError()
            return data
        except (ValueError, UnicodeDecodeError):
            raise HTTPException(422, 'Nieprawidłowy JSON konfiguracji') from None

    @app.get('/api/v1/configuration')
    async def configuration_status():
        return await configuration_call('status')

    @app.get('/api/v1/configuration/snapshots')
    async def configuration_list(cursor: str = ''):
        return await configuration_call('list', cursor=cursor)

    @app.get('/api/v1/configuration/snapshots/{identifier}')
    async def configuration_get(identifier: str):
        return await configuration_call('get', id=identifier)

    @app.delete('/api/v1/configuration/snapshots/{identifier}')
    async def configuration_delete(identifier: str):
        return await configuration_call('delete', id=identifier)

    @app.post('/api/v1/configuration/snapshots')
    async def configuration_create(request: Request):
        data = await configuration_body(request)
        return await configuration_call('create', document=data.get('document'))

    @app.post('/api/v1/configuration/validate')
    async def configuration_validate(request: Request):
        data = await configuration_body(request)
        return await configuration_call('validate', document=data.get('document'))

    @app.post('/api/v1/configuration/apply')
    async def configuration_apply(request: Request):
        data = await configuration_body(request)
        return await configuration_call('apply', document=data.get('document'), expected_revision=data.get('expected_revision'))

    @app.post('/api/v1/configuration/reconcile')
    async def configuration_reconcile():
        return await configuration_call('reconcile')

    @app.get('/api/v1/robot/description')
    async def robot_description():
        model = description_store.snapshot() if description_store else None
        if model is None:
            raise HTTPException(status_code=503, detail='Oczekiwanie na opis modelu z ROS')
        return Response(model[0], media_type='application/xml',
                        headers={'ETag': f'"{model[1]}"', 'Cache-Control': 'no-cache'})

    @app.get('/api/v1/can/status')
    async def can_status():
        if can_store is None:
            raise HTTPException(status_code=503, detail='Diagnostyka CAN nie jest uruchomiona')
        return JSONResponse(can_store.snapshot(), headers={'Cache-Control': 'no-store'})

    def run_can_action(action):
        if can_action is None:
            raise HTTPException(status_code=503, detail='Diagnostyka CAN nie jest uruchomiona')
        accepted, message = can_action(action)
        if not accepted:
            raise HTTPException(status_code=409, detail=message)
        return JSONResponse({'accepted': True, 'message': message}, status_code=202,
                            headers={'Cache-Control': 'no-store'})

    @app.post('/api/v1/can/scan')
    async def scan():
        return run_can_action('scan')

    @app.post('/api/v1/can/reset')
    async def reset():
        return run_can_action('reset')

    def require_manual():
        if manual_store is None or manual_action is None:
            raise HTTPException(status_code=503, detail='Moduł sterowania ręcznego jest niedostępny')

    @app.get('/api/v1/manual/status')
    async def manual_status():
        require_manual()
        return JSONResponse(manual_store.snapshot(), headers={'Cache-Control': 'no-store'})

    async def motor_action(operation, axis, **values):
        require_manual()
        if not 1 <= axis <= 6:
            raise HTTPException(status_code=422, detail='Oś musi należeć do zakresu 1–6')
        accepted, message = await manual_action(operation, axis=axis, **values)
        if not accepted:
            raise HTTPException(status_code=409, detail=message)
        return JSONResponse({'accepted': True, 'message': message}, status_code=202,
                            headers={'Cache-Control': 'no-store'})

    @app.post('/api/v1/manual/axes/{axis}/enable')
    async def motor_enable(axis: int, body: EnableBody):
        return await motor_action('enable', axis, enabled=body.enabled)

    @app.post('/api/v1/manual/axes/{axis}/zero')
    async def motor_zero(axis: int):
        return await motor_action('zero', axis)

    @app.websocket('/api/v1/manual/ws')
    async def manual_stream(websocket: WebSocket):
        if manual_store is None or manual_action is None:
            await websocket.close(code=1013)
            return
        await websocket.accept()
        session = uuid.uuid4().hex
        connection = next(_connections)
        trace('ws_open', connection=connection)
        last_sequence = 0
        active_axis = 0
        active_space = 'motor'
        active_revision = ''
        send_lock = asyncio.Lock()
        stopped = False
        close_reason = 'completed'
        close_code = None

        async def send(body):
            async with send_lock:
                await asyncio.wait_for(websocket.send_json(body), timeout=.1)

        async def publish():
            while not stopped:
                state = manual_store.snapshot()
                begin = time.monotonic_ns()
                await send({'type': 'state', 'state': state})
                trace('ws_state_sent', connection=connection, duration_ns=time.monotonic_ns() - begin, state=state)
                await asyncio.sleep(.05)

        async def commands():
            nonlocal last_sequence, active_axis, active_space, active_revision, close_reason, close_code
            rate_limit = JogRateLimit()
            previous_trace = 0.0
            while not stopped:
                event = await websocket.receive()
                if event['type'] == 'websocket.disconnect':
                    close_reason = 'peer_disconnect'
                    close_code = event.get('code')
                    return
                raw = event.get('text', '')
                if len(raw) > 1024:
                    close_reason = 'message_too_large'
                    return
                import json
                try:
                    cmd = json.loads(raw)
                    if not isinstance(cmd, dict):
                        raise ValueError()
                    if cmd.get('type') == 'trace':
                        # Tylko liczby/ustalone nazwy, osobny limit; nie są to komendy napędów.
                        now = time.monotonic()
                        if now - previous_trace < .05:
                            continue
                        previous_trace = now
                        if recording() and cmd.get('event') in ('state_received', 'state_gap', 'client_stop', 'visibility'):
                            fields = {key: cmd[key] for key in ('client_epoch_ms', 'client_perf_ms', 'gap_ms', 'axis', 'reason')
                                      if type(cmd.get(key)) in (int, float) and abs(cmd[key]) < 1e16}
                            stamp = cmd.get('state_monotonic_ns')
                            if isinstance(stamp, str) and stamp.isdigit() and len(stamp) <= 20:
                                fields['state_monotonic_ns'] = stamp
                            trace('browser_' + cmd['event'], connection=connection, **fields)
                        continue
                    action, axis, sequence = cmd['action'], cmd['axis'], cmd['sequence']
                    direction, rpm = cmd.get('direction', 0), cmd.get('rpm', 0)
                    space = cmd.get('space', 'motor')
                    velocity = cmd.get('velocity_degrees_s', 0)
                    revision = cmd.get('revision', '')
                    if action not in ('start', 'keepalive', 'stop') or type(axis) is not int or not 1 <= axis <= 6 or type(sequence) is not int or not last_sequence < sequence < 2**53:
                        raise ValueError()
                    if type(direction) is not int or direction not in (-1, 0, 1) or type(rpm) is not int or not 0 <= rpm <= 3000:
                        raise ValueError()
                    import math
                    if space not in ('motor', 'joint') or (space == 'joint' and (
                            type(velocity) not in (int, float) or not math.isfinite(velocity) or not 0 <= velocity <= 1e9 or
                            not isinstance(revision, str) or not 0 < len(revision) <= 64)):
                        raise ValueError()
                    now = time.monotonic()
                    if action != 'stop' and not rate_limit.allow(now):
                        close_reason = 'command_rate_exceeded'
                        trace('ws_command_rejected', connection=connection, reason=close_reason,
                              action=action, sequence=sequence, axis=axis)
                        return
                except (ValueError, KeyError, TypeError):
                    close_reason = 'invalid_command'
                    trace('ws_command_rejected', connection=connection, reason=close_reason)
                    return
                last_sequence = sequence
                trace('ws_command_received', connection=connection, action=action, axis=axis,
                      sequence=sequence, direction=direction, rpm=rpm)
                if action == 'start' and active_axis and active_axis != axis:
                    await send({'type': 'ack', 'sequence': sequence, 'accepted': False, 'message': 'Zatrzymaj poprzednią oś przed wyborem kolejnej'})
                    continue
                # Zachowaj oś nawet gdy potwierdzenie START zaginie: finally zleci STOP.
                if action == 'start':
                    active_axis = axis
                    active_space, active_revision = space, revision
                values = {'velocity_rad_s': math.radians(velocity), 'revision': revision} if space == 'joint' else {'rpm': rpm}
                accepted, message = await manual_action('jog_joint' if space == 'joint' else 'jog', action={'start': 0, 'keepalive': 1, 'stop': 2}[action],
                    axis=axis, direction=direction, session=session, sequence=sequence, **values)
                if action == 'stop' and accepted:
                    active_axis = 0
                await send({'type': 'ack', 'sequence': sequence, 'accepted': accepted, 'message': message})

        tasks = [asyncio.create_task(publish()), asyncio.create_task(commands())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, asyncio.TimeoutError, RuntimeError, OSError, LocalProtocolError) as error:
            close_reason = type(error).__name__
            trace('ws_error', connection=connection, error=type(error).__name__)
        finally:
            trace('ws_close', connection=connection, active_axis=active_axis,
                  reason=close_reason, peer_code=close_code)
            stopped = True
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if active_axis:
                try:
                    values = {'velocity_rad_s': 0., 'revision': active_revision} if active_space == 'joint' else {'rpm': 0}
                    await asyncio.wait_for(manual_action('jog_joint' if active_space == 'joint' else 'jog', action=2, axis=active_axis, direction=0,
                        session=session, sequence=last_sequence + 1, **values), timeout=.1)
                except (asyncio.TimeoutError, RuntimeError, OSError):
                    pass
            try:
                await asyncio.wait_for(websocket.close(), timeout=.1)
            except (RuntimeError, OSError, LocalProtocolError):
                pass

    @app.api_route('/api/{path:path}', methods=['GET', 'POST', 'PUT', 'PATCH', 'DELETE'])
    async def missing_api(path):
        return JSONResponse({'detail': 'Nieznany endpoint API'}, status_code=404)

    async def stream_state(websocket, snapshot, interval):
        await websocket.accept()
        stopped = False

        async def publish():
            while not stopped:
                # Jeden snapshot na klienta; nie istnieje kolejka pomiarów.
                await asyncio.wait_for(websocket.send_json(snapshot()), timeout=1.0)
                if stopped:
                    return
                await asyncio.sleep(interval)

        async def disconnect():
            # Odbierz zdarzenie zamknięcia również wtedy, gdy transport ignoruje
            # send po rozłączeniu. v1 nie przyjmuje wiadomości od klienta.
            await websocket.receive()

        tasks = [asyncio.create_task(publish()), asyncio.create_task(disconnect())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, asyncio.TimeoutError, RuntimeError, OSError,
                LocalProtocolError):
            pass
        finally:
            # Python 3.10 może zgubić cancel w wyścigu z zakończeniem wait_for.
            # Flaga kończy pętlę również po równoczesnym send i disconnect.
            stopped = True
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if websocket.client_state != WebSocketState.DISCONNECTED:
                try:
                    await asyncio.wait_for(websocket.close(), timeout=0.25)
                except (RuntimeError, OSError, asyncio.TimeoutError, LocalProtocolError):
                    pass

    @app.websocket('/api/v1/ws')
    async def stream(websocket: WebSocket):
        await stream_state(websocket, store.snapshot, 1.0)

    @app.websocket('/api/v1/can/ws')
    async def can_stream(websocket: WebSocket):
        if can_store is None:
            await websocket.close(code=1013)
            return
        await stream_state(websocket, can_store.snapshot, 0.2)

    app.mount('/', PanelFiles(directory=str(static_dir), html=True), name='panel')
    return app
