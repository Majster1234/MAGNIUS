"""API tylko do odczytu, wspólne pochodzenie strony i WebSocket."""

import asyncio
from ipaddress import ip_address, ip_network
import mimetypes
import os
from pathlib import Path
from urllib.parse import unquote

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from starlette.responses import JSONResponse, Response
from starlette.websockets import WebSocketState
from wsproto.utilities import LocalProtocolError


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


def create_app(store, static_dir, can_store=None, can_action=None):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LanOnly, subnet=store.network['subnet'],
                       authority=f"{store.network['address']}:{store.network['port']}")

    @app.get('/healthz')
    async def health():
        return JSONResponse({'status': 'ok'}, headers={'Cache-Control': 'no-store'})

    @app.get('/api/v1/status')
    async def status():
        return JSONResponse(store.snapshot(), headers={'Cache-Control': 'no-store'})

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

    @app.api_route('/api/{path:path}', methods=['GET', 'POST', 'PUT', 'PATCH', 'DELETE'])
    async def missing_api(path):
        return JSONResponse({'detail': 'Nieznany endpoint API'}, status_code=404)

    @app.websocket('/api/v1/ws')
    async def stream(websocket: WebSocket):
        await websocket.accept()

        async def publish():
            while True:
                # Jeden snapshot na klienta; nie istnieje kolejka pomiarów.
                await asyncio.wait_for(websocket.send_json(store.snapshot()), timeout=1.0)
                await asyncio.sleep(1.0)

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
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if websocket.client_state != WebSocketState.DISCONNECTED:
                try:
                    await asyncio.wait_for(websocket.close(), timeout=0.25)
                except (RuntimeError, OSError, asyncio.TimeoutError, LocalProtocolError):
                    pass

    @app.websocket('/api/v1/can/ws')
    async def can_stream(websocket: WebSocket):
        if can_store is None:
            await websocket.close(code=1013)
            return
        await websocket.accept()
        try:
            while True:
                await asyncio.wait_for(websocket.send_json(can_store.snapshot()), timeout=0.5)
                await asyncio.sleep(0.2)
        except (WebSocketDisconnect, asyncio.TimeoutError, RuntimeError, OSError, LocalProtocolError):
            pass
        finally:
            if websocket.client_state != WebSocketState.DISCONNECTED:
                try:
                    await websocket.close()
                except (RuntimeError, OSError, LocalProtocolError):
                    pass

    app.mount('/', PanelFiles(directory=str(static_dir), html=True), name='panel')
    return app
