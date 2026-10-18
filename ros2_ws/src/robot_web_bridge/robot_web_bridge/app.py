"""API tylko do odczytu, wspólne pochodzenie strony i WebSocket."""

import asyncio
from ipaddress import ip_address, ip_network
import os

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from starlette.responses import JSONResponse
from starlette.staticfiles import StaticFiles
from starlette.websockets import WebSocketState
from wsproto.utilities import LocalProtocolError


class PanelFiles(StaticFiles):
    """Kontrola granicy katalogu także w starszym Starlette z Ubuntu 22.04."""

    def lookup_path(self, path):
        root = os.path.realpath(self.directory)
        target = os.path.realpath(os.path.join(root, path))
        if os.path.commonpath([root, target]) != root:
            return '', None
        return super().lookup_path(path)


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


def create_app(store, static_dir):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LanOnly, subnet=store.network['subnet'],
                       authority=f"{store.network['address']}:{store.network['port']}")

    @app.get('/healthz')
    def health():
        return JSONResponse({'status': 'ok'}, headers={'Cache-Control': 'no-store'})

    @app.get('/api/v1/status')
    def status():
        return JSONResponse(store.snapshot(), headers={'Cache-Control': 'no-store'})

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

    app.mount('/', PanelFiles(directory=str(static_dir), html=True), name='panel')
    return app
