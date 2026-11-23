"""Odczyt trace i CSV pod tą samą polityką LAN co reszta panelu."""
import asyncio
import math
from typing import Literal
from fastapi import HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse


def add_shaft_trace_routes(app, trace):
    base = '/api/v1/diagnostics/trace'
    exports = asyncio.Semaphore(1)

    def require():
        if trace is None:
            raise HTTPException(503, 'Rejestrator pozycji nie jest dostępny')

    def interval(start, end):
        if (start is not None and not math.isfinite(start)) or (end is not None and not math.isfinite(end)):
            raise HTTPException(422, 'Nieprawidłowy czas')
        if start is not None and end is not None and start > end:
            raise HTTPException(422, 'Początek zakresu jest późniejszy od końca')
        return (float('-inf') if start is None else start, float('inf') if end is None else end)

    @app.get(base)
    @app.get(base+'/status')
    async def status():
        require()
        return JSONResponse(trace.status(), headers={'Cache-Control': 'no-store'})

    @app.get(base+'/history')
    async def history(start_ms: float = None, end_ms: float = None,
                      max_points: int = Query(2000, ge=2, le=2000), axis: int = Query(None, ge=1, le=6),
                      after: int = Query(0, ge=0), reduction: Literal['minmax', 'none'] = 'minmax'):
        require()
        start, end = interval(start_ms, end_ms)
        result = await asyncio.to_thread(trace.history, [axis] if axis else range(1, 7), start, end,
                                         max_points, {a: after for a in range(1, 7)}, reduction)
        return JSONResponse(result, headers={'Cache-Control': 'no-store'})

    @app.websocket(base+'/ws')
    async def stream(socket: WebSocket):
        if trace is None:
            await socket.close(code=1013)
            return
        await socket.accept()
        cursor = trace.cursor
        stopped = False
        async def publish():
            nonlocal cursor
            while not stopped:
                frame = trace.live(cursor)
                cursor = frame['cursor']
                await asyncio.wait_for(socket.send_json(frame), timeout=2)
                await asyncio.sleep(.05)
        async def receive():
            while True:
                event = await socket.receive()
                if event['type'] == 'websocket.disconnect':
                    return
        tasks = [asyncio.create_task(publish()), asyncio.create_task(receive())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, RuntimeError, OSError, asyncio.TimeoutError):
            pass
        finally:
            stopped = True
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    @app.get(base+'/axes/{axis}/export.csv')
    async def export(axis: int, start_ms: float = None, end_ms: float = None):
        require()
        if axis not in range(1, 7):
            raise HTTPException(422, 'Oś musi należeć do zakresu 1–6')
        start, end = interval(start_ms, end_ms)
        if exports.locked():
            raise HTTPException(429, 'Inny eksport jest jeszcze przesyłany')
        async def body():
            async with exports:
                iterator = await asyncio.to_thread(trace.export, axis, start, end)
                # Grupowanie redukuje liczbę przełączeń wątku i fragmentów HTTP.
                def chunk():
                    result = []
                    for _ in range(512):
                        row = next(iterator, None)
                        if row is None:
                            break
                        result.append(row)
                    return ''.join(result)
                while True:
                    value = await asyncio.to_thread(chunk)
                    if not value:
                        break
                    yield value.encode('utf-8')
        return StreamingResponse(body(), media_type='text/csv; charset=utf-8', headers={
            'Content-Disposition': f'attachment; filename="shaft-J{axis}.csv"', 'Cache-Control': 'no-store'})
