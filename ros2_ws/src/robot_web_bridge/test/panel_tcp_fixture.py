"""Panel + produkcyjny WebSocket + rzeczywisty typ JogTcp; bez węzła ROS i bez CAN."""
import asyncio
import json
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace

import uvicorn
from fastapi.routing import APIRoute
from robot_control_interfaces.srv import JogTcp
from robot_web_bridge.app import create_app
from robot_web_bridge.description import DescriptionStore
from robot_web_bridge.manual import ManualStore
from robot_web_bridge.status import StatusStore

sock = socket.socket()
sock.bind(('127.0.0.1', 0))
port = sock.getsockname()[1]
manual = ManualStore()
description = DescriptionStore()
description.update(SimpleNamespace(data=Path(sys.argv[2]).read_text()))
requests = []
busy = fault = False
message = 'Atrapa sterownika — bez napędów'
tcp = dict(ready=True, busy=False, state='ready', message='JOG TCP gotowy', axis=0,
           direction=0, frame='base', impulse=False, progress=0., requested_step=0., velocity=0.,
           max_linear_velocity=.1, max_angular_velocity=.2, model_id='model', configuration_revision='initial', generation=0)


def publish():
    stamp = time.monotonic_ns()
    axes = [SimpleNamespace(axis=i+1, available=True, prepared=True, can_jog=not busy, can_zero=not busy,
        motion='ruch' if busy else 'zatrzymany', enable_known=True, enabled=True,
        position_known=True, position_degrees=0., position_age_seconds=0., speed_known=True,
        speed_rpm=0., speed_age_seconds=0., enable_age_seconds=0., joint_configured=True,
        calibration_valid=True, joint_position_known=True, joint_position_rad=q,
        joint_lower_rad=-3., joint_upper_rad=3., jog_min_velocity_rad_s=.001,
        jog_max_velocity_rad_s=1., configuration_message='Atrapa') for i, q in enumerate([.3, -.4, .7, .3, .6, -.2])]
    manual.update(SimpleNamespace(monotonic_ns=stamp, busy=busy, fault=fault, active_axis=0,
        max_rpm=30, operation='idle', message=message, axes=axes, configuration_revision='initial'))
    manual.update_tcp(SimpleNamespace(monotonic_ns=stamp, **tcp))


async def action(operation, **values):
    global busy, fault, message
    assert operation == 'jog_tcp'
    request = JogTcp.Request()
    for key, value in values.items():
        setattr(request, key, value)  # Tu zgłaszany był rzeczywisty AssertionError dla step=0.
    request.issued_monotonic_ns = time.monotonic_ns()
    requests.append(dict(action=request.action, step=request.step, velocity=request.velocity,
        step_type=type(request.step).__name__, velocity_type=type(request.velocity).__name__,
        emergency=request.emergency, impulse=request.impulse, sequence=request.sequence))
    if request.action == request.START:
        assert not busy and not fault
        busy = True
        tcp.update(ready=False, busy=True, state='running', message='Ruch atrapy',
            axis=request.axis, direction=request.direction, frame=request.frame, impulse=request.impulse,
            requested_step=request.step, velocity=request.velocity, generation=tcp['generation']+1)
    elif request.action == request.STOP:
        if request.emergency and busy:
            fault = True
            message = 'Awaryjne przerwanie JOG TCP przez panel/most'
        busy = False
        tcp.update(ready=not fault, busy=False, state='failed' if fault else 'stopped', message=message if fault else 'Zatrzymanie TCP potwierdzone')
    publish()
    return True, 'Potwierdzenie atrapy; brak komend CAN'


publish()
app = create_app(StatusStore('127.0.0.1', port, 'lo', '127.0.0.0/8'), Path(sys.argv[1]),
    manual_store=manual, manual_action=action, description_store=description)


@app.on_event('startup')
async def startup():
    async def heartbeat():
        while True:
            publish()
            await asyncio.sleep(.02)
    app.state.heartbeat = asyncio.create_task(heartbeat())


@app.on_event('shutdown')
async def shutdown():
    app.state.heartbeat.cancel()
    await asyncio.gather(app.state.heartbeat, return_exceptions=True)


async def received():
    return requests


# Testowy odczyt przed końcowym montowaniem statycznego panelu pod "/".
app.router.routes.insert(0, APIRoute('/test/requests', received, methods=['GET']))
print(json.dumps({'port': port}), flush=True)
uvicorn.Server(uvicorn.Config(app, log_level='error', ws='wsproto')).run(sockets=[sock])
