import asyncio
from pathlib import Path
from types import SimpleNamespace
from robot_web_bridge.app import create_app
from robot_web_bridge.description import DescriptionStore
from robot_web_bridge.status import StatusStore
from test_bridge import http

MODEL = Path(__file__).resolve().parents[2] / 'robot_description/urdf/preview_6dof.example.urdf'


def test_model_exactly_matches_ros_and_access_policy(tmp_path):
    store = DescriptionStore()
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, description_store=store)
    path = '/api/v1/robot/description'
    assert asyncio.run(http(app, path))[0] == 503
    text = MODEL.read_text(); store.update(SimpleNamespace(data=text))
    assert asyncio.run(http(app, path)) == (200, text.encode())
    assert asyncio.run(http(app, path, client='100.64.0.10'))[0] == 403
    assert asyncio.run(http(app, path, headers=[(b'origin', b'http://foreign.example')]))[0] == 403


def test_invalid_or_external_resources_cannot_replace_the_model():
    store = DescriptionStore()
    for xml in ['<robot', '<other/>', '<robot><link><visual><geometry><mesh filename="http://foreign/m.stl"/></geometry></visual></link></robot>', '<robot/>', 'x' * 262145]:
        store.update(SimpleNamespace(data=xml)); assert store.snapshot() is None
