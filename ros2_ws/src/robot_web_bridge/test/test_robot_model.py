"""Model 3D: odczyt STL/OBJ, siatka bezpieczeństwa (2 mm, rzadsza), magazyn złożenia i API wgrywania."""
import asyncio
import json
from pathlib import Path
import struct
import sys

import numpy as np
import pytest

from robot_web_bridge.app import create_app
from robot_web_bridge.robot_model import Conflict, RobotModelStore
from robot_web_bridge.robot_model_mesh import (MeshError, SurfaceDistance, load_mesh, parse_obj, parse_stl, safety_mesh,
                                               signed_volume, write_stl)
from robot_web_bridge.status import StatusStore
from test_bridge import scope

ROOT = str(Path(__file__).resolve().parents[1])


def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    v = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return v, f


def cylinder(r, h, n):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    v = np.concatenate([np.c_[ring, np.zeros(n)], np.c_[ring, np.full(n, h)], [[0, 0, 0], [0, 0, h]]])
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    return v, np.array(f)


def stl_bytes(v, f, scale=1.):
    out = bytearray(b'x' * 80) + struct.pack('<I', len(f))
    for tri in v[f] * scale:
        out += struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', *tri.ravel()) + b'\0\0'
    return bytes(out)


def test_parsers_units_and_degenerate_triangles(tmp_path):
    v, f = box([0, 0, 0], [100, 50, 20])
    binary = tmp_path / 'part.stl'; binary.write_bytes(stl_bytes(v, f))
    vertices, faces = load_mesh(binary, 'mm')
    assert len(faces) == 12 and np.allclose(vertices[faces].max(axis=(0, 1)), [.1, .05, .02])
    ascii_text = 'solid a\n' + ''.join('facet normal 0 0 0\nouter loop\n' + ''.join(f'vertex {x} {y} {z}\n' for x, y, z in tri) +
                                        'endloop\nendfacet\n' for tri in v[f]) + 'endsolid a\n'
    av, af = parse_stl(ascii_text.encode())
    assert len(af) == 12 and np.allclose(av[af].max(axis=(0, 1)), [100, 50, 20])
    # OBJ: czworokąt → dwa trójkąty, indeksy ujemne, tekstury/normalne pomijane.
    ov, of = parse_obj(b'v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nf 1/1 2/1 3/1 4/1\nf -4 -3 -2\n')
    assert of.tolist() == [[0, 1, 2], [0, 2, 3], [0, 1, 2]]
    degenerate = tmp_path / 'flat.obj'; degenerate.write_bytes(b'v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n')
    with pytest.raises(MeshError):
        load_mesh(degenerate, 'mm')
    with pytest.raises(MeshError):
        load_mesh(binary, 'furlong')


@pytest.mark.parametrize('name,mesh', [('sześcian', box([0, 0, 0], [.1, .1, .1])),
                                        ('czworościan (ostre kąty dwuścienne)', (np.array([(0, 0, 0), (.02, 0, 0), (0, .02, 0), (0, 0, .02)]), np.array([(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]))),
                                        ('walec', cylinder(.04, .12, 256)),
                                        ('L wklęsły', (np.concatenate([box([0, 0, 0], [.12, .03, .03])[0], box([0, 0, .03], [.03, .03, .12])[0]]),
                                                       np.concatenate([box([0, 0, 0], [.12, .03, .03])[1], box([0, 0, .03], [.03, .03, .12])[1] + 8])))])
def test_safety_mesh_is_2mm_outside_closed_and_outward(name, mesh):
    v, f = mesh
    result, triangles, stats = safety_mesh(v, f)
    distance, _ = SurfaceDistance(v, f, .0005).nearest(result)
    # Wierzchołki dokładnie 2 mm (albo dalej przy dosunięciu na ostrych krawędziach), nigdy wewnątrz bryły.
    assert distance.min() >= .00199 and stats['mean_mm'] == pytest.approx(2.02, abs=.05)
    assert stats['min_mm'] >= 1.98, f'{name}: środki ścian/krawędzi za blisko bryły'
    assert stats['max_mm'] <= 3.0 and stats['cell_mm'] <= 3.2 + 1e-9
    assert stats['closed'] and signed_volume(result, triangles) > signed_volume(v, f) > 0
    # Rzadsza niż gęsta siatka walca CAD (512 segmentów), opis porównania w statystykach.
    assert stats['surface_triangles'] == len(f)


def test_dense_cad_mesh_gets_coarser_safety_mesh():
    v, f = cylinder(.03, .05, 4096)
    _, triangles, stats = safety_mesh(v, f)
    assert stats['coarser_than_surface'] and len(triangles) < len(f)


def command():
    # Proces importu z tym samym kodem (bez instalacji pakietu w środowisku testu).
    return [sys.executable, '-c', f'import sys; sys.path.insert(0, {ROOT!r}); from robot_web_bridge.robot_model_mesh import run_job; run_job(sys.argv[1])']


def upload(store, data, name='link.stl', units='mm'):
    identifier, clean, path = store.begin_upload(name, units)
    path.write_bytes(data)
    store.finish_upload(identifier, clean, units, path, len(data))
    store.process(identifier)
    return store._meta(identifier)


def test_store_import_assembly_and_delete(tmp_path):
    store = RobotModelStore(tmp_path, start_worker=False, command=command())
    meta = upload(store, stl_bytes(*box([0, 0, 0], [80, 40, 40])))
    assert meta['state'] == 'ready' and meta['triangles'] == 12 and meta['safety']['state'] == 'ready'
    assert meta['bbox_max'] == pytest.approx([.08, .04, .04])
    assert store.file(meta['id'], 'surface').is_file() and store.file(meta['id'], 'safety').is_file()
    assert not list(store.incoming.iterdir()), 'pliki tymczasowe usunięte'
    bad = upload(store, b'not a mesh', name='broken.stl')
    assert bad['state'] == 'failed' and bad['safety']['state'] == 'failed'
    assembly = store.set_assembly({'up': 'y', 'transform': {'z': 12.5, 'yaw': 90}, 'links': {'link_2': [meta['id']]}}, 'initial')
    assert assembly['links']['link_2'] == [meta['id']] and assembly['transform']['z'] == 12.5
    with pytest.raises(Conflict):
        store.set_assembly({'links': {}}, 'initial')  # nieaktualna rewizja
    with pytest.raises(ValueError):
        store.set_assembly({'links': {'link_1': [meta['id']], 'link_3': [meta['id']]}}, assembly['revision'])
    with pytest.raises(ValueError):
        store.set_assembly({'links': {'link_1': [bad['id']]}}, assembly['revision'])
    with pytest.raises(ValueError):
        store.set_assembly({'links': {'elbow': []}}, assembly['revision'])
    with pytest.raises(Conflict):
        store.delete(meta['id'])  # przypisana do członu
    store.delete(bad['id'])
    assert [item['id'] for item in store.snapshot()['meshes']] == [meta['id']]
    with pytest.raises(ValueError):
        store.begin_upload('model.step', 'mm')


def test_restart_requeues_or_fails_interrupted_jobs(tmp_path):
    store = RobotModelStore(tmp_path, start_worker=False, command=command())
    identifier, clean, path = store.begin_upload('a.stl', 'mm')
    path.write_bytes(stl_bytes(*box([0, 0, 0], [30, 30, 30])))
    store.finish_upload(identifier, clean, 'mm', path, 1)
    restarted = RobotModelStore(tmp_path, start_worker=False, command=command())
    assert restarted.jobs.get_nowait() == identifier
    (restarted.incoming / f'{identifier}.job.json').unlink()
    again = RobotModelStore(tmp_path, start_worker=False, command=command())
    assert again._meta(identifier)['state'] == 'failed'


async def call(app, method, path, body=b'', query=b'', chunks=1):
    output, spec = [], scope('http', path)
    spec['method'], spec['query_string'] = method, query
    parts = [body[i::chunks] for i in range(chunks)] if False else [body[i * len(body) // chunks:(i + 1) * len(body) // chunks] for i in range(chunks)]
    async def receive():
        part = parts.pop(0) if parts else b''
        return {'type': 'http.request', 'body': part, 'more_body': bool(parts)}
    async def send(event):
        output.append(event)
    await app(spec, receive, send)
    return output[0]['status'], b''.join(item.get('body', b'') for item in output)


def test_api_upload_in_chunks_assembly_and_files(tmp_path):
    store = RobotModelStore(tmp_path, start_worker=False, command=command())
    app = create_app(StatusStore('192.168.1.69', 8080, 'wlan0', '192.168.1.0/24'), str(tmp_path), robot_model_store=store)
    data = stl_bytes(*box([0, 0, 0], [50, 50, 50]))
    status, body = asyncio.run(call(app, 'POST', '/api/v1/robot/model/meshes', data, b'name=base.stl&units=mm', chunks=4))
    assert status == 202
    identifier = json.loads(body)['id']
    store.process(identifier)
    status, body = asyncio.run(call(app, 'GET', '/api/v1/robot/model'))
    model = json.loads(body)
    assert status == 200 and model['meshes'][0]['state'] == 'ready' and model['assembly']['revision'] == 'initial'
    request = json.dumps({'expected_revision': 'initial', 'assembly': {'links': {'base': [identifier]}}}).encode()
    status, body = asyncio.run(call(app, 'PUT', '/api/v1/robot/model', request))
    assert status == 200 and json.loads(body)['links']['base'] == [identifier]
    status, body = asyncio.run(call(app, 'PUT', '/api/v1/robot/model', request))
    assert status == 409
    status, body = asyncio.run(call(app, 'GET', f'/api/v1/robot/model/meshes/{identifier}/surface.stl'))
    assert status == 200 and struct.unpack_from('<I', body, 80)[0] == 12
    status, _ = asyncio.run(call(app, 'GET', f'/api/v1/robot/model/meshes/{identifier}/safety.stl'))
    assert status == 200
    status, _ = asyncio.run(call(app, 'GET', '/api/v1/robot/model/meshes/../../etc/surface.stl'))
    assert status in (404, 422)
    status, _ = asyncio.run(call(app, 'POST', '/api/v1/robot/model/meshes', data, b'name=x.step&units=mm'))
    assert status == 422


def test_overlapping_bodies_with_inverted_shell():
    # Komponent z dwiema nakładającymi się bryłami (korpus + czop), jedna z normalnymi do wewnątrz.
    body, pin = box([0, 0, 0], [.08, .04, .04]), cylinder(.015, .06, 128)
    pin_vertices = pin[0][:, [0, 2, 1]] * [1, -1, 1] + [.06, .05, .02]   # oś walca wzdłuż Y, częściowo w korpusie
    v = np.concatenate([body[0], pin_vertices])
    f = np.concatenate([body[1], pin[1] + 8])                             # obrót bez odbicia: orientacja zachowana
    f_inverted = np.concatenate([body[1], pin[1][:, ::-1] + 8])          # ta sama bryła z odwróconym czopem
    for faces in (f, f_inverted):
        result, triangles, stats = safety_mesh(v, faces)
        distance, _ = SurfaceDistance(v, faces, .0005).nearest(result)
        assert distance.min() >= .00199 and stats['min_mm'] >= 1.8 and stats['closed']


def test_dense_export_is_simplified_with_conservative_offset(monkeypatch):
    import robot_web_bridge.robot_model_mesh as mesh
    monkeypatch.setattr(mesh, 'SAMPLE_BUDGET', 60_000)   # wymusza ścieżkę uproszczenia na małej bryle
    v, f = cylinder(.012, .05, 4096)
    result, triangles, stats = mesh.safety_mesh(v, f)
    assert stats['simplified_mm'] > 0 and stats['error_mm'] > 0 and stats['min_mm'] >= 1.98
    # Siłowo względem oryginału (wszystkie trójkąty): punkty kontrolne co najmniej 2 mm od bryły.
    probes = np.concatenate([result, result[triangles].mean(axis=1)])[::7]
    a, b, c = (v[f[:, i]] for i in range(3))
    nearest = [np.linalg.norm(mesh.closest_points(np.repeat(p[None], len(f), 0), a, b, c) - p, axis=1).min() for p in probes]
    assert min(nearest) >= .00199
