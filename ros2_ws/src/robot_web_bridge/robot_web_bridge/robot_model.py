"""Model 3D robota: siatki członów (import z CAD), siatki bezpieczeństwa i złożenie.

Katalog (domyślnie ``$XDG_STATE_HOME/robot/models``, parametr ``robot_model_directory``):
- ``meshes/<id>.json`` — opis siatki i stan przetwarzania (queued → processing → ready/failed; safety osobno),
- ``meshes/<id>.stl`` — siatka w metrach, w układzie złożenia CAD (pozycja zerowa robota),
- ``meshes/<id>.safety.stl`` — siatka bezpieczeństwa (odsunięta o 2 mm, rzadsza),
- ``meshes/<id>.element.json`` — definicja elementu (tryb modułowy): status „model osi”/„element konstrukcyjny” i oś
  kinematyczna w układzie pliku elementu [mm]; osobny plik — proces importu nadpisuje ``<id>.json``,
- ``incoming/`` — surowe pliki i zadania czekające na proces importu,
- ``assembly.json`` — przypisanie siatek do członów i przekształcenie układu złożenia do układu bazy robota (tryb
  „całe złożenie z Fusion” — do usunięcia w etapie 3 trybu modułowego),
- ``chain.json`` — tryb modułowy: łańcuch wystąpień elementów od podstawy; każde wystąpienie ma położenie względem
  poprzednika (``placement``: x/y/z [mm], roll/pitch/yaw [°], Rz·Ry·Rx) i dane spasowania (``mate``) do ponownej edycji.

Model 3D nie wchodzi do kinematycznego URDF (identyfikator modelu planera = SHA-256 URDF): zmiana wyglądu nie
unieważnia programów. Podgląd łączy URDF z modelem w przeglądarce. Przetwarzanie siatek działa w osobnym procesie
(``robot_model_mesh``, nice 10) — bez obciążania GIL mostu i KEEPALIVE JOG.
"""
import json
import math
import os
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading
import uuid
from datetime import datetime, timezone

LINKS = ('base', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6', 'tool')
UNITS = ('mm', 'cm', 'm', 'in')
EXTENSIONS = ('.stl', '.obj')
MAX_UPLOAD = 200 * 1024 * 1024
MAX_MESHES = 64
IDENTIFIER = re.compile(r'[0-9a-f]{32}')
TRANSFORM = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
ROLES = ('axis', 'structural')
MAX_CHAIN = 64


def default_chain():
    return {'revision': 'initial', 'up': 'z', 'elements': []}


def vector(value, name, unit=False):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{name}: wymagane trzy liczby')
    result = [finite(item, name) for item in value]
    if unit:
        norm = math.sqrt(sum(item * item for item in result))
        if abs(norm - 1) > 1e-3:
            raise ValueError(f'{name}: wymagany wektor jednostkowy')
        result = [item / norm for item in result]
    return result


def clean_plane(plane, name):
    if plane is None:
        return None
    if not isinstance(plane, dict) or set(plane) - {'point', 'normal'}:
        raise ValueError(f'{name}: wymagany obiekt {{point, normal}} albo null')
    return {'point': vector(plane.get('point'), f'{name} — punkt'), 'normal': vector(plane.get('normal'), f'{name} — normalna', True)}


MATE_KEYS = {'parent_point', 'parent_direction', 'element_point', 'element_direction', 'flip', 'angle_deg', 'offset_mm',
             'parent_plane', 'element_plane', 'needs_parent'}


def clean_mate(mate):
    """Spasowanie z poprzednikiem: walec na poprzedniku i na elemencie (punkt [mm] i kierunek w układach plików),
    odwrócenie, obrót wokół wspólnej osi [°], przesunięcie wzdłuż niej [mm], opcjonalny styk czół (płaszczyzny).
    ``needs_parent`` = poprzednik zmienił się (wstawienie, usunięcie, zamiana pliku) — trzeba wskazać mocowanie ponownie."""
    if mate is None:
        return None
    if not isinstance(mate, dict) or set(mate) - MATE_KEYS:
        raise ValueError('Spasowanie: nieznane pola')
    if type(mate.get('flip', False)) is not bool or type(mate.get('needs_parent', False)) is not bool:
        raise ValueError('Spasowanie: flip i needs_parent — wartości logiczne')
    return {'parent_point': vector(mate.get('parent_point'), 'Punkt osi poprzednika'),
            'parent_direction': vector(mate.get('parent_direction'), 'Kierunek osi poprzednika', True),
            'element_point': vector(mate.get('element_point'), 'Punkt osi elementu'),
            'element_direction': vector(mate.get('element_direction'), 'Kierunek osi elementu', True),
            'flip': mate.get('flip', False), 'angle_deg': finite(mate.get('angle_deg', 0), 'Obrót wokół osi', -360, 360),
            'offset_mm': finite(mate.get('offset_mm', 0), 'Przesunięcie wzdłuż osi', -1e4, 1e4),
            'parent_plane': clean_plane(mate.get('parent_plane'), 'Styk na poprzedniku'),
            'element_plane': clean_plane(mate.get('element_plane'), 'Styk na elemencie'),
            'needs_parent': mate.get('needs_parent', False)}


def default_element():
    return {'revision': 'initial', 'role': None, 'axis': None}


def finite(value, name, low=-1e5, high=1e5):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name}: wymagana skończona liczba w zakresie {low:g}…{high:g}')
    return float(value)


def clean_axis(axis):
    """Oś modelu osi w układzie pliku elementu: punkt [mm], kierunek (jednostkowy; zwrot = dodatni obrót wg prawej
    dłoni; ``reversed`` — zwrot odwrócony względem wskazanego walca) i parametry dopasowania walca."""
    if not isinstance(axis, dict):
        raise ValueError('Oś: wymagany obiekt {point_mm, direction, radius_mm, residual_mm, faces, coverage_deg}')
    point, direction = axis.get('point_mm'), axis.get('direction')
    if not isinstance(point, list) or len(point) != 3 or not isinstance(direction, list) or len(direction) != 3:
        raise ValueError('Oś: point_mm i direction — po trzy liczby')
    point = [finite(value, 'Punkt osi') for value in point]
    direction = [finite(value, 'Kierunek osi', -1.01, 1.01) for value in direction]
    norm = math.sqrt(sum(value * value for value in direction))
    if abs(norm - 1) > 1e-3:
        raise ValueError('Kierunek osi musi być wektorem jednostkowym')
    faces = axis.get('faces')
    if type(faces) is not int or faces < 1:
        raise ValueError('Oś: liczba ścian dopasowania — dodatnia liczba całkowita')
    if type(axis.get('reversed', False)) is not bool:
        raise ValueError('Oś: reversed — wartość logiczna (zwrot odwrócony względem wskazanego walca)')
    return {'point_mm': point, 'direction': [value / norm for value in direction], 'reversed': axis.get('reversed', False),
            'radius_mm': finite(axis.get('radius_mm'), 'Promień walca', 0, 1e5),
            'residual_mm': finite(axis.get('residual_mm'), 'Odchyłka walca', 0, 1e5),
            'faces': faces, 'coverage_deg': finite(axis.get('coverage_deg'), 'Zasięg kątowy', 0, 360)}


class Conflict(ValueError):
    pass


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def default_assembly():
    return {'revision': 'initial', 'up': 'z', 'transform': dict.fromkeys(TRANSFORM, 0.0), 'links': {link: [] for link in LINKS}}


def atomic_json(path, value):
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


class RobotModelStore:
    def __init__(self, directory=None, start_worker=True, command=None):
        base = Path(directory) if directory else Path(os.environ.get('XDG_STATE_HOME') or str(Path.home() / '.local/state')) / 'robot/models'
        self.root, self.meshes, self.incoming = base, base / 'meshes', base / 'incoming'
        for path in (self.root, self.meshes, self.incoming):
            path.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.command = command or [sys.executable, '-m', 'robot_web_bridge.robot_model_mesh']
        self._recover()
        if start_worker:
            threading.Thread(target=self._worker, name='robot-model-import', daemon=True).start()

    # --- odczyt -------------------------------------------------------------------------------------------------
    def _meta(self, identifier):
        return json.loads((self.meshes / f'{identifier}.json').read_text(encoding='utf-8'))

    def element(self, identifier):
        path = self.meshes / f'{identifier}.element.json'
        if not path.exists():
            return default_element()
        try:
            value = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return default_element()
        return {**default_element(), **{key: value[key] for key in ('revision', 'role', 'axis') if key in value}}

    def mesh_list(self, elements=False):
        items = []
        for path in self.meshes.glob('*.json'):
            if IDENTIFIER.fullmatch(path.stem):
                try:
                    item = json.loads(path.read_text(encoding='utf-8'))
                except (OSError, ValueError):
                    continue
                if elements:
                    item['element'] = self.element(path.stem)
                items.append(item)
        return sorted(items, key=lambda item: item.get('created_at', ''))

    def assembly(self):
        path = self.root / 'assembly.json'
        if not path.exists():
            return default_assembly()
        value = json.loads(path.read_text(encoding='utf-8'))
        base = default_assembly()
        base.update({key: value[key] for key in ('revision', 'up', 'transform') if key in value})
        base['links'].update({link: list(value.get('links', {}).get(link, [])) for link in LINKS})
        return base

    def chain(self):
        path = self.root / 'chain.json'
        if not path.exists():
            return default_chain()
        value = json.loads(path.read_text(encoding='utf-8'))
        return {**default_chain(), **{key: value[key] for key in ('revision', 'up', 'elements') if key in value}}

    def snapshot(self):
        with self.lock:
            return {'assembly': self.assembly(), 'chain': self.chain(), 'meshes': self.mesh_list(elements=True)}

    def set_chain(self, data, expected_revision):
        """Łańcuch trybu modułowego: pierwsze wystąpienie = podstawa (bez spasowania, położenie zerowe), kolejne —
        położenie względem poprzednika. Siatki muszą być wczytane; ten sam plik może wystąpić wiele razy."""
        if not isinstance(data, dict) or set(data) - {'up', 'elements'}:
            raise ValueError('Łańcuch: wymagany obiekt {up, elements}')
        up = data.get('up', 'z')
        if up not in ('z', 'y'):
            raise ValueError('Oś „w górę” podstawy: z albo y')
        elements = data.get('elements', [])
        if not isinstance(elements, list) or len(elements) > MAX_CHAIN:
            raise ValueError(f'Łańcuch: lista do {MAX_CHAIN} elementów')
        with self.lock:
            current = self.chain()
            if expected_revision != current['revision']:
                raise Conflict('Łańcuch modelu zmienił się w innym panelu; wczytaj aktualny stan')
            known = {item['id']: item for item in self.mesh_list()}
            cleaned, seen = [], set()
            for index, item in enumerate(elements):
                if not isinstance(item, dict) or set(item) - {'id', 'mesh', 'mate', 'placement'}:
                    raise ValueError(f'Element {index + 1}: wymagany obiekt {{id, mesh, mate, placement}}')
                identifier, mesh = item.get('id'), item.get('mesh')
                if not IDENTIFIER.fullmatch(str(identifier or '')) or identifier in seen:
                    raise ValueError(f'Element {index + 1}: nieprawidłowy lub powtórzony identyfikator')
                seen.add(identifier)
                if mesh not in known:
                    raise ValueError(f'Element {index + 1}: siatka nie istnieje')
                if known[mesh].get('state') != 'ready':
                    raise ValueError(f'Element {index + 1}: siatka „{known[mesh].get("name")}” nie jest jeszcze wczytana')
                placement = item.get('placement') or {}
                if not isinstance(placement, dict) or set(placement) - set(TRANSFORM):
                    raise ValueError(f'Element {index + 1}: położenie x, y, z [mm], roll, pitch, yaw [°]')
                placement = {key: finite(placement.get(key, 0), f'Element {index + 1}: {key}') for key in TRANSFORM}
                mate = clean_mate(item.get('mate'))
                if index == 0 and (mate is not None or any(placement.values())):
                    raise ValueError('Podstawa nie ma spasowania ani przesunięcia — to początek łańcucha')
                cleaned.append({'id': identifier, 'mesh': mesh, 'mate': mate, 'placement': placement})
            result = {'revision': uuid.uuid4().hex, 'up': up, 'elements': cleaned}
            atomic_json(self.root / 'chain.json', result)
            return result

    def file(self, identifier, kind):
        if not IDENTIFIER.fullmatch(identifier or '') or kind not in ('surface', 'safety'):
            raise ValueError('Nieprawidłowy identyfikator siatki')
        path = self.meshes / (f'{identifier}.stl' if kind == 'surface' else f'{identifier}.safety.stl')
        if not path.is_file():
            raise FileNotFoundError(identifier)
        return path

    # --- zapis --------------------------------------------------------------------------------------------------
    def begin_upload(self, filename, units):
        name = Path(str(filename or '')).name.strip()[:120]
        extension = Path(name).suffix.lower()
        if extension not in EXTENSIONS:
            raise ValueError('Obsługiwane pliki: STL (binarny lub ASCII) i OBJ')
        if units not in UNITS:
            raise ValueError('Jednostki pliku: mm, cm, m albo in')
        with self.lock:
            if len(self.mesh_list()) >= MAX_MESHES:
                raise Conflict(f'Limit {MAX_MESHES} siatek — usuń nieużywane')
        identifier = uuid.uuid4().hex
        return identifier, name, self.incoming / f'{identifier}{extension}'

    def finish_upload(self, identifier, name, units, raw, size):
        meta = {'id': identifier, 'name': name, 'units': units, 'size_bytes': size, 'created_at': now_iso(),
                'state': 'queued', 'message': 'Oczekiwanie na przetworzenie', 'safety': {'state': 'queued', 'message': 'Po wczytaniu siatki'}}
        job = {'id': identifier, 'raw': str(raw), 'units': units, 'directory': str(self.meshes), 'offset': .002}
        with self.lock:
            atomic_json(self.meshes / f'{identifier}.json', meta)
            atomic_json(self.incoming / f'{identifier}.job.json', job)
        self.jobs.put(identifier)
        return meta

    def set_assembly(self, data, expected_revision):
        if not isinstance(data, dict):
            raise ValueError('Wymagany obiekt złożenia')
        up = data.get('up', 'z')
        if up not in ('z', 'y'):
            raise ValueError('Oś „w górę” złożenia: z albo y')
        transform = data.get('transform', {})
        if not isinstance(transform, dict) or set(transform) - set(TRANSFORM):
            raise ValueError('Przekształcenie złożenia: x, y, z [mm], roll, pitch, yaw [°]')
        values = {}
        for key in TRANSFORM:
            value = transform.get(key, 0)
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1e5:
                raise ValueError(f'Przekształcenie złożenia {key}: wymagana skończona liczba w zakresie ±1e5')
            values[key] = float(value)
        links = data.get('links', {})
        if not isinstance(links, dict) or set(links) - set(LINKS):
            raise ValueError('Człony złożenia: ' + ', '.join(LINKS))
        with self.lock:
            current = self.assembly()
            if expected_revision != current['revision']:
                raise Conflict('Model 3D zmienił się w innym panelu; wczytaj aktualny stan')
            known = {item['id']: item for item in self.mesh_list()}
            assigned, seen = {}, set()
            for link in LINKS:
                ids = links.get(link, [])
                if not isinstance(ids, list) or any(not isinstance(item, str) for item in ids):
                    raise ValueError(f'{link}: wymagana lista identyfikatorów siatek')
                for item in ids:
                    if item not in known:
                        raise ValueError(f'{link}: siatka {item[:8]}… nie istnieje')
                    if known[item].get('state') != 'ready':
                        raise ValueError(f'{link}: siatka „{known[item].get("name")}” nie jest jeszcze wczytana')
                    if item in seen:
                        raise ValueError(f'Siatka „{known[item].get("name")}” przypisana do dwóch członów')
                    seen.add(item)
                assigned[link] = list(ids)
            result = {'revision': uuid.uuid4().hex, 'up': up, 'transform': values, 'links': assigned}
            atomic_json(self.root / 'assembly.json', result)
            return result

    def set_element(self, identifier, data, expected_revision):
        """Status elementu (``axis`` = model osi, ``structural`` = element konstrukcyjny, ``None`` = nieokreślony) i oś
        kinematyczna modelu osi. Oś wymaga wczytanej siatki; zmiana statusu na inny niż model osi usuwa oś."""
        if not IDENTIFIER.fullmatch(identifier or ''):
            raise ValueError('Nieprawidłowy identyfikator siatki')
        if not isinstance(data, dict) or set(data) - {'role', 'axis'}:
            raise ValueError('Element: wymagany obiekt {role, axis}')
        role = data.get('role')
        if role is not None and role not in ROLES:
            raise ValueError('Status elementu: axis (model osi), structural (element konstrukcyjny) albo null')
        axis = data.get('axis')
        if axis is not None and role != 'axis':
            raise ValueError('Oś kinematyczną ma tylko model osi')
        axis = clean_axis(axis) if axis is not None else None
        with self.lock:
            meta = self._meta(identifier)
            current = self.element(identifier)
            if expected_revision != current['revision']:
                raise Conflict('Element zmienił się w innym panelu; wczytaj aktualny stan')
            if axis is not None and meta.get('state') != 'ready':
                raise ValueError(f'Siatka „{meta.get("name")}” nie jest jeszcze wczytana — oś można wskazać po wczytaniu')
            result = {'revision': uuid.uuid4().hex, 'role': role, 'axis': axis}
            atomic_json(self.meshes / f'{identifier}.element.json', result)
            return result

    def rename(self, identifier, name):
        if not IDENTIFIER.fullmatch(identifier or ''):
            raise ValueError('Nieprawidłowy identyfikator siatki')
        name = str(name or '').strip()[:120]
        if not name:
            raise ValueError('Nazwa siatki nie może być pusta')
        with self.lock:
            meta = self._meta(identifier); meta['name'] = name
            atomic_json(self.meshes / f'{identifier}.json', meta)
            return meta

    def delete(self, identifier):
        if not IDENTIFIER.fullmatch(identifier or ''):
            raise ValueError('Nieprawidłowy identyfikator siatki')
        with self.lock:
            if not (self.meshes / f'{identifier}.json').exists():
                raise FileNotFoundError(identifier)
            if any(item.get('mesh') == identifier for item in self.chain()['elements']):
                raise Conflict('Siatka jest użyta w łańcuchu modelu; najpierw usuń ją z łańcucha i zapisz model')
            if self._meta(identifier).get('state') in ('queued', 'processing'):
                raise Conflict('Siatka jest w trakcie przetwarzania')
            for suffix in ('.json', '.stl', '.safety.stl', '.element.json'):
                (self.meshes / f'{identifier}{suffix}').unlink(missing_ok=True)

    # --- przetwarzanie ------------------------------------------------------------------------------------------
    def _recover(self):
        """Po restarcie mostu: zadania z zachowanym plikiem wracają do kolejki, pozostałe są oznaczane jako nieudane."""
        for item in self.mesh_list():
            if item.get('state') not in ('queued', 'processing') and item.get('safety', {}).get('state') not in ('queued', 'processing'):
                continue
            job = self.incoming / f'{item["id"]}.job.json'
            if job.exists() and item.get('state') in ('queued', 'processing'):
                self.jobs.put(item['id'])
            else:
                item.update(state=item['state'] if item.get('state') == 'ready' else 'failed',
                            message=item.get('message') if item.get('state') == 'ready' else 'Przerwane przetwarzanie — wgraj plik ponownie')
                item['safety'] = {'state': 'failed', 'message': 'Przerwane generowanie — wgraj plik ponownie'}
                atomic_json(self.meshes / f'{item["id"]}.json', item)

    def process(self, identifier, timeout=1800):
        job = self.incoming / f'{identifier}.job.json'
        if not job.exists():
            return
        try:
            # Proces importu widzi ten sam pakiet niezależnie od sposobu instalacji (overlay z dowiązaniami).
            package_root = str(Path(__file__).resolve().parents[1])
            env = {**os.environ, 'PYTHONPATH': os.pathsep.join(filter(None, [package_root, os.environ.get('PYTHONPATH', '')]))}
            subprocess.run([*self.command, str(job)], timeout=timeout, check=False, env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        except subprocess.TimeoutExpired:
            pass
        with self.lock:
            try:
                meta = self._meta(identifier)
            except (OSError, ValueError):
                return
            changed = False
            if meta.get('state') in ('queued', 'processing'):
                meta.update(state='failed', message='Proces importu zakończył się bez wyniku'); changed = True
            if meta.get('safety', {}).get('state') in ('queued', 'processing'):
                meta['safety'] = {'state': 'failed', 'message': 'Proces siatki bezpieczeństwa zakończył się bez wyniku'}; changed = True
            if changed:
                atomic_json(self.meshes / f'{identifier}.json', meta)
            for leftover in self.incoming.glob(f'{identifier}*'):
                leftover.unlink(missing_ok=True)

    def _worker(self):
        while True:
            self.process(self.jobs.get())


def add_robot_model_routes(app, store):
    """API modelu 3D: /api/v1/robot/model (stan, złożenie), /meshes (wgrywanie, pliki, definicja elementu, usuwanie)."""
    import asyncio
    from fastapi import HTTPException, Request
    from starlette.responses import FileResponse, JSONResponse

    def reply(data, status=200):
        return JSONResponse(data, status_code=status, headers={'Cache-Control': 'no-store'})

    def available():
        if store is None:
            raise HTTPException(503, 'Magazyn modelu 3D jest niedostępny')

    async def run(function, *args):
        try:
            return await asyncio.get_running_loop().run_in_executor(None, function, *args)
        except Conflict as error:
            raise HTTPException(409, str(error)) from None
        except FileNotFoundError:
            raise HTTPException(404, 'Siatka nie istnieje') from None
        except ValueError as error:
            raise HTTPException(422, str(error)) from None

    @app.get('/api/v1/robot/model')
    async def robot_model():
        available()
        return reply(await run(store.snapshot))

    @app.put('/api/v1/robot/model')
    async def robot_model_assembly(request: Request):
        available()
        raw = await request.body()
        if len(raw) > 65536:
            raise HTTPException(413, 'Treść przekracza 64 KiB')
        try:
            data = json.loads(raw or b'{}')
        except ValueError:
            raise HTTPException(422, 'Wymagany JSON') from None
        if not isinstance(data, dict):
            raise HTTPException(422, 'Wymagany obiekt JSON')
        return reply(await run(store.set_assembly, data.get('assembly'), data.get('expected_revision')))

    @app.put('/api/v1/robot/model/chain')
    async def robot_model_chain(request: Request):
        available()
        raw = await request.body()
        if len(raw) > 262144:
            raise HTTPException(413, 'Treść przekracza 256 KiB')
        try:
            data = json.loads(raw or b'{}')
        except ValueError:
            raise HTTPException(422, 'Wymagany JSON') from None
        if not isinstance(data, dict):
            raise HTTPException(422, 'Wymagany obiekt JSON')
        return reply(await run(store.set_chain, data.get('chain'), data.get('expected_revision')))

    @app.post('/api/v1/robot/model/meshes')
    async def robot_model_upload(request: Request, name: str = '', units: str = 'mm'):
        available()
        identifier, clean, path = await run(store.begin_upload, name, units)
        size = 0
        try:
            with path.open('xb') as stream:
                async for chunk in request.stream():
                    size += len(chunk)
                    if size > MAX_UPLOAD:
                        raise HTTPException(413, f'Plik przekracza {MAX_UPLOAD // 1048576} MiB')
                    stream.write(chunk)
            if not size:
                raise HTTPException(422, 'Pusty plik')
        except BaseException:
            path.unlink(missing_ok=True)
            raise
        return reply(await run(store.finish_upload, identifier, clean, units, path, size), 202)

    @app.patch('/api/v1/robot/model/meshes/{identifier}')
    async def robot_model_rename(identifier: str, request: Request):
        available()
        try:
            data = json.loads(await request.body() or b'{}')
        except ValueError:
            raise HTTPException(422, 'Wymagany JSON') from None
        return reply(await run(store.rename, identifier, data.get('name') if isinstance(data, dict) else None))

    @app.put('/api/v1/robot/model/meshes/{identifier}/element')
    async def robot_model_element(identifier: str, request: Request):
        available()
        raw = await request.body()
        if len(raw) > 65536:
            raise HTTPException(413, 'Treść przekracza 64 KiB')
        try:
            data = json.loads(raw or b'{}')
        except ValueError:
            raise HTTPException(422, 'Wymagany JSON') from None
        if not isinstance(data, dict):
            raise HTTPException(422, 'Wymagany obiekt JSON')
        return reply(await run(store.set_element, identifier, data.get('element'), data.get('expected_revision')))

    @app.delete('/api/v1/robot/model/meshes/{identifier}')
    async def robot_model_delete(identifier: str):
        available()
        await run(store.delete, identifier)
        return reply({'deleted': True})

    @app.get('/api/v1/robot/model/meshes/{identifier}/{kind}.stl')
    async def robot_model_file(identifier: str, kind: str):
        available()
        path = await run(store.file, identifier, kind)
        # Pliki są niezmienne dla danego identyfikatora (nowy import = nowy identyfikator).
        return FileResponse(path, media_type='model/stl', headers={'Cache-Control': 'private, max-age=31536000, immutable'})
