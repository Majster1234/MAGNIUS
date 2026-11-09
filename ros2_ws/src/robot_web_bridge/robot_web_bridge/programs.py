"""Programy ruchowe (edytor blokowy) i biblioteka rozkazów, zmiennych i pozycji.

Struktura katalogu (domyślnie ``$XDG_STATE_HOME/robot/programs``, parametr ``programs_directory``):

    library/commands.json   rozkazy (definicje bloków); przy pierwszym starcie kopia domyślnej z pakietu
    library/variables.json  biblioteka zmiennych wielokrotnego użytku
    library/positions.json  biblioteka pozycji wielokrotnego użytku
    programs/<id>.json      programy: zmienne, pozycje i drzewo bloków
    programs/.trash/        usunięte programy (kopia z sygnaturą czasu)

Zapis atomowy (plik tymczasowy + fsync + rename), rewizja = skrót treści. Język programu i wykonanie: później.
"""
import hashlib
import json
import math
import os
import re
import shutil
import threading
import time
import unicodedata
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = 1
TYPES = ('INT', 'REAL', 'BOOL', 'POZYCJA', 'TEKST')
NAME = re.compile(r'^[^\W\d]\w{0,39}$')
ID = re.compile(r'^[a-z0-9][a-z0-9_-]{0,63}$')
MAX_BYTES = 1024 * 1024
MAX_BLOCKS = 2000
MAX_DEPTH = 16
MAX_ITEMS = 500
DEFAULT_COMMANDS = Path(__file__).resolve().parent / 'program_library' / 'commands.json'


class ProgramError(ValueError):
    pass


class ProgramConflict(ProgramError):
    pass


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def revision(document):
    # Zatwierdzenie operatora nie wchodzi do rewizji (zatwierdzenie dotyczy konkretnej treści programu).
    body = {k: v for k, v in document.items() if k not in ('revision', 'updated_at', 'approval')}
    return hashlib.sha256(json.dumps(body, sort_keys=True, ensure_ascii=False).encode()).hexdigest()[:16]


def slug(name):
    text = unicodedata.normalize('NFKD', name.replace('ł', 'l').replace('Ł', 'L'))
    text = ''.join(c for c in text if not unicodedata.combining(c)).lower()
    return re.sub(r'[^a-z0-9]+', '_', text).strip('_')[:48] or 'program'


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=1)
        stream.flush(); os.fsync(stream.fileno())
    os.replace(tmp, path)


def finite(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ProgramError(f'{label}: wymagana skończona liczba')
    return value


def text(value, label, limit=200, empty=True):
    if not isinstance(value, str) or len(value) > limit or (not empty and not value.strip()):
        raise ProgramError(f'{label}: wymagany tekst do {limit} znaków')
    return value


def check_name(name, label):
    if not isinstance(name, str) or not NAME.match(name):
        raise ProgramError(f'{label}: nazwa musi zaczynać się literą, zawierać litery, cyfry lub _ (do 40 znaków)')
    return name


def validate_variable(item, label='Zmienna'):
    if not isinstance(item, dict) or set(item) - {'name', 'type', 'init', 'unit', 'note', 'min', 'max'}:
        raise ProgramError(f'{label}: nieprawidłowa struktura')
    name = check_name(item.get('name'), label)
    kind = item.get('type')
    if kind not in TYPES:
        raise ProgramError(f'{label} {name}: typ musi być jednym z {", ".join(TYPES)}')
    init = item.get('init')
    if kind == 'INT' and (type(init) is not int):
        raise ProgramError(f'{label} {name}: wartość INT musi być liczbą całkowitą')
    if kind == 'REAL':
        finite(init, f'{label} {name}')
    if kind == 'BOOL' and type(init) is not bool:
        raise ProgramError(f'{label} {name}: wartość BOOL musi być true/false')
    if kind in ('POZYCJA', 'TEKST'):
        text(init if init is not None else '', f'{label} {name}', 200)
    for key in ('min', 'max'):
        if item.get(key) is not None:
            finite(item[key], f'{label} {name} {key}')
    if item.get('min') is not None and item.get('max') is not None and item['min'] > item['max']:
        raise ProgramError(f'{label} {name}: min większe od max')
    text(item.get('unit', ''), f'{label} {name} jednostka', 16)
    text(item.get('note', ''), f'{label} {name} opis', 200)
    return item


def validate_position(item, label='Pozycja'):
    if not isinstance(item, dict) or set(item) - {'name', 'joints_deg', 'tcp', 'note', 'source', 'created_at'}:
        raise ProgramError(f'{label}: nieprawidłowa struktura')
    name = check_name(item.get('name'), label)
    joints = item.get('joints_deg')
    if not isinstance(joints, list) or len(joints) != 6:
        raise ProgramError(f'{label} {name}: wymagane 6 kątów przegubów [°]')
    for k, value in enumerate(joints):
        if abs(finite(value, f'{label} {name} J{k + 1}')) > 3600:
            raise ProgramError(f'{label} {name} J{k + 1}: kąt poza zakresem ±3600°')
    if item.get('tcp') is not None:
        if not isinstance(item['tcp'], list) or len(item['tcp']) != 6:
            raise ProgramError(f'{label} {name}: TCP wymaga 6 wartości (X, Y, Z mm, Roll, Pitch, Yaw °)')
        for value in item['tcp']:
            finite(value, f'{label} {name} TCP')
    text(item.get('note', ''), f'{label} {name} opis', 200)
    text(item.get('source', ''), f'{label} {name} źródło', 40)
    text(item.get('created_at', ''), f'{label} {name} czas', 40)
    return item


def unique(items, label):
    names = [item['name'] for item in items]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise ProgramError(f'{label}: powtórzone nazwy {", ".join(duplicates)}')
    if len(items) > MAX_ITEMS:
        raise ProgramError(f'{label}: najwyżej {MAX_ITEMS} pozycji listy')


def validate_value(value, label):
    """Wartość parametru: literał albo odwołanie {"var": nazwa} / {"position": nazwa}."""
    if isinstance(value, dict):
        if len(value) != 1 or not ({'var', 'position'} & set(value)):
            raise ProgramError(f'{label}: odwołanie musi mieć postać {{"var": …}} albo {{"position": …}}')
        check_name(next(iter(value.values())), label)
    elif value is not None and type(value) not in (int, float, bool, str):
        raise ProgramError(f'{label}: nieobsługiwany typ wartości')
    elif type(value) in (int, float):
        finite(value, label)
    elif isinstance(value, str):
        text(value, label, 200)


def check_commands(library):
    if not isinstance(library, dict) or library.get('schema_version') != SCHEMA or not isinstance(library.get('commands'), list):
        raise ProgramError('Biblioteka rozkazów: wymagane schema_version 1 i lista commands')
    ids = set()
    for command in library['commands']:
        if not isinstance(command, dict) or not isinstance(command.get('id'), str) or not ID.match(command['id']):
            raise ProgramError('Biblioteka rozkazów: każdy rozkaz wymaga identyfikatora [a-z0-9_-]')
        if command['id'] in ids:
            raise ProgramError(f'Biblioteka rozkazów: powtórzony rozkaz {command["id"]}')
        ids.add(command['id'])
        if not isinstance(command.get('label'), str) or not isinstance(command.get('params', []), list):
            raise ProgramError(f'Rozkaz {command["id"]}: wymagana etykieta i lista parametrów')
    return {c['id']: c for c in library['commands']}


def validate_blocks(blocks, commands, path='Program', depth=0, counter=None, ids=None):
    counter = counter if counter is not None else [0]
    ids = ids if ids is not None else set()
    if not isinstance(blocks, list):
        raise ProgramError(f'{path}: lista bloków wymagana')
    if depth > MAX_DEPTH:
        raise ProgramError(f'{path}: zagnieżdżenie głębsze niż {MAX_DEPTH}')
    for k, block in enumerate(blocks):
        label = f'{path} › blok {k + 1}'
        if not isinstance(block, dict) or not isinstance(block.get('id'), str) or not (1 <= len(block['id']) <= 40):
            raise ProgramError(f'{label}: wymagany identyfikator bloku')
        if block['id'] in ids:
            raise ProgramError(f'{label}: powtórzony identyfikator {block["id"]}')
        ids.add(block['id'])
        command = commands.get(block.get('command'))
        if command is None:
            raise ProgramError(f'{label}: nieznany rozkaz {block.get("command")!r}')
        slots = [s['name'] for s in command.get('slots', [])]
        if set(block) - {'id', 'command', 'params', 'disabled', *slots}:
            raise ProgramError(f'{label}: nieznane pola bloku')
        params = block.get('params', {})
        allowed = {p['name'] for p in command.get('params', [])}
        if not isinstance(params, dict) or set(params) - allowed:
            raise ProgramError(f'{label}: parametry spoza definicji rozkazu {command["id"]}')
        for name, value in params.items():
            validate_value(value, f'{label} {name}')
        counter[0] += 1
        if counter[0] > MAX_BLOCKS:
            raise ProgramError(f'Program: najwyżej {MAX_BLOCKS} bloków')
        for slot in slots:
            validate_blocks(block.get(slot, []), commands, f'{label} › {slot}', depth + 1, counter, ids)


def references(blocks, out=None):
    out = out if out is not None else {'var': set(), 'position': set()}
    for block in blocks:
        for value in block.get('params', {}).values():
            if isinstance(value, dict):
                for key in ('var', 'position'):
                    if key in value:
                        out[key].add(value[key])
        for key, child in block.items():
            if isinstance(child, list):
                references(child, out)
    return out


def param_problems(blocks, commands, out=None):
    """Nieuzupełnione pola i literały spoza zakresu definicji rozkazu (min/max/integer). Blokują zatwierdzenie, nie zapis."""
    out = out if out is not None else []
    for block in blocks:
        command = commands.get(block.get('command'))
        if command is not None:
            for param in command.get('params', []):
                value = block.get('params', {}).get(param['name'])
                label = f'Blok {block["id"]}: {param.get("label", param["name"])}'
                if value is None or value == '':
                    out.append(f'{label} — pole nieuzupełnione')
                elif type(value) in (int, float) and not block.get('disabled'):
                    unit = f' {param["unit"]}' if param.get('unit') else ''
                    if param.get('min') is not None and value < param['min']:
                        out.append(f'{label} — {value}{unit} poniżej minimum {param["min"]}{unit}')
                    elif param.get('max') is not None and value > param['max']:
                        out.append(f'{label} — {value}{unit} powyżej maksimum {param["max"]}{unit}')
                    elif param.get('integer') and value != int(value):
                        out.append(f'{label} — wymagana liczba całkowita')
        for key, child in block.items():
            if isinstance(child, list):
                param_problems(child, commands, out)
    return out


class ProgramStore:
    def __init__(self, directory=None):
        base = Path(directory) if directory else Path(os.environ.get('XDG_STATE_HOME') or str(Path.home() / '.local/state')) / 'robot/programs'
        self.base, self.library, self.programs = base, base / 'library', base / 'programs'
        self.lock = threading.Lock()
        self.library.mkdir(parents=True, exist_ok=True); self.programs.mkdir(parents=True, exist_ok=True)
        if not (self.library / 'commands.json').exists():
            shutil.copyfile(DEFAULT_COMMANDS, self.library / 'commands.json')
        for name in ('variables', 'positions'):
            if not (self.library / f'{name}.json').exists():
                write_json(self.library / f'{name}.json', {'schema_version': SCHEMA, 'items': []})

    # ----- biblioteka -----
    def commands(self):
        try:
            data = json.loads((self.library / 'commands.json').read_text(encoding='utf-8'))
            check_commands(data)
            return data, None
        except (OSError, ValueError) as error:
            data = json.loads(DEFAULT_COMMANDS.read_text(encoding='utf-8'))
            return data, f'library/commands.json nieprawidłowy — użyto domyślnej biblioteki ({error})'

    def library_snapshot(self):
        commands, warning = self.commands()
        out = {'directory': str(self.base), 'commands': commands, 'warning': warning}
        for name in ('variables', 'positions'):
            data = json.loads((self.library / f'{name}.json').read_text(encoding='utf-8'))
            out[name] = {'items': data.get('items', []), 'revision': revision(data)}
        return out

    def set_library(self, name, items, expected=None):
        if name not in ('variables', 'positions'):
            raise KeyError('Nieznana biblioteka')
        if not isinstance(items, list):
            raise ProgramError('Wymagana lista elementów biblioteki')
        check = validate_variable if name == 'variables' else validate_position
        items = [check(deepcopy(item), 'Biblioteka') for item in items]
        unique(items, 'Biblioteka')
        with self.lock:
            path = self.library / f'{name}.json'
            current = json.loads(path.read_text(encoding='utf-8'))
            if expected is not None and expected != revision(current):
                raise ProgramConflict('Biblioteka zmieniła się w międzyczasie; odśwież i powtórz')
            data = {'schema_version': SCHEMA, 'items': items}
            write_json(path, data)
            return {'items': items, 'revision': revision(data)}

    # ----- programy -----
    def path(self, identifier):
        if not isinstance(identifier, str) or not ID.match(identifier):
            raise KeyError('Nieprawidłowy identyfikator programu')
        return self.programs / f'{identifier}.json'

    def validate(self, document):
        if not isinstance(document, dict):
            raise ProgramError('Wymagany obiekt programu')
        allowed = {'schema_version', 'id', 'name', 'description', 'created_at', 'updated_at', 'revision', 'variables', 'positions', 'blocks', 'approval'}
        if set(document) - allowed:
            raise ProgramError(f'Nieznane pola programu: {", ".join(sorted(set(document) - allowed))}')
        if document.get('schema_version', SCHEMA) != SCHEMA:
            raise ProgramError('Nieobsługiwana wersja programu')
        text(document.get('name'), 'Nazwa programu', 80, empty=False)
        text(document.get('description', ''), 'Opis programu', 2000)
        variables = [validate_variable(deepcopy(v)) for v in document.get('variables', [])]
        positions = [validate_position(deepcopy(p)) for p in document.get('positions', [])]
        unique(variables, 'Zmienne'); unique(positions, 'Pozycje')
        commands, _ = self.commands()
        validate_blocks(document.get('blocks', []), check_commands(commands))
        result = deepcopy(document)
        result.pop('approval', None)   # zatwierdzenie ustawia wyłącznie operator (approve), nie zapis programu
        result.update(schema_version=SCHEMA, variables=variables, positions=positions, blocks=document.get('blocks', []),
                      description=document.get('description', ''))
        if len(json.dumps(result, ensure_ascii=False).encode()) > MAX_BYTES:
            raise ProgramError('Program przekracza 1 MiB')
        return result

    def problems(self, document):
        refs = references(document.get('blocks', []))
        names = {v['name'] for v in document.get('variables', [])}
        positions = {p['name'] for p in document.get('positions', [])}
        out = [f'Brak zmiennej „{n}”' for n in sorted(refs['var'] - names)]
        out += [f'Brak pozycji „{n}”' for n in sorted(refs['position'] - positions)]
        commands, _ = self.commands()
        try:
            out += param_problems(document.get('blocks', []), check_commands(commands))
        except ProgramError:
            pass
        return out

    def summary(self, document):
        def count(blocks):
            return sum(1 + sum(count(v) for v in b.values() if isinstance(v, list)) for b in blocks)
        return {key: document.get(key) for key in ('id', 'name', 'description', 'created_at', 'updated_at', 'revision')} | {
            'approved': self.approved(document),
            'blocks': count(document.get('blocks', [])), 'variables': len(document.get('variables', [])),
            'positions': len(document.get('positions', []))}

    def list(self):
        items = []
        for path in sorted(self.programs.glob('*.json')):
            try:
                items.append(self.summary(json.loads(path.read_text(encoding='utf-8'))))
            except (OSError, ValueError):
                items.append({'id': path.stem, 'name': path.stem, 'broken': True})
        return sorted(items, key=lambda item: item.get('updated_at') or '', reverse=True)

    def get(self, identifier):
        path = self.path(identifier)
        if not path.exists():
            raise KeyError('Program nie istnieje')
        document = json.loads(path.read_text(encoding='utf-8'))
        return {'program': document, 'problems': self.problems(document)}

    def create(self, document):
        document = dict(document or {})
        document.setdefault('variables', []); document.setdefault('positions', []); document.setdefault('blocks', [])
        document = self.validate(document)
        with self.lock:
            base = slug(document['name']); identifier = base; k = 2
            while self.path(identifier).exists():
                identifier = f'{base}_{k}'; k += 1
            stamp = now_iso()
            document.update(id=identifier, created_at=stamp, updated_at=stamp)
            document['revision'] = revision(document)
            write_json(self.path(identifier), document)
        return {'program': document, 'problems': self.problems(document)}

    def save(self, identifier, document, expected):
        path = self.path(identifier)
        document = self.validate(document)
        with self.lock:
            if not path.exists():
                raise KeyError('Program nie istnieje')
            current = json.loads(path.read_text(encoding='utf-8'))
            if expected != current.get('revision'):
                raise ProgramConflict('Program został zmieniony w innym oknie; odśwież przed zapisem')
            document.update(id=identifier, created_at=current.get('created_at'), updated_at=now_iso())
            document['revision'] = revision(document)
            if document['revision'] == current.get('revision'):
                return {'program': current, 'problems': self.problems(current), 'unchanged': True}
            if current.get('approval'):
                document['approval'] = current['approval']   # zostaje w pliku jako historia, ale nie pasuje do nowej rewizji
            write_json(path, document)
        return {'program': document, 'problems': self.problems(document)}

    @staticmethod
    def approved(document):
        approval = document.get('approval') or {}
        return bool(approval.get('revision')) and approval.get('revision') == document.get('revision')

    def check(self, document):
        document = self.validate(document)
        document['revision'] = revision(document)
        return {'problems': self.problems(document), 'revision': document['revision']}

    def approve(self, identifier, expected, client=''):
        """Zatwierdzenie operatora: konkretna rewizja programu może zostać wykonana (scripts/run_program.py)."""
        path = self.path(identifier)
        with self.lock:
            if not path.exists():
                raise KeyError('Program nie istnieje')
            document = json.loads(path.read_text(encoding='utf-8'))
            if expected != document.get('revision'):
                raise ProgramConflict('Zatwierdzana rewizja różni się od zapisanej; zapisz i odśwież program')
            problems = self.problems(document)
            if problems:
                raise ProgramError('Program ma nierozwiązane uwagi: ' + '; '.join(problems[:5]))
            document['approval'] = {'revision': document['revision'], 'approved_at': now_iso(), 'client': str(client)[:64]}
            write_json(path, document)
        return {'program': document, 'problems': [], 'approved': True}

    def revoke(self, identifier):
        path = self.path(identifier)
        with self.lock:
            if not path.exists():
                raise KeyError('Program nie istnieje')
            document = json.loads(path.read_text(encoding='utf-8'))
            document.pop('approval', None)
            write_json(path, document)
        return {'program': document, 'problems': self.problems(document), 'approved': False}

    def delete(self, identifier):
        path = self.path(identifier)
        with self.lock:
            if not path.exists():
                raise KeyError('Program nie istnieje')
            trash = self.programs / '.trash'; trash.mkdir(exist_ok=True)
            os.replace(path, trash / f'{identifier}-{time.strftime("%Y%m%d-%H%M%S")}.json')
        return {'deleted': identifier}


def add_program_routes(app, store):
    from fastapi import HTTPException, Request
    from starlette.responses import JSONResponse

    def reply(data, status=200):
        return JSONResponse(data, status_code=status, headers={'Cache-Control': 'no-store'})

    async def body(request):
        raw = bytearray()
        async for chunk in request.stream():
            raw.extend(chunk)
            if len(raw) > MAX_BYTES + 4096:
                raise HTTPException(413, 'Treść przekracza 1 MiB')
        try:
            return json.loads(raw or b'{}')
        except ValueError:
            raise HTTPException(422, 'Nieprawidłowy JSON') from None

    def call(function, *args, status=200):
        if store is None:
            raise HTTPException(503, 'Magazyn programów jest niedostępny')
        try:
            return reply(function(*args), status)
        except ProgramConflict as error:
            raise HTTPException(409, str(error)) from None
        except ProgramError as error:
            raise HTTPException(422, str(error)) from None
        except KeyError as error:
            raise HTTPException(404, str(error.args[0] if error.args else 'Nie znaleziono')) from None
        except OSError as error:
            raise HTTPException(500, f'Błąd zapisu programu: {error}') from None

    @app.get('/api/v1/program-library')
    async def library():
        return call(store.library_snapshot if store else None)

    @app.put('/api/v1/program-library/{name}')
    async def set_library(name: str, request: Request):
        data = await body(request)
        return call(store.set_library if store else None, name, data.get('items'), data.get('expected_revision'))

    @app.get('/api/v1/programs')
    async def programs():
        return call(lambda: {'items': store.list(), 'directory': str(store.programs)} if store else None)

    @app.post('/api/v1/programs')
    async def create(request: Request):
        data = await body(request)
        return call(store.create if store else None, data.get('program', data), status=201)

    @app.post('/api/v1/programs/check')
    async def check(request: Request):
        data = await body(request)
        return call(store.check if store else None, data.get('program', data))

    @app.post('/api/v1/programs/{identifier}/approve')
    async def approve(identifier: str, request: Request):
        data = await body(request)
        client = request.client.host if request.client else ''
        return call(store.approve if store else None, identifier, data.get('revision'), client)

    @app.delete('/api/v1/programs/{identifier}/approve')
    async def revoke(identifier: str):
        return call(store.revoke if store else None, identifier)

    @app.get('/api/v1/programs/{identifier}')
    async def get(identifier: str):
        return call(store.get if store else None, identifier)

    @app.put('/api/v1/programs/{identifier}')
    async def save(identifier: str, request: Request):
        data = await body(request)
        return call(store.save if store else None, identifier, data.get('program'), data.get('expected_revision'))

    @app.delete('/api/v1/programs/{identifier}')
    async def delete(identifier: str):
        return call(store.delete if store else None, identifier)
