"""Kopie zapasowe nastaw sterowników MKS SERVO42D.

Trzy kontenery na dysku (katalog drive-backups obok katalogu konfiguracji):
- base.json      — nastawy podstawowe, NIETYKALNE: tworzone jeden raz z odczytu napędów, plik tylko do odczytu,
                   suma SHA-256; nadpisanie przez API jest niemożliwe (zmiana wymaga ręcznego usunięcia pliku);
- pre_test.json  — kopia stanu przed testami; nowa kopia archiwizuje poprzednią w archive/;
- test.json      — nastawy testowe (tylko podzbiór TEST_PARAMETERS z walidacją zakresów).

Oś Jn = adres CAN n (1–6). Przywracanie zapisuje do każdego adresu wyłącznie jego własne nastawy:
odczyt bieżący → zapis tylko różnic (kolejność RESTORE_ORDER) → odczyt kontrolny. Nastawy tylko do odczytu
(bitrate, tryb odpowiedzi, SYNC, brak CRC, bazowanie, autostart) są w kopii i w porównaniu, ale nie są zapisywane.
Zapis wymaga bezczynności i ENABLE=0 osi (wyjątek: PID 96h/97h także przy ENABLE=1) — egzekwuje to właściciel CAN (C++).
Kolejne kandydaty testowe po jednej kopii pre_test: zgodność z pre_test sprawdzana poza nastawami bieżącego kontenera test.
"""
import copy
import hashlib
import json
import os
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = 1
AXES = range(1, 7)
# Zgodne z mks_servo_can/drive_parameters.cpp: (kod, sub, długość, zapisywalna, nazwa).
CATALOG = (
    (0x82, -1, 1, True, 'work_mode'), (0x83, -1, 2, True, 'work_current_ma'), (0x9b, -1, 1, True, 'hold_current_ratio'),
    (0x84, -1, 1, True, 'microstep'), (0x85, -1, 1, True, 'en_active_level'), (0x86, -1, 1, True, 'direction'),
    (0x87, -1, 1, True, 'auto_screen_off'), (0x88, -1, 1, True, 'stall_protection'),
    (0x89, -1, 1, True, 'subdivision_interpolation'), (0x8a, -1, 1, False, 'can_bitrate'),
    (0x8c, -1, 2, False, 'response_mode'), (0x8d, -1, 2, True, 'group_can_id'), (0x8f, -1, 1, True, 'key_lock'),
    (0x95, -1, 3, True, 'position_reached_threshold'), (0x96, 0, 5, True, 'vfoc_kp_ki'), (0x96, 1, 5, True, 'vfoc_kd_kv'),
    (0x97, 0, 5, True, 'close_kp_ki'), (0x97, 1, 5, True, 'close_kd_kv'), (0x98, -1, 4, True, 'heartbeat_ms'),
    (0x9d, -1, 5, True, 'position_error_protection'), (0x9f, -1, 1, True, 'in1_port_mode'), (0x42, -1, 4, True, 'user_id'),
    (0x4a, -1, 1, False, 'sync_mode'), (0x0f, -1, 1, False, 'no_crc_mode'), (0x90, -1, 6, False, 'home_parameters'),
    (0x94, -1, 6, False, 'home_offset_current'), (0x9a, -1, 4, False, 'single_turn_home'),
    (0x9e, -1, 1, False, 'limit_port_mapping'), (0x99, -1, 5, False, 'pulse_divider_output'),
    (0xff, -1, 1, False, 'power_on_autorun'),
)
SPECS = {name: dict(code=code, sub=sub, length=length, writable=writable) for code, sub, length, writable, name in CATALOG}
# Kolejność zapisu: tryb i mikrokrok przed prądami i PID, zabezpieczenia na końcu ruchowych, reszta potem.
RESTORE_ORDER = ('work_mode', 'microstep', 'work_current_ma', 'hold_current_ratio', 'vfoc_kp_ki', 'vfoc_kd_kv',
                 'close_kp_ki', 'close_kd_kv', 'stall_protection', 'position_error_protection',
                 'position_reached_threshold', 'heartbeat_ms', 'en_active_level', 'direction', 'auto_screen_off',
                 'subdivision_interpolation', 'key_lock', 'group_can_id', 'in1_port_mode', 'user_id')
assert set(RESTORE_ORDER) == {name for name, spec in SPECS.items() if spec['writable']}


def _u16(value, low=0, high=0xffff):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'wymagana liczba całkowita {low}–{high}')
    return [value >> 8, value & 0xff]


def _choice(value, allowed):
    if type(value) is not int or value not in allowed:
        raise ValueError(f'dozwolone wartości: {", ".join(str(a) for a in allowed)}')
    return [value]


def _pid(sub):
    def encode(value):
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError('wymagana para [a, b] w zakresie 0–1024')
        return [sub] + _u16(value[0], 0, 1024) + _u16(value[1], 0, 1024)
    return encode


def _protection(value):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError('wymagane [enable 0/1, czas (×15 ms), liczba błędów]')
    return _choice(value[0], (0, 1)) + _u16(value[1]) + _u16(value[2])


def _threshold(value):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('wymagane [enable 0/1, próg 0–65535]')
    return _choice(value[0], (0, 1)) + _u16(value[1])


# Nastawy dopuszczone w kontenerze testowym (kierunek, poziom EN, adresy grup, heartbeat — tylko przywracanie).
TEST_PARAMETERS = {
    'work_mode': lambda v: _choice(v, (4, 5)),
    'work_current_ma': lambda v: _u16(v, 1, 3000),
    'hold_current_ratio': lambda v: _choice(v, tuple(range(9))),
    'microstep': lambda v: _choice(v, (16, 32, 64, 128)),
    'subdivision_interpolation': lambda v: _choice(v, (0, 1)),
    'stall_protection': lambda v: _choice(v, (0, 1)),
    'vfoc_kp_ki': _pid(0), 'vfoc_kd_kv': _pid(1), 'close_kp_ki': _pid(0), 'close_kd_kv': _pid(1),
    'position_error_protection': _protection,
    'position_reached_threshold': _threshold,
}
MODES = {0: 'CR_OPEN', 1: 'CR_CLOSE', 2: 'CR_vFOC', 3: 'SR_OPEN', 4: 'SR_CLOSE', 5: 'SR_vFOC'}


def describe(name, value_hex):
    """Wartość czytelna dla człowieka (tylko do podglądu; źródłem prawdy są bajty)."""
    if value_hex is None:
        return None
    b = bytes.fromhex(value_hex)
    if name == 'work_mode':
        return MODES.get(b[0], b[0])
    if name in ('work_current_ma', 'group_can_id'):
        return int.from_bytes(b, 'big')
    if name == 'hold_current_ratio':
        return f'{(b[0] + 1) * 10}%'
    if name == 'microstep':
        return 256 if b[0] == 0 else b[0]
    if name in ('vfoc_kp_ki', 'vfoc_kd_kv', 'close_kp_ki', 'close_kd_kv'):
        return [int.from_bytes(b[1:3], 'big'), int.from_bytes(b[3:5], 'big')]
    if name in ('heartbeat_ms', 'user_id'):
        return int.from_bytes(b, 'big')
    if name == 'position_error_protection':
        return [b[0], int.from_bytes(b[1:3], 'big'), int.from_bytes(b[3:5], 'big')]
    if name == 'position_reached_threshold':
        return [b[0], int.from_bytes(b[1:3], 'big')]
    if name == 'can_bitrate':
        return {0: '125K', 1: '250K', 2: '500K', 3: '1M'}.get(b[0], b[0])
    return list(b) if len(b) > 1 else b[0]


def _now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def _digest(axes):
    return hashlib.sha256(json.dumps(axes, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


class Conflict(RuntimeError):
    """Warunek bezpieczeństwa niespełniony (HTTP 409)."""


class DriveBackups:
    KINDS = ('base', 'pre_test', 'test')

    def __init__(self, root, device):
        """device: read(address, code, sub) -> (supported, bytes) ; write(address, code, sub, bytes) -> None (wyjątek przy błędzie)."""
        self.root = Path(root).expanduser()
        (self.root / 'archive').mkdir(mode=0o700, parents=True, exist_ok=True)
        self.device = device
        self.lock = threading.Lock()
        self.operation = {'id': '', 'action': '', 'phase': 'idle', 'message': 'Brak operacji', 'success': None,
                          'progress': 0.0, 'result': None}

    # --- pliki ---
    def path(self, kind):
        if kind not in self.KINDS:
            raise ValueError('Nieznany kontener nastaw')
        return self.root / f'{kind}.json'

    def _write(self, path, value, readonly=False):
        temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.chmod(temporary, 0o400 if readonly else 0o600)
        if readonly:
            # Bez nadpisywania: link() zawodzi, jeśli plik już istnieje (wyścig dwóch zapisów kopii bazowej).
            try:
                os.link(temporary, path)
            finally:
                temporary.unlink()
        else:
            os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def load(self, kind):
        path = self.path(kind)
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('schema') != SCHEMA or data.get('kind') != kind or set(data.get('axes', {})) != {str(a) for a in AXES}:
            raise ValueError(f'Kontener {kind}: nieprawidłowa struktura')
        if data.get('sha256') != _digest(data['axes']):
            raise ValueError(f'Kontener {kind}: suma SHA-256 niezgodna — plik uszkodzony lub zmieniony ręcznie')
        for axis, entry in data['axes'].items():
            if entry.get('address') != int(axis):
                raise ValueError(f'Kontener {kind}: J{axis} przypisany do innego adresu CAN')
        return data

    def audit(self, event, **fields):
        row = {'utc': _now(), 'event': event, **fields}
        with (self.root / 'audit.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n'); stream.flush(); os.fsync(stream.fileno())

    # --- odczyt / porównanie ---
    def read_axis(self, address):
        parameters, unsupported = {}, []
        for name, spec in SPECS.items():
            supported, value = self.device.read(address, spec['code'], spec['sub'])
            if supported:
                if len(value) != spec['length']:
                    raise RuntimeError(f'J{address} {name}: nieoczekiwana długość {len(value)}')
                parameters[name] = bytes(value).hex()
            else:
                unsupported.append(name)
        return {'address': address, 'parameters': parameters, 'unsupported': sorted(unsupported)}

    def read_all(self, progress=None):
        axes = {}
        for n, address in enumerate(AXES):
            axes[str(address)] = self.read_axis(address)
            if progress:
                progress((n + 1) / len(AXES))
        return axes

    @staticmethod
    def differences(reference, live):
        out = {}
        for axis in reference:
            ref, cur = reference[axis]['parameters'], live[axis]['parameters']
            diff = {name: {'expected': ref.get(name), 'actual': cur.get(name), 'writable': SPECS[name]['writable'],
                           'expected_value': describe(name, ref.get(name)), 'actual_value': describe(name, cur.get(name))}
                    for name in SPECS if name in ref and ref.get(name) != cur.get(name)}
            if diff:
                out[axis] = diff
        return out

    # --- kontenery ---
    def capture(self, kind, progress=None):
        if kind not in ('base', 'pre_test'):
            raise ValueError('Kopię z napędów tworzy się tylko dla base i pre_test')
        if kind == 'base' and self.path('base').exists():
            raise Conflict('Kopia podstawowa jest nietykalna i już istnieje; zmiana wymaga ręcznego usunięcia pliku przez operatora')
        axes = self.read_all(progress)
        value = {'schema': SCHEMA, 'kind': kind, 'created_utc': _now(), 'axes': axes, 'sha256': _digest(axes)}
        if kind == 'pre_test' and self.path('pre_test').exists():
            previous = self.load('pre_test')
            stamp = previous['created_utc'].replace(':', '').replace('-', '').replace('.', '')
            self._write(self.root / 'archive' / f'pre_test-{stamp}.json', previous)
        self._write(self.path(kind), value, readonly=kind == 'base')
        self.audit('capture', kind=kind, sha256=value['sha256'])
        return {'kind': kind, 'created_utc': value['created_utc'], 'sha256': value['sha256'],
                'unsupported': {a: e['unsupported'] for a, e in axes.items() if e['unsupported']}}

    def set_test(self, request):
        axes = request.get('axes') if isinstance(request, dict) else None
        if not isinstance(axes, dict) or not axes or set(axes) - {str(a) for a in AXES}:
            raise ValueError('Wymagane axes: {"1".."6": {nazwa: wartość}}')
        encoded = {}
        for axis, params in axes.items():
            if not isinstance(params, dict) or not params:
                raise ValueError(f'J{axis}: wymagany słownik nastaw')
            encoded[axis] = {'address': int(axis), 'values': {}, 'parameters': {}}
            for name, value in params.items():
                if name not in TEST_PARAMETERS:
                    raise ValueError(f'J{axis}: nastawa {name} niedozwolona w kontenerze testowym')
                try:
                    data = TEST_PARAMETERS[name](value)
                except ValueError as error:
                    raise ValueError(f'J{axis} {name}: {error}') from None
                assert len(data) == SPECS[name]['length']
                encoded[axis]['values'][name] = value
                encoded[axis]['parameters'][name] = bytes(data).hex()
        full = {str(a): encoded.get(str(a), {'address': a, 'values': {}, 'parameters': {}}) for a in AXES}
        value = {'schema': SCHEMA, 'kind': 'test', 'created_utc': _now(), 'axes': full, 'sha256': _digest(full)}
        self._write(self.path('test'), value)
        self.audit('test_set', axes={a: e['values'] for a, e in full.items() if e['values']})
        return value

    def _write_verify(self, address, name, target_hex):
        spec = SPECS[name]
        self.device.write(address, spec['code'], spec['sub'], list(bytes.fromhex(target_hex)))
        supported, value = self.device.read(address, spec['code'], spec['sub'])
        if not supported or bytes(value).hex() != target_hex:
            raise RuntimeError(f'J{address} {name}: odczyt kontrolny {bytes(value).hex() if supported else "brak"} ≠ {target_hex}')

    def _apply(self, address, targets, source):
        """Zapis różnic jednej osi; zwraca listę zapisanych nastaw (wyjątek przerywa — stan reszty bez zmian)."""
        written = []
        live = self.read_axis(address)['parameters']
        for name in RESTORE_ORDER:
            target = targets.get(name)
            if target is None or live.get(name) == target:
                continue
            self._write_verify(address, name, target)
            written.append(name)
            self.audit('write', source=source, address=address, parameter=name, before=live.get(name), after=target)
        return written

    def restore(self, source, axes=None, progress=None):
        container = self.load(source) if source in ('base', 'pre_test') else None
        if container is None:
            raise Conflict(f'Brak kontenera {source} — przywracanie niemożliwe')
        selected = [int(a) for a in (axes or AXES)]
        if any(a not in AXES for a in selected):
            raise ValueError('Osie 1–6')
        report = {}
        for n, address in enumerate(selected):
            entry = container['axes'][str(address)]
            targets = {k: v for k, v in entry['parameters'].items() if SPECS[k]['writable']}
            try:
                written = self._apply(address, targets, source)
                report[str(address)] = {'written': written, 'error': None}
            except Exception as error:
                report[str(address)] = {'written': [], 'error': str(error)}
                self.audit('restore_failed', source=source, address=address, error=str(error))
            if progress:
                progress((n + 1) / len(selected))
        live = {str(a): self.read_axis(a) for a in selected}
        reference = {str(a): container['axes'][str(a)] for a in selected}
        remaining = self.differences(reference, live)
        self.audit('restore', source=source, axes=selected, remaining=list(remaining))
        return {'source': source, 'axes': report, 'remaining_differences': remaining}

    def apply_test(self, progress=None):
        base, pre, test = self.load('base'), self.load('pre_test'), self.load('test')
        if base is None:
            raise Conflict('Brak nietykalnej kopii podstawowej (base) — zapis nastaw testowych zablokowany')
        if pre is None:
            raise Conflict('Brak kopii przed testami (pre_test) — zapis nastaw testowych zablokowany')
        if test is None or not any(e['parameters'] for e in test['axes'].values()):
            raise Conflict('Brak nastaw testowych')
        selected = [int(a) for a, e in test['axes'].items() if e['parameters']]
        live = {str(a): self.read_axis(a) for a in selected}
        # Nastawy objęte kontenerem testowym mogą już nosić poprzedniego kandydata — i tak zostaną nadpisane.
        reference = {str(a): {'parameters': {k: v for k, v in pre['axes'][str(a)]['parameters'].items()
                                             if k not in test['axes'][str(a)]['parameters']}} for a in selected}
        stale = self.differences(reference, live)
        if stale:
            raise Conflict('Stan napędów różni się od kopii przed testami — wykonaj nową kopię pre_test przed testem: ' +
                           ', '.join(f'J{a}: {", ".join(d)}' for a, d in stale.items()))
        report, failed = {}, None
        for n, address in enumerate(selected):
            try:
                report[str(address)] = {'written': self._apply(address, test['axes'][str(address)]['parameters'], 'test'), 'error': None}
            except Exception as error:
                failed = address
                report[str(address)] = {'written': [], 'error': str(error)}
                break
            if progress:
                progress((n + 1) / len(selected))
        if failed is not None:
            # Awaryjny powrót wszystkich dotkniętych osi do kopii przed testami.
            touched = [int(a) for a in report]
            rollback = self.restore('pre_test', touched)
            self.audit('test_apply_failed', address=failed, rollback=rollback['axes'])
            return {'applied': False, 'axes': report, 'rollback': rollback}
        self.audit('test_apply', axes={a: r['written'] for a, r in report.items()})
        return {'applied': True, 'axes': report}

    def verify(self, kind, progress=None):
        container = self.load(kind)
        if container is None:
            raise Conflict(f'Brak kontenera {kind}')
        live = self.read_all(progress)
        if kind == 'test':
            reference = {a: {'parameters': e['parameters']} for a, e in container['axes'].items()}
        else:
            reference = container['axes']
        return {'kind': kind, 'differences': self.differences(reference, live)}

    # --- stan i operacje asynchroniczne ---
    def status(self):
        containers = {}
        for kind in self.KINDS:
            try:
                data = self.load(kind)
                containers[kind] = None if data is None else {
                    'created_utc': data['created_utc'], 'sha256': data['sha256'], 'valid': True,
                    'readonly': kind == 'base' and not os.access(self.path(kind), os.W_OK),
                    'axes': {a: {name: {'hex': v, 'value': describe(name, v)} for name, v in e['parameters'].items()}
                             for a, e in data['axes'].items()},
                    'unsupported': {a: e.get('unsupported', []) for a, e in data['axes'].items() if e.get('unsupported')}}
            except (ValueError, OSError, KeyError) as error:
                containers[kind] = {'valid': False, 'error': str(error)}
        with self.lock:
            operation = copy.deepcopy(self.operation)
        return {'containers': containers, 'operation': operation, 'directory': str(self.root),
                'test_parameters': sorted(TEST_PARAMETERS), 'restore_order': list(RESTORE_ORDER)}

    def start(self, action, data):
        actions = {
            'capture_base': lambda p: self.capture('base', p),
            'capture_pre_test': lambda p: self.capture('pre_test', p),
            'apply_test': lambda p: self.apply_test(p),
            'restore': lambda p: self.restore(data.get('source'), data.get('axes'), p),
            'verify': lambda p: self.verify(data.get('kind', 'base'), p),
        }
        if action not in actions:
            raise ValueError('Nieznana operacja kopii nastaw')
        if action == 'capture_base' and self.path('base').exists():
            raise Conflict('Kopia podstawowa jest nietykalna i już istnieje; zmiana wymaga ręcznego usunięcia pliku przez operatora')
        if action == 'restore' and data.get('source') not in ('base', 'pre_test'):
            raise ValueError('Przywracanie z base albo pre_test')
        with self.lock:
            if self.operation['phase'] == 'running':
                raise Conflict('Inna operacja na nastawach napędów jeszcze trwa')
            self.operation = {'id': uuid.uuid4().hex, 'action': action, 'phase': 'running', 'message': 'W toku',
                              'success': None, 'progress': 0.0, 'result': None, 'started_utc': _now()}
            snapshot = copy.deepcopy(self.operation)

        def progress(value):
            with self.lock:
                self.operation['progress'] = round(value, 3)

        def run():
            try:
                result = actions[action](progress)
                ok = not (isinstance(result, dict) and (result.get('applied') is False or result.get('remaining_differences')
                                                        or any(r.get('error') for r in result.get('axes', {}).values()
                                                               if isinstance(r, dict))))
                message = 'Zakończono' if ok else 'Zakończono z różnicami lub błędami — sprawdź wynik'
            except Exception as error:
                result, ok, message = None, False, str(error)
                self.audit('operation_failed', action=action, error=message)
            with self.lock:
                self.operation.update(phase='done' if ok else 'failed', success=ok, message=message, result=result,
                                      progress=1.0 if ok else self.operation['progress'], finished_utc=_now())
        threading.Thread(target=run, name=f'drive-backup-{action}', daemon=True).start()
        return snapshot
