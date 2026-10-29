"""Trwały zapis JSON przez fsync + rename. Snapshoty nie są bazą danych ani historią klawiszy."""
import heapq
import json
import os
from pathlib import Path
import re
import threading
import uuid
from .schema import MAX_BYTES, empty_document, now_iso, validate

IDENTIFIER = re.compile(r'\d{8}T\d{9}-[a-f0-9]{32}')


def decode(text):
    if not isinstance(text, str) or len(text.encode('utf-8')) > MAX_BYTES:
        raise ValueError('Plik JSON przekracza 64 KiB')
    return json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f'Nieprawidłowa liczba {value}')))


class Storage:
    def __init__(self, directory):
        self.root = Path(directory).expanduser()
        self.snapshots = self.root / 'snapshots'
        self.snapshots.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.lock = threading.RLock()

    def atomic(self, name, value):
        target = self.root / name
        temporary = target.with_name('.' + target.name + '.' + uuid.uuid4().hex + '.tmp')
        try:
            with temporary.open('x', encoding='utf-8') as stream:
                os.chmod(temporary, 0o600)
                json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, target)
            descriptor = os.open(target.parent, os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            temporary.unlink(missing_ok=True)

    def active(self):
        path = self.root / 'active.json'
        if not path.exists():
            return {'revision': 'initial', 'document': empty_document()}
        saved = decode(path.read_text(encoding='utf-8'))
        if not isinstance(saved, dict) or not isinstance(saved.get('revision'), str) or len(saved['revision']) > 64:
            raise ValueError('Nieprawidłowy plik aktywnej konfiguracji; wymagane przywrócenie kopii')
        return {'revision': saved['revision'], 'document': validate(saved['document'])}

    def system_settings(self):
        with self.lock:
            path = self.root / 'system_settings.json'
            value = decode(path.read_text(encoding='utf-8')) if path.exists() else {'revision': 'initial', 'ignore_missing_drivers': False}
            if (not isinstance(value, dict) or type(value.get('ignore_missing_drivers')) is not bool or
                    not isinstance(value.get('revision'), str) or not 1 <= len(value['revision']) <= 64):
                raise ValueError('Nieprawidłowe ustawienia systemowe')
            value.setdefault('allow_missing_motion', False)
            if type(value['allow_missing_motion']) is not bool:
                raise ValueError('Nieprawidłowy tryb ruchu z niepełnym zestawem napędów')
            return value

    def set_system_settings(self, enabled=None, *, allow_missing_motion=None):
        updates = {}
        for key, value in (('ignore_missing_drivers', enabled), ('allow_missing_motion', allow_missing_motion)):
            if value is not None:
                if type(value) is not bool:
                    raise ValueError(f'{key} wymaga wartości logicznej')
                updates[key] = value
        if not updates:
            raise ValueError('Wymagane ustawienie logiczne')
        with self.lock:
            current = self.system_settings()
            if all(current[key] == value for key, value in updates.items()):
                return current
            result = {**current, **updates, 'revision': uuid.uuid4().hex}
            self.atomic('system_settings.json', result)
            return result

    def create(self, document):
        document = validate(document)
        document['created_at'] = now_iso()
        if not document['name']:
            document['name'] = 'Snapshot ' + document['created_at']
        identifier = document['created_at'].replace('-', '').replace(':', '').replace('.', '').replace('Z', '') + '-' + uuid.uuid4().hex
        metadata = {'id': identifier, 'name': document['name'], 'created_at': document['created_at']}
        with self.lock:
            self.atomic(f'snapshots/{identifier}.json', document)
            try:
                self.atomic(f'snapshots/{identifier}.meta.json', metadata)
            except Exception:
                (self.snapshots / f'{identifier}.json').unlink(missing_ok=True)
                raise
        return metadata

    def checked_id(self, identifier):
        if not isinstance(identifier, str) or not IDENTIFIER.fullmatch(identifier):
            raise ValueError('Nieprawidłowy identyfikator snapshotu')
        return identifier

    def list(self, cursor=''):
        if cursor:
            self.checked_id(cursor)
        with self.lock:
            # Stałe 21 nazw w pamięci, także przy dużej bibliotece snapshotów.
            candidates = (path.name[:-10] for path in self.snapshots.glob('*.meta.json')
                          if IDENTIFIER.fullmatch(path.name[:-10]) and (not cursor or path.name[:-10] < cursor))
            selected = heapq.nlargest(21, candidates)
            items = []
            for identifier in selected[:20]:
                if not (self.snapshots / f'{identifier}.json').is_file():
                    continue
                items.append(decode((self.snapshots / f'{identifier}.meta.json').read_text(encoding='utf-8')))
            return {'items': items, 'next_cursor': selected[19] if len(selected) > 20 else None}

    def get(self, identifier):
        path = self.snapshots / f'{self.checked_id(identifier)}.json'
        with self.lock:
            return validate(decode(path.read_text(encoding='utf-8')))

    def delete(self, identifier):
        identifier = self.checked_id(identifier)
        with self.lock:
            (self.snapshots / f'{identifier}.json').unlink()
            (self.snapshots / f'{identifier}.meta.json').unlink(missing_ok=True)

    def journal(self):
        path = self.root / 'pending.json'
        return decode(path.read_text(encoding='utf-8')) if path.exists() else None

    def clear_journal(self):
        (self.root / 'pending.json').unlink(missing_ok=True)

    def calibration(self, revision):
        path = self.root / 'calibration.json'
        if not path.exists():
            return [False] * 6
        record = decode(path.read_text(encoding='utf-8'))
        if record.get('revision') != revision:
            return [False] * 6
        flags = record.get('invalid')
        if not isinstance(flags, list) or len(flags) != 6 or any(type(flag) is not bool for flag in flags):
            raise ValueError('Nieprawidłowy stan kalibracji; wymagane jawne zatwierdzenie odniesienia')
        return flags
