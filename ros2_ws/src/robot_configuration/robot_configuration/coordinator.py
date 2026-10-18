"""Transakcja: blokada C++, opis ROS, commit, trwały zapis, zwolnienie blokady.

Transport ROS jest wstrzykiwany; testy błędów nie wymagają ROS ani napędów.
"""
from copy import deepcopy
import threading
import uuid
from .schema import build_urdf, changed_axes, validate, validate_transition


class Conflict(ValueError):
    pass


class Coordinator:
    def __init__(self, storage, transport):
        self.storage, self.transport = storage, transport
        self.active = storage.active()
        self.lock = threading.RLock()
        self.operation = {'id': '', 'phase': 'starting', 'message': 'Odtwarzanie konfiguracji w ROS', 'success': None}
        self.running = True

    def snapshot(self):
        with self.lock:
            return deepcopy({'active': self.active, 'operation': self.operation})

    def phase(self, phase, message, success=None):
        with self.lock:
            self.operation.update(phase=phase, message=message, success=success)

    def bootstrap(self):
        """Nowy runtime zaczyna z blokadą; restart koordynatora uzgadnia istniejącą transakcję."""
        operation_id = uuid.uuid4().hex
        try:
            active = self.storage.active()  # FINISH mógł zaginąć po trwałym zapisie nowej rewizji.
            status = self.transport.control('status', operation_id, active, None)
            journal = self.storage.journal()
            if status['locked'] and status['stage'] != 'boot':
                if not journal or journal['id'] != status['operation_id']:
                    raise RuntimeError('Runtime ma nierozpoznaną transakcję; sterowanie pozostaje zablokowane')
                operation_id = journal['id']
                if status['revision'] == active['revision'] and status['stage'] == 'committed':
                    self.transport.model(build_urdf(active['document'], active['revision']))
                    self.transport.control('finish', operation_id, active, None)
                else:
                    active = deepcopy(journal['previous'])
                    active['document'] = validate(active['document'])
                    self.transport.control('rollback', operation_id, active, None)
                    self.transport.model(build_urdf(active['document'], active['revision']))
                    self.storage.atomic('calibration.json', {'revision': active['revision'], 'invalid': journal.get('previous_calibration', [False] * 6)})
                    self.storage.atomic('active.json', active)
                    self.transport.control('finish', operation_id, active, None)
            elif status['stage'] == 'boot':
                if journal and journal['previous']['revision'] == active['revision'] and 'previous_calibration' in journal:
                    self.storage.atomic('calibration.json', {'revision': active['revision'], 'invalid': journal['previous_calibration']})
                self.storage.atomic('pending.json', {'id': operation_id, 'previous': active, 'candidate': active,
                                                    'previous_calibration': self.storage.calibration(active['revision'])})
                self.transport.control('bootstrap', operation_id, active, [True] * 6)
                self.transport.model(build_urdf(active['document'], active['revision']))
                self.storage.atomic('active.json', active)
                self.transport.control('finish', operation_id, active, None)
            elif status['revision'] != active['revision']:
                # Zakończony rollback/commit mógł stracić ACK i zapis na dysku.
                matching = next((journal[key] for key in ('previous', 'candidate') if journal and journal[key]['revision'] == status['revision']), None)
                if matching is None:
                    raise RuntimeError('Rewizja runtime różni się od pliku; nie znaleziono zgodnego dokumentu w dzienniku')
                active = deepcopy(matching); active['document'] = validate(active['document'])
                self.transport.model(build_urdf(active['document'], active['revision']))
                self.storage.atomic('active.json', active)
                if journal['previous']['revision'] == active['revision']:
                    self.storage.atomic('calibration.json', {'revision': active['revision'], 'invalid': journal.get('previous_calibration', [False] * 6)})
            else:
                self.transport.model(build_urdf(active['document'], active['revision']))
            with self.lock:
                self.active = active
            self.storage.clear_journal()
            self.phase('idle', 'Konfiguracja ROS gotowa', True)
        except Exception as error:
            self.phase('uncertain', str(error), None)
        finally:
            with self.lock:
                self.running = False

    def begin(self, document, expected_revision):
        document = validate(document)
        with self.lock:
            if self.running or self.operation['phase'] == 'uncertain':
                raise Conflict('Trwa zastosowanie lub uzgadnianie konfiguracji; sprawdź stan operacji')
            if expected_revision != self.active['revision']:
                raise Conflict('Konfiguracja zmieniła się w innym panelu. Wczytaj aktywną rewizję; szkic zachowano')
            validate_transition(self.active['document'], document)
            previous = deepcopy(self.active)
            candidate = {'revision': uuid.uuid4().hex, 'document': document}
            operation_id = uuid.uuid4().hex
            self.operation = {'id': operation_id, 'phase': 'preparing', 'message': 'Sprawdzanie warunków zastosowania', 'success': None}
            self.running = True
        return operation_id, previous, candidate

    def apply(self, operation_id, previous, candidate):
        journal = {'id': operation_id, 'previous': previous, 'candidate': candidate}
        attempted = False
        finishing = False
        try:
            journal['previous_calibration'] = self.storage.calibration(previous['revision'])
            self.storage.atomic('pending.json', journal)
            attempted = True
            self.transport.control('prepare', operation_id, candidate, changed_axes(previous['document'], candidate['document']), previous['revision'])
            self.phase('model', 'Aktualizowanie opisu modelu ROS')
            self.transport.model(build_urdf(candidate['document'], candidate['revision']))
            self.phase('committing', 'Zastosowanie parametrów w C++')
            self.transport.control('commit', operation_id, candidate, None)
            self.storage.atomic('calibration.json', {'revision': candidate['revision'], 'invalid': [False] * 6})
            self.storage.atomic('active.json', candidate)
            finishing = True
            self.transport.control('finish', operation_id, candidate, None)
            with self.lock:
                self.active = candidate
            self.storage.clear_journal()
            self.phase('done', 'Konfiguracja zapisana w ROS i na dysku', True)
        except Exception as error:
            try:
                status = self.transport.control('status', operation_id, candidate, None) if attempted else None
                if finishing and status and status['revision'] == candidate['revision'] and not status['locked']:
                    # ACK FINISH mógł zaginąć. Plik, model i runtime potwierdzają już nową rewizję.
                    with self.lock:
                        self.active = candidate
                    self.storage.clear_journal()
                    self.phase('done', 'Konfiguracja zapisana; potwierdzono wynik po utracie odpowiedzi', True)
                elif status and status['operation_id'] == operation_id and status['locked']:
                    self.phase('rolling_back', f'Wycofywanie: {error}')
                    self.transport.control('rollback', operation_id, previous, None)
                    self.transport.model(build_urdf(previous['document'], previous['revision']))
                    self.storage.atomic('calibration.json', {'revision': previous['revision'], 'invalid': journal['previous_calibration']})
                    self.storage.atomic('active.json', previous)
                    self.transport.control('finish', operation_id, previous, None)
                    self.storage.clear_journal()
                    self.phase('failed', f'Nie zastosowano; przywrócono poprzednią konfigurację: {error}', False)
                else:
                    if status and status['locked']:
                        raise RuntimeError('Blokada należy do innej transakcji')
                    if status and status['revision'] != previous['revision']:
                        raise RuntimeError('Runtime ma inną rewizję; nie nadpisano jej wycofaniem')
                    if status:
                        self.transport.control('rollback', operation_id, previous, None)
                        self.transport.control('finish', operation_id, previous, None)
                    self.storage.clear_journal()
                    self.phase('failed', f'Nie zastosowano konfiguracji: {error}', False)
            except Exception as recovery_error:
                self.phase('uncertain', f'Wynik niepotwierdzony; wymagane uzgodnienie przed kolejnym zapisem: {recovery_error}', None)
        finally:
            with self.lock:
                self.running = False
