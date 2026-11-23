"""Sprawdzenie nastaw napędów po inicjalizacji: porównanie z nietykalną kopią base i decyzja operatora.

Właściciel CAN wyłącza ENABLE i wykonuje komendy startowe, a w diagnostyce publikuje generację inicjalizacji.
Po każdej nowej generacji nastawy gotowych osi są odczytywane i porównywane z base. Zapis następuje wyłącznie na
polecenie operatora, po świeżej kopii pre_test, tylko dla osi z różnicami. Nastawy niezapisywalne są tylko raportowane.
"""
import copy
import threading
import time

from .drive_backup import AXES, Conflict, DriveBackups, _now

CHECK_ATTEMPTS = 3
RETRY_SECONDS = 5.0
# Po zmianie mikrokroku poza pozycją zero układ F5 jest przesunięty do restartu zasilania (docs/can_protocol.md).
POWER_CYCLE_PARAMETERS = {'microstep'}


def _mask_axes(mask):
    return [axis for axis in AXES if mask & (1 << (axis - 1))]


class DriveStartup:
    def __init__(self, backups, busy=lambda: False, clock=time.monotonic):
        self.backups, self.busy, self.clock = backups, busy, clock
        self.lock = threading.Lock()
        self.handled = None         # generacja z zakończonym sprawdzeniem
        self.attempts, self.retry_at = 0, 0.0
        self.acknowledged_failed = []
        self.state = {'phase': 'waiting', 'generation': None, 'axes': [], 'differences': {}, 'manual_differences': {},
                      'failed_axes': [], 'message': 'Oczekiwanie na inicjalizację napędów', 'checked_utc': None,
                      'last_apply': None}

    def status(self):
        with self.lock:
            state = copy.deepcopy(self.state)
            unacknowledged = [axis for axis in state['failed_axes'] if axis not in self.acknowledged_failed]
        state['attention'] = state['phase'] in ('differences', 'check_failed', 'applying', 'applied') or bool(unacknowledged)
        return state

    def _update(self, **fields):
        with self.lock:
            self.state.update(fields)

    # --- stan z właściciela CAN (wątek subskrypcji ROS) ---
    def can_status(self, enabled, generation, busy_mask, ready_mask, failed_mask):
        failed = _mask_axes(failed_mask)
        with self.lock:
            self.state['failed_axes'] = failed
            self.acknowledged_failed = [axis for axis in self.acknowledged_failed if axis in failed]
            if not enabled:
                self.state.update(phase='disabled', message='Inicjalizacja napędów wyłączona w konfiguracji CAN')
                return
            if generation != self.state['generation']:
                self.state['generation'] = generation
                self.attempts, self.retry_at = 0, 0.0
            due = generation > 0 and generation != self.handled and not busy_mask and self.clock() >= self.retry_at
            axes = _mask_axes(ready_mask)
        if not due or not axes or self.busy():
            return
        try:
            self.backups.run_exclusive('startup_check', lambda progress: self._check(generation, axes, progress))
        except Conflict:
            pass  # inna operacja na nastawach; kolejna próba przy następnym statusie

    # --- porównanie ---
    def _compare(self, axes, progress=None):
        base = self.backups.load('base')
        if base is None:
            return None
        live = {}
        for n, axis in enumerate(axes):
            live[str(axis)] = self.backups.read_axis(axis)
            if progress:
                progress((n + 1) / len(axes))
        found = DriveBackups.differences({str(a): base['axes'][str(a)] for a in axes}, live)
        writable, manual = {}, {}
        for axis, parameters in found.items():
            for name, entry in parameters.items():
                (writable if entry['writable'] else manual).setdefault(axis, {})[name] = entry
        return writable, manual

    def _publish(self, generation, axes, compared, ok_phase='ok'):
        if compared is None:
            self._update(phase='no_reference', axes=axes, differences={}, manual_differences={}, checked_utc=_now(),
                         message='Brak kopii podstawowej base — nie ma z czym porównać nastaw')
            return
        writable, manual = compared
        phase = 'differences' if writable or manual else ok_phase
        message = 'Nastawy napędów różnią się od zapamiętanych' if phase == 'differences' else 'Nastawy zgodne z zapamiętanymi'
        self._update(phase=phase, axes=axes, differences=writable, manual_differences=manual, checked_utc=_now(), message=message)

    def _check(self, generation, axes, progress):
        self._update(phase='checking', axes=axes, message='Sprawdzanie nastaw napędów')
        try:
            compared = self._compare(axes, progress)
        except Exception as error:
            with self.lock:
                self.attempts += 1
                self.retry_at = self.clock() + RETRY_SECONDS
                final = self.attempts >= CHECK_ATTEMPTS
                if final:
                    self.handled = generation
                self.state.update(phase='check_failed' if final else 'waiting', message=f'Nie udało się sprawdzić nastaw: {error}')
            return {'checked': False, 'error': str(error)}
        self._publish(generation, axes, compared)
        with self.lock:
            self.handled = generation
        self.backups.audit('startup_check', generation=generation, axes=axes,
                           different=sorted(compared[0]) if compared else [], manual=sorted(compared[1]) if compared else [])
        return {'checked': True}

    # --- decyzje operatora ---
    def _require_generation(self, generation):
        if generation != self.state['generation']:
            raise Conflict('Stan napędów zmienił się — sprawdź aktualne różnice')

    def apply(self, generation):
        with self.lock:
            self._require_generation(generation)
            if self.state['phase'] != 'differences' or not self.state['differences']:
                raise Conflict('Brak różnic, które można zapisać')
            targets = sorted(int(axis) for axis in self.state['differences'])
            axes = list(self.state['axes'])
        if self.busy():
            raise Conflict('Trwa zapis konfiguracji robota')
        return self.backups.run_exclusive('startup_apply', lambda progress: self._apply(generation, axes, targets, progress))

    def _apply(self, generation, axes, targets, progress):
        self._update(phase='applying', message='Kopia pre_test przed zapisem')
        report = {'axes': {}, 'error': None, 'requires_power_cycle': False, 'utc': _now()}
        try:
            self.backups.capture('pre_test')  # obowiązkowa kopia przed każdą ingerencją w nastawy
            self._update(message='Zapisywanie nastaw do napędów')
            result = self.backups.restore('base', targets, progress)
            report['axes'] = result['axes']
            written = {name for entry in result['axes'].values() for name in entry['written']}
            report['requires_power_cycle'] = bool(written & POWER_CYCLE_PARAMETERS)
            self._publish(generation, axes, self._compare(axes), ok_phase='applied')
        except Exception as error:
            report['error'] = str(error)
            self.backups.audit('startup_apply_failed', axes=targets, error=str(error))
            self._update(phase='differences', message=f'Zapis nastaw nieudany: {error}')
        self._update(last_apply=report)
        return {'applied': report['error'] is None and not any(e['error'] for e in report['axes'].values()), 'axes': report['axes']}

    def dismiss(self, generation):
        with self.lock:
            self._require_generation(generation)
            phase = self.state['phase']
            if phase == 'applying' or phase == 'checking':
                raise Conflict('Operacja na nastawach jeszcze trwa')
            self.acknowledged_failed = list(self.state['failed_axes'])
            if phase in ('differences', 'check_failed'):
                self.state.update(phase='skipped', message='Różnice pominięte do następnego startu napędów')
            elif phase == 'applied':
                self.state.update(phase='ok', message='Nastawy zgodne z zapamiętanymi')
        return self.status()

    def recheck(self):
        with self.lock:
            if self.state['phase'] in ('applying', 'checking'):
                raise Conflict('Operacja na nastawach jeszcze trwa')
            self.handled = None
            self.attempts, self.retry_at = 0, 0.0
            self.state.update(phase='waiting', message='Ponowne sprawdzanie nastaw')
        return self.status()
