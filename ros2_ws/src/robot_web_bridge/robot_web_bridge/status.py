"""Wspólny snapshot; liczba klientów nie wpływa na pomiary ani węzły ROS."""

from copy import deepcopy
from datetime import datetime, timezone
import json
import threading
import time

COMPONENTS = ('host', 'can', 'controllers', 'drives')
LEVELS = {0: 'ok', 1: 'warn', 2: 'error', 3: 'stale'}
STALE_AFTER = 3.0


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class StatusStore:
    def __init__(self, address, port, interface, subnet, clock=time.monotonic):
        self.clock = clock
        self.started = clock()
        self.lock = threading.Lock()
        self.received = None
        self.updated_at = None
        self.components = {}
        self.network = {'address': address, 'port': port, 'interface': interface,
                        'subnet': subnet, 'url': f'http://{address}:{port}'}

    def update(self, message):
        components = {}
        for item in message.status:
            if item.name not in COMPONENTS:
                continue
            values = {}
            for pair in item.values:
                try:
                    values[pair.key] = json.loads(pair.value)
                except (ValueError, TypeError):
                    values[pair.key] = pair.value
            state = LEVELS.get(item.level[0], 'error')
            if item.name in ('controllers', 'drives') and values.get('configured') is False:
                state = 'not_configured'
            components[item.name] = {
                'id': item.name, 'state': state, 'message': item.message,
                'sampled_at': values.pop('sampled_at', None), 'values': values,
            }
        with self.lock:
            self.components = components
            self.received = self.clock()
            self.updated_at = utc_now()

    def snapshot(self):
        now = self.clock()
        with self.lock:
            age = None if self.received is None else max(0, now - self.received)
            fresh = age is not None and age < STALE_AFTER
            components = deepcopy(self.components)
            updated_at = self.updated_at
        for name in COMPONENTS:
            component = components.setdefault(name, {
                'id': name, 'state': 'stale', 'message': 'Oczekiwanie na dane monitora',
                'sampled_at': None, 'values': {},
            })
            if not fresh:
                component['state'] = 'stale'
        components['api'] = {
            'id': 'api', 'state': 'ok', 'message': 'Backend odpowiada',
            'sampled_at': utc_now(), 'values': {'uptime_seconds': now - self.started},
        }
        components['ros'] = {
            'id': 'ros', 'state': 'ok' if fresh else 'stale',
            'message': ('Łączność z węzłem diagnostycznym potwierdzona' if fresh else
                        'Brak aktualnych wiadomości z węzła diagnostycznego'),
            'sampled_at': updated_at, 'values': {'topic': '/robot/system_status'},
        }
        return {'schema_version': 1, 'updated_at': updated_at,
                'age_seconds': age, 'fresh': fresh, 'network': self.network,
                'components': components}


class CanStatusStore:
    """Ostatni stan diagnostyki CAN, niezależny od monitora 1 Hz."""

    def __init__(self, interface, clock=time.monotonic):
        self.interface = interface
        self.clock = clock
        self.lock = threading.Lock()
        self.received = None
        self.updated_at = None
        self.summary = {}
        self.scan = {}
        self.devices = {}

    @staticmethod
    def _values(item):
        result = {}
        for pair in item.values:
            try:
                result[pair.key] = json.loads(pair.value)
            except (TypeError, ValueError):
                result[pair.key] = pair.value
        return result

    def update(self, message):
        summary, scan, devices = None, None, {}
        for item in message.status:
            values = self._values(item)
            if item.name == 'can':
                summary = {'state': LEVELS.get(item.level[0], 'error'),
                           'message': item.message, 'values': values}
            elif item.name == 'scan':
                scan = {'state': LEVELS.get(item.level[0], 'error'),
                        'message': item.message, 'values': values}
            elif item.name.startswith('device/'):
                try:
                    address = int(item.name.split('/', 1)[1])
                except ValueError:
                    continue
                devices[address] = {
                    'id': address, 'state': LEVELS.get(item.level[0], 'error'),
                    'message': item.message, 'values': values,
                }
        if summary is None or scan is None:
            return
        with self.lock:
            self.summary, self.scan, self.devices = summary, scan, devices
            self.received = self.clock()
            self.updated_at = utc_now()

    def snapshot(self):
        now = self.clock()
        with self.lock:
            age = None if self.received is None else max(0, now - self.received)
            fresh = age is not None and age < STALE_AFTER
            summary, scan, devices, updated_at = (deepcopy(self.summary), deepcopy(self.scan),
                                                   deepcopy(self.devices), self.updated_at)
        if not fresh:
            if not summary:
                summary = {'state': 'stale', 'message': 'Oczekiwanie na diagnostykę CAN',
                           'values': {'interface': self.interface}}
            else:
                summary['state'] = 'stale'
            if not scan:
                scan = {'state': 'stale', 'message': 'Brak aktualnych danych', 'values': {}}
            for device in devices.values():
                device['state'] = 'stale'
                device['message'] = 'Brak aktualnych danych CAN'
                device['values']['enable'] = None
                device['values']['stall'] = None
                device['values']['motor_state'] = 'Niedostępny'
        return {'schema_version': 1, 'updated_at': updated_at, 'age_seconds': age,
                'fresh': fresh, 'interface': self.interface, 'summary': summary,
                'scan': scan, 'devices': [devices[key] for key in sorted(devices)]}
