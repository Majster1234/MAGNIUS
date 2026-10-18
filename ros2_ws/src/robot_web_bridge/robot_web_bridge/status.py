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
