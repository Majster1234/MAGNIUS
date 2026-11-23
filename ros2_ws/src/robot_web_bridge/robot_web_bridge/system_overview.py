"""Przegląd architektury tylko do odczytu: procesy ROS, węzły, usługi systemd i klienci panelu.

Odczyt nie wysyła ramek CAN ani poleceń do ROS; dane są zbierane leniwie i buforowane.
"""

import os
import platform
import subprocess
import threading
import time

import psutil

DEFAULT_UNITS = ('robot-web-status.service', 'robot-can@can0.service')
CACHE_SECONDS = 1.0


def node_name(cmdline):
    """Nazwa węzła z `__node:=`, w przeciwnym razie nazwa pliku wykonywalnego."""
    for token in cmdline:
        if token.startswith('__node:='):
            return token.split(':=', 1)[1]
    for token in cmdline:
        if token and not token.startswith('-') and not token.endswith('python3'):
            return os.path.basename(token)
    return None


def executable_origin(cmdline):
    """Pakiet i nakładka z ścieżki `<prefiks>/<pakiet>/lib/<pakiet>/<plik>` lub `/opt/ros/<dystr>/lib/<pakiet>/<plik>`."""
    for token in cmdline:
        parts = token.split('/')
        if 'lib' in parts[:-1] and len(parts) - parts.index('lib') == 3:
            index = parts.index('lib')
            prefix = '/'.join(parts[:index])
            return parts[index + 1], overlay_name(prefix)
    return None, None


def parse_systemctl(text):
    units, current = [], {}
    for line in text.splitlines() + ['']:
        if not line.strip():
            if current:
                units.append(current)
            current = {}
            continue
        key, _, value = line.partition('=')
        current[key] = value
    return [{'unit': item.get('Id'), 'active_state': item.get('ActiveState'),
             'sub_state': item.get('SubState'), 'result': item.get('Result'),
             'main_pid': int(item['MainPID']) if item.get('MainPID', '').isdigit() and item['MainPID'] != '0' else None}
            for item in units if item.get('Id')]


def overlay_name(prefix_path):
    """Pierwszy wpis AMENT_PREFIX_PATH to aktywna nakładka, np. install/stop-recovery-ros/<pakiet>."""
    first = (prefix_path or '').split(os.pathsep)[0]
    parts = first.rstrip('/').split('/')
    if 'install' in parts:
        index = parts.index('install')
        return '/'.join(parts[index:index + 2])
    return first or None


def os_release():
    try:
        return platform.freedesktop_os_release().get('PRETTY_NAME')
    except (OSError, AttributeError):
        return None


class ConnectionCounter:
    """Liczba otwartych WebSocketów panelu według strumienia."""

    def __init__(self):
        self.lock = threading.Lock()
        self.counts = {}

    def opened(self, kind):
        with self.lock:
            self.counts[kind] = self.counts.get(kind, 0) + 1

    def closed(self, kind):
        with self.lock:
            self.counts[kind] = max(0, self.counts.get(kind, 0) - 1)

    def snapshot(self):
        with self.lock:
            return dict(self.counts)


class SystemOverview:
    def __init__(self, node=None, units=DEFAULT_UNITS, connections=None, clock=time.monotonic,
                 run=subprocess.run, processes=psutil.process_iter, environ=os.environ):
        self.node = node
        self.units = tuple(units)
        self.connections = connections or ConnectionCounter()
        self.clock = clock
        self.run = run
        self.process_iter = processes
        self.environ = environ
        self.started = clock()
        self.lock = threading.Lock()
        self.tracked = {}
        self.cached = None
        self.cached_at = None

    def _process(self, proc, now_wall):
        """cpu_percent(None) mierzy od poprzedniego wywołania; pierwsze daje None."""
        pid = proc.pid
        known = self.tracked.get(pid)
        if known is None or known.create_time() != proc.create_time():
            self.tracked[pid] = known = proc
            known.cpu_percent(None)
            cpu = None
        else:
            cpu = known.cpu_percent(None)
        with known.oneshot():
            return {'pid': pid, 'cpu_percent': cpu, 'memory_rss_bytes': known.memory_info().rss,
                    'threads': known.num_threads(), 'uptime_seconds': max(0., now_wall - known.create_time())}

    def _processes(self):
        own, result, seen = os.getpid(), [], set()
        now_wall = time.time()
        for proc in self.process_iter(['pid', 'cmdline']):
            try:
                cmdline = proc.info.get('cmdline') or []
                if '--ros-args' not in cmdline and proc.pid != own:
                    continue
                item = self._process(proc, now_wall)
                item['name'] = node_name(cmdline)
                item['package'], item['overlay'] = executable_origin(cmdline)
                item['backend'] = proc.pid == own
                result.append(item)
                seen.add(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        for pid in set(self.tracked) - seen:
            del self.tracked[pid]
        return sorted(result, key=lambda item: (item['name'] or '', item['pid']))

    def _nodes(self):
        if self.node is None:
            return None
        try:
            return sorted(f"{namespace.rstrip('/')}/{name}"
                          for name, namespace in self.node.get_node_names_and_namespaces())
        except Exception:
            return None

    def _services(self):
        try:
            result = self.run(['systemctl', 'show', '--no-pager', '-p', 'Id', '-p', 'ActiveState',
                               '-p', 'SubState', '-p', 'MainPID', '-p', 'Result', *self.units],
                              capture_output=True, text=True, timeout=0.75, check=False)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return parse_systemctl(result.stdout) if result.returncode == 0 else None

    def snapshot(self):
        with self.lock:
            now = self.clock()
            if self.cached is None or now - self.cached_at >= CACHE_SECONDS:
                processes = self._processes()
                self.cached = {
                    'host': {'os': os_release(), 'kernel': platform.release(), 'hostname': platform.node()},
                    'ros': {'distro': self.environ.get('ROS_DISTRO'),
                            'overlay': overlay_name(self.environ.get('AMENT_PREFIX_PATH')),
                            'nodes': self._nodes(),
                            'processes': [item for item in processes if not item['backend']]},
                    'backend': next((item for item in processes if item['backend']), None),
                    'services': self._services(),
                }
                self.cached_at = now
            data = dict(self.cached)
        data['backend'] = dict(data['backend'] or {}, websocket_clients=self.connections.snapshot())
        return {'schema_version': 1, 'age_seconds': now - self.cached_at, **data}
