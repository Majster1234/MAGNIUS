"""Odczyty systemowe. iproute2 pobiera stan CAN z Netlink, bez ramek CAN."""

import json
import subprocess
import time

import psutil


def host_status():
    memory = psutil.virtual_memory()
    return {
        'cpu_percent': psutil.cpu_percent(interval=None),
        'memory_percent': memory.percent,
        'memory_used_bytes': memory.used,
        'memory_total_bytes': memory.total,
        'uptime_seconds': max(0, time.time() - psutil.boot_time()),
    }


def parse_can_link(links):
    if not links:
        return 2, 'Brak interfejsu CAN', {'present': False}
    link = links[0]
    info = link.get('linkinfo', {})
    if info.get('info_kind') != 'can':
        return 2, 'Interfejs nie jest fizycznym interfejsem CAN', {'present': True}
    data = info.get('info_data', {})
    stats = link.get('stats64', link.get('stats', {}))
    up = 'UP' in link.get('flags', [])
    state = data.get('state', 'UNKNOWN')
    values = {
        'present': True,
        'interface': link['ifname'],
        'link_up': up,
        'bus_state': state,
        'bitrate': data.get('bittiming', {}).get('bitrate'),
        'rx_packets': stats.get('rx', {}).get('packets'),
        'tx_packets': stats.get('tx', {}).get('packets'),
        'rx_errors': stats.get('rx', {}).get('errors'),
        'tx_errors': stats.get('tx', {}).get('errors'),
        'bus_errors': info.get('info_xstats', {}).get('bus_error'),
        'bus_off_count': info.get('info_xstats', {}).get('bus_off'),
    }
    if not up:
        return 2, 'Interfejs CAN jest wyłączony', values
    if state in ('BUS-OFF', 'STOPPED', 'SLEEPING'):
        return 2, f'Magistrala CAN: {state}', values
    if state != 'ERROR-ACTIVE':
        return 1, f'Magistrala CAN: {state}', values
    return 0, 'Interfejs aktywny; łączność z napędami niepotwierdzona', values


def can_status(interface):
    result = subprocess.run(
        ['ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', interface],
        capture_output=True, text=True, timeout=0.75, check=False,
    )
    if result.returncode:
        # Nie interpretuj błędu uprawnień jako odłączenia urządzenia.
        if 'does not exist' in result.stderr or 'Cannot find device' in result.stderr:
            return 2, 'Brak interfejsu CAN', {'interface': interface, 'present': False}
        raise RuntimeError(result.stderr.strip() or 'Nieudany odczyt Netlink')
    return parse_can_link(json.loads(result.stdout))
