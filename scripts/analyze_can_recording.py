#!/usr/bin/env python3
"""Analiza ograniczonych JSONL bez ładowania historii ramek do pamięci."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path

COMMANDS = {0x31: 'pozycja', 0x32: 'prędkość', 0x3a: 'ENABLE', 0x3e: 'stall',
            0xf1: 'stan pracy', 0xf3: 'załączenie', 0xf6: 'JOG', 0x40: 'identyfikacja'}


def events(directory):
    try:
        session = json.loads((Path(directory) / 'session.json').read_text())
        session_start, recording_id = session.get('started_mono_ns', 0), session.get('recording_id')
    except (OSError, ValueError):
        session_start, recording_id = 0, None
    def first_time(path):
        with path.open() as file:
            for line in file:
                try:
                    return json.loads(line).get('mono_ns', 0)
                except (ValueError, TypeError):
                    pass
        return 0
    paths = [*Path(directory).glob('can-*.jsonl'), *Path(directory).glob('bridge-*.jsonl')]
    for path in sorted(paths, key=first_time):
        with path.open() as file:
            for line in file:
                try:
                    row = json.loads(line)
                    if (isinstance(row, dict) and isinstance(row.get('mono_ns'), int) and row['mono_ns'] >= session_start
                            and (not recording_id or row.get('recording_id') == recording_id)):
                        yield row
                except ValueError:
                    # Plik aktualnie nagrywany może kończyć się niepełną linią.
                    continue


def utc(ns):
    return datetime.fromtimestamp(ns // 10**9, timezone.utc).strftime('%Y-%m-%dT%H:%M:%S') + f'.{ns % 10**9:09d}Z'


def analyze(directory):
    counts, timeouts, replies = Counter(), Counter(), {}
    last_rx, gaps = {}, defaultdict(int)
    metrics = defaultdict(lambda: {'count': 0, 'sum_ms': 0., 'max_ms': 0.})
    coverage = {}
    faults, stops, transitions = [], [], []
    ws_closures, command_rejections = [], []
    connections = {}
    previous_axis, previous_ws = {}, {}
    dropped, overflow, first, last = 0, 0, None, 0

    def measure(name, ns):
        if ns is None or ns < 0:
            return
        value = ns / 1e6
        m = metrics[name]; m['count'] += 1; m['sum_ms'] += value; m['max_ms'] = max(m['max_ms'], value)

    for e in events(directory):
        kind, stamp = e.get('event'), e['mono_ns']
        counts[kind] += 1; first = stamp if first is None else min(first, stamp); last = max(last, stamp)
        source = e.get('source', 'unknown')
        covered = coverage.setdefault(source, {'first_mono_ns': stamp, 'last_mono_ns': stamp})
        covered['first_mono_ns'] = min(covered['first_mono_ns'], stamp)
        covered['last_mono_ns'] = max(covered['last_mono_ns'], stamp)
        dropped = max(dropped, e.get('dropped_total', 0))
        axis, command = e.get('axis', 0), e.get('command', 0)
        key = f'CAN {axis} / 0x{command:02X} ({COMMANDS.get(command, "inne")})'
        if kind == 'request_timeout':
            timeouts[key] += 1
        if kind == 'reply_ok':
            r = replies.setdefault(key, {'count': 0, 'max_ms': 0.})
            r['count'] += 1; r['max_ms'] = max(r['max_ms'], e.get('duration_ns', 0) / 1e6)
        if kind == 'can_rx':
            if key in last_rx:
                gaps[key] = max(gaps[key], stamp - last_rx[key])
            last_rx[key] = stamp
            if e.get('kernel_utc_ns'):
                measure('kernel → odbiór w C++', e['utc_ns'] - e['kernel_utc_ns'])
        if kind in ('io_loop_gap', 'io_tick_slow', 'ros_publish', 'ros_publish_slow', 'ws_state_sent', 'ros_service_result', 'ros_service_timeout_or_error'):
            measure(kind, e.get('duration_ns'))
        if kind == 'ros_state_received':
            measure('ROS publikacja → callback backendu', e.get('transport_age_ns'))
        if kind == 'ws_state_sent':
            connection = e.get('connection', 0)
            c = connections.setdefault(connection, {'states': 0, 'stale_states': 0, 'max_send_gap_ms': 0., 'max_browser_gap_ms': 0.})
            c['states'] += 1
            if not e.get('state', {}).get('fresh', False): c['stale_states'] += 1
            if connection in previous_ws:
                measure('odstęp statusów WS', stamp - previous_ws[connection])
                c['max_send_gap_ms'] = max(c['max_send_gap_ms'], (stamp - previous_ws[connection]) / 1e6)
            previous_ws[connection] = stamp
        if kind == 'browser_state_gap':
            measure('odstęp statusów w przeglądarce', e.get('gap_ms', 0) * 1e6)
            c = connections.setdefault(e.get('connection', 0), {'states': 0, 'stale_states': 0, 'max_send_gap_ms': 0., 'max_browser_gap_ms': 0.})
            c['max_browser_gap_ms'] = max(c['max_browser_gap_ms'], e.get('gap_ms', 0))
        if kind == 'ws_close' and len(ws_closures) < 64:
            ws_closures.append(e)
        if kind == 'ws_command_rejected' and len(command_rejections) < 64:
            command_rejections.append(e)
        if kind == 'socket_rx_overflow':
            overflow = max(overflow, e.get('flags', 0))
        if kind in ('bus_fault', 'manual_fault') and len(faults) < 32:
            faults.append(e)
        if kind == 'manual_stop' and len(stops) < 32:
            stops.append(e)
        if kind == 'ros_axis':
            available = bool(e.get('flags', 0) & 1)
            if previous_axis.get(axis) is True and not available and len(transitions) < 64:
                transitions.append(e)
            previous_axis[axis] = available
            measure(f'wiek pozycji CAN {axis}', e.get('position_age_ns'))
    for m in metrics.values():
        m['mean_ms'] = m.pop('sum_ms') / m['count']
    report = dict(events=sum(counts.values()), duration_seconds=(last - first) / 1e9 if first is not None else 0,
                  counts=dict(counts), timeouts=dict(timeouts), reply_rtt=replies,
                  max_rx_gap_ms={key: value / 1e6 for key, value in gaps.items()}, metrics=dict(metrics),
                  faults=faults, stops=stops, unavailable=transitions, coverage=coverage,
                  connections=connections, ws_closures=ws_closures, command_rejections=command_rejections,
                  recorder_dropped_max=dropped, socket_rx_dropped=overflow)
    # Drugi przebieg: małe okno przyczyny, bez trzymania historii całego nagrania.
    reference = next((e['mono_ns'] for e in faults if e['event'] == 'bus_fault'), None)
    report['fault_window'] = []
    if reference is not None:
        relevant = {'request_timeout', 'reply_late', 'reply_invalid', 'io_loop_gap', 'io_tick_slow',
                    'manual_stop', 'manual_fault', 'bus_fault', 'can_error_frame', 'can_send_error', 'ros_service_timeout_or_error', 'browser_client_stop'}
        for e in events(directory):
            if abs(e['mono_ns'] - reference) <= 500_000_000 and e.get('event') in relevant and len(report['fault_window']) < 128:
                report['fault_window'].append(e)
        report['fault_window'].sort(key=lambda e: e['mono_ns'])
    return report


def render(report):
    lines = [f"Zdarzenia: {report['events']}, okres: {report['duration_seconds']:.3f} s",
             f"Utracone wpisy rejestratora (maks./proces): {report['recorder_dropped_max']}; przepełnienie socketu RX: {report['socket_rx_dropped']}",
             '\nTimeouty zapytań:']
    for source, coverage in report['coverage'].items():
        lines.insert(2, f"Zachowane okno {source}: mono={coverage['first_mono_ns']}…{coverage['last_mono_ns']} (rotacja).")
    lines += [f'  {key}: {value}' for key, value in sorted(report['timeouts'].items()) if '0x40' not in key] or ['  brak poza skanowaniem']
    scan_timeouts = sum(value for key, value in report['timeouts'].items() if '0x40' in key)
    lines += [f'  SCAN: {scan_timeouts} zapytań identyfikacji bez odpowiedzi (nie potwierdza wolnego adresu).']
    lines += ['\nOpóźnienia (średnia / maksimum):']
    lines += [f"  {key}: {m['mean_ms']:.3f} / {m['max_ms']:.3f} ms ({m['count']} pomiarów)" for key, m in report['metrics'].items()]
    lines += ['\nOdczyty napędów: maks. RTT / maks. odstęp odpowiedzi:']
    feedback = [(key, value) for key, value in report['reply_rtt'].items()
                if any(f'0x{command:02X}' in key for command in (0x31, 0x32, 0x3a, 0x3e, 0xf1))]
    for key, value in sorted(feedback)[:40]:
        lines.append(f"  {key}: {value['max_ms']:.3f} / {report['max_rx_gap_ms'].get(key, 0):.3f} ms")
    if len(feedback) > 40:
        lines.append('  Dalsze adresy w raporcie --json.')
    lines += ['\nZatrzymania i błędy:']
    for e in sorted(report['stops'] + report['faults'], key=lambda x: x['mono_ns']):
        lines.append(f"  {utc(e['utc_ns'])} mono={e['mono_ns']} {e['event']} CAN {e.get('axis', 0)}: {e.get('detail', '')}")
    lines += ['\nPołączenia WebSocket:']
    for key, c in report['connections'].items():
        lines.append(f"  Połączenie {key}: {c['states']} stanów, nieaktualne {c['stale_states']}, maks. odstęp wysyłki {c['max_send_gap_ms']:.3f} ms / odbioru klienta {c['max_browser_gap_ms']:.3f} ms")
    for e in sorted(report['ws_closures'] + report['command_rejections'], key=lambda x: x['mono_ns']):
        lines.append(f"  {utc(e['utc_ns'])} {e['event']} połączenie {e.get('connection')}, oś {e.get('active_axis', e.get('axis', 0))}, przyczyna: {e.get('reason', 'niezapisana w tej wersji')}")
    lines += ['\nOkno przy błędzie (±500 ms):']
    for e in report['fault_window']:
        lines.append(f"  {utc(e['utc_ns'])} mono={e['mono_ns']} {e['event']} CAN {e.get('axis', 0)} cmd=0x{e.get('command', 0):02X} czas={e.get('duration_ns', 0)/1e6:.3f} ms {e.get('detail', '')}")
    lines += [f"\nPrzejścia osi do niedostępności w ROS: {len(report['unavailable'])} (raport przechowuje maks. 64)."]
    if any(e['event'] == 'bus_fault' for e in report['faults']):
        lines += ['Zatrzaśnięto globalny błąd CAN. Cisza TX i niedostępność wszystkich osi po nim mogą wynikać z logiki ochrony; sprawdź zdarzenia bezpośrednio przed bus_fault.']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    if not args.directory.is_dir():
        parser.error('Brak katalogu nagrania')
    report = analyze(args.directory)
    print(json.dumps(report, ensure_ascii=False, indent=2) if args.json else render(report))


if __name__ == '__main__':
    main()
