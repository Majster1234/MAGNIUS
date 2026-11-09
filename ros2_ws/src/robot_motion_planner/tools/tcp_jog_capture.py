#!/usr/bin/env python3
"""Wyłącznie odczyt telemetrii prób 5/6. Nie tworzy klienta poleceń ruchu."""
import argparse
import json
from pathlib import Path
import time
import urllib.request

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=int, choices=(5, 6), required=True)
    parser.add_argument('--api', required=True)
    parser.add_argument('--directory', default='/tmp/tcp-jog-2026-10-26')
    parser.add_argument('--wait-seconds', type=float, default=120.)
    args = parser.parse_args()
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / f'iteration-{args.trial}.json'
    if destination.exists():
        raise RuntimeError('Próba już zapisana; nie nadpisuję danych')

    def get(path, raw=False):
        with urllib.request.urlopen(args.api.rstrip('/') + path, timeout=3) as response:
            return response.read().decode() if raw else json.load(response)

    before = get('/api/v1/manual/status')
    if not before['fresh'] or before['busy'] or before['fault'] or not before['tcp_jog']['ready']:
        raise RuntimeError('Brak gotowości do samej rejestracji; nie wydano żadnych poleceń')
    result = dict(iteration=args.trial, axis='x', direction=1 if args.trial == 5 else -1,
                  frame='base', step_m=.01, velocity_m_s=.002, max_seconds=10,
                  before=before, description=get('/api/v1/robot/description', raw=True), telemetry=[])
    rclpy.init()
    node = rclpy.create_node(f'tcp_jog_capture_{args.trial}')
    generation = None
    terminal = None
    received_start = None

    def receive(message):
        nonlocal generation, terminal, received_start
        record = json.loads(message.data)
        if generation is None:
            if record['event'] != 'start':
                return
            expected_direction = result['direction']
            if record.get('axis') != 0 or record.get('direction') != expected_direction or record.get('frame') != 'base' or abs(record.get('requested_step', 0)-.01) > 1e-12:
                raise RuntimeError('Inny ruch niż karta próby; rejestrator nie wydaje STOP ani START')
            generation = record['generation']
            received_start = time.monotonic()
            result['start'] = record
        if record['generation'] != generation:
            return
        result['telemetry'].append(record)
        if record['event'] in ('stopped', 'fault'):
            terminal = time.monotonic()
            result['terminal_event'] = record

    subscription = node.create_subscription(String, '/robot/manual/tcp_telemetry', receive,
        QoSProfile(depth=5000, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        ready = time.monotonic()+5
        while node.count_publishers('/robot/manual/tcp_telemetry') < 1 and time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=.05)
        if node.count_publishers('/robot/manual/tcp_telemetry') < 1:
            raise RuntimeError('Brak publikatora pełnej telemetrii; nie uruchamiaj próby')
        print(json.dumps({'capture_ready': True, 'trial': args.trial, 'motion_commands': 0}), flush=True)
        deadline = time.monotonic()+args.wait_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if terminal is not None and time.monotonic()-terminal >= .3:
                break
            if received_start is not None and time.monotonic()-received_start > 15:
                result['capture_timeout'] = True
                break
        result['after'] = get('/api/v1/manual/status')
        result['stop_confirmed'] = bool(result.get('terminal_event', {}).get('event') == 'stopped'
            and result['after']['fresh'] and not result['after']['busy'] and not result['after']['fault']
            and all(a['speed_known'] and a['speed_rpm'] == 0 for a in result['after']['axes']))
        result['accepted'] = generation is not None
        if generation is None:
            destination = directory / f'iteration-{args.trial}-no-start.json'
        temporary = destination.with_suffix('.tmp')
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        temporary.replace(destination)
        print(json.dumps({'file': str(destination), 'samples': sum(r['event']=='sample' for r in result['telemetry']),
                          'stop_confirmed': result['stop_confirmed'], 'motion_commands': 0}), flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
