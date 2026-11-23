#!/usr/bin/env python3
"""Test odczytowej komunikacji MKS; brak komend ruchu i zapisów parametrów."""
import argparse
import collections
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import socket
import statistics
import struct
import subprocess
import time

READS = {0x40: 7, 0x31: 8, 0x32: 4, 0x3a: 3, 0x3e: 3, 0xf1: 3}
PARAMETERS = {0x82: 3, 0x84: 3, 0x8a: 3, 0x8c: 4, 0x98: 6, 0x4a: 3}

def packet(axis, command, parameter=None):
    if parameter is None:
        if command not in READS:
            raise ValueError('Komenda spoza listy odczytów')
        data = bytes([command])
    else:
        if command != 0 or parameter not in PARAMETERS:
            raise ValueError('Nieobsługiwany odczyt parametru')
        data = bytes([0, parameter])
    return data + bytes([(axis + sum(data)) & 255])

def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return dict(count=len(values), minimum=min(values), mean=statistics.mean(values),
                p95=ordered[min(len(ordered)-1, int(.95*len(ordered)))], maximum=max(values))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=10, choices=range(1, 31))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    link = lambda: json.loads(subprocess.check_output(['ip', '-j', '-details', '-statistics', 'link', 'show', 'can0'], text=True))[0]
    before = link()
    if before['linkinfo']['info_data']['bittiming']['bitrate'] != 1000000:
        raise RuntimeError('Test wymaga rzeczywistego bitrate 1000000 na can0')
    lock = open(f'/tmp/robot-can-{os.getuid()}-can0.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, 0x1fc)
    sock.bind(('can0',)); sock.setblocking(False)
    log = (args.output/'frames.jsonl').open('x', buffering=1)
    counters = collections.Counter(); rtts = collections.defaultdict(list)
    invalid = []; errors = []; timeouts = []; unsupported = []
    def event(kind, **values):
        log.write(json.dumps(dict(event=kind, mono_ns=time.monotonic_ns(), **values))+'\n')
    def exchange(axes, command, parameter=None, timeout=.03):
        opcode = command if parameter is None else parameter
        length = READS[command] if parameter is None else PARAMETERS[parameter]
        pending = {}; result = {}
        for axis in axes:
            data = packet(axis, command, parameter)
            sent = time.monotonic_ns()
            sock.send(struct.pack('=IB3x8s', axis, len(data), data))
            event('tx', axis=axis, data=data.hex()); counters['tx'] += 1
            counters['model_bits'] += 55+10*len(data)
            pending[axis] = sent
        deadline = time.monotonic()+timeout
        while pending and time.monotonic()<deadline:
            if not select.select([sock], [], [], max(0, deadline-time.monotonic()))[0]:
                break
            address, size, raw = struct.unpack('=IB3x8s', sock.recv(16)); data = raw[:size]
            stamp = time.monotonic_ns()
            event('rx', can_id=address, data=data.hex()); counters['rx'] += 1
            counters['model_bits'] += 55+10*len(data)
            if address & 0x20000000:
                recovered = (address & 0x1fffffff) == 4 and len(data)==8 and data[1]==0x40 and not any(data[i] for i in (0,2,3,4,5))
                if recovered: counters['error_active_reports'] += 1
                else: errors.append(dict(can_id=address, data=data.hex()))
                continue
            if address & 0xc0000000 or len(data)<2 or (address+sum(data[:-1]))&255 != data[-1]:
                invalid.append(dict(can_id=address, data=data.hex())); continue
            if address not in pending or data[0] != opcode:
                counters['unmatched'] += 1; continue
            if parameter is not None and data[1:-1] == b'\xff\xff':
                unsupported.append(dict(axis=address, parameter=parameter)); pending.pop(address); continue
            if len(data) != length:
                invalid.append(dict(can_id=address, data=data.hex(), expected_length=length)); continue
            rtts[address].append((stamp-pending.pop(address))/1e6)
            result[address] = data[1:-1]
            counters['validated'] += 1
        for axis in pending:
            timeouts.append(dict(axis=axis, command=opcode))
        return result
    started = time.monotonic_ns()
    try:
        found = {}; absent = []
        for axis in range(1,7):
            response = exchange([axis], 0x40)
            if axis in response: found[axis] = dict(identity_hex=response[axis].hex())
            else: absent.append(axis)
        if not found:
            raise RuntimeError('Brak odpowiedzi napędów przy 1 Mbit/s')
        discovery_timeouts = len(timeouts)
        for axis in found:
            params = {}
            for parameter in PARAMETERS:
                response = exchange([axis], 0, parameter)
                if axis in response: params[f'{parameter:02x}'] = response[axis].hex()
            state = {}
            for command in (0x31,0x32,0x3a,0x3e,0xf1):
                response = exchange([axis], command)
                if axis in response: state[f'{command:02x}'] = response[axis].hex()
            found[axis].update(parameters=params, before=state)
        start_test = time.monotonic(); due = start_test
        cycles = 0; skipped = 0; positions = collections.Counter()
        while time.monotonic()-start_test < args.seconds:
            response = exchange(list(found), 0x31, timeout=.02)
            positions.update(response.keys()); cycles += 1; due += .01
            now = time.monotonic()
            if due < now:
                missed = int((now-due)/.01)+1; skipped += missed; due += missed*.01
            time.sleep(max(0, due-time.monotonic()))
        elapsed = time.monotonic()-start_test
        for axis in found:
            response = exchange([axis], 0x3a)
            found[axis]['enable_after'] = response.get(axis, b'').hex()
            found[axis]['position_responses'] = positions[axis]
        non_discovery_timeouts = timeouts[discovery_timeouts:]
        stable_enable = all(v['before'].get('3a') == v['enable_after'] and v['enable_after'] in ('00','01') for v in found.values())
        passed = not errors and not invalid and not non_discovery_timeouts and stable_enable
        after = link()
        report = dict(date=datetime.now().astimezone().isoformat(), success=passed, bitrate=1000000,
                      duration_seconds=elapsed, cycles=cycles, skipped_cycles=skipped,
                      position_responses_per_second={a:positions[a]/elapsed for a in found},
                      discovered=found, absent=absent, counters=dict(counters),
                      rtt_ms={a:stats(v) for a,v in rtts.items()}, unsupported_parameters=unsupported,
                      discovery_timeouts=discovery_timeouts, timeouts=non_discovery_timeouts,
                      invalid_frames=invalid, can_errors=errors, enable_unchanged=stable_enable,
                      commands_sent='wyłącznie40/31/32/3A/3E/F1 i00+parametr; bez F5/F6/F7/FE/F3/92 oraz zapisów',
                      link_before=before, link_after=after)
        (args.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps({k:report[k] for k in ('success','bitrate','duration_seconds','cycles','position_responses_per_second','absent','counters','rtt_ms','timeouts','invalid_frames','can_errors','enable_unchanged')}, ensure_ascii=False), flush=True)
        return 0 if passed else 1
    finally:
        event('finished', duration_ns=time.monotonic_ns()-started)
        log.close(); sock.close(); lock.close()

if __name__ == '__main__':
    raise SystemExit(main())
