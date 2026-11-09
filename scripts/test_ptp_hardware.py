#!/usr/bin/env python3
"""Odbiór F5/ACK/SYNC na rozsprzęglonych wałach; uruchomienie wymaga osobnej zgody.

Nie jest testem planera ani wykonawcy ROS. Właściciel panelu musi być zatrzymany.
Stała procedura: CAN 1, 1+2, 1+2+3+5; +5° i powrót, maks. 1 RPM.
Nie zmienia zera, mikrokroków, prądu, adresów ani konfiguracji robota.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import time

IDS = (1, 2, 3, 5)
TICKS = 16384
STEP = round(5 * TICKS / 360)


def packet(axis, code, payload=b''):
    body = bytes([code]) + bytes(payload)
    return body + bytes([(axis + sum(body)) & 255])


def position_payload(target, rpm):
    if not -8388607 <= target <= 8388607 or not 1 <= rpm <= 3000:
        raise ValueError('Nieprzedstawialny cel/prędkość')
    return rpm.to_bytes(2, 'big') + b'\x01' + (target & 0xffffff).to_bytes(3, 'big')


class Bench:
    def __init__(self, output):
        self.log = output.open('x', buffering=1)
        self.saved = {}
        self.anchor = {}
        self.lock = open(f'/tmp/robot-can-{os.getuid()}-can0.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # BUSOFF, controller/protocol/ACK/bus errors: fail on any received error frame.
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, 0x1fc)
        self.sock.bind(('can0',))
        self.sock.setblocking(False)

    def event(self, name, **values):
        value = dict(event=name, mono_ns=time.monotonic_ns(), **values)
        self.log.write(json.dumps(value) + '\n')
        if name not in ('tx', 'rx', 'sample'):
            print(json.dumps(value), flush=True)

    def send(self, axis, code, payload=b''):
        data = packet(axis, code, payload)
        self.sock.send(struct.pack('=IB3x8s', axis, len(data), data))
        self.event('tx', axis=axis, data=data.hex())

    def receive(self, timeout):
        if not select.select([self.sock], [], [], max(0, timeout))[0]:
            return None
        axis, length, data = struct.unpack('=IB3x8s', self.sock.recv(16))
        data = data[:length]
        if axis & 0x20000000:
            raise RuntimeError(f'Ramka błędu CAN {axis:x}')
        self.event('rx', axis=axis, data=data.hex())
        if axis & 0xc0000000 or len(data) < 2 or (axis + sum(data[:-1])) & 255 != data[-1]:
            raise RuntimeError('Nieprawidłowa ramka/CRC')
        if data[0] == 0xf5 and data[1] in (0, 3):
            raise RuntimeError(f'CAN {axis}: błąd/krańcówka F5 {data[1]}')
        return axis, data

    def query(self, axis, code, payload=b'', expected=None):
        self.send(axis, code, payload)
        end = time.monotonic() + .1
        opcode = payload[0] if code == 0 else code
        while time.monotonic() < end:
            frame = self.receive(end - time.monotonic())
            if frame and frame[0] == axis and frame[1][0] == opcode:
                data = frame[1][1:-1]
                if opcode == 0xf5 and expected == b'\x05' and data == b'\x02':
                    # Raport zakończenia poprzedniego celu nie jest ACK nowego.
                    continue
                if expected is not None and data != expected:
                    raise RuntimeError(f'CAN {axis} {opcode:02x}: oczekiwano {expected.hex()}, otrzymano {data.hex()}')
                return data
        raise RuntimeError(f'Timeout CAN {axis} opcode {opcode:02x}')

    def read(self, axis, code):
        value = self.query(axis, 0, bytes([code]))
        if value == b'\xff\xff':
            raise RuntimeError(f'Nieobsługiwany odczyt {code:02x}')
        return value

    def write(self, axis, code, value):
        self.query(axis, code, value, b'\x01')

    def pos(self, axis):
        data = self.query(axis, 0x31)
        if len(data) != 6:
            raise RuntimeError('Niepełna pozycja enkodera')
        value = int.from_bytes(data, 'big', signed=True)
        if axis in self.anchor and abs(value - self.anchor[axis]) > STEP + 24:
            raise RuntimeError(f'CAN {axis}: przekroczony zakres +5° względem startu etapu')
        return value

    def sample(self, axes):
        values = {a: self.pos(a) for a in axes}
        self.event('sample', positions=values)
        return values

    def observe(self, axes, seconds, stationary=None):
        end = time.monotonic() + seconds
        rows = []
        while time.monotonic() < end:
            started = time.monotonic()
            values = self.sample(axes)
            if stationary and any(abs(values[a] - stationary[a]) > 5 for a in axes):
                raise RuntimeError('Ruch przed SYNC lub po skasowaniu bufora')
            rows.append((time.monotonic_ns(), values))
            time.sleep(max(0, .01 - (time.monotonic() - started)))
        return rows

    def prepare_axis(self, axis):
        self.write(axis, 0x8c, b'\x01\x01')
        self.write(axis, 0x4a, b'\x00')
        self.write(axis, 0xf7, b'')
        self.write(axis, 0x82, b'\x05')
        self.write(axis, 0x98, (300).to_bytes(4, 'big'))
        if self.read(axis, 0x98) != (300).to_bytes(4, 'big'):
            raise RuntimeError('Niepotwierdzony watchdog')
        microstep = int.from_bytes(self.read(axis, 0x84), 'big')
        if microstep not in (1, 2, 4, 8, 16, 32, 64, 128, 256):
            raise RuntimeError('Nieobsługiwany mikrokrok')
        scale = 1 if microstep in (16, 32, 64) else 16 / microstep
        rpm = math.floor(1 / scale)
        if rpm < 1:
            raise RuntimeError('Nie da się uzyskać prędkości <=1 RPM bez zmiany mikrokroku')
        self.anchor[axis] = self.pos(axis)
        self.write(axis, 0xf3, b'\x01')
        self.query(axis, 0x3a, expected=b'\x01')
        self.query(axis, 0x3e, expected=b'\x00')
        self.query(axis, 0xf1, expected=b'\x01')
        # Ustalenie rzeczywistego startu po załączeniu momentu.
        rows = []
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rows.extend(self.observe([axis], .02))
            recent = [p[1][axis] for p in rows if p[0] >= rows[-1][0]-100_000_000]
            if rows[-1][0]-rows[0][0] >= 100_000_000 and max(recent)-min(recent) <= 3:
                break
        else:
            raise RuntimeError('Niestabilna pozycja po ENABLE przez 2 s')
        p = rows[-1][1][axis]
        self.write(axis, 0x4a, b'\x01')
        if self.read(axis, 0x4a) != b'\x01':
            raise RuntimeError('Niepotwierdzony tryb SYNC')
        self.event('axis_ready', axis=axis, position=p, microstep=microstep, raw_rpm=rpm, shaft_rpm=rpm*scale)
        return p, rpm

    def stage(self, targets, speeds):
        before = self.sample(targets)
        start = time.monotonic_ns()
        for a, target in targets.items():
            if abs(target - self.anchor[a]) > STEP:
                raise RuntimeError('Cel poza zatwierdzonym zakresem')
            self.query(a, 0xf5, position_payload(target, speeds[a]), b'\x05')
        self.event('all_staged', axes=list(targets), staging_ms=(time.monotonic_ns()-start)/1e6)
        self.observe(targets, .12, stationary=before)

    def stop_all_zero(self, required=()):
        """Wszystkie F7 przed wszystkimi F6(0), bez SYNC i bez zmiany ENABLE."""
        errors, results = [], {}
        for code, payload in ((0xf7, b''), (0xf6, b'\x00\x00\x01')):
            replies = {}
            for axis in range(1, 7):
                try:
                    self.send(axis, code, payload)
                except Exception as exc:
                    errors.append(str(exc))
            deadline = time.monotonic() + .02
            while time.monotonic() < deadline:
                try:
                    frame = self.receive(deadline - time.monotonic())
                except Exception as exc:
                    errors.append(str(exc))
                    break
                if frame and frame[1][0] == code:
                    replies[frame[0]] = frame[1][1:-1]
                if set(self.saved).issubset(replies):
                    break
            results[f'{code:02x}'] = {str(axis): value.hex() for axis, value in replies.items()}
            for axis in required:
                if replies.get(axis) != b'\x01':
                    errors.append(f'CAN {axis}: niepotwierdzone {code:02x}')
        self.event('stop_f7_zero', replies=results, errors=errors,
                   note='ACK nie potwierdza opróżnienia przygotowanego celu')
        if errors:
            raise RuntimeError('; '.join(errors))

    def move(self, targets, speeds, label, stop_early=False):
        self.stage(targets, speeds)
        before = self.sample(targets)
        self.send(0, 0x4b)
        sync = time.monotonic_ns()
        if stop_early:
            self.observe(targets, .2)
            self.stop_all_zero(targets)
            stopped = time.monotonic_ns()
            reference = self.sample(targets)
            rows = self.observe(targets, .2, stationary=reference)
            for a in targets:
                self.query(a, 0xf1, expected=b'\x01')
            self.event('stop_pass', stop_ack_ms=(stopped-sync)/1e6, stopped_positions=rows[-1][1])
            return
        rows = []
        stable_since = None
        while time.monotonic_ns() - sync < 3_000_000_000:
            rows.extend(self.observe(targets, .02))
            values = rows[-1][1]
            if all(abs(values[a] - targets[a]) <= 8 for a in targets):
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= .15:
                    for a in targets:
                        self.query(a, 0xf1, expected=b'\x01')
                    starts = {a: next(((t-sync)/1e6 for t, p in rows if abs(p[a]-before[a]) > 5), None) for a in targets}
                    self.event('move_pass', label=label, axes=list(targets), duration_ms=(time.monotonic_ns()-sync)/1e6,
                               target=targets, final=values, error_degrees={a:(values[a]-targets[a])*360/TICKS for a in targets},
                               first_observed_motion_ms=starts)
                    return
            else:
                stable_since = None
        raise RuntimeError(f'{label}: brak osiągnięcia celu w 3 s')

    def run(self):
        for a in IDS:
            if self.query(a, 0x3a) != b'\x00':
                raise RuntimeError('Test wymaga początkowo wyłączonych napędów')
            identity = self.query(a, 0x40)
            saved = {c:self.read(a,c) for c in (0x82,0x84,0x8c,0x98,0x4a)}
            self.saved[a] = saved
            self.event('initial', axis=a, identity=identity.hex(), parameters={f'{c:02x}':v.hex() for c,v in saved.items()}, position=self.pos(a))
        for axes in ((1,), (1,2), IDS):
            self.event('stage_begin', axes=axes)
            pairs = {a:self.prepare_axis(a) for a in axes}
            self.anchor = {a:p[0] for a,p in pairs.items()}
            speeds = {a:p[1] for a,p in pairs.items()}
            targets = {a:p+STEP for a,p in self.anchor.items()}
            if axes == (1,):
                self.stage(targets, speeds)
                self.stop_all_zero(axes)
                baseline = self.sample(axes)
                self.send(0, 0x4b)
                self.observe(axes, .35, stationary=baseline)
                self.event('staged_reset_pass', axis=1)
                self.move(targets, speeds, 'stop', stop_early=True)
                self.move(self.anchor, speeds, 'return_after_stop')
            self.move(targets, speeds, 'forward')
            self.move(self.anchor, speeds, 'return')
            self.stop_all_zero(axes)
            for a in axes:
                self.write(a, 0xf3, b'\x00')
                self.write(a, 0x4a, b'\x00')
            self.event('stage_pass', axes=axes)
            self.anchor = {}

    def cleanup(self):
        errors = []
        self.anchor = {}
        # Najpierw zatrzymanie wszystkich; potem wyłączenie momentu i odtworzenie parametrów.
        try:
            self.stop_all_zero()
        except Exception as exc:
            errors.append(str(exc))
        for a in self.saved:
            for code,payload in ((0xf3,b'\x00'),(0x4a,b'\x00')):
                try:
                    self.write(a,code,payload)
                except Exception as exc:
                    errors.append(str(exc))
        for a, saved in self.saved.items():
            for code in (0x82,0x8c,0x98,0x4a):
                try:
                    self.write(a,code,saved[code])
                except Exception as exc:
                    errors.append(str(exc))
            try:
                self.query(a,0x3a,expected=b'\x00')
            except Exception as exc:
                errors.append(str(exc))
        self.event('cleanup', success=not errors, errors=errors)
        self.sock.close()
        self.lock.close()
        self.log.close()
        return not errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Fizyczny ruch; tylko po zatwierdzeniu operatora')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--owner-stopped', action='store_true', help='Właściciel CAN zamknięty osobno, launch może nadal działać; blokada CAN pozostaje obowiązkowa')
    args = parser.parse_args()
    if not args.execute or args.output is None:
        parser.error('Wymagane --execute i nowy plik --output; brak trybu automatycznego')
    if not args.owner_stopped and subprocess.run(['systemctl','is-active','--quiet','robot-web-status.service']).returncode == 0:
        raise SystemExit('Najpierw zatrzymaj usługę właściciela CAN')
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Sygnał {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    bench = Bench(args.output)
    passed = False
    try:
        bench.run()
        bench.event('all_pass')
        passed = True
    except BaseException as exc:
        bench.event('failed', message=str(exc))
    finally:
        passed = bench.cleanup() and passed
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
