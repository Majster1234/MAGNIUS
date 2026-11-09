#!/usr/bin/env python3
"""Odbiór sekwencji neutralnego F5 STOP na rozsprzęgniętych wałach J1–J6.

Wyłącznie po zgodzie na fizyczny test. Jeden właściciel CAN musi być zwolniony
przed uruchomieniem. Nie zmienia kalibracji, geometrii, mikrokroku ani prądu.
Rejestruje TX, własne potwierdzenia nadania, RX i pomiary z monotonicznym czasem.
"""
import argparse
import json
import math
from pathlib import Path
import select
import signal
import socket
import struct
import time

from test_ptp_hardware import Bench, packet, position_payload

AXES = tuple(range(1, 7))
TICKS = 16384
STEP = round(5 * TICKS / 360)
REST = math.floor(.5 * TICKS / 360)
RANGE = math.floor(6 * TICKS / 360)
READ_CODES = (0x31, 0x32, 0x3a, 0x3e, 0xf1, 0x40)


def speed_code(microstep):
    if microstep not in (1, 2, 4, 8, 16, 32, 64, 128, 256):
        raise RuntimeError(f'Nieobsługiwany mikrokrok: {microstep}')
    scale = 1. if microstep in (16, 32, 64) else 16. / microstep
    raw = math.floor(1. / scale)
    if raw < 1:
        raise RuntimeError('Nie można uzyskać <=1 RPM bez zmiany mikrokroku')
    return raw, raw * scale


class RecoveryBench(Bench):
    def __init__(self, output):
        super().__init__(output)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), 1)
        self.guard = {}
        self.speeds = {}
        self.window = -1
        self.spent = 0
        self.transmit_confirmations = 0
        self.results = []
        self.read_timeout = .1
        self.query_latencies = []
        self.stop_gap = .006  # Przetestowane 5 ms +20%.
        self.enable_settle_min = 1.61  # Zmierzone 1340,809 ms +20%, w górę do 10 ms.
        self.enable_settle_timeout = 4.49  # Maks. zmierzone 3738,952 ms +20%, w górę do 10 ms.
        self.move_timeout = 6.0  # Próba dłuższej obserwacji: baza 5 s +20%.
        self.motion_stable_window = .6  # 500 ms +20%, bez zwiększania 0,5°.
        self.control_detection_only = False

    def event(self, name, **values):
        value = dict(event=name, mono_ns=time.monotonic_ns(), **values)
        self.log.write(json.dumps(value) + '\n')
        if name not in ('tx', 'rx', 'sample', 'tx_confirmation', 'query_latency'):
            print(json.dumps(value), flush=True)

    def query(self, axis, code, payload=b'', expected=None):
        self.send(axis, code, payload)
        sent = time.monotonic_ns()
        opcode = payload[0] if code == 0 else code
        read = code == 0 or code in READ_CODES
        timeout = self.read_timeout if read else .12
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            frame = self.receive(end - time.monotonic())
            if frame and frame[0] == axis and frame[1][0] == opcode:
                value = frame[1][1:-1]
                if opcode == 0xf5 and expected == b'\x05' and value == b'\x02':
                    continue
                latency_ms = (time.monotonic_ns() - sent) / 1e6
                self.query_latencies.append((axis, opcode, read, latency_ms))
                self.event('query_latency', axis=axis, opcode=opcode, read=read, latency_ms=latency_ms, timeout_ms=1000*timeout)
                if expected is not None and value != expected:
                    raise RuntimeError(f'CAN {axis} {opcode:02x}: oczekiwano {expected.hex()}, otrzymano {value.hex()}')
                return value
        self.event('query_timeout', axis=axis, opcode=opcode, read=read, timeout_ms=1000*timeout)
        raise TimeoutError(f'Timeout CAN {axis} opcode {opcode:02x}, limit {1000*timeout:.1f} ms')

    def calibrate_read_timeout(self):
        # Bazą jest istniejące 20 ms. Powiększamy wyłącznie przy rzeczywistym
        # timeout; nie dotyczy to świeżości sprzężenia zwrotnego w ruchu.
        base_ms = 20
        for attempt in range(5):
            self.read_timeout = base_ms / 1000
            try:
                for _ in range(30):
                    for axis in AXES:
                        for code in READ_CODES:
                            self.query(axis, code)
                        for code in (0x82, 0x84, 0x8c, 0x98, 0x4a):
                            self.read(axis, code)
            except TimeoutError:
                end = time.monotonic() + .12
                while time.monotonic() < end:
                    self.receive(end - time.monotonic())
                base_ms = math.ceil(base_ms * 1.5)
                self.event('read_timeout_calibration_retry', base_ms=base_ms)
                continue
            final_ms = math.ceil(base_ms * 1.2)
            self.read_timeout = final_ms / 1000
            for _ in range(10):
                for axis in AXES:
                    for code in READ_CODES:
                        self.query(axis, code)
                    for code in (0x82, 0x84, 0x8c, 0x98, 0x4a):
                        self.read(axis, code)
            self.event('read_timeout_calibrated', passing_base_ms=base_ms, margin_percent=20,
                       final_timeout_ms=final_ms, samples=len(self.query_latencies),
                       maximum_ms=max(item[3] for item in self.query_latencies), stop_gap_ms=1000*self.stop_gap)
            return
        raise RuntimeError('Nie uzyskano stabilnego odczytu w ograniczonej kalibracji timeoutów')

    def send(self, axis, code, payload=b''):
        # Konserwatywna rezerwacja odpowiedzi 8 B. Do 40% w oknie 10 ms.
        data = packet(axis, code, payload)
        cost = 55 + 10 * len(data) + (135 if axis else 0)
        window = time.monotonic_ns() // 10_000_000
        if window != self.window:
            self.window, self.spent = window, 0
        if self.spent + cost > 4000:
            time.sleep(max(0, ((window + 1) * 10_000_000 - time.monotonic_ns()) / 1e9))
            self.window, self.spent = time.monotonic_ns() // 10_000_000, 0
        self.spent += cost
        super().send(axis, code, payload)

    def receive(self, timeout):
        if not select.select([self.sock], [], [], max(0, timeout))[0]:
            return None
        raw, ancillary, flags, _ = self.sock.recvmsg(16, 128)
        axis, length, data = struct.unpack('=IB3x8s', raw)
        data = data[:length]
        kernel_ns = None
        for level, kind, value in ancillary:
            if level == socket.SOL_SOCKET and kind == getattr(socket, 'SO_TIMESTAMPNS', 35):
                sec, ns = struct.unpack('=qq', value[:16]); kernel_ns = sec * 10**9 + ns
        confirmation = bool(flags & socket.MSG_CONFIRM)
        self.event('tx_confirmation' if confirmation else 'rx', axis=axis, data=data.hex(), kernel_utc_ns=kernel_ns)
        if confirmation:
            self.transmit_confirmations += 1
            return None
        if axis & 0x20000000:
            raise RuntimeError(f'Ramka błędu CAN {axis:x}')
        if axis & 0xc0000000 or len(data) < 2 or (axis + sum(data[:-1])) & 255 != data[-1]:
            raise RuntimeError('Nieprawidłowa ramka/CRC')
        # Status odrzucenia F5 również trafia do trwałego zapisu i do query().
        return axis, data

    def pos(self, axis):
        data = self.query(axis, 0x31)
        if len(data) != 6:
            raise RuntimeError(f'CAN {axis}: niepełna pozycja')
        value = int.from_bytes(data, 'big', signed=True)
        if axis in self.guard and abs(value - self.guard[axis]) > RANGE:
            raise RuntimeError(f'CAN {axis}: przekroczono zakres ±6° wału')
        return value

    def snapshot(self):
        values = {}
        for axis in AXES:
            position = self.pos(axis); position_ns = time.monotonic_ns()
            speed = self.query(axis, 0x32); speed_ns = time.monotonic_ns()
            enable = self.query(axis, 0x3a); enable_ns = time.monotonic_ns()
            motor = self.query(axis, 0xf1); motor_ns = time.monotonic_ns()
            if len(speed) != 2 or len(enable) != 1 or len(motor) != 1:
                raise RuntimeError(f'CAN {axis}: niepełne pomiary')
            values[axis] = dict(position=position, position_rx_ns=position_ns,
                                rpm=int.from_bytes(speed, 'big', signed=True), speed_rx_ns=speed_ns,
                                enabled=bool(enable[0]), enable_rx_ns=enable_ns,
                                motor_state=motor[0], motor_rx_ns=motor_ns)
        self.event('sample', axes=values)
        return values

    def stationary(self, baseline, seconds=.4, enabled=()):
        rows = []
        start = time.monotonic()
        end = start + seconds + self.enable_settle_timeout
        confirmed = None
        while time.monotonic() < end:
            began = time.monotonic()
            values = self.snapshot()
            for axis, value in values.items():
                if abs(value['position'] - baseline[axis]) > REST:
                    raise RuntimeError(f'CAN {axis}: ruch >0,5° w oknie postoju')
                if value['enabled'] != (axis in enabled):
                    raise RuntimeError(f'CAN {axis}: niezgodny ENABLE')
            stamp = time.monotonic_ns()
            rows.append((stamp, values))
            all_stopped = all(v['rpm']==0 and v['motor_state']==1 for v in values.values())
            if not all_stopped:
                confirmed = None
            elif confirmed is None:
                confirmed = stamp
            if time.monotonic()-start>=seconds and confirmed is not None and stamp-confirmed>=100_000_000:
                return rows
            time.sleep(max(0, .02 - (time.monotonic() - began)))
        raise RuntimeError('Nie potwierdzono 0 RPM/F1=1 i dwóch zestawów >=100 ms w terminie postoju')

    def settle_enable(self, baseline, enabled=(), transition_axes=(), reason='enable'):
        """Przejście momentu jest rejestrowane osobno, przed oceną bufora.

        Tolerancja postoju zaczyna obowiązywać po doświadczalnym okresie ustalania
        oraz >=100 ms stabilnych
        pozycji, 0 RPM i F1=1. Globalny zakres ±6° nadal obowiązuje w przejściu.
        Zmiana odniesienia dotyczy tylko testu; nie zapisuje offsetu/kalibracji.
        """
        start = time.monotonic_ns()
        anchors = {}
        while time.monotonic_ns() - start < self.enable_settle_timeout*1e9:
            values = self.snapshot();now = time.monotonic_ns()
            for axis, value in values.items():
                if axis not in transition_axes and abs(value['position']-baseline[axis]) > REST:
                    raise RuntimeError(f'CAN {axis}: ruch osi bez przejścia ENABLE')
                if value['enabled'] != (axis in enabled):
                    raise RuntimeError(f'CAN {axis}: niezgodny ENABLE podczas ustalania')
                anchor = anchors.get(axis)
                if value['rpm'] != 0 or value['motor_state'] != 1:
                    anchors.pop(axis, None)
                elif anchor is None or abs(value['position']-anchor[1]) > REST:
                    anchors[axis] = (now, value['position'])
            if now-start >= self.enable_settle_min*1e9 and len(anchors)==6 and all(now-a[0]>=100_000_000 for a in anchors.values()):
                result = {axis: value['position'] for axis,value in values.items()}
                for axis in transition_axes:
                    self.guard[axis] = result[axis]
                self.event('enable_transition_settled', reason=reason, axes=list(transition_axes),
                           elapsed_ms=(now-start)/1e6, before=baseline, after=result,
                           note='Przejście ENABLE wyłączone z kryterium błędu postoju; pomiary zachowane')
                return result
            time.sleep(.01)
        raise RuntimeError(f'Nie potwierdzono uspokojenia po ENABLE w {self.enable_settle_timeout} s')

    def set_parameter(self, axis, code, value):
        self.write(axis, code, value)
        actual = self.read(axis, code)
        if actual != value:
            raise RuntimeError(f'CAN {axis}: niezgodny odczyt {code:02x}: {actual.hex()}')

    def set_enable(self, axis, enabled):
        self.write(axis, 0xf3, bytes([int(enabled)]))
        self.query(axis, 0x3a, expected=bytes([int(enabled)]))

    def safety_stop(self):
        errors = []
        for code, payload in ((0xf7, b''), (0xf5, bytes(6)), (0xf3, b'\0')):
            for axis in AXES:
                try:
                    self.send(axis, code, payload)
                except Exception as exc:
                    errors.append(f'CAN {axis} {code:02x}: {exc}')
            # Natywny MKS używa tego samego CAN ID dla żądania i odpowiedzi.
            # Pozostawiamy okno na odpowiedzi przed kolejną fazą tego samego ID.
            end = time.monotonic() + self.stop_gap
            while time.monotonic() < end:
                try:
                    self.receive(end - time.monotonic())
                except Exception as exc:
                    errors.append(str(exc))
        self.event('stop_sent', errors=errors, note='F7 → F5 STOP → ENABLE=0; wymaga pomiarów')
        if errors:
            raise RuntimeError('; '.join(errors))

    def recover(self, baseline):
        self.safety_stop()
        baseline = self.settle_enable(baseline, transition_axes=AXES, reason='stop_enable_off')
        self.stationary(baseline, .25)
        for axis in AXES:
            self.set_parameter(axis, 0x8c, b'\x01\x00')
            mode = self.read(axis, 0x4a)
            if mode == b'\x01':
                self.query(axis, 0xf5, bytes(6), b'\x05')
                self.event('neutral_accepted', axis=axis)
            elif mode != b'\x00':
                raise RuntimeError(f'CAN {axis}: nieznany SYNC {mode.hex()}')
        self.stationary(baseline, .25)
        for axis in AXES:
            self.set_parameter(axis, 0x4a, b'\0')
        for axis in AXES:
            self.set_parameter(axis, 0x8c, b'\0\0')
        self.stationary(baseline, .4)
        self.event('recovery_sequence_verified', baseline=baseline)
        return baseline

    def enable_probe(self, axis, baseline):
        self.set_parameter(axis, 0x8c, b'\x01\x00')
        self.set_enable(axis, True)
        baseline = self.settle_enable(baseline, enabled=(axis,), transition_axes=(axis,), reason='enable_on')
        self.stationary(baseline, .4, enabled=(axis,))
        self.set_parameter(axis, 0x4a, b'\x01')
        for sequence in range(3):
            self.send(0, 0x4b)
            self.stationary(baseline, .4, enabled=(axis,))
            self.event('residual_sync_probe_pass', axis=axis, sequence=sequence + 1)
        self.set_enable(axis, False)
        baseline = self.settle_enable(baseline, transition_axes=(axis,), reason='enable_off')
        self.stationary(baseline, .25)
        self.set_enable(axis, True)
        baseline = self.settle_enable(baseline, enabled=(axis,), transition_axes=(axis,), reason='reenable_on')
        self.stationary(baseline, .4, enabled=(axis,))
        self.event('reenable_probe_pass', axis=axis)
        return baseline

    def move(self, axis, target, baseline, label):
        self.query(axis, 0xf5, position_payload(target, self.speeds[axis]), b'\x05')
        self.stationary(baseline, .2, enabled=(axis,))
        self.send(0, 0x4b)
        end = time.monotonic() + self.move_timeout
        anchor = None
        while time.monotonic() < end:
            values = self.snapshot(); now = time.monotonic_ns()
            for other, value in values.items():
                if value['enabled'] != (other == axis):
                    raise RuntimeError(f'CAN {other}: niezgodny ENABLE w ruchu')
                if other != axis and abs(value['position'] - baseline[other]) > REST:
                    raise RuntimeError(f'CAN {other}: ruch nieuczestniczącej osi (pozycja {value["position"]}, baza {baseline[other]})')
                if abs(value['rpm']) > 2:
                    raise RuntimeError(f'CAN {other}: RPM przekracza limit próby')
            value = values[axis]
            # Pojedyncze 32H=-1 przy nieruchomym enkoderze nie oznacza
            # potwierdzonego ruchu, ale wstrzymuje potwierdzenie postoju.
            all_stopped = all(v['rpm'] == 0 and v['motor_state'] == 1 for v in values.values())
            direction = 1 if target > baseline[axis] else -1
            detectable = direction*(value['position']-baseline[axis]) >= STEP-2*REST
            in_position = abs(value['position'] - target) <= REST
            if (in_position or (self.control_detection_only and detectable)) and all_stopped:
                if anchor is None:
                    anchor = (now, value['position'])
                elif abs(value['position'] - anchor[1]) > REST:
                    anchor = (now, value['position'])
                elif now - anchor[0] >= self.motion_stable_window*1e9:
                    if abs(value['position'] - baseline[axis]) < STEP - 2 * REST:
                        raise RuntimeError('Kontrola dodatnia nie wykazała wyraźnego ruchu wału')
                    self.event('positive_control_pass', axis=axis, label=label, target=target, measured=value['position'],
                               target_error_degrees=(value['position']-target)*360/TICKS,
                               criterion='detectable_motion_and_standstill' if self.control_detection_only else 'target_and_standstill')
                    return {a: v['position'] for a, v in values.items()}
            else:
                anchor = None
            time.sleep(.01)
        raise RuntimeError(f'CAN {axis}: timeout dojścia {label}')

    def run(self, axes=AXES):
        initial = self.snapshot()
        baseline = {axis: value['position'] for axis, value in initial.items()}
        self.guard = baseline.copy()
        self.stationary(baseline, .4)
        for axis in AXES:
            identity = self.query(axis, 0x40)
            if len(identity) != 5 or identity[:4] != bytes([0x12, 1, 0, 9]):
                raise RuntimeError(f'CAN {axis}: wymagany MKS SERVO42D firmware 1.0.9, odczyt {identity.hex()}')
            saved = {code: self.read(axis, code) for code in (0x82, 0x84, 0x8c, 0x98, 0x4a)}
            self.saved[axis] = saved
            if saved[0x82] != b'\x05':
                raise RuntimeError(f'CAN {axis}: wymagany istniejący SR_vFOC=05')
            raw, rpm = speed_code(int.from_bytes(saved[0x84], 'big'))
            self.speeds[axis] = raw
            self.event('initial', axis=axis, identity=identity.hex(), parameters={f'{c:02x}': v.hex() for c, v in saved.items()},
                       position=baseline[axis], raw_rpm=raw, shaft_rpm=rpm)
        # Najpierw nadpisanie wszystkich nieznanych buforów, bez broadcast SYNC.
        self.calibrate_read_timeout()
        baseline = self.recover(baseline)
        for axis in axes:
            self.event('axis_test_begin', axis=axis, step_degrees=5, rpm=1, rest_tolerance_degrees=.5,
                       move_timeout_seconds=self.move_timeout, enable_settle_min_ms=1000*self.enable_settle_min,
                       motion_stable_window_ms=1000*self.motion_stable_window,
                       control_detection_only=self.control_detection_only,
                       enable_settle_timeout_ms=1000*self.enable_settle_timeout, response_timeout_ms=1000*self.read_timeout,
                       stop_gap_ms=1000*self.stop_gap)
            baseline = self.enable_probe(axis, baseline)
            start = baseline[axis]
            baseline = self.move(axis, start + STEP, baseline, 'plus_5_degrees')
            baseline = self.move(axis, start, baseline, 'return_minus_5_degrees')
            for repetition in range(2):
                # Znany, wykrywalny rozkaz +5° pozostaje oczekujący bez 4B.
                self.set_parameter(axis, 0x8c, b'\x01\x00')
                expected_enabled = baseline[axis]
                poisoned_target = expected_enabled + (STEP if repetition == 0 else -STEP)
                self.query(axis, 0xf5, position_payload(poisoned_target, self.speeds[axis]), b'\x05')
                self.event('known_buffer_staged', axis=axis, repetition=repetition+1, target=poisoned_target, before=expected_enabled)
                self.stationary(baseline, .2, enabled=(axis,))
                baseline = self.recover(baseline)
                baseline = self.enable_probe(axis, baseline)
                # Dwa pomiary po ustaleniu mają osobno pasmo ±0,5°. Ich różnica
                # >1° nie daje rozstrzygającej powtarzalności ani wykluczenia
                # wykonania znanego celu w samym przejściu ENABLE.
                if abs(baseline[axis]-expected_enabled) > 2*REST:
                    raise RuntimeError(f'CAN {axis}: niejednoznaczna zmiana pozycji po ustaleniu ENABLE względem kontroli bez oczekującego ruchu')
                self.event('overwrite_trial_pass', axis=axis, repetition=repetition + 1)
            self.set_enable(axis, False)
            baseline = self.settle_enable(baseline, transition_axes=(axis,), reason='axis_test_enable_off')
            self.set_parameter(axis, 0x4a, b'\0')
            self.set_parameter(axis, 0x8c, b'\0\0')
            self.stationary(baseline, .4)
            self.results.append(dict(axis=axis, passed=True, repetitions=2))
            self.event('axis_test_pass', axis=axis)
        if not self.transmit_confirmations:
            raise RuntimeError('Brak potwierdzeń nadania własnych ramek')
        self.event('all_pass' if tuple(axes)==AXES else 'selected_axes_pass', axes=self.results, tx_confirmations=self.transmit_confirmations)

    def latch_failed_recovery(self, baseline):
        self.stationary(baseline, .25)
        for axis in AXES:
            self.set_parameter(axis, 0x8c, b'\x01\x00')
            self.set_parameter(axis, 0x4a, b'\x01')
        self.stationary(baseline, .25)
        self.event('failed_recovery_latched', axes=list(AXES), sync=1, enabled=False,
                   note='Znacznik blokady dla właściciela po restarcie; nie wysłano 4B')

    def cleanup(self, keep_block=True):
        errors = []
        try:
            self.safety_stop()
        except Exception as exc:
            errors.append(str(exc))
        self.guard = {}
        try:
            final = self.snapshot()
            baseline = {a: v['position'] for a, v in final.items()}
            baseline = self.settle_enable(baseline, transition_axes=AXES, reason='cleanup_enable_off')
            self.stationary(baseline, .4)
            if keep_block:
                self.latch_failed_recovery(baseline)
        except Exception as exc:
            errors.append(str(exc))
        self.event('cleanup', success=not errors, errors=errors,
                   keep_block=keep_block,
                   note='ENABLE=0; nie wyzwalano bufora; niezaliczona próba pozostawia SYNC=1')
        self.sock.close();self.lock.close();self.log.close()
        return not errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--axes', type=int, choices=AXES, nargs='+', default=list(AXES))
    parser.add_argument('--keep-block', action='store_true', help='Zachowaj SYNC=1 także po udanym teście części osi')
    parser.add_argument('--control-detection-only', action='store_true', help='Kontrola dodatnia bada wykrywalny ruch i postój; błąd dojazdu zapisuje osobno')
    args = parser.parse_args()
    if not args.execute:
        parser.error('Wymagane --execute i wcześniejsza zgoda na fizyczne próby')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    def interrupted(number, _):
        raise KeyboardInterrupt(f'Sygnał {number}')
    signal.signal(signal.SIGTERM, interrupted)
    bench = RecoveryBench(args.output)
    bench.control_detection_only = args.control_detection_only
    passed = False
    try:
        bench.event('authorization', scope='J1–J6; wały odłączone; ±5°; <=1 RPM; zgoda operatora na dobór parametrów i bezpośrednie testy CAN w tej sesji')
        bench.run(tuple(dict.fromkeys(args.axes)));passed = True
    except BaseException as exc:
        bench.event('failed', message=str(exc))
    finally:
        passed = bench.cleanup(keep_block=not passed or args.keep_block) and passed
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
