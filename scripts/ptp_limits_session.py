#!/usr/bin/env python3
"""Przygotowanie/cleanup zatwierdzonej próby CAN 1; właściciel CAN musi być zamknięty."""
import argparse
import json
from pathlib import Path
import sys
import time

from test_ptp_hardware import Bench, TICKS, position_payload


class ClearBench(Bench):
    """F6=05 to odbiór synchroniczny; skuteczność ocenia dopiero pomiar po SYNC."""
    def event(self, name, **values):
        if name == 'stop_f7_zero':
            self.stop_replies = values['replies']
        super().event(name, **values)

    def stop_all_zero(self, required=()):
        super().stop_all_zero()
        for axis in required:
            f7 = self.stop_replies['f7'].get(str(axis))
            f6 = self.stop_replies['f6'].get(str(axis))
            if f7 != '01' or f6 not in ('01', '02', '05'):
                raise RuntimeError(f'CAN {axis}: odrzucono F7/F6: {f7}/{f6}')
            self.event('stop_receipt', axis=axis, f7=f7, f6=f6,
                       meaning='05 oznacza dane przyjęte do synchronizacji, nie potwierdzenie zatrzymania')


def session(directory, phase):
    directory.mkdir(parents=True, exist_ok=True)
    baseline = directory/'drive-baseline.json'
    bench = ClearBench(directory/f'{phase}.jsonl')
    passed = False
    try:
        if phase == 'preflight':
            for axis in (1, 2, 3, 5):
                parameters = {c: bench.read(axis, c) for c in (0x82, 0x84, 0x8c, 0x98, 0x4a)}
                bench.saved[axis] = parameters
                bench.event('baseline', axis=axis, parameters={f'{c:02x}': v.hex() for c, v in parameters.items()},
                            enabled=bench.query(axis, 0x3a).hex(), position=bench.pos(axis))
            baseline.write_text(json.dumps({str(a): {str(c): v.hex() for c, v in values.items()}
                                            for a, values in bench.saved.items()}, indent=2))
            if int.from_bytes(bench.saved[1][0x84], 'big') != 16:
                raise RuntimeError('Próba 1 RPM wymaga potwierdzonego mikrokroku 16; nie zmieniam mikrokroku')
            # Wstępne wyciszenie przed świeżym ENABLE. Odmowa F6 jest zapisana;
            # wymagany ACK F6 dotyczy właściwego testu po prepare_axis poniżej.
            bench.stop_all_zero()
            for axis in bench.saved:
                bench.write(axis, 0xf3, b'\x00')
            start, rpm = bench.prepare_axis(1)
            target = start+round(10*TICKS/360)
            bench.query(1, 0xf5, position_payload(target, rpm), b'\x05')
            bench.event('clear_test_staged', target=target, start=start)
            bench.stop_all_zero(required=(1,))
            reference = bench.sample((1,))
            bench.send(0, 0x4b)
            # Pasmo z bieżącego zlecenia: 0,3°, zamiast starego progu 5 impulsów.
            # Dodatkowo stan napędu i RPM po 100 ms od kontrolnego SYNC.
            rows = []
            sync = time.monotonic()
            checked = False
            while time.monotonic()-sync < 1.:
                values = bench.sample((1,));rows.append((time.monotonic_ns(), values))
                delta = abs(values[1]-reference[1])*360/TICKS
                if delta > .3:
                    raise RuntimeError(f'Ruch po kasowaniu bufora: {delta:.6f} stopnia > 0.3')
                if not checked and time.monotonic()-sync >= .1:
                    state = bench.query(1, 0xf1)
                    speed = int.from_bytes(bench.query(1, 0x32), 'big', signed=True)
                    bench.event('clear_state_after_100ms', state=state.hex(), speed_rpm=speed, delta_degrees=delta)
                    if state != b'\x01' or speed != 0:
                        raise RuntimeError(f'Po STOP i SYNC brak postoju: F1={state.hex()}, RPM={speed}')
                    checked = True
                time.sleep(.02)
            bench.query(1, 0xf1, expected=b'\x01')
            bench.event('clear_test_pass', final=rows[-1][1], maximum_delta_ticks=max(abs(p[1][1]-reference[1]) for p in rows))
        elif phase.startswith('cleanup'):
            bench.saved = {int(a): {int(c): bytes.fromhex(v) for c, v in values.items()}
                           for a, values in json.loads(baseline.read_text()).items()}
        else:
            raise ValueError(phase)
        passed = True
    except BaseException as exc:
        bench.event('failed', message=str(exc))
    finally:
        bench.anchor = {}
        passed = bench.cleanup() and passed
    return passed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', required=True)
    args = parser.parse_args()
    if not args.execute:
        parser.error('Wymagana jawna zgoda --execute')
    sys.exit(0 if session(args.output, args.phase) else 1)
