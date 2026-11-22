#!/usr/bin/env python3
"""Pojedynczy rozkaz F5 (jazda do celu ze stałą prędkością) do jednej osi — czy napęd jedzie płynnie przy małych prędkościach.

Bez pętli strumieniowej i korekt co 10 ms: jedna ramka F5 [prędkość, acc, cel absolutny], potem tylko odczyt położenia 31h
co ~10 ms. Odróżnia zachowanie samego napędu (schodki co pełny krok) od wpływu sterowania.

Wymaga ZATRZYMANEGO właściciela CAN (usługi panelu): `sudo systemctl stop robot-web-status.service` — inaczej zewnętrzny F5
uruchomi nadzór postoju i koliduje ze sterowaniem. Skrypt nie zmienia żadnych nastaw napędu (tylko 31h, 3Ah, 84h odczyt, F3, F5,
F7); nie zmienia zera, mikrokroku, trybu, prądu ani konfiguracji. Tryb odpowiedzi 8C00 (bez ACK F5) i SYNC 4A=0 — jak po pracy usługi.

Prędkość F5 = liczba całkowita jednostek MKS (mikrokrok 128: 1 jedn. = 0,125 RPM wału). Domyślnie 8 / 3 / 2 jedn. = 1 / 0,375 /
0,25 RPM (0,3 RPM nie jest przedstawialne). Każdy rozkaz: cel ±delta (domyślnie 10° wału, kierunek naprzemienny — powrót do startu).

    python3 scripts/f5_single_command_test.py                  # karta + sprawdzenie warunków (bez wysyłania ramek ruchu)
    python3 scripts/f5_single_command_test.py --execute        # test (usługa zatrzymana, zgoda operatora)
    python3 scripts/f5_single_command_test.py --execute --axes 1,2,3,4,5,6   # te same rozkazy kolejno na każdej osi
"""
import argparse, fcntl, json, os, select, socket, struct, subprocess, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
TICKS = 16384
SERVICE = 'robot-web-status.service'


def packet(axis, code, payload=b''):
    body = bytes([code]) + bytes(payload)
    return body + bytes([(axis + sum(body)) & 255])


class Bus:
    def __init__(self, log):
        self.log = log
        self.lock = open(f'/tmp/robot-can-{os.getuid()}-can0.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)   # właściciel CAN trzyma tę samą blokadę
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, 0x1fc)
        self.sock.bind(('can0',)); self.sock.setblocking(False)

    def send(self, axis, code, payload=b''):
        data = packet(axis, code, payload)
        self.sock.send(struct.pack('=IB3x8s', axis, len(data), data))
        self.log.write(json.dumps({'tx': data.hex(), 'axis': axis, 'ns': time.monotonic_ns()}) + '\n')

    def query(self, axis, code, payload=b'', timeout=.05):
        self.send(axis, code, payload)
        end = time.monotonic() + timeout; opcode = payload[0] if code == 0 else code
        while time.monotonic() < end:
            if not select.select([self.sock], [], [], max(0, end - time.monotonic()))[0]:
                break
            can_id, length, data = struct.unpack('=IB3x8s', self.sock.recv(16)); data = data[:length]
            if can_id & 0x20000000:
                raise RuntimeError(f'Ramka błędu CAN {can_id:x}')
            if can_id == axis and len(data) >= 2 and data[0] == opcode and (axis + sum(data[:-1])) & 255 == data[-1]:
                return data[1:-1]
        raise RuntimeError(f'Brak odpowiedzi CAN {axis} na {opcode:02x}')

    def position(self, axis):
        data = self.query(axis, 0x31)
        if len(data) != 6:
            raise RuntimeError('Niepełna pozycja enkodera')
        return int.from_bytes(data, 'big', signed=True)

    def enabled(self, axis):
        return self.query(axis, 0x3a) == b'\x01'

    def f5(self, axis, speed_units, acc, target):
        if not 1 <= speed_units <= 3000 or not 0 <= acc <= 255 or not -8388607 <= target <= 8388607:
            raise ValueError('Nieprzedstawialny rozkaz F5')
        self.send(axis, 0xf5, speed_units.to_bytes(2, 'big') + bytes([acc]) + (target & 0xffffff).to_bytes(3, 'big'))

    def stop(self, axis):
        self.send(axis, 0xf7)


def service_active():
    return subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE]).returncode == 0


def analyse(rows, start, target, units, scale):
    t = np.array([r[0] for r in rows]) / 1e9; p = np.array([r[1] for r in rows], float)
    t -= t[0]; sign = 1 if target > start else -1
    commanded = units * scale * 6.0                       # ° wału/s
    deg = (p - start) * 360 / TICKS * sign                # postęp w stronę celu [° wału]
    total = abs(target - start) * 360 / TICKS
    # Między ruszeniem a dojazdem: koniec = pierwsze osiągnięcie najdalszego położenia − 0,3° (napęd potrafi stanąć przed
    # celem albo za nim; próg względem celu wliczał postój po dojeździe — poprawione 19.11.2026).
    t_reach = t[np.argmax(deg >= deg.max() - 0.3)]
    moving = (deg > 0.3) & (t <= t_reach)
    out = {'speed_units': units, 'commanded_rpm': round(units * scale, 3), 'commanded_deg_s': round(commanded, 3),
           'distance_deg': round(total, 2), 'reached': bool(abs(p[-1] - target) <= 10), 'final_error_ticks': int(p[-1] - target)}
    if moving.sum() < 10:
        out.update(moved=bool(deg.max() > 0.3), note='za mało próbek w ruchu'); return out
    tm, dm = t[moving], deg[moving]
    out.update(moved=True, start_delay_s=round(float(t[np.argmax(deg > 0.3)]), 3),
               mean_deg_s=round(float((dm[-1] - dm[0]) / max(tm[-1] - tm[0], 1e-6)), 3))
    # Prędkość z okna 50 ms; postoje (< 50 % zadanej) i skoki położenia między postojami.
    g = np.arange(tm[0], tm[-1], .005); d = np.interp(g, tm, dm); w = 10
    v = (d[w:] - d[:-w]) / .05
    stall = v < .5 * commanded
    out['stall_pct'] = round(float(stall.mean() * 100), 1)
    # Odchylenie od ruchu jednostajnego (linia od pierwszej do ostatniej próbki ruchu).
    line = dm[0] + (tm - tm[0]) * (dm[-1] - dm[0]) / max(tm[-1] - tm[0], 1e-6)
    out['fluct_deg'] = round(float(np.std(dm - line)), 3); out['max_dev_deg'] = round(float(np.max(np.abs(dm - line))), 3)
    edges = np.flatnonzero(np.diff(stall.astype(int)) == -1)
    if len(edges) >= 3:
        out['steps_period_s'] = round(float(np.median(np.diff(g[edges]))), 3)
        out['step_deg'] = round(float(np.median(np.diff(d[edges]))), 3)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--axis', type=int, default=1, choices=range(1, 7))
    ap.add_argument('--axes', default=None, help='lista osi testowanych kolejno, np. 1,2,3,4,5,6 (zastępuje --axis)')
    ap.add_argument('--speeds', default='8,3,2', help='prędkości F5 w jednostkach MKS (mikrokrok 128: 1 = 0,125 RPM)')
    ap.add_argument('--acc', type=int, default=254, help='kod acc w ramce F5 (konfiguracja robota: 254)')
    ap.add_argument('--delta-deg', type=float, default=10.0, help='droga każdego rozkazu [° wału], maks. 20')
    ap.add_argument('--margin-deg', type=float, default=2.0, help='zapas zabezpieczenia ponad drogę [° wału], maks. 10')
    ap.add_argument('--execute', action='store_true')
    a = ap.parse_args()
    speeds = [int(x) for x in a.speeds.split(',')]
    if not 0 < a.delta_deg <= 20 or any(not 1 <= s <= 80 for s in speeds) or not 0 <= a.acc <= 255 or not 0 < a.margin_deg <= 10:
        sys.exit('Parametry poza zakresem testu (delta ≤ 20° wału, prędkość 1–80 jedn. ≤ 10 RPM)')
    scale = 16 / 128
    delta = round(a.delta_deg * TICKS / 360)
    durations = [a.delta_deg / (s * scale * 6) for s in speeds]
    print(f'KARTA — pojedyncze rozkazy F5, acc {a.acc}, droga {a.delta_deg:g}° wału '
          f'({a.delta_deg / 25:.2f}° przegubu przy przełożeniu 25), kierunek naprzemienny (+, −, + …):')
    for s, d in zip(speeds, durations):
        print(f'  F5 prędkość {s} jedn. = {s * scale:g} RPM wału ({s * scale * 6:.2f} °/s wału), czas jazdy ≈ {d:.1f} s')
    n_axes = len(a.axes.split(',')) if a.axes else 1
    print(f'  Osie: {a.axes or a.axis} — kolejno, ENABLE tylko testowanej osi; pozostałe osie bez komend. Odczyt 31h co ~10 ms. STOP (F7) i ENABLE=0: wyjście poza start ±'
          f'{a.delta_deg + a.margin_deg:g}° wału, brak odpowiedzi CAN, Ctrl+C; na końcu ENABLE=0. Bez zmian nastaw napędu.')
    print(f'  Razem ≈ {n_axes * (sum(durations) * 1.3 + 3 * len(speeds) + 2) / 60:.1f} min. Warunek: usługa {SERVICE} zatrzymana.')
    active = service_active()
    print(f'  Usługa: {"DZIAŁA — zatrzymaj: sudo systemctl stop " + SERVICE if active else "zatrzymana"}')
    if not a.execute or active:
        if a.execute:
            sys.exit('Przerwano: właściciel CAN działa.')
        return
    axes = [int(x) for x in a.axes.split(',')] if a.axes else [a.axis]
    if any(not 1 <= x <= 6 for x in axes):
        sys.exit('--axes: osie 1–6')
    print(f'  Osie kolejno: {", ".join(f"J{x}" for x in axes)} (zawsze tylko jedna z ENABLE=1).')
    out = ROOT / 'log' / 'f5_single' / datetime.now().strftime('%Y%m%d-%H%M%S'); out.mkdir(parents=True)
    log = (out / 'frames.jsonl').open('w', buffering=1)
    bus = Bus(log); all_results = {}
    for axis in axes:
        print(f'=== J{axis} ===', flush=True)
        all_results[axis] = run_axis(bus, axis, speeds, a, delta, durations, scale, out)
        (out / 'results.json').write_text(json.dumps(all_results, ensure_ascii=False, indent=1))
        if any('abort' in r for r in all_results[axis]):
            print('Przerwano serię po błędzie osi.', flush=True); break
    lines = ['| Oś | ' + ' | '.join(f'{u} jedn. ({u * scale:g} RPM): średnia/zadana, postoje, odchylenie std/maks., skok' for u in speeds) + ' |',
             '|---|' + '---|' * len(speeds)]
    for axis, rs in all_results.items():
        cells = []
        for r in rs:
            if 'abort' in r:
                cells.append('PRZERWANO: ' + r['abort'][:60]); continue
            ratio = r.get('mean_deg_s', 0) / r['commanded_deg_s']
            cells.append(f"{ratio * 100:.0f} %, {r.get('stall_pct', '—')} %, {r.get('fluct_deg', '—')}°/{r.get('max_dev_deg', '—')}°, "
                         f"{r.get('step_deg', '—')}°")
        lines.append(f'| J{axis} | ' + ' | '.join(cells) + ' |')
    (out / 'summary.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines) + f'\nWyniki: {out}')


def run_axis(bus, axis, speeds, a, delta, durations, scale, out):
    results = []; partial = None
    try:
        micro = int.from_bytes(bus.query(axis, 0, b'\x84'), 'big')
        if micro != 128:
            raise RuntimeError(f'Mikrokrok {micro}, test zakłada 128')
        home = bus.position(axis); limit = round((a.delta_deg + a.margin_deg) * TICKS / 360)
        if not bus.enabled(axis):
            bus.send(axis, 0xf3, b'\x01'); time.sleep(.3)
            if not bus.enabled(axis):
                raise RuntimeError('Brak potwierdzenia ENABLE')
        time.sleep(.5)
        current = bus.position(axis)
        for i, units in enumerate(speeds):
            target = home + (delta if i % 2 == 0 else 0)
            start = current
            if target == start:
                target = home + delta
            rows = [(time.monotonic_ns(), start)]; partial = (i, units, rows)
            bus.f5(axis, units, a.acc, target)
            end = time.monotonic() + durations[i] * 1.5 + 1.0
            while time.monotonic() < end:
                tick = time.monotonic(); p = bus.position(axis); rows.append((time.monotonic_ns(), p))
                if abs(p - home) > limit:
                    raise RuntimeError(f'J{axis} poza zakresem testu ({(p - home) * 360 / TICKS:.1f}° wału)')
                if len(rows) > 30 and all(abs(r[1] - target) <= 3 for r in rows[-30:]):
                    break   # ~0,3 s w celu
                time.sleep(max(0, .01 - (time.monotonic() - tick)))
            current = rows[-1][1]
            res = {'axis': axis, **analyse(rows, start, target, units, scale)}; results.append(res)
            np.savetxt(out / f'J{axis}_f5_{i + 1}_{units}.csv', np.array([((r[0] - rows[0][0]) / 1e9, r[1]) for r in rows]),
                       delimiter=',', fmt='%.4f', header='t_s,ticks')
            print(json.dumps(res, ensure_ascii=False), flush=True)
            time.sleep(.5)
    except (RuntimeError, KeyboardInterrupt, ValueError) as error:
        print(f'PRZERWANO J{axis}: {error}', flush=True); results.append({'abort': str(error)})
        if partial:   # przebieg przerwanego ruchu — do analizy przyczyny
            i, units, rows = partial
            np.savetxt(out / f'J{axis}_f5_{i + 1}_{units}_przerwany.csv',
                       np.array([((r[0] - rows[0][0]) / 1e9, r[1]) for r in rows]), delimiter=',', fmt='%.4f', header='t_s,ticks')
    finally:
        try:
            bus.stop(axis); time.sleep(.05); bus.send(axis, 0xf3, b'\x00'); time.sleep(.2)
            print(f'ENABLE J{axis} = {int(bus.enabled(axis))}', flush=True)
        except Exception as error:
            print(f'!!! Nie potwierdzono STOP/ENABLE=0 J{axis}: {error}', flush=True)
    return results


if __name__ == '__main__':
    main()
