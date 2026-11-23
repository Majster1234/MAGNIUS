#!/usr/bin/env python3
"""Wykresy testu pojedynczego F5 (scripts/f5_single_command_test.py): dla każdej osi położenie wału (zmierzone wobec idealnego
ruchu jednostajnego z zadaną prędkością) i prędkość wału (zmierzona z okna 50 ms wobec zadanej). Jeden plik PNG na prędkość F5.

    python3 scripts/plot_f5_single.py log/f5_single/<czas>        # → <czas>/wykres_<jedn>_jedn.png
"""
import json, sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

TICKS = 16384
SCALE = 16 / 128                     # RPM wału na jednostkę F5 (mikrokrok 128)
MEASURED, IDEAL = '#2a78d6', '#52514e'   # seria 1 palety referencyjnej; zadane — tekst drugorzędny (neutralny)
TEXT, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#e6e5e1', '#fcfcfb'


def load(run, axis, index, units):
    files = sorted(run.glob(f'J{axis}_f5_{index}_{units}.csv')) or (sorted(run.glob(f'f5_{index}_{units}.csv')) if axis == 1 else [])
    if not files:
        return None
    a = np.loadtxt(files[0], delimiter=',')
    t, ticks = a[:, 0], a[:, 1]
    deg = (ticks - ticks[0]) * 360 / TICKS
    sign = 1 if deg[-1] >= 0 else -1
    return t, deg * sign


def velocity(t, deg, window=0.05):
    g = np.arange(t[0], t[-1], 0.005)
    d = np.interp(g, t, deg); w = int(round(window / 0.005))
    return g[w // 2: w // 2 + len(g) - w], (d[w:] - d[:-w]) / window


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID); ax.spines[side].set_linewidth(1)
    ax.grid(True, color=GRID, linewidth=1, linestyle='-'); ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=8)


def main():
    run = Path(sys.argv[1])
    results = json.loads((run / 'results.json').read_text())
    if isinstance(results, list):            # wynik jednej osi (stary format)
        results = {'1': results}
    axes_ids = sorted(int(a) for a in results)
    speeds = [r['speed_units'] for r in results[str(axes_ids[0])] if 'speed_units' in r]
    for index, units in enumerate(speeds, start=1):
        commanded = units * SCALE * 6.0      # ° wału/s
        fig, grid = plt.subplots(len(axes_ids), 2, figsize=(12, 2.1 * len(axes_ids) + 1), facecolor=SURFACE,
                                 gridspec_kw={'width_ratios': [1, 1]}, squeeze=False)
        fig.suptitle(f'Pojedynczy rozkaz F5: {units} jedn. = {units * SCALE:g} RPM wału ({commanded:.2f} °/s), droga 10° wału, acc 254',
                     color=TEXT, fontsize=12, x=0.01, ha='left')
        for row, axis in enumerate(axes_ids):
            data = load(run, axis, index, units)
            res = next((r for r in results[str(axis)] if r.get('speed_units') == units), {})
            ax_p, ax_v = grid[row]
            for ax in (ax_p, ax_v):
                style(ax)
            if data is None:
                ax_p.text(.5, .5, 'brak danych', transform=ax_p.transAxes, ha='center', color=MUTED); continue
            t, deg = data
            moving = np.flatnonzero(deg > 0.3)
            t0 = t[moving[0]] if len(moving) else 0.0
            ideal = np.clip((t - t0) * commanded, 0, deg.max() if deg.max() > 0 else 10) + (deg[moving[0]] if len(moving) else 0) * 0
            ax_p.plot(t, ideal, color=IDEAL, linewidth=1.5, linestyle=(0, (4, 3)), label='ruch jednostajny (zadana prędkość)')
            ax_p.plot(t, deg, color=MEASURED, linewidth=2, solid_joinstyle='round', label='pomiar wału')
            ax_p.set_ylabel(f'J{axis}\npołożenie [°]', color=TEXT, fontsize=9)
            tv, v = velocity(t, deg)
            ax_v.axhline(commanded, color=IDEAL, linewidth=1.5, linestyle=(0, (4, 3)), label='zadana')
            ax_v.plot(tv, v, color=MEASURED, linewidth=2, solid_joinstyle='round', label='pomiar (okno 50 ms)')
            ax_v.set_ylabel('prędkość [°/s]', color=TEXT, fontsize=9)
            reach = np.flatnonzero(deg >= deg.max() - 0.15)
            t_end = (t[reach[0]] if len(reach) else t[-1]) + 0.3
            for ax in (ax_p, ax_v):
                ax.set_xlim(0, t_end)
            note = (f"średnio {res.get('mean_deg_s', float('nan')) / commanded * 100:.0f} % zadanej · postoje {res.get('stall_pct', '—')} % · "
                    f"odchylenie {res.get('fluct_deg', '—')}° (maks. {res.get('max_dev_deg', '—')}°)" +
                    (f" · skok {res['step_deg']}° co {res['steps_period_s']} s" if res.get('step_deg') else ''))
            ax_p.set_title(note, color=MUTED, fontsize=8, loc='left')
            if row == 0:
                ax_p.legend(frameon=False, fontsize=8, labelcolor=TEXT, loc='upper left')
                ax_v.legend(frameon=False, fontsize=8, labelcolor=TEXT, loc='upper right')
            if row == len(axes_ids) - 1:
                ax_p.set_xlabel('czas od rozkazu [s]', color=TEXT, fontsize=9); ax_v.set_xlabel('czas od rozkazu [s]', color=TEXT, fontsize=9)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        out = run / f'wykres_{index}_{units}_jedn.png'
        fig.savefig(out, dpi=130, facecolor=SURFACE); plt.close(fig)
        print(out)


if __name__ == '__main__':
    main()
