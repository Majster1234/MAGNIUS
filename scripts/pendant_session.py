#!/usr/bin/env python3
"""SPRZĘTOWY test długiej sesji teach pendanta (ruch robota; docs/teach_pendant.md „Test długiej sesji”).

1. Zerowanie pozycji sterowników MKS (92, J1–J6) i potwierdzenie odniesienia (jak scripts/tcp_motion_quality.py).
2. ENABLE=1; pozę startową (domyślnie 0/−30/40/0/45/0°, poza osobliwością nadgarstka) ustawia JOG osi z pendanta.
3. web/test/pendant-session.hw.cjs: prawdziwa strona #pendant w Chromium (dotyk jak telefon, spowolnione CPU),
   ruchy JOG TCP tam i z powrotem przez zadany czas; zliczenie przerwań, awarii i ponownych ENABLE.
4. Lista nowych migawek awarii (log/manual-can/faults) z czasu sesji.

Stała zgoda operatora dotyczy wyłącznie rozsprzęglonej mechaniki (AGENTS.md §5, pamięć projektu).
Użycie: python3 scripts/pendant_session.py --minutes 10 [--cpu 4] [--seed 1] [--home 0,-30,40,0,45,0] [--no-zero]
"""
import argparse, json, os, subprocess, sys, time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from tcp_motion_quality import Robot, Log  # noqa: E402

NODE = ROOT / 'build/node22/runtime/bin/node'


def snapshots():
    out = subprocess.run([sys.executable, '-m', 'robot_web_bridge.fault_snapshot', '--directory', str(ROOT / 'log/manual-can'), '--list'],
                         cwd=ROOT / 'ros2_ws/src/robot_web_bridge', capture_output=True, text=True)
    return [line for line in out.stdout.splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--minutes', type=float, default=10)
    parser.add_argument('--cpu', type=float, default=4, help='spowolnienie CPU przeglądarki (telefon ≈ 4)')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--home', default='0,-30,40,0,45,0')
    parser.add_argument('--no-zero', action='store_true', help='bez zerowania MKS (tylko gdy poprzedni test zerował)')
    args = parser.parse_args()
    home = [float(v) for v in args.home.split(',')]
    out = ROOT / 'log/pendant-sessions' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = Log(out)
    robot = Robot(log)
    before = set(snapshots())
    robot.ensure_ready()
    if not args.no_zero:
        robot.zero_and_confirm()
    robot.enable_all(True)
    env = dict(os.environ, PATH=f"{NODE.parent}:{os.environ['PATH']}")
    result = subprocess.run([str(NODE), 'test/pendant-session.hw.cjs', '--minutes', str(args.minutes), '--cpu', str(args.cpu),
                             '--seed', str(args.seed), '--home', json.dumps(home), '--out', str(out)], cwd=ROOT / 'web', env=env)
    time.sleep(8)   # migawka awarii powstaje ok. 6,5 s po zdarzeniu
    new = [line for line in snapshots() if line not in before]
    log('fault_snapshots', count=len(new), items=new)
    print(f'\nWynik: {out}/summary.json; nowe migawki awarii: {len(new)}')
    for line in new:
        print('  ' + line[:220])
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
