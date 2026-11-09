#!/usr/bin/env python3
"""Sprawdzenie i wykonanie programu ruchowego z edytora „Programowanie” — klient wykonawcy programów w moście.

    python3 scripts/run_program.py <id>                 # sprawdzenie na sucho: ciągi ruchów + oś czasu per blok (bez ruchu)
    python3 scripts/run_program.py --file prog.json     # to samo dla pliku (np. przygotowanego przez agenta przed zapisem)
    python3 scripts/run_program.py <id> --execute       # wykonanie na robocie — TYLKO zatwierdzona rewizja

Logika (docs/programming.md): interpreter w moście liczy z wyprzedzeniem ciągi ruchów PTP/LIN (promień pominięcia
punktu: 0 = postój w punkcie, > 0 = płynne przejście) i wysyła je do wykonawcy jako jedną trajektorię na ciąg.
Wykonanie wymaga: (1) zatwierdzenia tej rewizji przez operatora w GUI („✔ Zatwierdź do wykonania”; każda zmiana
programu je unieważnia), (2) wyraźnej zgody operatora na ruch w rozmowie z agentem (AGENTS.md). Ctrl+C → STOP.
"""
import argparse, json, sys, time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tcp_motion_quality as q   # klient API (api, Robot.enable_all/ensure_ready, Log)


class ProgramFailure(q.Abort):
    pass


def require_approval(program):
    approval = program.get('approval') or {}
    if not approval.get('revision') or approval.get('revision') != program.get('revision'):
        raise ProgramFailure('Ta rewizja programu NIE jest zatwierdzona przez operatora. Zatwierdzenie: GUI → Programowanie → '
                             '„✔ Zatwierdź do wykonania” (każda zmiana programu je unieważnia).')
    return approval


def follow(log, verbose=True):
    """Odpytywanie /api/v1/program-run do końca przebiegu; wypisuje zmiany kursorów."""
    last = None
    while True:
        status = q.api('/api/v1/program-run')
        robot = (status.get('robot_cursor') or {})
        program = status.get('program_cursor') or {}
        line = (status['state'], robot.get('block_id'), robot.get('phase'), program.get('block_id'), program.get('state'))
        if verbose and line != last:
            blend = robot.get('blend')
            extra = f" przejście {blend['from']}→{blend['to']} {blend['progress']:.0%}" if blend else ''
            print(f"[{status['state']:>9}] robot: {robot.get('block_id') or '—'} ({robot.get('phase') or '—'}){extra} | "
                  f"program: {program.get('block_id') or '—'} ({program.get('state')})")
            log('cursor', state=status['state'], robot=robot, program=program)
            last = line
        if not status.get('active'):
            return status
        time.sleep(0.2)


def report(status, log):
    for warning in status.get('warnings', []):
        print(f"  uwaga [{warning.get('block_id') or '—'}] {warning['code']}: {warning['text']}")
    if status.get('timeline'):
        print('\n  blok         ciąg  start [s]  koniec [s]  postój  promień [mm] żądany→zastosowany')
        for entry in status['timeline']:
            print(f"  {entry['block_id']:<12} {entry['chain']:>4}  {entry['start_s']:>9.2f}  {entry['transition_end_s']:>10.2f}  "
                  f"{'tak' if entry['stop'] else 'nie':>6}  {entry['blend_mm_requested']:g} → {entry['blend_mm_applied']:g}")
    log('result', **{k: status.get(k) for k in ('state', 'message', 'error', 'warnings', 'timeline', 'total_s', 'chains')})
    if status['state'] != 'succeeded':
        error = status.get('error') or {}
        raise ProgramFailure(f"{status['state']}: {status.get('message')} (blok {error.get('block_id') or '—'})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('program', nargs='?', help='identyfikator programu (nazwa pliku bez .json w …/programs/)')
    parser.add_argument('--file', help='plik programu JSON zamiast magazynu (tylko sprawdzenie)')
    parser.add_argument('--execute', action='store_true', help='wykonaj na robocie (wymaga zatwierdzenia rewizji przez operatora)')
    args = parser.parse_args()
    if not args.program and not args.file:
        parser.error('podaj identyfikator programu albo --file')
    if args.execute and args.file:
        parser.error('--execute działa tylko dla programu z magazynu (zatwierdzonego w GUI)')
    out = q.ROOT / 'log/programs' / datetime.now().strftime('%Y%m%d-%H%M%S')
    log = q.Log(out)
    if args.file:
        program = json.loads(Path(args.file).read_text(encoding='utf-8'))
        log('program', name=program.get('name'), mode='check-file')
        q.api('/api/v1/program-run/check-document', {'program': program})
        status = follow(log, verbose=False)
        report(status, log)
        print(f"\nSprawdzenie OK: {status.get('message')}. Dziennik: {out}")
        return
    reply = q.api(f'/api/v1/programs/{args.program}')
    program, problems = reply['program'], reply.get('problems', [])
    if problems:
        raise ProgramFailure('Program ma uwagi: ' + '; '.join(problems))
    log('program', name=program.get('name'), revision=program.get('revision'), mode='execute' if args.execute else 'check')
    if not args.execute:
        q.api(f'/api/v1/programs/{args.program}/run', {'mode': 'check', 'revision': program['revision']})
        status = follow(log, verbose=False)
        report(status, log)
        print(f"\nSprawdzenie OK: {status.get('message')}. Wykonanie: --execute (po zatwierdzeniu w GUI i zgodzie operatora). "
              f'Dziennik: {out}')
        return
    approval = require_approval(program)
    log('approval_ok', revision=program['revision'], approved_at=approval.get('approved_at'), client=approval.get('client'))
    robot = q.Robot(log)
    robot.ensure_ready()
    robot.wait_quiet(20)
    if not all(a['enabled'] for a in robot.manual()['axes'] if a['available']):
        robot.enable_all(True)
    try:
        q.api(f'/api/v1/programs/{args.program}/run', {'mode': 'execute', 'revision': program['revision']})
        status = follow(log)
    except (q.Abort, KeyboardInterrupt) as error:
        log('aborted', reason=str(error) or 'przerwane przez operatora')
        try:
            q.api('/api/v1/program-run/stop', {})
        except q.Abort:
            pass
        raise
    report(status, log)
    print(f"\nProgram „{program.get('name')}” wykonany: {status.get('message')}. Dziennik: {out}")


if __name__ == '__main__':
    try:
        main()
    except q.Abort as error:
        raise SystemExit(f'PRZERWANO: {error}')
