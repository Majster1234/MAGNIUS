#!/usr/bin/env python3
"""Buduje izolowany wykonawca stanowiskowy; nie zmienia biblioteki ani usług ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def replace_once(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError(f'Zmieniony kod źródłowy: oczekiwano {count} wystąpień {old!r}')
    return text.replace(old, new)


def build(output, sync_us=1000, response_ms=20, feedback_ms=60, encoder_end=14, diag_end=18, stage_ms=8,
          allow_stale_prestart_speed=False, diag_guard_ms=2, fix_completion_handover=False):
    root = Path(__file__).resolve().parents[1]
    source = root/'ros2_ws/src/mks_servo_can'
    if output.exists():
        raise ValueError('Katalog wyniku już istnieje; użyj nowego dla kolejnej próby')
    output.mkdir(parents=True)
    shutil.copytree(source/'include', output/'include')
    timing = output/'include/mks_servo_can/motion_timing.hpp'
    text = timing.read_text()
    for old, new in (('kStageWindow{8}', f'kStageWindow{{{stage_ms}}}'),
                     ('kFeedbackWindowEnd{14}', f'kFeedbackWindowEnd{{{encoder_end}}}'),
                     ('kDiagnosticWindowEnd{18}', f'kDiagnosticWindowEnd{{{diag_end}}}')):
        text = replace_once(text, old, new)
    timing.write_text(text)
    program = (source/'src/program_bus.cpp').read_text()
    program = replace_once(program, 'e.due+1ms', f'e.due+std::chrono::microseconds({sync_us})', 2)
    if allow_stale_prestart_speed:
        # Postój przed startem potwierdzają F1 i stabilność enkodera w procedurze
        # zerowania. Dokładne RPM=0 wymagamy po końcu +100 ms, jak zlecono.
        program = replace_once(program, 'good=speed && *speed==0;', 'good=bool(speed);')
    # Zlecone kryteria próby: przeregulowanie dozwolone, końcowe ±0,3°,
    # pierwsze wymaganie RPM=0 dopiero 100 ms po nominalnym końcu.
    program = replace_once(program, 'if(actual<bounds[0]-a.tracking_error || actual>bounds[1]+a.tracking_error)',
                           'if(false && (actual<bounds[0]-a.tracking_error || actual>bounds[1]+a.tracking_error))')
    program = replace_once(program, 'if(s.elapsed>=program.duration && e.frame==program.frames.size())',
                           'if(s.elapsed>=program.duration+.1 && e.frame==program.frames.size())')
    program = replace_once(program, '!d.speed.fresh(now,150ms) || d.speed.value!=0) reached=false;',
                           '!d.speed.fresh(now,150ms) || d.speed.stamp<e.started+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(program.duration+.1)) || d.speed.value!=0) reached=false;')
    (output/'program_bus.cpp').write_text(program)
    sources = [output/'program_bus.cpp', root/'scripts/ptp_limits_hardware.cpp']
    if (stage_ms, encoder_end, diag_end, diag_guard_ms) != (8, 14, 18, 2) or fix_completion_handover:
        manager = (source/'src/bus_manager.cpp').read_text()
        manager = replace_once(manager, 'now-cycle_start_>=kDiagnosticWindowEnd-std::chrono::milliseconds(2)',
                               f'now-cycle_start_>=kDiagnosticWindowEnd-std::chrono::milliseconds({diag_guard_ms})')
        pieces = []
        for begin, end in (('bool CanBusManager::send_query(', 'bool CanBusManager::send_manual('),
                           ('void CanBusManager::schedule_feedback(', 'void CanBusManager::schedule_background('),
                           ('void CanBusManager::schedule_background(', 'void CanBusManager::schedule_motion(')):
            pieces.append(manager[manager.index(begin):manager.index(end)])
        if fix_completion_handover:
            tick = manager[manager.index('void CanBusManager::tick('):manager.index('BusSnapshot CanBusManager::snapshot(')]
            tick = replace_once(tick, 'if(source_==Source::program) schedule_program(now); else schedule_motion(now);',
                                'const bool program_iteration=source_==Source::program;\n'
                                '      if(program_iteration) schedule_program(now); else schedule_motion(now);')
            tick = replace_once(tick, 'if(source_==Source::program) continue; // Wykonawca',
                                'if(program_iteration) continue; // Wykonawca')
            pieces.append(tick)
        override = output/'schedule_override.cpp'
        override.write_text('#include "mks_servo_can/bus_manager.hpp"\n'
                            '#include "mks_servo_can/manual_motor_controller.hpp"\n'
                            '#include "mks_servo_can/motion_sample.hpp"\n'
                            '#include "mks_servo_can/trace.hpp"\n'
                            '#include <algorithm>\nnamespace mks_servo_can {\n'
                            'namespace { double milliseconds(Time a,Time b) { return std::chrono::duration<double,std::milli>(a-b).count(); } }\n'
                            +''.join(pieces)+'\n}\n')
        sources.append(override)
    library_dir = root/'install/can-schedule-ros/mks_servo_can/lib'
    library = library_dir/'libmks_servo_can.so'
    if not library.exists():
        raise ValueError('Brak zweryfikowanej biblioteki w install/can-schedule-ros')
    command = ['g++', '-std=c++17', '-O0', '-DNDEBUG', '-Wall', '-Wextra', '-pthread',
               f'-DPTP_BENCH_FEEDBACK_MS={feedback_ms}', f'-DPTP_BENCH_RESPONSE_MS={response_ms}',
               '-I'+str(output/'include'), *map(str, sources), '-L'+str(library_dir),
               '-Wl,-rpath,'+str(library_dir), '-lmks_servo_can', '-o', str(output/'ptp_limits')]
    metadata = dict(sync_us=sync_us, response_ms=response_ms, feedback_ms=feedback_ms,
                    stage_ms=stage_ms, encoder_end=encoder_end, diag_end=diag_end, diag_guard_ms=diag_guard_ms,
                    allow_stale_prestart_speed=allow_stale_prestart_speed,
                    fix_completion_handover=fix_completion_handover,
                    criteria=dict(target_shaft_degrees=10, maximum_rpm=1, final_tolerance_degrees=.3,
                                  speed_grace_ms=100, transient_tracking_check=False, duration_seconds=3.2),
                    command=command, library=str(library), library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                    sha256={str(p.relative_to(root)) if p.is_relative_to(root) else p.name:
                            hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (output/'build.json').write_text(json.dumps(metadata, indent=2))
    subprocess.run(command, check=True)
    return output/'ptp_limits'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sync-us', type=int, default=1000)
    parser.add_argument('--response-ms', type=int, default=20)
    parser.add_argument('--feedback-ms', type=int, default=60)
    parser.add_argument('--encoder-end', type=int, default=14)
    parser.add_argument('--diag-end', type=int, default=18)
    parser.add_argument('--stage-ms', type=int, default=8)
    parser.add_argument('--diag-guard-ms', type=int, default=2)
    parser.add_argument('--allow-stale-prestart-speed', action='store_true')
    parser.add_argument('--fix-completion-handover', action='store_true')
    args = parser.parse_args()
    print(build(**vars(args)))
