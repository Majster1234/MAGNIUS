#!/usr/bin/env bash
# Jedno polecenie startu panelu; nie wymaga GNOME Terminal.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --record ]]; then
    shift
    umask 077
    export ROBOT_TRACE_DIR=${ROBOT_TRACE_DIR:-"$project_dir/log/manual-can"}
    export ROBOT_TRACE_SESSION=$(date -u +%s%N)
    # Pętla ciągła (bez zatrzymania po awarii): CAN 2048 MiB (128 × 16 MiB), most 1024 MiB (128 × 8 MiB).
    export ROBOT_TRACE_CAN_MIB=${ROBOT_TRACE_CAN_MIB:-2048}
    export ROBOT_TRACE_BRIDGE_MIB=${ROBOT_TRACE_BRIDGE_MIB:-1024}
    mkdir -p -- "$ROBOT_TRACE_DIR"
    # Poprzednia sesja zostaje w previous/ (migawka zaległej awarii nadal ją znajdzie); starsza jest usuwana.
    shopt -s nullglob
    previous=("$ROBOT_TRACE_DIR"/can-*.jsonl "$ROBOT_TRACE_DIR"/bridge-*.jsonl)
    if (( ${#previous[@]} )); then
        rm -rf -- "$ROBOT_TRACE_DIR/previous"
        mkdir -p -- "$ROBOT_TRACE_DIR/previous"
        mv -- "${previous[@]}" "$ROBOT_TRACE_DIR/previous/"
        if [[ -f $ROBOT_TRACE_DIR/session.json ]]; then mv -- "$ROBOT_TRACE_DIR/session.json" "$ROBOT_TRACE_DIR/previous/"; fi
    fi
    shopt -u nullglob
    python3 - <<'PY'
import json, os, time
from pathlib import Path
Path(os.environ['ROBOT_TRACE_DIR'], 'session.json').write_text(json.dumps({
    'schema_version': 1, 'recording_id': os.environ['ROBOT_TRACE_SESSION'],
    'started_mono_ns': time.monotonic_ns(), 'started_utc_ns': time.time_ns(),
    'clock': 'Linux CLOCK_MONOTONIC + CLOCK_REALTIME', 'bitrate': 500000,
    'position_deadline_ms': 60, 'response_timeout_default_ms': 20,
    'limits_mib': {'can': int(os.environ['ROBOT_TRACE_CAN_MIB']), 'bridge': int(os.environ['ROBOT_TRACE_BRIDGE_MIB'])},
    'fault_snapshot': {'file': 'faults/faults.rfs', 'pre_s': 10, 'post_s': 5, 'keep_s': 900},
}, indent=2))
PY
    echo "Nagrywanie diagnostyki CAN/ROS/API: $ROBOT_TRACE_DIR (pętla CAN ${ROBOT_TRACE_CAN_MIB} MiB, most ${ROBOT_TRACE_BRIDGE_MIB} MiB; migawki awarii: faults/faults.rfs)"
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$project_dir/ros2_ws/install/local_setup.bash" ]]; then
    echo 'Brak ROS 2 Humble lub zbudowanego workspace. Patrz docs/web_status.md.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$project_dir/ros2_ws/install/local_setup.bash"
if [[ -n ${ROBOT_OVERLAY_SETUP:-} ]]; then
    if [[ ! -r $ROBOT_OVERLAY_SETUP ]]; then
        echo "Brak setup overlay: $ROBOT_OVERLAY_SETUP" >&2
        exit 1
    fi
    source "$ROBOT_OVERLAY_SETUP"
fi
if ! python3 -c 'import fastapi, uvicorn.protocols.websockets.wsproto_impl, aiofiles, psutil, rclpy' 2>/dev/null; then
    echo 'Brak zależności panelu. Uruchom: sudo ./scripts/install_web_dependencies.sh' >&2
    exit 1
fi
export ROS_LOCALHOST_ONLY=1
# Nowy kontrakt parametrów nie może trafić do starego binarnego właściciela CAN.
# Jawna ścieżka użytkownika i jawny overlay mają pierwszeństwo.
if [[ -z ${ROBOT_OVERLAY_SETUP:-} && ! -f "$project_dir/ros2_ws/install/mks_servo_can/include/mks_servo_can/streaming_motion.hpp" ]]; then
    explicit_execution=false
    for argument in "$@"; do
        if [[ $argument == execution_parameters:=* ]]; then explicit_execution=true; fi
    done
    if [[ $explicit_execution == false ]]; then
        set -- "execution_parameters:=$project_dir/ros2_ws/src/robot_bringup/config/execution.legacy.example.yaml" "$@"
    fi
fi
exec ros2 launch robot_bringup web_status.launch.py "$@"
