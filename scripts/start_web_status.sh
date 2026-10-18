#!/usr/bin/env bash
# Jedno polecenie startu panelu; nie wymaga GNOME Terminal.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --record ]]; then
    shift
    umask 077
    export ROBOT_TRACE_DIR=${ROBOT_TRACE_DIR:-"$project_dir/log/manual-can"}
    export ROBOT_TRACE_SESSION=$(date -u +%s%N)
    mkdir -p -- "$ROBOT_TRACE_DIR"
    python3 - <<'PY'
import json, os, time
from pathlib import Path
Path(os.environ['ROBOT_TRACE_DIR'], 'session.json').write_text(json.dumps({
    'schema_version': 1, 'recording_id': os.environ['ROBOT_TRACE_SESSION'],
    'started_mono_ns': time.monotonic_ns(), 'started_utc_ns': time.time_ns(),
    'clock': 'Linux CLOCK_MONOTONIC + CLOCK_REALTIME', 'bitrate': 500000,
    'position_deadline_ms': 60, 'response_timeout_default_ms': 20,
    'limits_mib': {'can': 64, 'bridge': 32},
}, indent=2))
PY
    echo "Nagrywanie diagnostyki CAN/ROS/API: $ROBOT_TRACE_DIR (rotacja, maks. 96 MiB)"
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$project_dir/ros2_ws/install/local_setup.bash" ]]; then
    echo 'Brak ROS 2 Humble lub zbudowanego workspace. Patrz docs/web_status.md.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$project_dir/ros2_ws/install/local_setup.bash"
if ! python3 -c 'import fastapi, uvicorn.protocols.websockets.wsproto_impl, aiofiles, psutil, rclpy' 2>/dev/null; then
    echo 'Brak zależności panelu. Uruchom: sudo ./scripts/install_web_dependencies.sh' >&2
    exit 1
fi
export ROS_LOCALHOST_ONLY=1
exec ros2 launch robot_bringup web_status.launch.py "$@"
