#!/usr/bin/env bash
# Jedno polecenie startu panelu; nie wymaga GNOME Terminal.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
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
