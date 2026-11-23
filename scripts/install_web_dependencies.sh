#!/usr/bin/env bash
# Instalacja wyłącznie zależności panelu; bez konfiguracji sieci i napędów.
set -euo pipefail

if [[ $# -ne 0 ]]; then
    echo "Użycie: sudo $0" >&2
    exit 2
fi
if [[ $EUID -ne 0 ]]; then
    echo "Uruchom: sudo $0" >&2
    exit 1
fi
apt-get update
apt-get install --yes --no-install-recommends \
    python3-fastapi python3-uvicorn python3-aiofiles \
    python3-psutil iproute2 ros-humble-rclpy ros-humble-diagnostic-msgs \
    ros-humble-controller-manager-msgs ros-humble-rclcpp ros-humble-std-srvs
