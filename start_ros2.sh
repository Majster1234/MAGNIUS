#!/usr/bin/env bash
# Otwiera terminale projektu; może być wywoływany przez przyszłą aplikację.
set -euo pipefail

usage() {
    cat <<'HELP'
Użycie:
  ./start_ros2.sh [--dry-run] [-- POLECENIE ARGUMENTY...]

Otwiera trzy okna GNOME Terminal: Aplikacja, Diagnostyka i Workspace.
Każde wczytuje ROS 2 Humble i lokalny workspace, jeśli został zbudowany.
POLECENIE jest uruchamiane wyłącznie w oknie Aplikacja. Po jego zakończeniu
terminal pozostaje otwarty. Bez polecenia wszystkie okna są gotowe do pracy.

Opcje:
  --dry-run  Sprawdź wymagania i pokaż polecenia bez otwierania okien.
  -h, --help Pokaż tę pomoc.

Przykłady:
  ./start_ros2.sh
  ./start_ros2.sh --dry-run
  ./start_ros2.sh -- ros2 launch NAZWA_PAKIETU PLIK.launch.py
HELP
}

dry_run=false
startup_command=()
while (( $# > 0 )); do
    case "$1" in
        --dry-run)
            dry_run=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            if (( $# == 0 )); then
                printf 'Błąd: po -- podaj polecenie do uruchomienia.\n' >&2
                exit 2
            fi
            startup_command=("$@")
            break
            ;;
        *)
            printf 'Błąd: nieznany argument: %s\nUżyj --help.\n' "$1" >&2
            exit 2
            ;;
    esac
done

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${project_dir}/ros2_ws"
session_rc="${project_dir}/scripts/ros2_terminal.bash"
ros_setup=/opt/ros/humble/setup.bash

if [[ ! -r "$ros_setup" ]]; then
    printf 'Błąd: brak ROS 2 Humble: %s\n' "$ros_setup" >&2
    exit 1
fi
if [[ ! -d "$workspace/src" || ! -r "$session_rc" ]]; then
    printf 'Błąd: niekompletny projekt w %s.\n' "$project_dir" >&2
    exit 1
fi
if ! command -v gnome-terminal >/dev/null 2>&1; then
    printf 'Błąd: wymagany jest GNOME Terminal (gnome-terminal).\n' >&2
    exit 1
fi
if [[ "$dry_run" == false && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    printf 'Błąd: brak sesji graficznej. Uruchom skrypt na pulpicie lub użyj --dry-run.\n' >&2
    exit 1
fi
if [[ ! -r "$workspace/install/local_setup.bash" ]]; then
    printf 'Uwaga: workspace nie jest zbudowany. Terminale wczytają sam ROS 2 Humble.\n' >&2
fi

open_terminal() {
    local role="$1"
    local title="$2"
    shift 2
    local -a terminal_command=(
        gnome-terminal --window
        "--title=$title"
        "--working-directory=$workspace"
        -- env
        "ROBOT_ROS2_WORKSPACE=$workspace"
        "ROBOT_ROS2_SETUP=$ros_setup"
        "ROBOT_ROS2_ROLE=$role"
        "ROBOT_ROS2_TITLE=$title"
        bash --rcfile "$session_rc" -i -s -- "$@"
    )

    if [[ "$dry_run" == true ]]; then
        printf '%q ' "${terminal_command[@]}"
        printf '\n'
    elif ! "${terminal_command[@]}"; then
        printf 'Błąd: nie udało się otworzyć terminala %s. Sprawdź sesję GNOME.\n' "$title" >&2
        return 1
    fi
}

open_terminal application 'ROS 2 | Aplikacja' "${startup_command[@]}"
open_terminal diagnostics 'ROS 2 | Diagnostyka'
open_terminal development 'ROS 2 | Workspace'

if [[ "$dry_run" == false ]]; then
    printf 'Zlecono otwarcie trzech terminali ROS 2.\n'
fi
