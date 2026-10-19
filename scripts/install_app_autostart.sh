#!/usr/bin/env bash
# Instaluje start panelu przy rozruchu, bez sesji graficznej.
set -euo pipefail

usage() {
    cat <<'HELP'
Użycie:
  ./scripts/install_app_autostart.sh --dry-run [--user UŻYTKOWNIK]
  sudo ./scripts/install_app_autostart.sh [--user UŻYTKOWNIK]

Instaluje i włącza robot-web-status.service na następny rozruch komputera.
Panel działa jako użytkownik wywołujący sudo, bez logowania do pulpitu.
Nie uruchamia ani nie restartuje obecnie działającej aplikacji.
Wymaga przygotowanego ROS 2 Humble, zbudowanego panelu i workspace.

Opcje:
  --user NAZWA  Konto uruchamiające panel; wymagane przy wywołaniu z konta root.
  --dry-run     Sprawdź pliki i pokaż usługę bez zmian w systemie.
  -h, --help    Pokaż pomoc.
HELP
}

dry_run=false
app_user=${SUDO_USER:-$(id -un)}
while (( $# > 0 )); do
    case "$1" in
        --user)
            if (( $# < 2 )) || [[ -z "$2" || "$2" == -* ]]; then
                printf 'Błąd: po --user podaj nazwę konta.\n' >&2
                exit 2
            fi
            app_user=$2
            shift 2
            ;;
        --dry-run) dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Błąd: nieznany argument %s. Użyj --help.\n' "$1" >&2; exit 2 ;;
    esac
done

if [[ ! "$app_user" =~ ^[a-zA-Z_][a-zA-Z0-9_.-]*\$?$ ]] || ! id "$app_user" >/dev/null 2>&1; then
    printf 'Błąd: nieprawidłowe lub nieistniejące konto: %s.\n' "$app_user" >&2
    exit 2
fi
if [[ $(id -u "$app_user") == 0 ]]; then
    printf 'Błąd: panel powinien działać na zwykłym koncie. Podaj --user NAZWA.\n' >&2
    exit 2
fi
app_home=$(getent passwd "$app_user" | cut -d: -f6)
if [[ "$app_home" != /* || ! -d "$app_home" ]]; then
    printf 'Błąd: brak katalogu domowego konta %s.\n' "$app_user" >&2
    exit 1
fi
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
unit_template="$project_dir/system/application/robot-web-status.service.in"
for required_file in \
    "$unit_template" \
    "$project_dir/scripts/start_web_status.sh" \
    /opt/ros/humble/setup.bash \
    "$project_dir/ros2_ws/install/local_setup.bash" \
    "$project_dir/web/dist/index.html"; do
    if [[ ! -r "$required_file" ]]; then
        printf 'Błąd: brak pliku %s. Przygotuj aplikację według docs/web_status.md.\n' "$required_file" >&2
        exit 1
    fi
done
for required_command in python3 systemctl systemd-analyze; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        printf 'Błąd: wymagane polecenie %s.\n' "$required_command" >&2
        exit 1
    fi
done

temp_dir=$(mktemp -d)
trap 'rm -rf -- "$temp_dir"' EXIT
unit_file="$temp_dir/robot-web-status.service"
python3 - "$unit_template" "$unit_file" "$app_user" "$app_home" "$project_dir" <<'PY'
import re
import sys
from pathlib import Path

template, target, user, home, project = sys.argv[1:]


def quote(value, *, command=False):
    # systemd interpretuje specyfikatory %; ExecStart dodatkowo rozwija $.
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        sys.exit('Błąd: ścieżki nie mogą zawierać znaków sterujących.')
    value = value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    if command:
        value = value.replace('$', '$$')
    return '"' + value + '"'


values = {
    'USER': user,
    'HOME_ENV': quote('HOME=' + home),
    'START_SCRIPT': quote(project + '/scripts/start_web_status.sh', command=True),
}
rendered = re.sub(r'@(USER|HOME_ENV|START_SCRIPT)@',
                  lambda match: values[match[1]], Path(template).read_text())
Path(target).write_text(rendered)
PY
systemd-analyze --generators=no verify "$unit_file"
printf 'Usługa: robot-web-status.service; konto: %s; projekt: %s\n' "$app_user" "$project_dir"
if [[ "$dry_run" == true ]]; then
    cat "$unit_file"
    printf '\nPodgląd: instalacja do /etc/systemd/system/ i włączenie na następny rozruch.\n'
    exit 0
fi
if (( EUID != 0 )); then
    printf 'Błąd: instalacja wymaga sudo; do podglądu użyj --dry-run.\n' >&2
    exit 1
fi

unit_target=/etc/systemd/system/robot-web-status.service
if [[ -e "$unit_target" || -L "$unit_target" ]]; then
    cp -a -- "$unit_target" "$unit_target.bak-$(date -u +%Y%m%dT%H%M%S)-$$"
fi
install -m 0644 "$unit_file" "$unit_target"
systemctl daemon-reload
systemctl enable robot-web-status.service
systemctl is-enabled --quiet robot-web-status.service
printf '\nAutostart włączony. Panel uruchomi się po ponownym włączeniu komputera.\n'
printf 'Stan: systemctl status robot-web-status.service\n'
printf 'Logi: journalctl -u robot-web-status.service -b\n'
printf 'Start teraz (po zamknięciu ręcznie uruchomionej aplikacji): sudo systemctl start robot-web-status.service\n'
