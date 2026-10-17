#!/usr/bin/env bash
# Instaluje konfigurację systemową SocketCAN dla adaptera gs_usb.
set -euo pipefail

usage() {
    cat <<'HELP'
Użycie:
  sudo ./scripts/install_socketcan.sh --bitrate BIT_S [--interface can0]
  ./scripts/install_socketcan.sh --bitrate BIT_S [--interface can0] --dry-run

Instaluje can-utils oraz usługę systemd konfigurującą istniejący interfejs gs_usb.
Usługa uruchamia się wraz z urządzeniem, również po ponownym podłączeniu USB.
Instalacja uruchamia lub restartuje usługę i włącza interfejs z podanym bitrate.
Nie wysyła komend CAN i nie aktualizuje firmware.

Opcje:
  --bitrate BIT_S    Wymagana prędkość zgodna z napędami, w bit/s.
  --interface NAME   Nazwa interfejsu CAN; domyślnie can0.
  --dry-run          Sprawdź wymagania i pokaż plan bez zmian systemu.
  -h, --help         Pokaż pomoc.
HELP
}

interface=can0
bitrate=''
dry_run=false
while (( $# > 0 )); do
    case "$1" in
        --bitrate|--interface)
            if (( $# < 2 )); then
                printf 'Błąd: brak wartości opcji %s.\n' "$1" >&2
                exit 2
            fi
            if [[ "$1" == --bitrate ]]; then bitrate="$2"; else interface="$2"; fi
            shift 2
            ;;
        --dry-run) dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Błąd: nieznany argument %s. Użyj --help.\n' "$1" >&2; exit 2 ;;
    esac
done

if [[ ! "$bitrate" =~ ^[1-9][0-9]{0,6}$ ]] || (( bitrate > 1000000 )); then
    printf 'Błąd: --bitrate musi podawać prędkość Classical CAN od 1 do 1000000 bit/s.\n' >&2
    exit 2
fi
if [[ ! "$interface" =~ ^can[0-9]+$ || ${#interface} -gt 15 ]]; then
    printf 'Błąd: oczekiwano nazwy interfejsu can0, can1 itd.\n' >&2
    exit 2
fi

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$project_dir/system/can/robot-can@.service"
if [[ ! -r "$unit_source" ]]; then
    printf 'Błąd: brak szablonu usługi %s.\n' "$unit_source" >&2
    exit 1
fi
if [[ ! -r "/sys/class/net/$interface/type" ]] || [[ "$(cat "/sys/class/net/$interface/type")" != 280 ]]; then
    printf 'Błąd: interfejs %s nie istnieje lub nie jest interfejsem CAN. Podłącz U2C.\n' "$interface" >&2
    exit 1
fi
driver="$(basename -- "$(readlink -f -- "/sys/class/net/$interface/device/driver")")"
if [[ "$driver" != gs_usb ]]; then
    printf 'Błąd: %s używa sterownika %s; wymagany gs_usb.\n' "$interface" "$driver" >&2
    exit 1
fi

systemd-analyze verify "$unit_source"
printf 'Interfejs: %s, sterownik: %s, bitrate: %s bit/s.\n' "$interface" "$driver" "$bitrate"
printf 'Pliki: /etc/systemd/system/robot-can@.service i /etc/robot-can/%s.conf\n' "$interface"
printf 'Pakiet: can-utils. Usługa: robot-can@%s.service (start wraz z urządzeniem).\n' "$interface"
if [[ "$dry_run" == true ]]; then
    exit 0
fi
if (( EUID != 0 )); then
    printf 'Błąd: instalacja wymaga sudo; do samego podglądu użyj --dry-run.\n' >&2
    exit 1
fi

apt-get install --yes --no-remove can-utils
config_dir=/etc/robot-can
unit_target=/etc/systemd/system/robot-can@.service
config_target="$config_dir/$interface.conf"
install -d -m 0755 "$config_dir"
backup_suffix="$(date -u +%Y%m%dT%H%M%S)-$$"
for target in "$unit_target" "$config_target"; do
    if [[ -e "$target" ]]; then
        cp -a -- "$target" "$target.bak-$backup_suffix"
    fi
done
install -m 0644 "$unit_source" "$unit_target"
temp_config="$(mktemp)"
trap 'rm -f -- "$temp_config"' EXIT
printf 'BITRATE=%s\n' "$bitrate" > "$temp_config"
install -m 0644 "$temp_config" "$config_target"
systemctl daemon-reload
systemctl enable "robot-can@$interface.service"
systemctl restart "robot-can@$interface.service"
systemctl is-active --quiet "robot-can@$interface.service"
ip -details -statistics link show dev "$interface"
printf '\nSocketCAN skonfigurowany. Podgląd ruchu: candump %s\n' "$interface"
