#!/usr/bin/env bash
# Wdrożenie nowego builda bez ruchu i bez zmiany bitrate napędów.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:---dry-run}
if [[ $# -gt 1 || ( $mode != --dry-run && $mode != --apply ) ]]; then
    echo 'Użycie: deploy_streaming100.sh [--dry-run|--apply]' >&2; exit 2
fi
profile="$project_dir/config/local/execution.production.yaml"
overlay="$project_dir/install/streaming100-ros/local_setup.bash"
for file in "$profile" "$overlay" "$project_dir/web/dist/index.html"; do
    [[ -r $file ]] || { echo "Brak: $file" >&2; exit 1; }
done
# Nie pobieramy konta ani adresu LAN z domysłów.
app_user=$(systemctl show robot-web-status.service --property=User --value)
[[ -n $app_user && $app_user != root ]] || { echo 'Brak użytkownika usługi.' >&2; exit 1; }
interface=enp2s0
app_address=$(ip -j -4 address show "$interface" | python3 -c 'import json,sys; print(next(a["local"] for n in json.load(sys.stdin) for a in n["addr_info"] if a["family"]=="inet"))')
bitrate=$(ip -j -details link show can0 | python3 -c 'import json,sys; print(json.load(sys.stdin)[0]["linkinfo"]["info_data"]["bittiming"]["bitrate"])')
tx_before=$(ip -j link show can0 | python3 -c 'import json,sys; print(json.load(sys.stdin)[0]["txqlen"])')
[[ $bitrate == 500000 || $bitrate == 1000000 ]] || { echo 'Nieobsługiwany bitrate.' >&2; exit 1; }
python3 - "$profile" "$app_address" <<'PY'
import json,sys,urllib.request,yaml
profile,address=sys.argv[1:]
params=yaml.safe_load(open(profile))['/**']['ros__parameters']
assert params['execution_enabled'] is False,'Wdrożenie wymaga execution_enabled=false'
assert params['execution_use_ack'] is False and params['execution_position_period_ms']==10
with urllib.request.urlopen(f'http://{address}:8080/api/v1/manual/status',timeout=5) as r:manual=json.load(r)
with urllib.request.urlopen(f'http://{address}:8080/api/v1/motion/status',timeout=5) as r:motion=json.load(r)
with urllib.request.urlopen(f'http://{address}:8080/api/v1/configuration',timeout=5) as r:configuration=json.load(r)
assert manual['fresh'] and not manual['busy'] and not manual['fault'],'Wymagany świeży stan bez ruchu i fault'
assert not motion['busy'],'Program nadal działa'
operation=configuration['operation']
assert operation['phase'] in ('idle','done') and operation['success'] is True,'Konfiguracja nie jest uzgodniona'
assert manual['configuration_revision']==configuration['active']['revision'],'Niezgodna rewizja konfiguracji i stanu osi'
PY
printf 'Nowy overlay: %s\nProfil: %s\nBitrate zachowany: %s; TX: %s → 1000\n' "$overlay" "$profile" "$bitrate" "$tx_before"
echo 'Operacje: kopia konfiguracji, stop aplikacji, instalacja drop-in/CAN, daemon-reload, start i kontrola HTTP.'
echo 'Bez ENABLE, zerowania, F5/F6/FE i zmiany CanRate napędów. execution_enabled=false.'
[[ $mode == --apply ]] || exit 0
(( EUID == 0 )) || { echo 'Instalacja wymaga administratora: sudo ./scripts/deploy_streaming100.sh --apply' >&2; exit 1; }
backup="$project_dir/log/streaming100-deployment/$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$backup"
dropin=/etc/systemd/system/robot-web-status.service.d/streaming100.conf
can_unit=/etc/systemd/system/robot-can@.service
can_config=/etc/robot-can/can0.conf
paths=("$dropin" "$can_unit" "$can_config")
for index in "${!paths[@]}"; do
    if [[ -e ${paths[$index]} ]]; then cp -a -- "${paths[$index]}" "$backup/$index"; fi
done
systemctl cat robot-web-status.service > "$backup/service-before.txt"
systemctl show robot-web-status.service --property=MainPID,NRestarts > "$backup/state-before.txt"
rollback() {
    status=$?
    trap - ERR
    echo 'Wdrożenie nie powiodło się; odtwarzam konfigurację.' >&2
    systemctl stop robot-web-status.service || true
    for index in "${!paths[@]}"; do
        if [[ -e $backup/$index ]]; then cp -a -- "$backup/$index" "${paths[$index]}"; else rm -f -- "${paths[$index]}"; fi
    done
    ip link set dev can0 txqueuelen "$tx_before" || true
    systemctl daemon-reload || true
    systemctl start robot-web-status.service || true
    exit "$status"
}
trap rollback ERR
systemctl stop robot-web-status.service
mkdir -p /etc/systemd/system/robot-web-status.service.d /etc/robot-can
# Nie zatrzymujemy magistrali i nie zmieniamy jej prędkości.
install -m 0644 "$project_dir/system/can/robot-can@.service" "$can_unit"
printf 'BITRATE=%s\nTX_QUEUE_LEN=1000\n' "$bitrate" > "$can_config"
python3 - "$dropin" "$project_dir" "$bitrate" <<'PY'
from pathlib import Path
import sys
output,project,bitrate=sys.argv[1:]
def quote(text):
    return '"'+text.replace('\\','\\\\').replace('"','\\"').replace('%','%%').replace('$','$$')+'"'
Path(output).write_text('[Service]\nEnvironment='+quote('ROBOT_OVERLAY_SETUP='+project+'/install/streaming100-ros/local_setup.bash')+'\nExecStart=\nExecStart=/usr/bin/bash '+quote(project+'/scripts/start_web_status.sh')+' '+quote('execution_parameters:='+project+'/config/local/execution.production.yaml')+' '+quote('can_bitrate:='+bitrate)+'\n')
PY
ip link set dev can0 txqueuelen 1000
systemctl daemon-reload
systemctl start robot-web-status.service
ready=false
for attempt in {1..40}; do
    if python3 - "$app_address" <<'PY'
import json,sys,urllib.request
try:
    with urllib.request.urlopen(f'http://{sys.argv[1]}:8080/api/v1/can/status',timeout=1) as r:data=json.load(r)
    v=data['summary']['values']
    ready=data['fresh'] and v['command_period_ms']==10 and v['motion_use_ack'] is False
except (OSError,ValueError,KeyError):
    ready=False
sys.exit(0 if ready else 1)
PY
    then ready=true; break; fi
    sleep 1
done
[[ $ready == true ]]
systemctl show robot-web-status.service --property=MainPID,ActiveState,NRestarts > "$backup/state-after.txt"
ip -j -details link show can0 > "$backup/can-after.json"
chown -R "$app_user:$(id -gn "$app_user")" "$backup"
trap - ERR
printf 'Wdrożono nowy build bez ruchu. Bitrate: %s; TX=1000. Kopia: %s\n' "$bitrate" "$backup"
