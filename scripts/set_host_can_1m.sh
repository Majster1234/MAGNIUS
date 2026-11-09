#!/usr/bin/env bash
# Zmienia wyłącznie hosta; napędy użytkownik przestawił wcześniej na1M.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:---dry-run}
[[ $# -le 1 && ( $mode == --dry-run || $mode == --apply ) ]] || { echo 'Użycie: set_host_can_1m.sh [--dry-run|--apply]' >&2; exit 2; }
output="$project_dir/log/can-1m-20261024/$(date -u +%Y%m%dT%H%M%SZ)-$$"
config=/etc/robot-can/can0.conf
dropin=/etc/systemd/system/robot-web-status.service.d/streaming100.conf
profile="$project_dir/config/local/execution.production.yaml"
python3 - "$config" "$dropin" "$profile" <<'PY'
from pathlib import Path
import re,sys,yaml
config,dropin,profile=map(Path,sys.argv[1:])
assert len(re.findall(r'^BITRATE=\d+$',config.read_text(),re.M))==1
assert len(re.findall(r'can_bitrate:=\d+',dropin.read_text()))==1
assert yaml.safe_load(profile.read_text())['/**']['ros__parameters']['execution_enabled'] is False
PY
printf 'Host: BITRATE=1000000, TX=1000. Pliki: %s oraz %s\n' "$config" "$dropin"
echo 'Stop aplikacji → zmiana hosta → test samych odczytów CAN10 s → przywrócenie aplikacji.'
echo 'Bez zapisów do MKS, ENABLE, zerowania i komend ruchu.'
[[ $mode == --apply ]] || exit 0
(( EUID == 0 )) || { echo 'Wymagane sudo ./scripts/set_host_can_1m.sh --apply' >&2; exit 1; }
app_user=$(systemctl show robot-web-status.service --property=User --value)
[[ -n $app_user && $app_user != root ]]
mkdir -p "$output"
cp -a "$config" "$output/can0.conf.before"
cp -a "$dropin" "$output/streaming100.conf.before"
ip -j -details -statistics link show can0 > "$output/host-before.json"
systemctl stop robot-web-status.service
# Nie cofamy automatycznie do500k: napędy są już1M. Przy błędzie aplikacja
# pozostaje zatrzymana, a wynik i konfiguracja są dostępne do diagnostyki.
python3 - "$config" "$dropin" <<'PY'
from pathlib import Path
import re,sys
config,dropin=map(Path,sys.argv[1:])
s=config.read_text();s=re.sub(r'^BITRATE=\d+$','BITRATE=1000000',s,flags=re.M)
if re.search(r'^TX_QUEUE_LEN=',s,re.M):s=re.sub(r'^TX_QUEUE_LEN=.*$','TX_QUEUE_LEN=1000',s,flags=re.M)
else:s+='\nTX_QUEUE_LEN=1000\n'
config.write_text(s)
dropin.write_text(re.sub(r'can_bitrate:=\d+','can_bitrate:=1000000',dropin.read_text()))
PY
systemctl daemon-reload
systemctl restart robot-can@can0.service
ip -j -details -statistics link show can0 > "$output/host-after.json"
chown -R "$app_user:$(id -gn "$app_user")" "$output"
passed=false
if runuser -u "$app_user" -- python3 "$project_dir/scripts/test_can_communication.py" --output "$output/probe" --seconds 10 > "$output/probe-console.log" 2>&1; then passed=true; fi
cat "$output/probe-console.log"
# Aplikacja pozostaje z execution_enabled=false; jej uruchomienie jest odczytowe.
systemctl start robot-web-status.service
systemctl show robot-web-status.service --property=ActiveState,MainPID,NRestarts > "$output/application-after.txt"
chown -R "$app_user:$(id -gn "$app_user")" "$output"
printf 'Host1000000/TX1000; aplikacja przywrócona; test komunikacji=%s. Wyniki: %s\n' "$passed" "$output"
[[ $passed == true ]]
