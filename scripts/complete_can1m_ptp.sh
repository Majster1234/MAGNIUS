#!/usr/bin/env bash
# Kończy zlecone wdrożenie hosta1M i odblokowanie PTP bez ruchu.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ ${1:-} == --apply && $# == 1 ]] || { echo 'Użycie: sudo ./scripts/complete_can1m_ptp.sh --apply'; exit 2; }
(( EUID == 0 )) || { echo 'Wymagane uprawnienia administratora.' >&2; exit 1; }
app_user=$(systemctl show robot-web-status.service --property=User --value)
[[ -n $app_user && $app_user != root ]]
output="$project_dir/log/can-1m-20261024/unlock-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$output"
cp -a "$project_dir/config/local/execution.production.yaml" "$output/execution-before.yaml"
# Najpierw komunikacja, nadal z execution_enabled=false.
"$project_dir/scripts/set_host_can_1m.sh" --apply
address=$(ip -j -4 address show enp2s0 | python3 -c 'import json,sys;print(next(a["local"] for n in json.load(sys.stdin) for a in n["addr_info"] if a["family"]=="inet"))')
ready=false
for attempt in {1..40}; do
    if python3 - "$address" <<'PY'
import json,sys,urllib.request
try:
 with urllib.request.urlopen(f'http://{sys.argv[1]}:8080/api/v1/manual/status',timeout=1) as r:s=json.load(r)
 ready=s['fresh'] and not s['busy'] and not s['fault']
except (OSError,ValueError,KeyError):ready=False
sys.exit(0 if ready else 1)
PY
    then ready=true; break; fi
    sleep 1
done
[[ $ready == true ]]
chown -R "$app_user:$(id -gn "$app_user")" "$output"
if ! runuser -u "$app_user" -- python3 "$project_dir/scripts/prepare_ptp_test_profile.py" --address "$address" --output "$output/unlock" > "$output/unlock-console.log" 2>&1; then
    cp -a "$output/execution-before.yaml" "$project_dir/config/local/execution.production.yaml"
    cat "$output/unlock-console.log" >&2
    echo 'Nie odblokowano PTP; host pozostaje1M, profil wykonania nadal wyłączony.' >&2
    exit 1
fi
cat "$output/unlock-console.log"
# Profil zmienia się dopiero przy restarcie. Bez ENABLE i rozkazów ruchu.
if ! systemctl restart robot-web-status.service; then
    cp -a "$output/execution-before.yaml" "$project_dir/config/local/execution.production.yaml"
    systemctl start robot-web-status.service || true
    exit 1
fi
systemctl show robot-web-status.service --property=ActiveState,MainPID,NRestarts > "$output/service-after.txt"
chown -R "$app_user:$(id -gn "$app_user")" "$output"
printf 'Host1M, test CAN zaliczony, PTP stanowiska odblokowane1 RPM/acc1. Bez ruchu. Raport: %s\n' "$output"
