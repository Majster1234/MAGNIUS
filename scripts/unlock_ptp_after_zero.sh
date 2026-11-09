#!/usr/bin/env bash
# Zerowanie zlecone przez operatora; żadnego ruchu/ENABLE.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ $# == 1 && $1 == --zero-decoupled ]] || { echo 'Użycie: sudo ./scripts/unlock_ptp_after_zero.sh --zero-decoupled';exit 2; }
(( EUID == 0 )) || { echo 'Wymagane sudo.' >&2;exit 1; }
app_user=$(systemctl show robot-web-status.service --property=User --value)
[[ -n $app_user && $app_user != root ]]
output="$project_dir/log/can-1m-20261024/zero-unlock-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$output"
profile="$project_dir/config/local/execution.production.yaml"
cp -a "$profile" "$output/execution-before.yaml"
# Zerowanie i włączanie mają oddzielne etapy; błędy nie włączą wykonawcy.
python3 - "$profile" <<'PY'
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1]);s=yaml.safe_load(p.read_text());s['/**']['ros__parameters']['execution_enabled']=False;p.write_text(yaml.safe_dump(s,sort_keys=False))
PY
systemctl stop robot-web-status.service
chown -R "$app_user:$(id -gn "$app_user")" "$output"
zero_ok=false
if runuser -u "$app_user" -- python3 "$project_dir/scripts/zero_ptp_encoders.py" --zero-decoupled --output "$output/zero" > "$output/zero-console.log" 2>&1; then zero_ok=true;fi
cat "$output/zero-console.log"
systemctl start robot-web-status.service
[[ $zero_ok == true ]] || { echo 'Nie potwierdzono zera; PTP pozostaje wyłączone.' >&2;exit 1; }
address=$(ip -j -4 address show enp2s0 | python3 -c 'import json,sys;print(next(a["local"] for n in json.load(sys.stdin) for a in n["addr_info"] if a["family"]=="inet"))')
ready=false
for attempt in {1..40};do
 if python3 - "$address" <<'PY'
import json,sys,urllib.request
try:
 with urllib.request.urlopen(f'http://{sys.argv[1]}:8080/api/v1/manual/status',timeout=1) as r:s=json.load(r)
 ready=s['fresh'] and not s['busy'] and not s['fault']
except (OSError,ValueError,KeyError):ready=False
sys.exit(0 if ready else 1)
PY
 then ready=true;break;fi
 sleep 1
done
[[ $ready == true ]]
if ! runuser -u "$app_user" -- python3 "$project_dir/scripts/prepare_ptp_test_profile.py" --address "$address" --output "$output/unlock" --zero-proof "$output/zero/zero.json" > "$output/unlock-console.log" 2>&1;then
 cat "$output/unlock-console.log" >&2
 python3 - "$profile" <<'PY'
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1]);s=yaml.safe_load(p.read_text());s['/**']['ros__parameters']['execution_enabled']=False;p.write_text(yaml.safe_dump(s,sort_keys=False))
PY
 exit 1
fi
cat "$output/unlock-console.log"
if ! systemctl restart robot-web-status.service; then
 python3 - "$profile" <<'PY'
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1]);s=yaml.safe_load(p.read_text());s['/**']['ros__parameters']['execution_enabled']=False;p.write_text(yaml.safe_dump(s,sort_keys=False))
PY
 echo 'Restart nie powiódł się; profil PTP ponownie wyłączony.' >&2
 exit 1
fi
chown -R "$app_user:$(id -gn "$app_user")" "$output"
printf 'Zero zweryfikowane, nowa rewizja przyjęta, profil PTP włączony1 RPM/acc1. Bez uruchamiania ruchu. Wynik: %s\n' "$output"
