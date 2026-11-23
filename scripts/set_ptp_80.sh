#!/usr/bin/env bash
# Zatwierdzony limit 80 stopni/s; bez zerowania, ENABLE=1 ani ruchu.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:---dry-run}
[[ $# -le 1 && ( $mode == --dry-run || $mode == --apply ) ]] || { echo 'Użycie: sudo ./scripts/set_ptp_80.sh --apply'; exit 2; }
address=$(ip -j -4 address show enp2s0 | python3 -c 'import json,sys;print(next(a["local"] for n in json.load(sys.stdin) for a in n["addr_info"] if a["family"]=="inet"))')
echo 'Plan: wyłączenie dostępnych napędów w postoju, limit przegubów 80 stopni/s, holding1/acc1, zgodna rewizja i restart panelu. Bez ruchu i zerowania.'
[[ $mode == --apply ]] || exit 0
(( EUID == 0 )) || { echo 'Obsługa usługi wymaga sudo.' >&2; exit 1; }
app_user=$(systemctl show robot-web-status.service --property=User --value)
[[ -n $app_user && $app_user != root ]]
output="$project_dir/log/can-1m-20261024/ptp80-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$output"
profile="$project_dir/config/local/execution.production.yaml"
cp -a "$profile" "$output/execution-before.yaml"
chown -R "$app_user:$(id -gn "$app_user")" "$output"
# Zmiana konfiguracji wymaga wyłączonych osi. Nie przerywamy aktywnego programu/JOG.
runuser -u "$app_user" -- python3 - "$address" "$output" <<'PY'
import json,sys,time,urllib.request
from pathlib import Path
base=f'http://{sys.argv[1]}:8080/api/v1/'
def call(path,body=None):
 req=urllib.request.Request(base+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'},method='GET' if body is None else 'POST')
 with urllib.request.urlopen(req,timeout=6) as r:return json.load(r)
motion=call('motion/status');state=call('manual/status')
assert not motion['busy'] and motion.get('preparation',{}).get('state')!='planning', 'Trwa program lub przygotowanie PTP'
assert state['fresh'] and not state['busy'] and not state['fault'], 'Wymagany świeży postój bez fault'
Path(sys.argv[2],'manual-before.json').write_text(json.dumps(state,indent=2,ensure_ascii=False))
for axis in state['axes']:
 if not axis['available']:continue
 assert axis['enable_known'] and axis['calibration_valid'], 'Niepotwierdzony stan lub kalibracja'
 if axis['enabled']:
  result=call(f'manual/axes/{axis["axis"]}/enable',{'enabled':False});assert result['accepted']
  deadline=time.monotonic()+6
  while time.monotonic()<deadline:
   time.sleep(.1);current=call('manual/status');a=current['axes'][axis['axis']-1]
   if current['fresh'] and not current['busy'] and not current['fault'] and a['enable_known'] and not a['enabled']:break
  else:raise RuntimeError('Brak potwierdzenia wyłączenia napędu')
print('Potwierdzono wyłączenie napędów; bez zerowania i bez zadawania pozycji.')
PY
disable_profile() {
 python3 - "$profile" <<'PY'
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1]);s=yaml.safe_load(p.read_text());s['/**']['ros__parameters']['execution_enabled']=False;p.write_text(yaml.safe_dump(s,sort_keys=False))
PY
}
if ! runuser -u "$app_user" -- python3 "$project_dir/scripts/prepare_ptp_test_profile.py" --address "$address" --output "$output/configuration" --joint-velocity-deg-s 80 > "$output/configure.log" 2>&1; then
 cat "$output/configure.log" >&2
 disable_profile
 systemctl restart robot-web-status.service
 echo 'Nie potwierdzono zmiany; wykonawca PTP wyłączony.' >&2
 exit 1
fi
cat "$output/configure.log"
if ! systemctl restart robot-web-status.service; then disable_profile; exit 1; fi
chown -R "$app_user:$(id -gn "$app_user")" "$output"
printf 'Przyjęto limit PTP 80 stopni/s; acc1. Napędy wyłączone; bez automatycznego ruchu. Wynik: %s\n' "$output"
