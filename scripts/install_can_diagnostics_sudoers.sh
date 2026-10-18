#!/usr/bin/env bash
# Nadaje wyłącznie prawo restartu usługi can0 używanej przez diagnostykę.
set -euo pipefail

if [[ $# -gt 1 ]]; then
    echo "Użycie: sudo $0 [konto_aplikacji]" >&2
    exit 2
fi
if [[ $EUID -ne 0 ]]; then
    echo "Uruchom: sudo $0 [konto_aplikacji]" >&2
    exit 1
fi
app_user=${1:-${SUDO_USER:-}}
if [[ -z $app_user ]] || ! getent passwd "$app_user" >/dev/null; then
    echo "Podaj istniejące konto uruchamiające panel." >&2
    exit 2
fi
rule="${app_user} ALL=(root) NOPASSWD: /usr/bin/systemctl restart robot-can@can0.service"
temp_file=$(mktemp)
trap 'rm -f "$temp_file"' EXIT
printf '%s\n' "$rule" >"$temp_file"
visudo -cf "$temp_file"
install -o root -g root -m 0440 "$temp_file" /etc/sudoers.d/robot-can-diagnostics
echo "Zainstalowano ograniczone uprawnienie resetu dla: $app_user"
