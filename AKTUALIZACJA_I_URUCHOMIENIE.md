# Aktualizacja i uruchomienie aplikacji robota

Instrukcja dla istniejącej instalacji w `/home/mk/Desktop/Projekty`, Ubuntu 22.04 i ROS 2 Humble. Stan sprawdzony 29.10.2026. Polecenia wykonuj w terminalu Bash.

Źródłem aktualizacji jest **bieżący kod w tym folderze**, również zmiany jeszcze niezapisane w Git. Sam restart usługi nie buduje kodu.

| Co zostało zmienione | Co wykonać |
| --- | --- |
| Tylko panel `web/`, np. restart pomarańczowego modelu | Punkt 2: build panelu i Ctrl+F5. |
| Backend, pakiety ROS, pliki launch albo obie części aplikacji | Punkt 3: pełna aktualizacja do nowej nakładki. |
| Kod jest już zbudowany, potrzebny tylko restart | Punkt 4. |
| Potrzebne logi lub sprawdzenie uruchomionej wersji | Punkt 5. |
| Trzeba wrócić do wersji sprzed pełnej aktualizacji | Punkt 6. |

Przed zatrzymaniem lub restartem usługi zakończ program i JOG oraz potwierdź postój robota. Instrukcja nie zawiera poleceń ENABLE, ruchu, kalibracji ani resetu CAN.

## 1. Przygotowanie terminala i Node.js

```bash
cd /home/mk/Desktop/Projekty
```

Przechodzi do folderu głównego projektu. Polecenia w kolejnych punktach zakładają tę lokalizację.

Do budowania panelu wymagany jest Node.js co najmniej 22.12.0. Sprawdzenie dostępnych narzędzi:

```bash
node --version
npm --version
```

Na tym komputerze ostatnie buildy korzystały z Node.js 22.14.0 w katalogu tymczasowym. Jeśli `node` nie jest dostępny, poniższy blok zachowuje tę istniejącą instalację w stałym katalogu użytkownika i udostępnia ją w bieżącym terminalu:

```bash
if [[ ! -x "$HOME/.local/opt/node-v22.14.0-linux-x64/bin/node" ]]; then
    mkdir -p "$HOME/.local/opt"
    cp -a /tmp/stop-recovery-node.a9mozG/node-v22.14.0-linux-x64 "$HOME/.local/opt/"
fi
export PATH="$HOME/.local/opt/node-v22.14.0-linux-x64/bin:$PATH"
node --version
npm --version
```

Kopiowanie wykonuje się tylko przy braku stałej kopii. Jeśli kopii nie ma, a katalog `/tmp/stop-recovery-node.a9mozG` został już usunięty, trzeba najpierw dostarczyć Node.js w wymaganej wersji. Nie kontynuuj budowania po błędzie tego kroku. W kolejnych terminalach wystarczy powtórzyć `export PATH=...` z powyższego bloku. Node.js jest potrzebny przy budowaniu, nie podczas pracy zbudowanej aplikacji.

## 2. Aktualizacja samego panelu — ostatnia poprawka pomarańczowego modelu

Po przygotowaniu Node.js:

```bash
cd /home/mk/Desktop/Projekty/web
npm run build
```

`npm run build` sprawdza TypeScript i generuje panel produkcyjny w `web/dist`. Przy błędzie budowania przerwij aktualizację i popraw wskazany problem.

```bash
npm test
node test/motion.browser.cjs
node test/cartesian.browser.cjs
cd /home/mk/Desktop/Projekty
```

`npm test` uruchamia testy jednostkowe panelu. Dwa kolejne polecenia sprawdzają w Chromium wykonanie PTP/LIN, restart pętli i podgląd trajektorii. Korzystają z atrap API, bez ruchu robota. Stan ostatniej weryfikacji: build i oba testy przeglądarkowe OK; 144/145 testów jednostkowych OK. Zastany błąd dotyczy `controlMessages.test.ts`, przypadku „nie przypisuje nieznanemu błędowi niepotwierdzonej przyczyny”. Inne błędy wymagają sprawdzenia.

Nie wykonuj rutynowo `npm ci`. Jest potrzebne przy pierwszym przygotowaniu `node_modules` albo po zmianie zależności w `package.json` / `package-lock.json`:

```bash
cd /home/mk/Desktop/Projekty/web
npm ci
cd /home/mk/Desktop/Projekty
```

Aktualna usługa korzysta przez symlink z `web/dist`. Możesz to sprawdzić:

```bash
readlink -f install/stop-recovery-ros/robot_web_bridge/share/robot_web_bridge/web/index.html
```

W instalacji sprawdzonej 29.10.2026 wynik to `/home/mk/Desktop/Projekty/web/dist/index.html`. W takiej konfiguracji **restart usługi nie jest potrzebny**. Po udanym buildzie otwórz panel i naciśnij **Ctrl+F5**. Po przełączeniu na inną nakładkę sprawdzaj analogiczną ścieżkę w tej nakładce.

## 3. Pełna aktualizacja aplikacji do nowej nakładki ROS

Ten wariant buduje wszystkie 11 pakietów aplikacji z bieżących źródeł. Przy zwykłej poprawce jednego modułu buduj tylko zmienione pakiety i zależnych konsumentów; pełna procedura służy aktualizacji całej aplikacji.

Użyj nowego terminala, przygotuj Node.js według punktu 1 i **nie wczytuj wcześniej starych nakładek projektu**. Poniższe bloki wykonuj w jednym terminalu, po kolei. Zatrzymaj się po każdym błędzie. Nazwa nowej nakładki zawiera czas, aby nie nadpisywać działających bibliotek.

### 3.1. Ścieżki i lista pakietów

```bash
cd /home/mk/Desktop/Projekty
export ROBOT_PROJECT_DIR="$PWD"
export ROBOT_RELEASE_ID="release-$(date +%Y%m%d-%H%M%S)"
export ROBOT_BUILD_BASE="$ROBOT_PROJECT_DIR/build/$ROBOT_RELEASE_ID"
export ROBOT_INSTALL_BASE="$ROBOT_PROJECT_DIR/install/$ROBOT_RELEASE_ID"
export ROBOT_BACKUP_DIR="$ROBOT_PROJECT_DIR/install/deployment-backups/$ROBOT_RELEASE_ID"
ROBOT_PACKAGES=(
    robot_control_interfaces
    mks_servo_can
    robot_kinematics
    robot_can_diagnostics
    robot_configuration
    robot_hardware
    robot_motion_planner
    robot_description
    robot_bringup
    robot_system_monitor
    robot_web_bridge
)
```

Zmienne określają katalogi buildu, instalacji i kopii poprzedniego wdrożenia. Tablica zawiera pakiety aplikacji; `colcon` ustali kolejność ich budowania według zależności.

### 3.2. Kopia poprzedniej wersji

```bash
mkdir -p "$ROBOT_BACKUP_DIR"
systemctl cat robot-web-status.service > "$ROBOT_BACKUP_DIR/service-before.txt"
systemctl show robot-web-status.service --property=Environment --property=ExecStart \
    > "$ROBOT_BACKUP_DIR/runtime-before.txt"
cp -a /etc/systemd/system/robot-web-status.service.d "$ROBOT_BACKUP_DIR/service.d"
cp -a "$ROBOT_PROJECT_DIR/config/local/execution.production.yaml" "$ROBOT_BACKUP_DIR/"
cp -a "$ROBOT_PROJECT_DIR/web/dist" "$ROBOT_BACKUP_DIR/web-dist"
if [[ -d "$HOME/.local/state/robot/configuration" ]]; then
    cp -a "$HOME/.local/state/robot/configuration" "$ROBOT_BACKUP_DIR/configuration"
fi
printf 'Kopia poprzedniej wersji: %s\n' "$ROBOT_BACKUP_DIR"
```

Zachowuje ustawienia usługi, profil produkcyjny, poprzedni panel i zapisane konfiguracje robota. Zapisz wyświetloną ścieżkę — przyda się po zamknięciu terminala. Poprzednia nakładka ROS pozostaje na dysku.

### 3.3. Zatrzymanie aplikacji i build panelu

**Dopiero po potwierdzeniu postoju robota:**

```bash
sudo systemctl stop robot-web-status.service
systemctl is-active robot-web-status.service
```

Pierwsze polecenie zatrzymuje procesy aplikacji. Drugie powinno pokazać `inactive` i wtedy zwraca niezerowy kod wyjścia — to oczekiwany wynik, nie błąd buildu. Nie kontynuuj, jeśli usługa nadal działa.

```bash
cd "$ROBOT_PROJECT_DIR/web"
npm run build
cd "$ROBOT_PROJECT_DIR"
```

Buduje panel przed pakietem `robot_web_bridge`, który instaluje jego zasoby. Usługa jest zatrzymana, ponieważ panel z `web/dist` jest współdzielony także z poprzednią instalacją.

### 3.4. Build pakietów ROS w Release

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base "$ROBOT_PROJECT_DIR/log/$ROBOT_RELEASE_ID-build" build \
    --base-paths "$ROBOT_PROJECT_DIR/ros2_ws/src" \
    --build-base "$ROBOT_BUILD_BASE" \
    --install-base "$ROBOT_INSTALL_BASE" \
    --symlink-install \
    --parallel-workers 2 \
    --packages-select "${ROBOT_PACKAGES[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`source` udostępnia ROS 2 Humble. `colcon build` buduje wskazane pakiety, `--symlink-install` wiąże odpowiednie pliki z kodem/zasobami, a `Release` włącza produkcyjny wariant C++. Dwa ograniczenia równoległości ograniczają obciążenie komputera. Wynik trafia do nowej nakładki, nie do starej `install/stop-recovery-ros`.

### 3.5. Sprawdzenie nowego buildu

```bash
source "$ROBOT_INSTALL_BASE/setup.bash"
ros2 pkg prefix robot_can_diagnostics
ros2 pkg prefix robot_motion_planner
ros2 pkg prefix robot_web_bridge
readlink -f "$ROBOT_INSTALL_BASE/robot_web_bridge/share/robot_web_bridge/web/index.html"
```

Prefiksy powinny wskazywać nową nakładkę `install/release-...`. Ostatnie polecenie pokazuje rzeczywiste położenie panelu; przy symlinkach powinien to być plik w `web/dist`.

Testy pakietów zbudowanych w tej procedurze:

```bash
colcon --log-base "$ROBOT_PROJECT_DIR/log/$ROBOT_RELEASE_ID-test" test \
    --build-base "$ROBOT_BUILD_BASE" \
    --packages-select "${ROBOT_PACKAGES[@]}" \
    --return-code-on-test-failure
colcon test-result --test-result-base "$ROBOT_BUILD_BASE" --verbose
```

Pierwsze polecenie uruchamia testy wybranych pakietów, drugie pokazuje ich wyniki. Nie uruchamiaj skryptów odbioru sprzętowego jako części tej procedury. Testy panelu z punktu 2 wykonaj po zmianach w `web/`.

### 3.6. Przełączenie usługi i uruchomienie

```bash
test -r "$ROBOT_INSTALL_BASE/setup.bash"
sudo mkdir -p /etc/systemd/system/robot-web-status.service.d
sudo tee /etc/systemd/system/robot-web-status.service.d/zz-application-release.conf > /dev/null <<EOF
[Service]
Environment="ROBOT_OVERLAY_SETUP=$ROBOT_INSTALL_BASE/setup.bash"
EOF
sudo systemctl daemon-reload
systemctl show robot-web-status.service --property=Environment --property=ExecStart
```

`test` sprawdza obecność nowej nakładki — przy błędzie nie wykonuj dalszych poleceń. `tee` zapisuje dodatkowy plik ustawień systemd, który wybiera nową nakładkę. `daemon-reload` wczytuje zmianę definicji usługi, ale sam nie uruchamia aplikacji.

Sprawdź, czy `Environment` wskazuje nową ścieżkę. Zachowany `ExecStart` powinien nadal uruchamiać `scripts/start_web_status.sh` z `execution_parameters:=/home/mk/Desktop/Projekty/config/local/execution.production.yaml` oraz `can_bitrate:=1000000`, tak jak w aktualnej instalacji. Procedura nie zmienia tego profilu, geometrii, przełożeń ani ustawień CAN.

```bash
sudo systemctl start robot-web-status.service
systemctl status robot-web-status.service --no-pager
journalctl -u robot-web-status.service -n 100 --no-pager
```

`start` uruchamia usługę z nową nakładką. `status` pokazuje stan procesów, a `journalctl` ostatnie logi. Po starcie przejdź do punktu 5 i odśwież panel przez Ctrl+F5. Stan `active` potwierdza działanie usługi, nie gotowość robota do ruchu.

## 4. Sam restart, zatrzymanie i autostart

Przy gotowym buildzie, po potwierdzonym postoju robota:

```bash
sudo systemctl restart robot-web-status.service
```

Zatrzymuje i ponownie uruchamia aplikację. Stosuj po zmianie backendu lub parametrów startowych; dla samego panelu podłączonego do `web/dist` wystarczy build i Ctrl+F5.

```bash
sudo systemctl stop robot-web-status.service
sudo systemctl start robot-web-status.service
```

To dwa osobne działania: zatrzymanie oraz późniejsze uruchomienie. Nie uruchamiaj równocześnie drugiej kopii aplikacji w terminalu.

```bash
systemctl is-enabled robot-web-status.service
sudo systemctl enable robot-web-status.service
```

Pierwsze polecenie sprawdza autostart, drugie go włącza na kolejne uruchomienia komputera. `enable` bez `--now` nie restartuje działającej aplikacji. Istniejącej usługi nie trzeba ponownie instalować skryptem instalacyjnym.

## 5. Kontrola działania i logi

```bash
systemctl status robot-web-status.service --no-pager
systemctl show robot-web-status.service --property=Environment --property=MainPID --property=NRestarts
systemctl cat robot-web-status.service
journalctl -u robot-web-status.service -b -n 100 --no-pager
```

Kolejno: stan usługi, wybrana nakładka/PID/liczba restartów, pełna definicja wraz z plikami dodatkowych ustawień oraz ostatnie logi od bieżącego uruchomienia komputera.

```bash
journalctl -u robot-web-status.service -f
```

Pokazuje logi na żywo. Ctrl+C kończy tylko ich podgląd.

```bash
ss -ltn 'sport = :8080'
ip -details link show can0
```

Pierwsze polecenie pokazuje adres nasłuchu panelu. Drugie odczytuje stan i bitrate SocketCAN; niczego nie przełącza. Aktualnie panel jest dostępny pod **http://192.168.1.69:8080**. Jeśli adres się zmieni, użyj adresu z `ss` i logów.

```bash
curl --fail --show-error http://192.168.1.69:8080/ -o /dev/null
```

Sprawdza odpowiedź strony przez HTTP, bez poleceń do napędów. Po otwarciu strony sprawdź również komunikaty oraz świeżość danych w panelu. Sam poprawny HTTP nie potwierdza komunikacji z robotem.

## 6. Powrót do poprzedniej wersji po punkcie 3

Po potwierdzeniu postoju robota użyj `ROBOT_BACKUP_DIR` z punktu 3.2. Jeśli otworzyłeś nowy terminal, najpierw ustaw tę zmienną na **dokładną zapisaną ścieżkę kopii**. Nie wybieraj katalogu na podstawie domysłu.

```bash
test -d "$ROBOT_BACKUP_DIR/service.d"
test -f "$ROBOT_BACKUP_DIR/web-dist/index.html"
```

Oba sprawdzenia muszą się powieść przed kontynuacją.

```bash
sudo systemctl stop robot-web-status.service
if [[ -f "$ROBOT_BACKUP_DIR/service.d/zz-application-release.conf" ]]; then
    sudo cp -a "$ROBOT_BACKUP_DIR/service.d/zz-application-release.conf" \
        /etc/systemd/system/robot-web-status.service.d/zz-application-release.conf
else
    sudo rm -f /etc/systemd/system/robot-web-status.service.d/zz-application-release.conf
fi
cp -a "$ROBOT_BACKUP_DIR/web-dist/." /home/mk/Desktop/Projekty/web/dist/
sudo systemctl daemon-reload
sudo systemctl start robot-web-status.service
systemctl status robot-web-status.service --no-pager
```

Przywraca poprzednie wskazanie nakładki albo usuwa dodane wskazanie, jeśli wcześniej go nie było. Następnie odtwarza poprzedni panel i uruchamia usługę. Pozostałe pliki ustawień systemd pozostają bez zmian. Nie przywraca automatycznie konfiguracji robota ani kalibracji. Nowy katalog buildu i instalacji pozostaje na dysku do analizy. Po cofnięciu wersji użyj Ctrl+F5.

## 7. Uruchomienie ręczne zamiast usługi — opcjonalnie

Po potwierdzeniu postoju zatrzymaj usługę. Ustawienie nakładki w systemd nie przenosi się automatycznie do terminala:

```bash
sudo systemctl stop robot-web-status.service
cd /home/mk/Desktop/Projekty
export ROBOT_OVERLAY_SETUP=/home/mk/Desktop/Projekty/install/stop-recovery-ros/setup.bash
./scripts/start_web_status.sh \
    execution_parameters:=/home/mk/Desktop/Projekty/config/local/execution.production.yaml \
    can_bitrate:=1000000
```

Podana ścieżka to nakładka używana przed pełną aktualizacją opisaną w tym pliku. Po wykonaniu punktu 3 zastąp ją rzeczywistą ścieżką nowej nakładki, widoczną w `systemctl show robot-web-status.service --property=Environment`. Skrypt wczytuje Humble, bazowy workspace i wybraną nakładkę. Wymaga istniejącego `ros2_ws/install/local_setup.bash`.

Ctrl+C w tym terminalu zamyka aplikację. Aby wrócić do usługi, zakończ ręczną kopię, a następnie wykonaj:

```bash
sudo systemctl start robot-web-status.service
```

`./start_ros2.sh` służy do otwierania terminali roboczych; samo jego wywołanie nie buduje ani nie wdraża najnowszej aplikacji.
