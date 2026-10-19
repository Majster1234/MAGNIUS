# Panel diagnostyczny w sieci LAN

Panel tylko odczytuje stan systemu. Backend i monitor działają na komputerze
robota, a inne urządzenia w tej samej podsieci otwierają stronę w przeglądarce.
Nie wymaga modelu robota, CAN ID ani konfiguracji limitów osi.

Strona **Konfiguracja** ma pustą zakładkę ustawień systemowych i tabelę parametrów robota.
Na tym etapie udostępnia lokalny szkic danych; zakres i ograniczenia opisuje
[instrukcja konfiguracji](configuration.md).

## Przygotowanie

Na Ubuntu 22.04 z ROS 2 Humble, z katalogu głównego repozytorium:

```bash
sudo ./scripts/install_web_dependencies.sh
```

Alternatywnie użyj `rosdep install` zgodnie z [setup.md](setup.md).
Zależności backendu pochodzą z Ubuntu/ROS; nie instaluj ich przez `sudo pip`.
Nie ma automatycznej zmiany konfiguracji zapory, adresów sieci ani CAN.
Backend wybiera transport `wsproto`, będący zależnością pakietu
`python3-uvicorn`. Nie wymaga pakietu `python3-websockets` z Jammy, którego
wersja 9 jest niezgodna z używanym API asyncio Pythona 3.10.

Do zbudowania panelu potrzebny jest Node.js 22.12 lub nowszy z linii 22
oraz npm. Node.js nie jest potrzebny podczas uruchamiania zbudowanego panelu.
Instalacja Node.js: [oficjalne pliki i instrukcje](https://nodejs.org/en/download).

```bash
cd web
npm ci
npm run build
npm test
cd ../ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
cd ..
```

Frontend jest budowany przed workspace, ponieważ `robot_web_bridge` instaluje
zasoby z `web/dist`. Po zmianie frontendu ponów build frontendu i `colcon build`.

Launch uruchamia również obliczeniowy `robot_motion_planner` (`planning_enabled`,
`planning_parameters`). Nowe trasy `/api/v1/planning/…` obliczają i zwracają
plany bez wykonania na sprzęcie. Obsługa i kontrakt: [IK i droga TCP](cartesian_planning.md).
Brak `web/dist/index.html` daje przy uruchomieniu backendu czytelny błąd.

## Uruchomienie

```bash
./scripts/start_web_status.sh
```

Skrypt wczytuje Humble i lokalny workspace. Odpowiada poleceniom:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
ros2 launch robot_bringup web_status.launch.py
```

Launch uruchamia monitor, diagnostykę CAN i backend jako osobne procesy, ustawia
`ROS_LOCALHOST_ONLY=1` i wypisuje URL. `Ctrl+C` zamyka oba procesy.
Wyjście backendu zamyka też monitor. Utrata monitora pozostawia panel dostępny,
ale jego dane stają się nieaktualne. Monitor można wtedy uruchomić ponownie
w drugim terminalu z tym samym środowiskiem ROS:

```bash
ROS_LOCALHOST_ONLY=1 ros2 run robot_system_monitor monitor
```

Domyślnie adres pochodzi z `enp2s0` i podsieci `192.168.1.0/24`.
Przy adresie `192.168.1.69` otwórz **http://192.168.1.69:8080**.
Wpisanie `localhost:8080` lub adresu Tailscale nie udostępnia panelu.
Zmiana parametrów, np. na innym komputerze:

```bash
./scripts/start_web_status.sh lan_interface:=enp2s0 lan_subnet:=192.168.1.0/24 port:=8080
```

Brak jednoznacznego adresu w podanej podsieci zatrzymuje serwer; aplikacja
nie przełącza się na nasłuch na wszystkich adresach. DHCP może zmienić URL
przy następnym uruchomieniu. Stały adres można zapewnić rezerwacją w routerze.

## Autostart systemd

Po przygotowaniu zależności, frontendu i workspace zainstaluj autostart:

```bash
./scripts/install_app_autostart.sh --dry-run
sudo ./scripts/install_app_autostart.sh
```

Instalator zapisuje `/etc/systemd/system/robot-web-status.service` i włącza ją
na następny rozruch. Nie uruchamia ani nie restartuje bieżącej aplikacji.
Przy ponownej instalacji zachowuje kopię dotychczasowego pliku `.bak-*`.
Usługa działa jako użytkownik wywołujący `sudo`, z jego katalogiem domowym
i zapisanymi konfiguracjami robota. Z konta root wskaż zwykłe konto opcją
`--user NAZWA`. Nie są potrzebne sesja graficzna ani logowanie użytkownika.

Usługa wywołuje istniejący `scripts/start_web_status.sh`, używając domyślnych
parametrów LAN/CAN opisanych wyżej. Uruchamia się po `network-online.target`
i, jeśli jest włączona, usłudze `robot-can@can0.service`. Sam autostart panelu
nie konfiguruje CAN; przygotuj go według [socketcan.md](socketcan.md).
Gdy backend zakończy działanie, np. zanim DHCP przydzieli adres LAN,
systemd ponawia start po 5 s, także przy kodzie wyjścia 0 z launch.
Polecenie zatrzymania usługi wyłącza te ponowienia do kolejnego startu.
Start panelu wykonuje odczyty; ENABLE, zero i JOG nadal wymagają operatora.

Po instalacji można uruchomić usługę od razu. Najpierw zamknij aplikację
uruchomioną ręcznie przez `Ctrl+C`, aby nie uruchamiać dwóch właścicieli CAN:

```bash
sudo systemctl start robot-web-status.service
systemctl status robot-web-status.service
journalctl -u robot-web-status.service -b -n 100 --no-pager
```

Zatrzymanie, ponowne uruchomienie oraz wyłączenie autostartu:

```bash
sudo systemctl stop robot-web-status.service
sudo systemctl restart robot-web-status.service
sudo systemctl disable --now robot-web-status.service
```

Podczas pracy usługi nie uruchamiaj równolegle `start_web_status.sh`.
Konflikt dwóch instancji może powodować naprzemienne alarmy sterowania,
znikanie dostępnych osi i oczekiwanie na model zgodny z rewizją ROS.
W logu usługi pojawiają się wtedy `address already in use` oraz komunikat
o innym właścicielu katalogu konfiguracji. Zamknij ręczną instancję przez
`Ctrl+C`; usługa ponowi start. Sprawdź, czy jej stan to `active (running)`
i licznik `systemctl show robot-web-status.service -p NRestarts` przestał rosnąć.
Po zmianie kodu zbuduj wymagane składniki i zrestartuj usługę; po przeniesieniu
repozytorium uruchom instalator ponownie, ponieważ zapisuje ścieżkę bezwzględną.
Autostart uruchamia serwer panelu; stronę otwierasz w przeglądarce samodzielnie.
Instalator nie buduje projektu ani nie instaluje zależności.

## Interpretacja

| Element | Co potwierdza |
| --- | --- |
| Backend API | Przeglądarka otrzymuje odpowiedź backendu |
| ROS 2 | Backend odbiera wiadomości od osobnego węzła monitora |
| Komputer | Rzeczywiste CPU, RAM i czas pracy systemu |
| CAN | Podsumowanie interfejsu; osobny ekran pokazuje skan i statusy urządzeń |
| Kontrolery | Odpowiedź `ListControllers`; przy braku usługi: nieuruchomione |
| Napędy | Szczegóły CAN oraz osobny ekran ręcznego sterowania wałami |

Ekran **Magistrala CAN** prowadzi automatyczny skan po starcie i pokazuje
wykryte adresy, ENABLE, stan pracy, RTT oraz szczegóły po rozwinięciu wiersza.
Opis protokołu, Reset CAN i ograniczenia są w [diagnostyce CAN](can_diagnostics.md).

Ekran **Sterowanie ręczne** obsługuje ENABLE, ustawienie bieżącej pozycji zero
i JOG pojedynczych silników. Mapowanie: CAN 1–6 → J1–J6. Moduł nie wymaga
modelu ani przekładni robota; szczegóły i zasady odbioru: [sterowanie ręczne](manual_control.md).
Nowe funkcje są rozwijane dla desktopu. Panel główny nadal odświeża się przy 1 Hz,
a ręczny przy 20 Hz; samo otwarcie ekranu nie załącza napędów.

`ERROR-ACTIVE` to normalny stan CAN, a nie zgłoszenie awarii.
Aktywny CAN ani licznik RX nie potwierdzają gotowości konkretnej osi.
Wartości po utracie danych pozostają widoczne, ale są oznaczone jako nieaktualne.
Automatyczne ponawianie połączenia działa bez odświeżenia strony.

## Dostęp i diagnostyka połączenia

Jeden port TCP 8080 obsługuje stronę, API i WebSocket. Serwer nasłuchuje tylko
na wybranym adresie IPv4, a aplikacja dopuszcza źródła z podanej podsieci.
Żądania z innego Host/Origin i nagłówki podszywające się pod adres klienta
nie omijają ograniczenia. Nie włączaj przekierowania portu w routerze ani
proxy, które przedstawiałoby zdalnych klientów jako urządzenia LAN.

```bash
curl --noproxy '*' http://192.168.1.69:8080/healthz
curl --noproxy '*' http://192.168.1.69:8080/api/v1/status
ss -ltn 'sport = :8080'
```

Jeżeli inne urządzenie nie może otworzyć strony, sprawdź właściwy adres,
trasę, izolację klientów Wi-Fi oraz istniejące reguły zapory. Odczyt zapory
może wymagać `sudo ufw status verbose`. Nie włączaj ani nie resetuj zapory
bez uwzględnienia pozostałych usług komputera.

## Weryfikacja odbiorcza

- Otwórz panel na drugim urządzeniu LAN; porównaj CAN z `ip -details -statistics link show can0`.
- Zatrzymaj monitor: po 3 s dane powinny być nieaktualne. Uruchom go ponownie: dane wracają.
- Zatrzymaj i uruchom backend: otwarta strona powinna wznowić połączenie.
- Otwórz pięć paneli: nadal działa jeden monitor i jeden bridge.
- Sprawdź odrzucenie dostępu przez adres Tailscale i spoza dozwolonej podsieci.
- Porównaj odczytowe ramki CAN z opisem diagnostyki; nowy manager wysyła zapytania
  pozycji i statusów. W trybie diagnostycznym nie mogą pojawić się komendy ruchu.

Testy automatyczne nie zastępują sprawdzenia na drugim fizycznym urządzeniu.

## Potwierdzona weryfikacja — 18.10.2026

- Wcześniej zbudowano sześć pakietów w ROS 2 Humble i produkcyjny panel; typy TypeScript
  sprawdzone. Testy: 38 Python i 3 frontend, bez błędów.
- Chromium: pulpit 1440 px i ekran mobilny 390 px, bez poziomego przepełnienia,
  błędów JavaScript i żądań do zewnętrznych serwerów.
- Pięć równoczesnych paneli: jeden `robot_system_monitor` i jeden `robot_web_bridge`.
- Wstrzymanie monitora oznaczyło dane jako nieaktualne po około 2,8 s;
  wznowienie przywróciło dane. Restart backendu odtworzył połączenie strony.
- Wyjście backendu zamknęło monitor; wspólne Ctrl+C zamknęło oba procesy bez błędów.
- Rzeczywisty `can0`: 500000 bit/s, ERROR-ACTIVE, RX/TX 0 przed i po testach.
- Rzeczywiste żądanie z adresem źródłowym Tailscale: HTTP 403; na adresie
  Tailscale nie było nasłuchu TCP 8080. Pozostałe podsieci sprawdzono w testach ASGI.
- Do testów brakujące pakiety Ubuntu rozpakowano w `/tmp`; nie zainstalowano
  ich systemowo, ponieważ `sudo` wymaga hasła. Zwykłe uruchomienie wymaga
  wykonania skryptu instalacji zależności. Node.js był narzędziem tymczasowym.
- Nie sprawdzono drugiego fizycznego urządzenia LAN, zapory wymagającej hasła,
  zdalnego GitHub Actions ani działania napędów. Procesy testowe zakończono.
