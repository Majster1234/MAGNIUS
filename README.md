# Robot 6-DOF — ROS 2 Humble / CAN

Diagnostyka zaniku danych podczas JOG: [nagrywanie i analiza CAN/ROS/API](docs/can_recording.md).

Projekt sześcioosiowego robota dla Ubuntu 22.04 i ROS 2 Humble.
Docelowa komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → napędy
MKS SERVO42D w wersji CAN.

Repozytorium zawiera pakiety ROS 2, panel React/TypeScript, diagnostykę CAN
oraz ręczne sterowanie pojedynczymi silnikami. Monitor i backend udostępniają
rzeczywiste statusy w LAN. Po starcie wykonywane są wyłącznie odczyty;
ENABLE, zerowanie i JOG wymagają jawnego działania operatora.
Istnieją manager C++, adapter `ros2_control` i arbitraż trajektorii/jog, z aktywacją
zablokowaną do czasu konfiguracji i odbioru sprzętu. Brakuje modelu i parametrów osi.
Hierarchia i uruchamianie: [strategia CAN](docs/can_strategy.md).

## Panel diagnostyczny

Po przygotowaniu zależności i zbudowaniu panelu oraz workspace według
[instrukcji panelu](docs/web_status.md), uruchom z katalogu repozytorium:

```bash
./scripts/start_web_status.sh
```

Jeżeli panel działa już przez autostart, przeładuj istniejącą usługę:
`sudo systemctl restart robot-web-status.service`.
Jej stan sprawdzisz przez `systemctl status robot-web-status.service`.
Równoczesne uruchomienie skryptu i usługi powoduje konflikt publikacji ROS oraz portu API.

Otwórz URL wypisany w terminalu, obecnie **http://192.168.1.69:8080**,
na komputerze robota lub innym urządzeniu w `192.168.1.0/24`.
Panel pokazuje API, łączność ROS, CPU/RAM, CAN, kontrolery i stan integracji
napędów. Strona [Konfiguracja](docs/configuration.md) oddziela ustawienia systemu
od danych robota. Udostępnia szkic, wspólne snapshoty JSON, import/eksport,
lokalny podgląd geometrii 3D oraz osobne zastosowanie parametrów do ROS.
Nad sterowaniem jest [podgląd URDF 3D](docs/robot_preview.md) na połowę wysokości okna,
z enkoderami, przełożeniami skonfigurowanych osi i domkiem dopasowania kamery. Pod tytułem strony
znajduje się przewijana konsola komunikatów, ostrzeżeń i awarii (do 100 wpisów).
Ekran [Sterowanie ręczne](docs/manual_control.md) obsługuje ENABLE,
zero i JOG wałów J1–J6 (CAN 1–6), bez modelu robota. Panel jest dostępny
bez logowania w LAN. `Ctrl+C` zamyka sesję.

Przycisk **Planowanie** na wizualizacji 3D przełącza JOG na półprzezroczystego
bliźniaka, rozpoczynającego od pozycji enkoderów. Domyślnie dostępne są osie z aktualnymi
pomiarami; ENABLE może być wyłączony. Plan jest lokalny, bez wykonania na robocie.
Obsługa: [planowanie lokalne](docs/robot_preview.md#planowanie-lokalne).

Wybór **PTP/LIN** automatycznie oblicza ruch od zapisanego startu do celu ustawionego
JOG i odtwarza go w pętli na trzecim, pomarańczowym modelu. Nowy JOG przelicza ruch
po puszczeniu strzałki. W **Konfiguracja → Ustawienia systemowe** wspólny suwak
pozwala symulować wszystkie osie bez napędów. C++ używa KDL/Eigen i TOTG.
Fizyczny PTP/LIN wymaga osobnego przygotowania od pomiarów i jawnego zatwierdzenia;
sam podgląd nie uruchamia robota. [Instrukcja PTP/LIN i API](docs/cartesian_planning.md).
Przy osobliwości nadgarstka trajektoria płynnie ogranicza J4/J6 do maks. 5°/s;
po oddaleniu dopuszczalna prędkość wraca do wartości zadanej.
W LIN ustawiasz prędkość przesuwu TCP w mm/s, a obrót TCP ma osobny limit w °/s.
W PTP pozostaje prędkość złączy w °/s.

## Autostart po włączeniu komputera

Po przygotowaniu aplikacji według [instrukcji panelu](docs/web_status.md)
zainstaluj usługę systemd:

```bash
./scripts/install_app_autostart.sh --dry-run
sudo ./scripts/install_app_autostart.sh
```

Panel uruchomi się przy następnym rozruchu komputera, bez logowania do pulpitu.
Działa na koncie użytkownika wywołującego `sudo`; instalacja nie przerywa
obecnej sesji aplikacji. Otwórz panel w przeglądarce pod jego adresem LAN.
Stan, logi i wyłączenie autostartu opisuje
[instrukcja autostartu](docs/web_status.md#autostart-systemd).

## Układ projektu

```text
.
├── docs/                         # Architektura, środowisko i protokół CAN
├── .github/                      # CI i szablony zgłoszeń / pull requestów
├── CONTRIBUTING.md               # Zasady pracy nad projektem
├── start_ros2.sh                  # Otwieranie terminali projektu
├── scripts/                      # Inicjalizacja środowiska terminali
├── web/                          # Panel React/TypeScript, budowany przez Vite
├── system/can/                   # Usługa systemd i wzór konfiguracji SocketCAN
├── system/application/           # Szablon usługi autostartu panelu
├── hardware/
│   ├── wiring/                   # Schematy połączeń
│   ├── datasheets/               # Dokumentacja posiadanych urządzeń
│   └── cad/                      # Modele mechaniczne i pliki źródłowe CAD
└── ros2_ws/src/
    ├── robot_description/        # Przyszły URDF/Xacro, meshes, RViz
    ├── mks_servo_can/            # SocketCAN, protokół, manager i moduł ręczny
    ├── robot_control_interfaces/ # Typowane usługi i stan sterowania ręcznego
    ├── robot_can_diagnostics/    # Skaner MKS i diagnostyka CAN 5 Hz
    ├── robot_hardware/           # Plugin sześciu osi, wspólny manager CAN
    ├── robot_bringup/            # Launch panelu i szablony konfiguracji
    ├── robot_system_monitor/     # Pomiary systemowe i diagnostyka ROS
    └── robot_web_bridge/         # API HTTP/WebSocket i serwowanie panelu
```

Każdy pakiet zawiera `package.xml`, konfigurację CMake lub setuptools i README.
Puste katalogi zachowują pliki `.gitkeep`. `build/`, `install/` i `log/`
powstaną dopiero podczas pracy z `colcon` i są ignorowane przez Git.

## Początek pracy

1. Przygotuj ROS 2 Humble i narzędzia według [instrukcji środowiska](docs/setup.md).
2. Zbuduj frontend i workspace poleceniami z tej instrukcji.
3. Uzupełnij dane sprzętowe i mechaniczne opisane w
   [architekturze](docs/architecture.md) oraz [opisie CAN](docs/can_protocol.md).
4. Uzupełnij model i konfigurację osi, następnie przeprowadź odbiór ruchu na stanowisku.

Szablony w `ros2_ws/src/robot_bringup/config/` mają rozszerzenie `.example.yaml`.
Wartość `null` oznacza brak ustalenia, a nie zero ani wartość domyślną.
Nie są to gotowe konfiguracje do załadowania przez ROS 2. Opis pól znajduje się
w [README pakietu robot_bringup](ros2_ws/src/robot_bringup/README.md).

## Terminale do pracy z ROS 2

Po instalacji ROS 2 Humble uruchom w sesji graficznej GNOME, z katalogu głównego repozytorium:

```bash
./start_ros2.sh
```

Skrypt otworzy trzy osobne okna: **Aplikacja**, **Diagnostyka** i **Workspace**.
W każdym wczyta ROS 2 Humble, następnie `ros2_ws/install/local_setup.bash`,
jeżeli workspace jest zbudowany, i przejdzie do katalogu `ros2_ws`.
Terminale zachowują ustawienia z `~/.bashrc`. Terminal diagnostyczny wyświetla
podpowiedzi poleceń, a terminal workspace instrukcję budowania.

Podgląd poleceń bez otwierania okien:

```bash
./start_ros2.sh --dry-run
```

Można przekazać polecenie panelu po `--`:

```bash
./start_ros2.sh -- ros2 launch robot_bringup web_status.launch.py
```

Polecenie uruchamia się tylko w terminalu aplikacji. Po jego zakończeniu lub
błędzie okno pozostaje otwarte. `Ctrl+C` przerywa działający proces, a `exit`
zamyka wybraną powłokę. Argumenty przekazywane są bez `eval`; operatory powłoki,
takie jak `|` lub `&&`, nie są interpretowane jako część przekazanego polecenia.

Przyszła aplikacja może wywołać ten sam plik jako proces, np. w Pythonie:

```python
from pathlib import Path
import subprocess

# project_dir: katalog projektu ustalony przez przyszłą aplikację.
subprocess.Popen([str(Path(project_dir) / "start_ros2.sh")])
```

Wywołuj skrypt raz przy starcie sesji pracy; każde kolejne wywołanie otwiera
następny zestaw okien. Skrypt wymaga GNOME Terminal i dostępu do sesji graficznej.
Zakończenie skryptu oznacza przekazanie żądań otwarcia okien, a nie potwierdzenie
gotowości węzłów ROS. Logi polecenia aplikacji są widoczne w jego oknie;
zamknięcie przyszłej aplikacji nie zamyka automatycznie tych terminali.

## Magistrala CAN w systemie

Podłączony U2C v2.1 jest wykrywany przez sterownik `gs_usb` jako `can0`.
Potwierdzona prędkość napędów wynosi **500000 bit/s**. Skrypt instalacyjny dodaje
narzędzia `can-utils` i usługę systemd konfigurującą link wraz z urządzeniem:

```bash
./scripts/install_socketcan.sh --bitrate 500000 --dry-run
sudo ./scripts/install_socketcan.sh --bitrate 500000
```

Szczegóły konfiguracji, diagnostyki i wyłączenia usługi są w
[instrukcji integracji SocketCAN](docs/socketcan.md).

## Repozytorium GitHub

Folder jest przygotowany jako lokalne repozytorium z gałęzią `main`.
Dodaj go w GitHub Desktop przez **File → Add Local Repository**, a następnie
użyj **Publish repository**, wybierając nazwę i widoczność repozytorium.
Pełna instrukcja, również dla terminala, znajduje się w
[opisie publikacji na GitHubie](docs/github.md).

GitHub Actions sprawdza składnię plików oraz buduje pakiety dla Humble.
Zasady pracy i lokalne polecenia weryfikacji opisuje [CONTRIBUTING.md](CONTRIBUTING.md).

## Metadane projektu

W manifestach wersja początkowa to `0.0.0`. Nazwa i adres opiekuna
(`Project maintainer (TODO)`, `maintainer@example.com`) są jawnymi placeholderami.
Pole licencji ma wartość `TODO`: licencja nie została jeszcze wybrana.
Przed wydaniem pakietów ROS należy zastąpić te metadane rzeczywistymi danymi.
Wybór licencji został odłożony; na tym etapie nie dodano pliku `LICENSE`.

Instalacja ROS 2 i aktualizowanie firmware nie są wykonywane automatycznie.
Konfiguracja systemowa CAN wymaga jawnego uruchomienia skryptu instalacyjnego z `sudo`.
