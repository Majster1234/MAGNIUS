# Robot 6-DOF — ROS 2 Humble / CAN

Szkielet projektu sześcioosiowego robota dla Ubuntu 22.04 i ROS 2 Humble.
Docelowa komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → napędy
MKS SERVO42D w wersji CAN.

Repozytorium zawiera cztery bazowe pakiety `ament_cmake`, dokumentację i szablony
konfiguracji. Nie zawiera jeszcze modelu robota, węzłów, bibliotek sterujących,
pluginu sprzętowego ani plików uruchomieniowych. Budowanie instaluje szkielet
pakietów i zasoby; nie uruchamia komunikacji z napędami.

## Układ projektu

```text
.
├── docs/                         # Architektura, środowisko i protokół CAN
├── .github/                      # CI i szablony zgłoszeń / pull requestów
├── CONTRIBUTING.md               # Zasady pracy nad projektem
├── start_ros2.sh                  # Otwieranie terminali projektu
├── scripts/                      # Inicjalizacja środowiska terminali
├── hardware/
│   ├── wiring/                   # Schematy połączeń
│   ├── datasheets/               # Dokumentacja posiadanych urządzeń
│   └── cad/                      # Modele mechaniczne i pliki źródłowe CAD
└── ros2_ws/src/
    ├── robot_description/        # Przyszły URDF/Xacro, meshes, RViz
    ├── mks_servo_can/            # Przyszła biblioteka C++: SocketCAN i protokół MKS
    ├── robot_hardware/           # Przyszły adapter sześciu osi do ros2_control
    └── robot_bringup/            # Miejsce na launch i szablony konfiguracji
```

Każdy pakiet zawiera `package.xml`, `CMakeLists.txt` i README.
Puste katalogi zachowują pliki `.gitkeep`. `build/`, `install/` i `log/`
powstaną dopiero podczas pracy z `colcon` i są ignorowane przez Git.

## Początek pracy

1. Przygotuj ROS 2 Humble i narzędzia według [instrukcji środowiska](docs/setup.md).
2. Zbuduj workspace poleceniami z tej instrukcji.
3. Uzupełnij dane sprzętowe i mechaniczne opisane w
   [architekturze](docs/architecture.md) oraz [opisie CAN](docs/can_protocol.md).
4. Na tej podstawie zaimplementuj model, bibliotekę CAN i adapter sprzętowy.

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

Docelowo można przekazać polecenie aplikacji po `--`, np. po dodaniu odpowiedniego
pakietu i pliku launch:

```bash
./start_ros2.sh -- ros2 launch NAZWA_PAKIETU PLIK.launch.py
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
gotowości przyszłych węzłów ROS. Logi polecenia aplikacji są widoczne w jego oknie;
zamknięcie przyszłej aplikacji nie zamyka automatycznie tych terminali.

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

Instalacja ROS 2, konfiguracja magistrali i aktualizowanie firmware nie są
wykonywane automatycznie przez ten projekt.
