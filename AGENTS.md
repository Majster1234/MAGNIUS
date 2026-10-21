# AGENTS.md — wskazówki i kontekst dla AI

Zakres: całe repozytorium; uwzględniaj instrukcje podkatalogów, systemu i użytkownika.
Zasady i ostatni kontekst poniżej; kod i wyniki narzędzi są źródłem prawdy.

## Utrzymanie kontekstu po każdym prompcie

1. Na początku zadania czytaj ten plik; sprawdzaj istotne fakty, aktualny Git, środowisko i testy.
2. Po każdym prompcie, także bez zmian kodu, przed odpowiedzią zapisz nową wersję pliku.
3. Nadpisuj **Bieżący kontekst** aktualnym podsumowaniem, bez dziennika i kopii rozmowy.
   Zachowuj obowiązujące zasady; aktualizuj je po zmianie ustaleń projektu.
4. Zapisuj zadanie/rezultat, implementację, decyzje, sprawdzenia/ograniczenia,
   otwarte zobowiązania i następny krok; nie usuwaj niewykonanych zobowiązań.
5. Oddzielaj fakty od założeń/planów; datuj sprawdzenia. Nie przypisuj dawnych testów nowej sesji.
6. Usuwaj stare dane, odsyłaj do docs. Limit: około 120 linii/8 KiB, kontekst 30 linii; ścieżki względne.
7. Nie zapisuj haseł, tokenów, prywatnych danych kont, pełnych logów ani toku rozumowania.
8. Gdy tryb lub uprawnienia zabraniają zapisu, nie omijaj ograniczeń; zgłoś brak aktualizacji.

Agent aktualizuje plik; nie jest to hook. W narzędziu bez obsługi AGENTS.md zleć jego odczytanie.

## Projekt i mapa repozytorium

- Cel: robot 6-DOF, Ubuntu 22.04, ROS 2 Humble, `colcon`, docelowo `ros2_control`.
- Komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → MKS SERVO42D CAN.
- Bitrate potwierdzony przez użytkownika: 500000 bit/s, interfejs `can0`, sterownik `gs_usb`.
- Protokół napędów jest protokołem producenta MKS; nie zakładaj CANopen/CiA 402.
- CAN 1–6 → J1–J6. ROS: SI; GUI: mm/°. JOG: °/s przegubu lub RPM wału. Mobile zakończone.

| Miejsce | Odpowiedzialność |
| --- | --- |
| `ros2_ws/src/robot_description` | URDF/Xacro, geometria, RViz |
| `ros2_ws/src/mks_servo_can` | Protokół, transport, manager, ManualMotorController |
| `ros2_ws/src/robot_can_diagnostics` | Diagnostyka CAN, usługi ręczne C++ |
| `ros2_ws/src/robot_hardware` | CanSystem, sześć osi, Humble |
| `ros2_ws/src/robot_kinematics`, `robot_motion_planner` | FK/IK KDL, TCP/TOTG, podgląd MKS |
| `ros2_ws/src/robot_bringup` | Launch diagnostyki i konfiguracje przykładowe |
| `ros2_ws/src/robot_system_monitor` | Monitor systemu i CAN |
| `ros2_ws/src/robot_configuration` | Snapshoty JSON, zapis do ROS |
| `ros2_ws/src/robot_web_bridge`, `web/` | FastAPI, WebSocket, React/TypeScript |
| `start_ros2.sh`, `scripts/ros2_terminal.bash` | Terminale aplikacji/diagnostyki/workspace |
| `scripts/install_app_autostart.sh`, `system/application/` | Autostart panelu, systemd |
| `hardware/`, `docs/` | Sprzęt i dokumentacja |
| `system/can/`, `scripts/install_socketcan.sh` | SocketCAN przez systemd |
| `.github/workflows/ci.yml` | Składnia i build Humble |

## Zasady pracy

- Domyślnie przed każdym testem ruchu opisz oś, kierunek, prędkość, czas/zakres
  i zatrzymanie oraz uzyskaj zgodę użytkownika przed wykonaniem.
- Rozmawiaj i pisz dokumentację po polsku; zachowuj istniejące nazwy pakietów i API.
- Zakładkę „Ustawienia systemowe” uzupełniaj tylko na konkretne polecenie użytkownika.
- Najpierw sprawdź odpowiedni kod i dokumentację. Wyszukuj przez `rg`; nie czytaj
  całego repozytorium, jeśli zadanie dotyczy małego fragmentu.
- Wykonuj zlecone zadanie do końca. Rutynowe, odwracalne decyzje podejmuj samodzielnie;
  pytaj o brakujące dane, które istotnie zmieniają rozwiązanie.
- Zachowuj rozdzielenie protokołu MKS, transportu SocketCAN i adaptera ROS.
  Zależności deklaruj w `package.xml` i CMake wraz z korzystającą z nich implementacją.
- Nie wymyślaj geometrii, CAN ID, bitrate, przełożeń, limitów ani wersji firmware.
  `null` w `.example.yaml` oznacza brak danych, a nie zero lub wartość domyślną.
  Wyjątek: zatwierdzony przykładowy URDF podglądu 1:1, oddzielony od fizycznego robota.
  W symulacji PTP/LIN dopuszczono ostatnią pozycję lub jawnie oznaczone 0° bez pomiaru;
  nie są to odczyty sprzętu, a limity profilu nadal obowiązują.
- Dopasuj sprawdzenia do zmiany: build nie jest testem sterowania, a atrapa terminala testem GUI.
- Zachowuj zmiany użytkownika. Sprawdzaj stan Git przed operacjami na repozytorium;
  nie wnioskuj o publikacji na podstawie samej obecności lokalnego commitu.
- Wybór licencji jest odłożony przez użytkownika. Pozostaw brak `LICENSE` i pola
  `TODO` w metadanych, dopóki użytkownik nie ustali właściwych danych.
- Nie dodawaj do historii katalogów `build/`, `install/`, `log/`, lokalnych ustawień,
  danych uwierzytelniających ani nagrań ROS pomijanych przez `.gitignore`.
- Przed zmianą logiki ROS/backendu ustal listę podprogramów z bieżącego promptu;
  dodaj brakujące scenariusze i zmierz je przed/po, na tych samych danych i cyklach.
  Badaj tylko zmienione podprogramy/cykle, bez całej kolekcji i wcześniejszych zmian.
- Po każdej optymalizacji uzupełniaj `docs/benchmark_results.md`: lista podprogramów,
  cel, przed/po, procent, rozrzut, metadane i wniosek, także pogorszenie/brak poprawy.
  Dla zmienionego cyklu procesu dodaj odczyt CPU przy porównywalnym obciążeniu;
  brak pomiaru/bazy opisz jawnie. Procedura i narzędzie: ten sam dokument; bez GUI/CI.

## Polecenia sprawdzające

Uruchamiaj odpowiedni podzbiór z katalogu głównego repozytorium:

```bash
bash -n start_ros2.sh scripts/*.sh scripts/*.bash
./start_ros2.sh --dry-run
(cd web && npm ci && npm run build && npm test)
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

`--dry-run` wymaga GNOME Terminal i nie otwiera okien. Po błędzie sprawdź przyczynę;
nie raportuj dalszych kroków jako sukcesów. Dokumentacja wymaga kontroli treści/ścieżek/formatu.
`0 tests` oznacza brak testów wykonywalnych.

## Dokumentacja źródłowa

- [README.md](README.md) — uruchamianie; [CONTRIBUTING.md](CONTRIBUTING.md) — zmiany i sprawdzenia.
- [docs/architecture.md](docs/architecture.md) — architektura i osie.
- [docs/web_status.md](docs/web_status.md) — LAN/API; [docs/can_diagnostics.md](docs/can_diagnostics.md) — skan CAN.
- [docs/can_strategy.md](docs/can_strategy.md) — harmonogram; [docs/manual_control.md](docs/manual_control.md) — JOG/ENABLE/zero.
- [docs/configuration.md](docs/configuration.md) — snapshoty i parametry; [docs/robot_preview.md](docs/robot_preview.md) — podgląd URDF.
- [docs/cartesian_planning.md](docs/cartesian_planning.md) — IK, droga TCP, API i granice pierwszego etapu.
- [docs/can_recording.md](docs/can_recording.md) — nagrywanie; wyniki w docs/can_recording_results.md.
- [docs/socketcan.md](docs/socketcan.md) — SocketCAN; [docs/setup.md](docs/setup.md), [docs/github.md](docs/github.md) — środowisko/Git.
- [docs/motion_execution.md](docs/motion_execution.md) — fizyczny PTP, niepełny zestaw i odbiór.
- [docs/benchmark_results.md](docs/benchmark_results.md) — benchmarki, pomiary CPU i historia optymalizacji.

## Bieżący kontekst

- 2026-10-21: zgoda na rozsprzęglone 1→2→4, +5°/powrót ≤1 RPM. CAN 1,2,3,5 obecne. Test CAN 1: F5 czeka, lecz F7 ACK=1 nie kasuje celu przed SYNC; przerwano przy 0,154°. Etapy 2/4 niewykonane. docs/motion_hardware_results.md, scripts/test_ptp_hardware.py.
- ENABLE=0 wszystkich, parametry odtworzone, usługa przywrócona, bez nowych błędów CAN/zmiany profilu. Execution nadal false. Następnie odbiór kasowania bufora/STOP na 1, dopiero 2/4; nie ustawiać flag bez pomiaru.
- PTP C++/panel i allow_missing_motion wdrożone; LIN/blending przyszłe. Wcześniejsze build/testy i regresje benchmarków: docs/motion_execution.md, docs/benchmark_results.md; dziś tylko test stanowiskowy i narzędzia, bez ponowienia całej kolekcji.
- Otwarte: Ctrl+F5/odbiór UI, JOG TCP zablokowany, IK/J4/J6, geometria/przekładnie/bazowanie, JOG/limity, USB/BUS-OFF/watchdog, drugi LAN, oscylacje, odbiór PTP. Three.js: rozmiar. Licencja TODO; feature/web-status, bez commit/push.
