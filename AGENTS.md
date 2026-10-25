# AGENTS.md — wskazówki i kontekst dla AI

Zakres: repozytorium; uwzględniaj instrukcje podkatalogów/systemu/użytkownika.
Źródłem prawdy są kod i narzędzia.

## Utrzymanie kontekstu po każdym prompcie

1. Na początku czytaj plik i sprawdzaj fakty, Git, środowisko i testy.
2. Przed odpowiedzią po każdym prompcie aktualizuj plik, także bez zmian kodu.
3. Nadpisuj **Bieżący kontekst** aktualnym podsumowaniem, bez dziennika i kopii rozmowy.
   Zachowuj obowiązujące zasady; aktualizuj je po zmianie ustaleń projektu.
4. Zapisuj zadanie/rezultat, implementację, decyzje, sprawdzenia/ograniczenia,
   otwarte zobowiązania i następny krok; nie usuwaj niewykonanych zobowiązań.
5. Oddzielaj fakty od założeń/planów; datuj sprawdzenia. Nie przypisuj dawnych testów nowej sesji.
6. Usuwaj stare dane, odsyłaj do docs. Limit: około 120 linii/8 KiB, kontekst 30 linii; ścieżki względne.
7. Nie zapisuj haseł, tokenów, prywatnych danych kont, pełnych logów ani toku rozumowania.
8. Gdy tryb lub uprawnienia zabraniają zapisu, nie omijaj ograniczeń; zgłoś brak aktualizacji.
Agent aktualizuje plik, bez hooka. Narzędziu bez obsługi AGENTS.md zleć odczyt.

## Projekt i mapa repozytorium

- Robot 6-DOF, Ubuntu 22.04, ROS 2 Humble, colcon/ros2_control.
- Komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → MKS SERVO42D CAN.
- Bitrate potwierdzony przez użytkownika i hosta (24.10): 1000000 bit/s, interfejs `can0`, sterownik `gs_usb`.
- Protokół napędów jest protokołem producenta MKS; nie zakładaj CANopen/CiA 402.
- CAN 1–6 → J1–J6. ROS: SI; GUI: mm/°. JOG: °/s przegubu lub RPM wału. Mobile zakończone.

| Miejsce | Odpowiedzialność |
| --- | --- |
| `ros2_ws/src/robot_description` | URDF/Xacro, geometria, RViz |
| `ros2_ws/src/mks_servo_can` | Protokół, transport, manager, ManualMotorController |
| `ros2_ws/src/robot_can_diagnostics` | Diagnostyka/usługi C++ |
| `ros2_ws/src/robot_hardware` | CanSystem, sześć osi, Humble |
| `ros2_ws/src/robot_kinematics`, `robot_motion_planner` | FK/IK KDL, TCP/TOTG, podgląd MKS |
| `ros2_ws/src/robot_bringup` | Launch i przykłady |
| `ros2_ws/src/robot_system_monitor` | Monitor systemu i CAN |
| `ros2_ws/src/robot_configuration` | Snapshoty JSON/ROS |
| `ros2_ws/src/robot_web_bridge`, `web/` | FastAPI, WebSocket, React/TypeScript |
| `start_ros2.sh`, `scripts/ros2_terminal.bash` | Terminale ROS |
| `scripts/install_app_autostart.sh`, `system/application/` | Autostart panelu, systemd |
| `hardware/`, `docs/` | Sprzęt i dokumentacja |
| `system/can/`, `scripts/install_socketcan.sh` | SocketCAN przez systemd |
| `.github/workflows/ci.yml` | Składnia i build Humble |

## Zasady pracy

- Domyślnie przed każdym testem ruchu opisz oś, kierunek, prędkość, czas/zakres
  i zatrzymanie oraz uzyskaj zgodę użytkownika przed wykonaniem.
- PTP/LIN streaming (25.10, nowy build): 100 Hz, F5/F6/F7/FE bez ACK, bez SYNC (4A=0),
  CanRSP/raporty wyłączone (8C00/00), SR_vFOC=05. Lookahead85 ms, FF1, Kp2,5,
  dithering per oś, holding1/max2500 RPM, acc135; limity mechaniczne obowiązują.
  Odczyty PTP bez startu zapytań 4 ms przed F5; ruch nie czeka na odczyt.
  Tolerancja ±0,5° tylko na końcu (dwa pomiary >=100 ms); bez pauz pośrednich.
  CAN: razem60%, robot40%, osobne20 punktów dla przyszłego modułu.
  Sześć osi100 Hz wymaga1 Mbit/s; fizyczny can0 ma1 Mbit/s, TX1000.
- Rozmawiaj i pisz dokumentację po polsku; zachowuj istniejące nazwy pakietów i API.
- Zakładkę „Ustawienia systemowe” uzupełniaj tylko na konkretne polecenie użytkownika.
- Najpierw sprawdź odpowiedni kod i dokumentację. Wyszukuj przez `rg`; nie czytaj
  całego repozytorium, jeśli zadanie dotyczy małego fragmentu.
- Wykonuj zlecone zadanie do końca. Rutynowe, odwracalne decyzje podejmuj samodzielnie;
  pytaj o brakujące dane, które istotnie zmieniają rozwiązanie.
- Po zmianach buduj i wdrażaj właściwe pakiety do konfiguracji używanej przez usługę.
  Restart może wykonać użytkownik; zawsze podaj w odpowiedzi komendę restartu
  właściwej usługi i zaznacz, czy nowy build został już uruchomiony.
- Zachowuj rozdzielenie protokołu MKS, transportu SocketCAN i adaptera ROS.
  Zależności deklaruj w `package.xml` i CMake wraz z korzystającą z nich implementacją.
- Nie wymyślaj geometrii, CAN ID, bitrate, przełożeń, limitów ani wersji firmware.
  `null` w `.example.yaml` oznacza brak danych, a nie zero lub wartość domyślną.
  Wyjątek: zatwierdzony przykładowy URDF podglądu 1:1, oddzielony od fizycznego robota.
  W symulacji PTP/LIN dopuszczono ostatnią pozycję lub jawnie oznaczone 0° bez pomiaru;
  nie są to odczyty sprzętu, a limity profilu nadal obowiązują.
- Uruchamiaj testy tylko wtedy, gdy dotyczą modyfikowanego kawałka kodu.
  Wybieraj konkretne scenariusze; nie uruchamiaj pełnych zestawów ani testów niezwiązanych ze zmianą.
- Dopasuj sprawdzenia do zmiany: build nie jest testem sterowania, a atrapa terminala testem GUI.
- Zachowuj zmiany użytkownika. Sprawdzaj stan Git przed operacjami na repozytorium;
  nie wnioskuj o publikacji na podstawie samej obecności lokalnego commitu.
- Wybór licencji jest odłożony przez użytkownika. Pozostaw brak `LICENSE` i pola
  `TODO` w metadanych, dopóki użytkownik nie ustali właściwych danych.
- Nie dodawaj do historii katalogów `build/`, `install/`, `log/`, lokalnych ustawień,
  danych uwierzytelniających ani nagrań ROS pomijanych przez `.gitignore`.
- Przed zmianą ROS/backendu określ podprogramy, dodaj brakujące scenariusze,
  zmierz przed/po te same dane/cykle. Bez całej kolekcji i cudzych zmian.
- Po optymalizacji uzupełnij docs/benchmark_results.md: zakres, cel, przed/po,
  procent, rozrzut, metadane, wniosek (także pogorszenie/brak poprawy).
  Zmieniony cykl procesu: CPU przy porównywalnym obciążeniu; brak bazy/pomiaru
  opisz jawnie. Procedura i narzędzie tamże; bez GUI/CI.

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

- [README.md](README.md), [CONTRIBUTING.md](CONTRIBUTING.md) — start i sprawdzenia.
- [Architektura](docs/architecture.md), [LAN/API](docs/web_status.md), [diagnostyka CAN](docs/can_diagnostics.md).
- [Harmonogram CAN](docs/can_strategy.md), [JOG/ENABLE/zero](docs/manual_control.md), [trace wałów](docs/shaft_trace.md).
- [Konfiguracja](docs/configuration.md), [podgląd URDF](docs/robot_preview.md), [planowanie TCP](docs/cartesian_planning.md).
- [Nagrywanie](docs/can_recording.md), [wyniki](docs/can_recording_results.md).
- [SocketCAN](docs/socketcan.md), [środowisko](docs/setup.md), [Git](docs/github.md).
- [Fizyczny PTP](docs/motion_execution.md), [benchmarki](docs/benchmark_results.md).

## Bieżący kontekst

- 25.10 LIN/PTP: wspólny wykonawca, nowa ComputePhysicalMotion i zgodne PTP, limity TCP i typ w HTTP/panelu. Niepełny zestaw oznaczony, nadzór jak PTP. docs/motion_execution.md.
- Sprawdzenia25.10:109 C++/Python/ROS+2 TS+2 Chromium OK;11 pakietów i frontend zbudowane. Bez fizycznego CAN/ruchu. Wyniki/benchmarki: log/lin-20261025, docs/benchmark_results.md.
- Wdrożone: frontend i punkt install/streaming-smooth-ros/local_setup.bash → install/lin-ros/local_setup.bash; kopie: log/lin-20261025/deployment. PID12021/NRestarts0 bez zmian. Dalej restart robot-web-status.service w postoju, prepare LIN bez execute. Profil500RPM/80°/s/3:1/acc135 bez zmian.
- Otwarte: odbiór toru LIN, płynność/prędkość/acc135/JTC/6osi/TCP, STOP/F6=05/skoki, ROS/panel, JOG TCP/limity, IK/J4/J6, geometria/bazowanie, USB/BUS-OFF/watchdog, LAN2/oscylacje, blending, Three.js/licencja. Git feature/web-status; zachować cudze zmiany, bez commit/push. Flagi po pomiarach; ruch wymaga osobnej zgody.
