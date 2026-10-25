# AGENTS.md — wskazówki i kontekst dla AI

Zakres: repozytorium; uwzględniaj instrukcje podkatalogów/systemu/użytkownika.
Źródłem prawdy są kod i narzędzia.

## Utrzymanie kontekstu po każdym prompcie

1. Na początku czytaj plik; sprawdź kod, Git, środowisko i testy.
2. Przed każdą odpowiedzią aktualizuj plik, także bez zmian kodu.
3. Zastępuj **Bieżący kontekst** aktualnym podsumowaniem, bez dziennika rozmowy.
   Zachowuj obowiązujące zasady; aktualizuj zmienione ustalenia projektu.
4. Zapisuj zadanie/wynik, zmiany, decyzje, sprawdzenia/ograniczenia,
   zobowiązania i następny krok; zachowuj niewykonane zobowiązania.
5. Datuj sprawdzenia; oddzielaj fakty od planów. Dawnych testów nie przypisuj nowej sesji.
6. Usuwaj stare dane, odsyłaj do docs. Limit około 120 linii/8 KiB, kontekst 30 linii;
   ścieżki względne.
7. Bez haseł, tokenów, danych kont, pełnych logów i toku rozumowania.
8. Gdy tryb/uprawnienia zabraniają zapisu, nie omijaj ograniczeń; zgłoś brak aktualizacji.
Agent aktualizuje plik bez hooka. Narzędziu bez obsługi AGENTS.md zleć odczyt.

## Projekt i mapa repozytorium

- Robot 6-DOF, Ubuntu 22.04, ROS 2 Humble, colcon/ros2_control.
- Komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → MKS SERVO42D CAN.
- Potwierdzone 24.10: `can0`, 1000000 bit/s, sterownik `gs_usb`.
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
- Najpierw sprawdź właściwy kod i docs przez `rg`; nie czytaj niepotrzebnie całego repo.
- Kończ zadanie; odwracalne, rutynowe decyzje podejmuj samodzielnie. Pytaj o dane
  istotnie zmieniające rozwiązanie.
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
- Zmianę przełożenia i offsetu/kierunku wykonuj osobno. Kąt zachowuje automatyczny
  offset z pomiaru CAN. Bez pomiaru blokuj kalibrację; potwierdzaj wskazane osie jawnie.
- Uruchamiaj testy tylko wtedy, gdy dotyczą modyfikowanego kawałka kodu.
  Wybieraj konkretne scenariusze; nie uruchamiaj pełnych zestawów ani testów niezwiązanych ze zmianą.
- Dopasuj sprawdzenia do zmiany: build nie jest testem sterowania, a atrapa terminala testem GUI.
- Zachowuj zmiany użytkownika; sprawdzaj Git przed operacjami. Lokalny commit nie oznacza publikacji.
- Licencja odłożona: pozostaw brak `LICENSE` i `TODO` w metadanych do ustalenia przez użytkownika.
- Nie dodawaj do historii katalogów `build/`, `install/`, `log/`, lokalnych ustawień,
  danych uwierzytelniających ani nagrań ROS pomijanych przez `.gitignore`.
- Przed zmianą ROS/backendu określ podprogramy i scenariusze; zmierz przed/po te same
  dane/cykle. Bez całej kolekcji i cudzych zmian.
- Po optymalizacji uzupełnij docs/benchmark_results.md: zakres/cel, przed/po, procent,
  rozrzut, metadane, wniosek (także pogorszenie/brak poprawy). Zmieniony cykl procesu:
  CPU przy porównywalnym obciążeniu; jawnie brak bazy/pomiaru. Procedura tamże; bez GUI/CI.

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
- [Konfiguracja](docs/configuration.md), [przełożenia](docs/gear_ratio_continuity.md), [podgląd URDF](docs/robot_preview.md), [planowanie TCP](docs/cartesian_planning.md).
- [Nagrywanie](docs/can_recording.md), [wyniki](docs/can_recording_results.md).
- [SocketCAN](docs/socketcan.md), [środowisko](docs/setup.md), [Git](docs/github.md).
- [Fizyczny PTP](docs/motion_execution.md), [dokładność streamingu](docs/streaming_accuracy.md), [benchmarki](docs/benchmark_results.md).

## Bieżący kontekst

- 25.10: zgoda operatora na PTP TCP Xbazy+100mm, wały rozsprzęglone. Cztery osie1/2/3/5, brak4/6. Pełny IK~0,002mm i PTP19,36s OK; z zamrożonymi brakującymi IK bez rozwiązania. Fizycznego TCP nie da się potwierdzić.
- Gotowy izolowany test1RPM/acc1, STOP przy błędzie1,25°/przeciwnym ruchu0,5°/starym RX/CAN. Walidacja1937klatek/3zakazy OK; log/tcp-plus-x-physical-preflight-20261025/README.md, build/guarded-tcp-plus-x-ptp. Pliki produkcyjne nietknięte.
- Ruch NIE wystartował: sudo wymaga hasła, PID79530 nadal active. Operator ma wykonać `sudo systemctl stop robot-web-status.service`; potem supervisor --execute --decoupled --owner-stopped. Jeden przejazd, zapisF5/31, STOP/ENABLE0, odtworzenie parametrów; restart operatora. Nie ponawiać wykonanej próby automatycznie.
- Wcześniejszy audytratio1/dir+1/offset0 i nieprawidłowego PTP: docs/planner_profile_diagnosis_20261025.md; przyczyna F5/31 otwarta. Otwarta diagnoza WS/LAN/CAN BIT1/EAGAIN: log/browser-can-diagnosis-20261025/diagnosis-summary.md.
- Overlay streaming-smooth-ros→gear-ratio-continuity-ros→motion-control-ui-ros; przełożenia/kalibracja/F5/panel zachowane. Dawne wyniki: docs/benchmark_results.md; UI86TS/5Chromium: log/motion-control-ui-20261025.
- Otwarte: LIN/płynność/prędkość/acc135/JTC/6osi/TCP, STOP/F6=05/skoki, ROS/panel/JOG TCP/limity, IK/J4/J6/geometria/bazowanie, USB/BUS-OFF/watchdog, LAN2/oscylacje/blending/Three.js/licencja. Git feature/web-status, bez commit/push; zachować zmiany. Dalszy ruch poza tą zgodą wymaga uzgodnienia.
