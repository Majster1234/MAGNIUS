# AGENTS.md — wskazówki i kontekst dla AI

Zakres: całe repozytorium, z uwzględnieniem instrukcji właściwych dla podkatalogów.
Stosuj ten plik wraz z instrukcjami systemowymi i bieżącymi poleceniami użytkownika.
Dokument opisuje zasady pracy oraz ostatni potwierdzony stan; kod i wyniki narzędzi
są źródłem prawdy o implementacji.

## Utrzymanie kontekstu po każdym prompcie

1. Na początku każdego zadania przeczytaj ten plik i sprawdź fakty istotne dla
   zadania. Nie zakładaj, że zapisany stan Git, środowiska lub testów jest aktualny.
2. Po każdym obsłużonym prompcie użytkownika, przed odpowiedzią końcową, zapisz
   nową wersję tego pliku — także po odpowiedzi niewymagającej zmian kodu.
3. Nadpisuj w całości sekcję **Bieżący kontekst** aktualnym podsumowaniem. Zachowuj
   nadal obowiązujące zasady; zmieniaj je, gdy zmieniły się ustalenia projektu.
   Nie dopisuj dziennika kolejnych promptów ani kopii rozmowy.
4. Zapisuj: ostatnie zadanie i rezultat, stan implementacji, istotne decyzje,
   wykonane sprawdzenia i ich ograniczenia, otwarte sprawy oraz następny krok.
   Zachowuj niezakończone zobowiązania, dopóki nie zostały wykonane lub odwołane.
5. Oddzielaj fakty potwierdzone narzędziami od założeń i planów. Podawaj datę
   sprawdzeń. Nie przedstawiaj wcześniejszych testów jako wykonanych ponownie.
6. Usuwaj nieaktualne informacje i odsyłaj do dokumentacji zamiast ją kopiować.
   Utrzymuj cały plik w granicach około 120 linii i 8 KiB, a kontekst do 30 linii.
7. Nie zapisuj haseł, tokenów, prywatnych danych kont, pełnych logów ani wewnętrznego
   toku rozumowania. Używaj ścieżek względnych do repozytorium.
8. Jeżeli tryb pracy lub uprawnienia zabraniają zapisu, nie omijaj ograniczeń.
   Poinformuj w odpowiedzi, że aktualizacja kontekstu nie została zapisana.

Agent aktualizuje plik; nie jest to hook. W narzędziu bez obsługi AGENTS.md zleć jego odczytanie.

## Projekt i mapa repozytorium

- Cel: robot 6-DOF, Ubuntu 22.04, ROS 2 Humble, `colcon`, docelowo `ros2_control`.
- Komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → MKS SERVO42D CAN.
- Bitrate potwierdzony przez użytkownika: 500000 bit/s, interfejs `can0`, sterownik `gs_usb`.
- Protokół napędów jest protokołem producenta MKS; nie zakładaj CANopen/CiA 402.
- CAN 1–6 → J1–J6. ROS: SI; GUI: mm/°. JOG: °/s przegubu lub RPM wału. Mobile zakończone.

| Miejsce | Odpowiedzialność |
| --- | --- |
| `ros2_ws/src/robot_description` | Model URDF/Xacro, geometria i konfiguracja RViz |
| `ros2_ws/src/mks_servo_can` | Protokół, transport, manager i ManualMotorController |
| `ros2_ws/src/robot_can_diagnostics` | Diagnostyka CAN i usługi ręczne w C++ |
| `ros2_ws/src/robot_hardware` | Plugin CanSystem dla sześciu osi, Humble |
| `ros2_ws/src/robot_kinematics`, `robot_motion_planner` | FK/IK KDL, droga TCP/TOTG i podgląd MKS bez wykonania |
| `ros2_ws/src/robot_bringup` | Launch diagnostyki i konfiguracje przykładowe |
| `ros2_ws/src/robot_system_monitor` | Monitor systemu i CAN |
| `ros2_ws/src/robot_configuration` | Snapshoty JSON i zapis do ROS |
| `ros2_ws/src/robot_web_bridge`, `web/` | API FastAPI, WebSocket i panel React/TypeScript |
| `start_ros2.sh`, `scripts/ros2_terminal.bash` | Terminale aplikacji, diagnostyki i workspace |
| `scripts/install_app_autostart.sh`, `system/application/` | Instalator autostartu panelu przez systemd |
| `hardware/`, `docs/` | Materiały sprzętowe i dokumentacja projektu |
| `system/can/`, `scripts/install_socketcan.sh` | Systemowa konfiguracja SocketCAN przez systemd |
| `.github/workflows/ci.yml` | Sprawdzenie składni i budowanie w ROS 2 Humble |

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

## Bieżący kontekst

- 2026-10-20: wspólny AppShell, 5 tras (#config i alias #configuration), menu 68/224 px i topbar. CAN→J1–J6 i obecność osi z diagnostyki; docs/web_status.md.
- Manual: osobne Z/Rz, PTP/LIN poza sceną, model ruchu 22% z checkboxem; dziennik grupuje świeżość <500 ms. JOG/TCP/WS/watchdog bez zmian. docs/manual_control.md, docs/robot_preview.md.
- 20.10: TS/Vite, 75 Vitest, pełne 7 zestawów Chromium OK. Układ 1280–2560; impulsy w SW WebGL 1366. Bez sprzętu/ROS/backendu. Ostrzeżenie rozmiaru Three.js.
- TCP: 0.1/1/5/10/20 mm i °, LIN na klik, bez kolejki; STOP/blur/kontekst anulują. Fizyczne TCP zablokowane; docs/cartesian_planning.md.
- Wcześniej 20.10 LAN: backend impulsów nieaktywny. Nadal potrzebny sudo systemctl restart robot-web-status.service; nie restartowano. Następnie Ctrl+F5 i odbiór UI.
- Otwarte: IK/J4/J6, geometria/przekładnie/bazowanie, odbiór JOG/limitów, USB/BUS-OFF/watchdog, drugi klient LAN, oscylacje. Licencja TODO; feature/web-status bez commit/push.
