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
- CAN 1–6 → J1–J6 / `joint_1`–`joint_6`. ROS: SI; panel desktop: stopnie wału i RPM; mobile zakończone.

| Miejsce | Odpowiedzialność |
| --- | --- |
| `ros2_ws/src/robot_description` | Model URDF/Xacro, geometria i konfiguracja RViz |
| `ros2_ws/src/mks_servo_can` | Protokół, transport, manager i ManualMotorController |
| `ros2_ws/src/robot_can_diagnostics` | Diagnostyka CAN i usługi ręczne w C++ |
| `ros2_ws/src/robot_hardware` | Plugin CanSystem dla sześciu osi, Humble |
| `ros2_ws/src/robot_bringup` | Launch diagnostyki i konfiguracje przykładowe |
| `ros2_ws/src/robot_system_monitor` | Monitor systemu i CAN, diagnostyka ROS w Pythonie |
| `ros2_ws/src/robot_web_bridge`, `web/` | API FastAPI, WebSocket i panel React/TypeScript |
| `start_ros2.sh`, `scripts/ros2_terminal.bash` | Terminale aplikacji, diagnostyki i workspace |
| `hardware/`, `docs/` | Materiały sprzętowe i dokumentacja projektu |
| `system/can/`, `scripts/install_socketcan.sh` | Systemowa konfiguracja SocketCAN przez systemd |
| `.github/workflows/ci.yml` | Sprawdzenie składni i budowanie w ROS 2 Humble |

## Zasady pracy

- Domyślnie przed każdym testem ruchu opisz oś, kierunek, prędkość, czas/zakres
  i zatrzymanie oraz uzyskaj zgodę użytkownika przed wykonaniem.
  Wyjątek: w obecnej rozmowie 2026-10-18 użytkownik zezwolił na testy sesji,
  w tym dostępne osie po 15 s w obu kierunkach; nie pytaj ponownie. Wyjątek nie przechodzi do nowej sesji.
- Rozmawiaj i pisz dokumentację po polsku; zachowuj istniejące nazwy pakietów i API.
- Najpierw sprawdź odpowiedni kod i dokumentację. Wyszukuj przez `rg`; nie czytaj
  całego repozytorium, jeśli zadanie dotyczy małego fragmentu.
- Wykonuj zlecone zadanie do końca. Rutynowe, odwracalne decyzje podejmuj samodzielnie;
  pytaj o brakujące dane, które istotnie zmieniają rozwiązanie.
- Zachowuj rozdzielenie protokołu MKS, transportu SocketCAN i adaptera ROS.
  Zależności deklaruj w `package.xml` i CMake wraz z korzystającą z nich implementacją.
- Nie wymyślaj geometrii, CAN ID, bitrate, przełożeń, limitów ani wersji firmware.
  `null` w `.example.yaml` oznacza brak danych, a nie zero lub wartość domyślną.
- Dopasowuj sprawdzenia do zmiany. Nie utożsamiaj budowania szkieletu z testem
  sterowania ani testu z atrapą terminala z uruchomieniem rzeczywistych okien GUI.
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

- [README.md](README.md) — uruchamianie i stan projektu.
- [CONTRIBUTING.md](CONTRIBUTING.md) — zasady zmian i lokalna weryfikacja.
- [docs/architecture.md](docs/architecture.md) — architektura i konwencje osi.
- [docs/web_status.md](docs/web_status.md) — panel LAN, uruchomienie i wyniki weryfikacji.
- [docs/can_diagnostics.md](docs/can_diagnostics.md) — skan, Reset CAN i ograniczenia odczytów.
- [docs/can_strategy.md](docs/can_strategy.md) — harmonogram, ruch, watchdog i konfiguracja.
- [docs/manual_control.md](docs/manual_control.md) — ENABLE, zero, JOG i interfejsy ROS/API.
- [docs/can_recording.md](docs/can_recording.md) — zapis/analiza incydentów; wyniki w can_recording_results.md.
- [docs/socketcan.md](docs/socketcan.md) — instalacja i diagnostyka systemowego CAN.
- [docs/setup.md](docs/setup.md), [docs/github.md](docs/github.md) — środowisko i GitHub.

## Bieżący kontekst

- Aktualizacja: 2026-10-18, Europe/Prague. Zmniejszono panel ręczny i zastosowano ciemny motyw całego GUI.
- Tytuł, połączenie i nagrywanie u góry; wynik operacji poniżej. Panel 279,5 px, wiersze 29 px; reszta miejsca wolna.
- TypeScript/build, 12 Vitest i Chromium OK: cztery rozdzielczości 1280×800–1920×1080, bez przepełnienia.
- Testy atrap obejmują nadal ENABLE, przytrzymanie/puszczenie, dwóch klientów, zanik danych i brak auto wznowienia.
- Działający LAN sprawdzony tylko odczytem: nowe pliki statyczne, statusy 20 Hz, ciemny przegląd i CAN, brak błędów JS.
- Bez restartu backendu i prób ruchu; nie powtarzano ROS/C++ ani API. Dokumentacja: docs/manual_control.md.
- Poprzednio JOG: F1=1, 60 ms, ±1° od stałego punktu; zero/ENABLE nadal 100 ms / ±3 impulsy. 50 GTest i J1/J2 sprzętowo OK.
- Nagrywanie: log/manual-can, do 96 MiB; analiza wcześniejszego WS: docs/can_incident_2026-10-18.md. Ostatnio dostępne 1/2/4, brak 3/5/6.
- Dalej: odbiór limitów/USB/BUS-OFF/watchdog, sześciu osi, modelu/parametrów i zapory. Licencja TODO, sudoers bez zmian.
- Git feature/web-status, bez commit/push. Panel 192.168.1.69:8080/#manual; odświeżyć stronę dla nowego wyglądu.
