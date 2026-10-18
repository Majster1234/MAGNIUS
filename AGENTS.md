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

To zobowiązanie wykonuje agent; ten plik nie jest skryptem ani automatycznym hookiem.
W narzędziu, które nie rozpoznaje `AGENTS.md`, trzeba jawnie polecić jego odczytanie.

## Projekt i mapa repozytorium

- Cel: robot 6-DOF, Ubuntu 22.04, ROS 2 Humble, `colcon`, docelowo `ros2_control`.
- Komunikacja: Linux SocketCAN → BIGTREETECH U2C v2.1 → MKS SERVO42D CAN.
- Bitrate potwierdzony przez użytkownika: 500000 bit/s, interfejs `can0`, sterownik `gs_usb`.
- Protokół napędów jest protokołem producenta MKS; nie zakładaj CANopen/CiA 402.
- Nazwy osi: `joint_1`–`joint_6`, od podstawy do narzędzia. Jednostki ROS: SI.

| Miejsce | Odpowiedzialność |
| --- | --- |
| `ros2_ws/src/robot_description` | Model URDF/Xacro, geometria i konfiguracja RViz |
| `ros2_ws/src/mks_servo_can` | Przyszła biblioteka C++: transport i protokół CAN |
| `ros2_ws/src/robot_hardware` | Przyszły adapter C++ `SystemInterface` dla Humble |
| `ros2_ws/src/robot_bringup` | Launch diagnostyki i konfiguracje przykładowe |
| `ros2_ws/src/robot_system_monitor` | Monitor systemu i CAN, diagnostyka ROS w Pythonie |
| `ros2_ws/src/robot_web_bridge`, `web/` | API FastAPI, WebSocket i panel React/TypeScript |
| `start_ros2.sh`, `scripts/ros2_terminal.bash` | Terminale aplikacji, diagnostyki i workspace |
| `hardware/`, `docs/` | Materiały sprzętowe i dokumentacja projektu |
| `system/can/`, `scripts/install_socketcan.sh` | Systemowa konfiguracja SocketCAN przez systemd |
| `.github/workflows/ci.yml` | Sprawdzenie składni i budowanie w ROS 2 Humble |

## Zasady pracy

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

`--dry-run` wymaga zainstalowanego GNOME Terminal, ale nie otwiera okien.
Po niepowodzeniu polecenia sprawdź przyczynę; nie raportuj dalszych kroków jako
zakończonych sukcesem. Dla zmian samej dokumentacji wystarczy sprawdzenie treści,
ścieżek i formatowania. Wynik `0 tests` oznacza brak testów wykonywalnych.

## Dokumentacja źródłowa

- [README.md](README.md) — uruchamianie i stan projektu.
- [CONTRIBUTING.md](CONTRIBUTING.md) — zasady zmian i lokalna weryfikacja.
- [docs/architecture.md](docs/architecture.md) — architektura i konwencje osi.
- [docs/web_status.md](docs/web_status.md) — panel LAN, uruchomienie i wyniki weryfikacji.
- [docs/can_protocol.md](docs/can_protocol.md) — dane wymagane do implementacji CAN.
- [docs/socketcan.md](docs/socketcan.md) — instalacja i diagnostyka systemowego CAN.
- [docs/setup.md](docs/setup.md), [docs/github.md](docs/github.md) — środowisko i GitHub.
- [OpenAI Docs: AGENTS.md](https://learn.chatgpt.com/docs/agent-configuration/agents-md) — wczytywanie instrukcji.

## Bieżący kontekst

- Aktualizacja: 2026-10-18, strefa Europe/Prague.
- Zadanie wykonane: panel diagnostyczny LAN, dwa pakiety ament_python i połączenie ROS.
- Instrukcja, schemat i szczegółowa weryfikacja: `docs/web_status.md`, `docs/architecture.md`.
- Start: `./scripts/start_web_status.sh`; enp2s0, 192.168.1.0/24, port 8080; wykryty URL http://192.168.1.69:8080.
- API/WebSocket tylko do odczytu, ROS_LOCALHOST_ONLY=1, pomiary 1 Hz, stale po 3 s.
- Wybrano wsproto z Uvicorn; pakiet websockets 9 z Jammy nie działa z Pythonem 3.10.
- Potwierdzono 2026-10-18: build 6 pakietów, 38 testów Python, 3 frontendu, build produkcyjny i typy TS.
- Sprawdzono Bash/XML/YAML, lokalne linki i git diff --check; CI rozszerzone, bez zdalnego uruchomienia.
- Chromium desktop/mobile: 5 paneli, 2 węzły ROS, utrata danych po ok. 2,8 s, powrót monitora i restart backendu.
- Ctrl+C czysto zamyka procesy; brak błędów JS i zewnętrznych żądań. Procesy testowe zakończono.
- Źródło Tailscale otrzymało HTTP 403; brak nasłuchu na jego adresie. Inne podsieci sprawdzono w ASGI.
- CAN: 500000 bit/s, ERROR-ACTIVE, RX/TX 0 przed i po; usługa active/enabled. Monitor tylko odczytuje Netlink.
- Brakuje systemowo FastAPI/Uvicorn/aiofiles: sudo wymaga hasła. Testy używały pakietów Ubuntu z /tmp i tymczasowego Node.js.
- Następny krok: `sudo ./scripts/install_web_dependencies.sh`, start panelu i odbiór na drugim urządzeniu LAN.
- Nie sprawdzono fizycznego drugiego klienta LAN, zapory wymagającej sudo, zdalnego CI ani napędów.
- Git: feature/web-status, zmiany bez commitu; nie wykonywano commit ani push.
- C++/ros2_control, model i protokół MKS do implementacji; firmware, CAN ID, geometria, przekładnie, bazowanie i limity nieustalone.
- Licencja/metadane TODO. Konflikt GNOME w VS Code Snap pozostaje; nowy start panelu nie wymaga GNOME Terminal.
