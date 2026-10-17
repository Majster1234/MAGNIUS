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
| `ros2_ws/src/robot_bringup` | Konfiguracje przykładowe i przyszłe pliki launch |
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
bash -n start_ros2.sh scripts/ros2_terminal.bash scripts/install_socketcan.sh
./start_ros2.sh --dry-run
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
- [docs/can_protocol.md](docs/can_protocol.md) — dane wymagane do implementacji CAN.
- [docs/socketcan.md](docs/socketcan.md) — instalacja i diagnostyka systemowego CAN.
- [docs/setup.md](docs/setup.md), [docs/github.md](docs/github.md) — środowisko i GitHub.
- [OpenAI Docs: AGENTS.md](https://learn.chatgpt.com/docs/agent-configuration/agents-md) — wczytywanie instrukcji.

## Bieżący kontekst

- Aktualizacja: 2026-10-17, strefa Europe/Prague.
- Ostatnie zadanie: integracja U2C v2.1 z systemowym SocketCAN; użytkownik podał 500000 bit/s.
- Rezultat: istniejący `gs_usb` obsługuje adapter; zainstalowano `can-utils` i usługę
  `robot-can@can0.service`, konfigurację `/etc/robot-can/can0.conf` z `BITRATE=500000`.
- Usługa jest aktywna i włączona jako zależność urządzenia; instrukcja: `docs/socketcan.md`.
- Weryfikacja tej zmiany: `can0` ma flagi UP/LOWER_UP, bitrate 500000, ERROR-ACTIVE;
  zwykły użytkownik otworzył socket CAN. Liczniki RX/TX: 0; nie wysyłano ramek.
- Sprawdzono składnię Bash, błędne argumenty, jednostkę systemd, YAML i lokalne linki.
  Nie wykonano restartu komputera, fizycznego ponownego podłączenia ani testu napędów.
- Implementacja ROS nadal obejmuje cztery szkielety; brak modelu, biblioteki CAN,
  adaptera sprzętowego i plików launch. W szablonie CAN ustawiono bitrate 500000.
- Wcześniejsze sprawdzenie (2026-10-17): czyste budowanie 4 pakietów udane,
  0 testów wykonywalnych. Nie powtarzano builda w tym zadaniu ani zdalnego CI.
- Git na początku zadania: `main` śledzi `origin/main`; `AGENTS.md` był bez commitu.
  W tym zadaniu nie tworzono commitu ani nie wykonywano push.
- Pozostaje konflikt bibliotek GNOME w środowisku VS Code Snap; skrypt terminali
  nie otrzymał poprawki, a jego testy GUI były symulowane. ROS, MoveIt i Desktop są zainstalowane.
- Otwarte dane: firmware, CAN ID, geometria, przekładnie, bazowanie i limity osi;
  licencja i metadane opiekuna nieustalone. Dalszy zakres określi użytkownik.
