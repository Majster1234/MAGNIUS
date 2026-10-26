# AGENTS.md — instrukcje operacyjne dla AI

Zakres: całe repozytorium. Źródłem prawdy jest kod. Komunikacja i dokumentacja: język polski (zachowuj angielskie nazwy w kodzie/API).

## 1. Dyrektywy efektywności (Krytyczne — zero zbędnych akcji)

- **Zakaz rutynowego skanowania:** NIE sprawdzaj stanu Git, środowiska, drzewa plików ani testów na początku promptu, dopóki zadanie bezpośrednio tego nie wymaga.
- **Selektywny odczyt:** Używaj `rg` (ripgrep) do punktowego szukania w kodzie i `docs/`. Nie czytaj całych plików ani dokumentacji na zapas.
- **Warunkowa aktualizacja tego pliku:** Aktualizuj sekcję `Bieżący kontekst` TYLKO wtedy, gdy w sesji zmienił się kod, konfiguracja lub stan testów sprzętowych. Przy pytaniach, analizie i planowaniu **nie edytuj `AGENTS.md`**.
- **Budowanie i testy celowane:** Buduj i testuj wyłącznie modyfikowane pakiety (`colcon build --packages-select <pkg>`). Nie uruchamiaj pełnych zestawów testów, `npm ci` ani benchmarków bez wyraźnego polecenia lub zmiany w danym module.
- **Brak zgadywania parametrów:** Nie wymyślaj geometrii, CAN ID, bitrate, przełożeń ani limitów (`null` w `.example.yaml` = brak danych, nie zero).
- **Decyzyjność:** Odwracalne decyzje techniczne podejmuj samodzielnie. Pytaj tylko przed operacjami fizycznymi (ruch robota) lub zmianami architektury.

## 2. Bezpieczeństwo sprzętu i zasady domenowe

- **Ruch fizyczny (WYMÓG ZGODY):** Przed każdym testem ruchu opisz: oś, kierunek, prędkość, czas/zakres oraz warunek zatrzymania. Zaczekaj na wyraźną zgodę użytkownika.
- **Architektura:** Zachowuj ścisłe rozdzielenie: protokół MKS (`mks_servo_can`) ↔ transport SocketCAN ↔ adapter ROS (`robot_hardware`). Zależności deklaruj w `package.xml` i `CMakeLists.txt`.
- **Wdrożenie zmian:** Po modyfikacji kodu zbuduj zmieniony pakiet. Nie restartuj usług systemowych z `sudo` w ciemno — podaj użytkownikowi gotową komendę restartu i zaznacz, czy nowy build już działa.
- **Kalibracja i przełożenia:** Zmianę przełożenia i offsetu/kierunku wykonuj osobno. Kąt zachowuje automatyczny offset z pomiaru CAN (bez pomiaru blokuj kalibrację).
- **Parametry PTP/LIN (build z 25.10):**
  - Streaming: 100 Hz, komendy F5/F6/F7/FE bez ACK, bez SYNC (`4A=0`), raporty CanRSP wyłączone (`8C00/00`), `SR_vFOC=05`.
  - Regulator i limity: Lookahead 85 ms, FF 1, Kp 2.5, dithering per oś, prąd holding 1, max 2500 RPM, acc 135. Limity mechaniczne obowiązują zawsze.
  - Odczyty PTP: wstrzymaj nowe zapytania 4 ms przed F5 (ruch nie czeka na odczyt). Tolerancja końcowa ±0.5° (dwa pomiary >= 100 ms), bez pauz pośrednich.
  - Pasmo CAN (1 Mbit/s, TX 1000): łącznie max 60% (robot 40%, rezerwa 20%).
- **Higiena repozytorium:** Zachowuj lokalne zmiany użytkownika w Git. Ignoruj `build/`, `install/`, `log/` i nagrania ROS. Nie ruszaj kwestii licencji (`LICENSE`) ani zakładki GUI „Ustawienia systemowe” bez polecenia.
- **Benchmarki:** Tylko przy zadaniach optymalizacyjnych zmierz stan przed/po na tych samych danych i dopisz wynik do `docs/benchmark_results.md`.

## 3. Architektura i mapa repozytorium

- **System:** Robot 6-DOF, Ubuntu 22.04, ROS 2 Humble (`colcon`/`ros2_control`).
- **Magistrala:** Linux SocketCAN (`can0`, 1 Mbit/s, sterownik `gs_usb`) → BIGTREETECH U2C v2.1 → MKS SERVO42D CAN (natywny protokół MKS, **nie** CANopen/CiA 402).
- **Konwencje:** CAN ID 1–6 → J1–J6. Wewnątrz ROS: jednostki SI; w GUI: mm oraz stopnie (°). JOG: °/s przegubu lub RPM wału.

| Ścieżka | Odpowiedzialność |
| --- | --- |
| `ros2_ws/src/mks_servo_can` | Protokół MKS, transport CAN, manager, ManualMotorController |
| `ros2_ws/src/robot_hardware` | Interfejs `CanSystem` (ros2_control, 6 osi) |
| `ros2_ws/src/robot_kinematics`, `robot_motion_planner` | FK/IK (KDL), planowanie TCP/TOTG, podgląd MKS |
| `ros2_ws/src/robot_description`, `robot_bringup` | URDF/Xacro, RViz, pliki launch |
| `ros2_ws/src/robot_can_diagnostics`, `robot_system_monitor` | Diagnostyka C++, monitor systemu i CAN |
| `ros2_ws/src/robot_configuration` | Snapshoty konfiguracji JSON/ROS |
| `ros2_ws/src/robot_web_bridge`, `web/` | Backend FastAPI/WebSocket, frontend React/TypeScript |
| `scripts/`, `system/`, `start_ros2.sh` | Skrypty uruchomieniowe, systemd (SocketCAN i autostart) |

## 4. Polecenia sprawdzające (Uruchamiaj tylko adekwatne do zmian!)

```bash
# Tylko przy zmianie skryptów bash:
bash -n start_ros2.sh scripts/*.sh scripts/*.bash

# Tylko przy zmianie w web/ (używaj npm test / build, NIE uruchamiaj npm ci bez zmiany zależności):
(cd web && npm run build && npm test)

# Tylko dla zmodyfikowanych pakietów ROS 2:
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select <nazwa_pakietu>
colcon test --packages-select <nazwa_pakietu> && colcon test-result --verbose
```

## Bieżący kontekst

- 26.10.2026, korekta panelu: operator wymaga wykonywania dalszych ruchów fizycznych wyłącznie przez panel operatorski; nie używać bezpośrednich skryptów WS/ROS/CAN do ruchu. Nadal obowiązuje opis i osobna zgoda przed każdym testem. Dotychczasowy licznik prób sprzętowych pozostaje 4/6; w tej korekcie brak ruchu fizycznego.
- Znaleziono w logach mostu `AssertionError: The 'step' field must be of type 'float'`: JSON `step: 0` z trybu ciągłego trafiał jako Python int do typu ROS. Naprawiono konwersję zweryfikowanych `velocity`/`step` na float. Panel pokazuje teraz konkretną przyczynę awarii w widocznym opisie. Trzy regresje odtworzyły błąd przed poprawką; WS→typowany ROS na atrapach OK.
- Build `robot_web_bridge` w `install/tcp-panel-ros` OK, colcon 177/0 błędów; frontend build i 142 testy TS OK. Nowy `npm run test:tcp-panel-ros` wymaga środowiska ROS i testuje kliknięcia panelu→produkcyjny WS→typ JogTcp z wykonawcą na localhost bez CAN. Przeszedł wraz z `test:tcp-live` (ciągły/impulsowy, przytrzymanie, puszczenie, klawiatura, blur, opis awarii, brak wznowienia).
- Poprawka panelu aktywna: frontend skopiowany atomowo do `install/tcp-settings-ros/robot_web_bridge/share/robot_web_bridge/web`, backend tej nakładki wskazuje na poprawione źródło. Odświeżono tylko most: SIGTERM utknął na zamykaniu WS, zakończono zawieszony PID 9460, launch respawn odtworzył PID 17337. Właściciel CAN bez restartu PID 9458, usługa PID 9419. API i Chromium (tylko odczyt: 0 komend ruchu, 0 zapisów HTTP) potwierdzają nowy frontend, gotowość TCP, postój i brak awarii. Karty operatora wymagają Ctrl+F5; nie trzeba ponownie restartować usługi. Szczegóły dopisano do `docs/tcp_jog_report_2026-10-26.md`.
- 26.10.2026: operator wcześniej aktywował `install/tcp-settings-ros` i uzupełnił cztery limity TCP (100 mm/s, 100 °/s, 100 mm/s², 100 °/s²). Na wyraźną zgodę wykonano próbę 1: +X/Baza, 10 mm przy 2 mm/s, maks. 10 s. Generator przerwał przy 3,820 mm z powodu anulowania obliczeń w budżecie 7 ms; FK enkoderów: +3,682/−0,047/+0,071 mm, postój potwierdzony, bez awarii CAN. Wały według operatora rozsprzęgnięte; FK nie jest pomiarem przestrzennego TCP.
- Poprawka ENABLE: końcowe F3 J1→J6 co najmniej co 200 ms, z potwierdzeniem 3A poprzedniej osi i przerwaniem po błędzie/STOP. Dodano dwa parametry dokładności IK TCP (m/rad), zmienne tylko na postoju; watchdogi i limity mechaniczne bez zmian. Domyślne tolerancje nadal 1e-9 m / 1e-8 rad.
- Build Release trzech zmienionych pakietów (`mks_servo_can`, `robot_kinematics`, `robot_can_diagnostics`) w `install/tcp-jog-fix-ros` OK; colcon 176/0 błędów. Offline na rzeczywistym modelu/startowej pozycji ukończono 12 impulsów 10 mm (X/Y/Z × 4 tolerancje). Nie wykonywano rzeczywistego przełączania ENABLE. Raport: `docs/tcp_jog_report_2026-10-26.md`, trwałe dane: `docs/tcp_jog_test_data_2026-10-26.json`.
- Operator aktywował poprawkę z 26.10; systemd potwierdza `install/tcp-jog-fix-ros` (PID 9419). Osobno zatwierdzone próby po poprawce ukończyły pełne impulsy przy 2 mm/s: 2) +Y 40 mm, IK 1e-8/1e-7, FK −0,040/+39,741/−0,070 mm, postój 20,314 s; 3) +Z 10 mm, IK 1e-7/1e-6, FK −0,036/−0,227/+9,897 mm, postój 5,237 s; 4) +X 10 mm, IK 1e-6/1e-5, FK +9,972/−0,289/−0,375 mm, postój 5,214 s. Bez awarii CAN. Zakończono na 4 próbach z limitu 6; nie uruchamiać kolejnych bez nowych opisów i zgody. Końcowa norma odchylenia poprzecznego w próbie +X: 0,474 mm; nie jest to odbiór dokładności TCP przy sprzęgniętym mechanizmie.
- Ostatnio sprawdzone tolerancje IK 1e-6 m / 1e-5 rad są aktywne w ROS i zapisane w `config/local/execution.production.yaml` na kolejne restarty; odczyt końcowy potwierdził zgodność YAML/ROS, postój sześciu osi, ENABLE i brak awarii. Nie restartowano usługi ponownie. Watchdogi i końcowa tolerancja przegubów ±0,5° bez zmian. Maksymalna chwilowa norma odchylenia poprzecznego z próbek FK w próbie +X: 1,081 mm; końcowa 0,474 mm. Raport zawiera cztery próby, dane przed/po i wykresy. Domyślne wartości biblioteki/węzła nadal 1e-9 m / 1e-8 rad.
- 25.10.2026: zaimplementowano fizyczny JOG TCP LIVE (Baza/Narzędzie, 12 kierunków, ciągły/impulsowy z przytrzymaniem). Generator/hamowanie w `robot_kinematics`, adapter przy `BusRuntime`, sesja/generacja i watchdog IO, ROS `JogTcp`/`TcpJogStatus`, istniejący WebSocket i panel. Wymagane 6 skalibrowanych osi, jawny TCP i fizyczne limity; domyślnie wyłączony.
- Build 9 pakietów w `install/tcp-jog-ros` i frontend OK; 159 GTest (colcon 178/0 błędów), 69 Python, 140 TS oraz Chromium LIVE i regresja planowania OK. Pięć kolejnych integracji TCP przeszło. Wyłącznie atrapy; bez fizycznego CAN/ruchu. `docs/tcp_jog.md` zawiera parametry, sprawdzenia i aktywację nakładki.
- 25.10.2026: dodano cztery fizyczne limity TCP do Konfiguracja → Parametry robota (mm/s, °/s, mm/s², °/s²). Snapshoty bez tych pól migrują do null; aktywna konfiguracja przelicza wartości na SI i aktualizuje parametry ROS. Dodano dynamiczną obsługę zezwolenia/limitów w adapterze; brak danych nadal blokuje LIVE.
- Sterowanie ręczne ma „Załącz wszystkie” / „Wyłącz wszystkie”: jedna usługa `SetAllMotorsEnable`, przygotowanie kolejnych wykrytych osi, końcowe F3 we wspólnym cyklu IO i potwierdzenie świeżymi odczytami 3A. Brakujące osie są pomijane; błędy/STOP blokują kontynuację. Testy wyłącznie na atrapach, bez rzeczywistych poleceń ENABLE.
- Nowy build 9 pakietów w `install/tcp-settings-ros` oraz frontend OK; 163 GTest (colcon 183/0 błędów), 72 Python, 141 TS i trzy testy Chromium konfiguracji/zbiorczego ENABLE OK. Frontend nowej nakładki zapisany jako samodzielne pliki, bez zmiany aktywnego `web/dist`.
- W lokalnym `config/local/execution.production.yaml` zapisano `tcp_jog_enabled: true`. Nowa nakładka czeka na aktywację: `sudo -n install` wymaga hasła, nie zainstalowano nowego drop-in ani nie restartowano usługi. Aktywna pozostaje wcześniejsza `install/tcp-jog-ros` z flagą TCP false. Gotowy `system/application/tcp-settings-ros.conf.example` i komendy w `docs/tcp_jog.md`. Przed próbą wdrożenia potwierdzono postój sześciu osi przez odczyty; bez ruchu. Pozostaje uzupełnienie zatwierdzonych limitów/geometrii i osobno autoryzowany odbiór sprzętowy.
- 25.10.2026: poprawiono sztuczne przerwy złotego śladu PTP/LIN w `web/`: spójny stan i znacznik odbioru przed klatką sceny, deduplikacja przed oceną wieku pozycji, zmiany maski osi dopiero dla zapisywanej próbki.
- Weryfikacja: trzy nowe scenariusze regresji odtworzyły błąd przed poprawką; po niej `npm run build`, 132 testy jednostkowe i `trails.browser.cjs` OK. Chromium używało wyłącznie atrap HTTP/WebSocket; bez ruchu robota.
- Frontend wdrożony do aktywnej nakładki `install/motion-control-ui-ros/robot_web_bridge/share/robot_web_bridge/web`; HTTP GET potwierdził nową stronę i zasoby. Bez restartu usługi; otwarte karty wymagają odświeżenia. Szczegóły: `docs/robot_preview.md`.
