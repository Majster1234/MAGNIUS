# AGENTS.md — instrukcje operacyjne dla AI

Zakres: całe repozytorium. Źródłem prawdy jest kod. Komunikacja i dokumentacja: język polski (zachowuj angielskie nazwy w kodzie/API).

## 0. Programy ruchowe — szybki start dla agenta (pisanie skryptów robota)

Edytor „⌖ Programowanie” (`#programming`) zapisuje programy jako JSON. Agent pisze program jako plik/JSON i wysyła
przez API; wykonuje `scripts/run_program.py` (po zatwierdzeniu). Pełny opis: `docs/programming.md`.

**Gdzie:** `~/.local/state/robot/programs/` → `programs/<id>.json` (programy), `library/commands.json` (rozkazy =
definicje bloków), `library/variables.json`, `library/positions.json` (biblioteki), `programs/.trash/` (kosz).
Backend: `ros2_ws/src/robot_web_bridge/robot_web_bridge/programs.py` (magazyn), `program_runner.py` (wykonawca:
kursor robota i kursor programu), `program_interpreter.py`, `motion_chain.py`; planer ciągu: akcja
`/robot/planning/compute_motion_chain`; frontend: `web/src/ProgrammingPage.tsx`, `programModel.ts`, `programRun.ts`.
API: `http://192.168.1.69:8080` (bez proxy).

**Format programu:** `{"name", "description", "variables": [...], "positions": [...], "blocks": [...]}`
- zmienna: `{"name", "type": INT|REAL|BOOL|POZYCJA|TEKST, "init", "unit"?, "note"?, "min"?, "max"?}` (POZYCJA → `init` = nazwa pozycji);
- pozycja: `{"name", "joints_deg": [J1..J6 w °], "note"?}` — kąty przegubów; nazwy: litera, potem litery/cyfry/_ (≤ 40), unikalne;
- blok: `{"id": unikalny, "command": id rozkazu, "params": {...}, "body"?: [bloki], "else"?: [bloki], "disabled"?: true}`;
- wartość parametru: literał albo `{"var": "nazwa"}` / `{"position": "nazwa"}`.

**Rozkazy (biblioteka budowana od nowa, 09.11.2026):** obecnie tylko ruchy
- `move_ptp`: `{"position": {"position": "P1"}, "speed": 1–100 [%], "blend_mm": 0–200}`;
- `move_lin`: `{"position": …, "speed": mm/s ≤ tcp.max_linear_velocity, "blend_mm": 0–200}`.

`blend_mm` = promień pominięcia punktu na torze TCP: **0 = dojazd i postój w punkcie**, > 0 = płynne przejście do
następnego ruchu (planer może zmniejszyć promień lub zamienić go na postój — uwagi w wyniku sprawdzenia). Zawsze
sprawdzaj aktualną listę w `GET /api/v1/program-library` (`commands.commands`); rozkazy spoza biblioteki backend
odrzuci. Logika wykonania (ciągi ruchów liczone z wyprzedzeniem, kursory): `docs/programming.md` („Logika wykonania”).

**Jak napisać i sprawdzić (bez ruchu, bez zgody):**
1. Pozycje: z pliku/biblioteki albo `GET /api/v1/manual/status` (`axes[].joint_position_degrees`) — nie zgaduj kątów.
2. `POST /api/v1/programs/check {"program": …}` → `problems` (walidacja bez zapisu);
   `python3 scripts/run_program.py --file prog.json` → symulacja w wykonawcy (ciągi ruchów, oś czasu, promienie).
3. Zapis: `POST /api/v1/programs {"program": …}` (nowy, zwraca `id`, `revision`) lub
   `PUT /api/v1/programs/{id} {"program": …, "expected_revision": …}` (409 = konflikt rewizji).
4. `python3 scripts/run_program.py <id>` — sprawdzenie zapisanego programu.

**Wykonanie (ruch fizyczny) — wyłącznie za zgodą operatora:**
- Operator zatwierdza konkretną rewizję w GUI („✔ Zatwierdź do wykonania”); każda zmiana programu unieważnia zatwierdzenie.
  **Agent nigdy nie wywołuje `/api/v1/programs/{id}/approve`.**
- Przed `python3 scripts/run_program.py <id> --execute` agent opisuje ruch (program, pozycje, prędkości, czas z
  sprawdzenia, warunek zatrzymania) i **czeka na wyraźną zgodę w rozmowie** — za każdym uruchomieniem. Skrypt sam
  odmawia wykonania niezatwierdzonej rewizji; Ctrl+C/błąd → STOP programu. Stała zgoda z §5 dotyczy tylko testu jakości TCP.

**Jeśli Twoim zadaniem jest wyłącznie napisanie (lub sprawdzenie) skryptu ruchowego — nie czytaj dalej tego pliku.**
Dalsze sekcje dotyczą rozwoju kodu, sprzętu i testów.

## 1. Dyrektywy efektywności (Krytyczne — zero zbędnych akcji)

- **Zakaz rutynowego skanowania:** NIE sprawdzaj stanu Git, środowiska, drzewa plików ani testów na początku promptu, dopóki zadanie bezpośrednio tego nie wymaga.
- **Selektywny odczyt:** Używaj `rg` (ripgrep) do punktowego szukania w kodzie i `docs/`. Nie czytaj całych plików ani dokumentacji na zapas.
- **Warunkowa aktualizacja tego pliku:** Aktualizuj sekcję `Bieżący kontekst` TYLKO wtedy, gdy w sesji zmienił się kod, konfiguracja lub stan testów sprzętowych.W Agents.md ma zostć tylko kontekst z aktualnego zapytania, nie prowadzimy tutaj historii wykraczającej poza ostatnie zapytanie. Przy pytaniach, analizie i planowaniu **nie edytuj `AGENTS.md`**.
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
  - Regulator i limity: Lookahead 25 ms, FF 1, Kp 4 (od 08.11.2026), dithering per oś, min. prędkość F5/dithering `motion.min_dither_rpm` (domyślnie 0,125 RPM, ≥ 1 jednostka MKS; F5 speed=0 to STOP), max 2500 RPM, acc 135. Limity mechaniczne obowiązują zawsze.
  - Odczyty PTP: wstrzymaj nowe zapytania 4 ms przed F5 (ruch nie czeka na odczyt). Tolerancja końcowa ±0.5° (dwa pomiary >= 100 ms), bez pauz pośrednich.
  - Pasmo CAN (1 Mbit/s, TX 1000): łącznie max 60% (robot 40%, rezerwa 20%).
- **Nastawy sterowników MKS (warunki testów parametrów silnika):**
  - Zmiana wyłącznie przez mechanizm kopii (`docs/drive_parameter_backup.md`). Kontener `base` (nietykalny) musi istnieć; **każda** ingerencja jest poprzedzona automatyczną, świeżą kopią `pre_test` (bez niej zapis blokowany).
  - Jedna oś na raz, adres CAN = numer osi. Zakres nastaw testowych ograniczony do walidowanych granic kontenera `test`; PID w przedziale ustalonym w karcie (domyślnie 0,5–2× wartości `base`). Bez zmiany kierunku, adresu, bitrate, kalibracji i resetu fabrycznego.
  - Weryfikacja obowiązkowa: ten sam zestaw ruchów testowych przed zmianą (linia bazowa) i po zmianie; nastawa zostaje tylko przy lepszym wskaźniku jakości i braku naruszeń twardych kryteriów (oscylacje w postoju, przeregulowanie, nadzór postoju, awaria). W przeciwnym razie — oraz przy każdym przerwaniu — automatyczny powrót do `pre_test` i odczyt kontrolny.
  - Ruch testowy wymaga karty i zgody operatora jak każdy ruch fizyczny (oś, amplitudy, prędkości, czas, zakres nastaw, warunki zatrzymania).
  - Wyniki (linia bazowa, kandydaci, decyzja, sumy SHA kontenerów) zapisywać w `docs/benchmark_results.md`.
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
| `~/.local/state/robot/programs/` | Programy ruchowe (`programs/*.json`) i biblioteka rozkazów/zmiennych/pozycji (`library/`); opis `docs/programming.md` |
| `scripts/`, `system/`, `start_ros2.sh` | Skrypty uruchomieniowe, systemd (SocketCAN i autostart) |
| `temp/` | Propozycje wyglądu GUI: samodzielne pliki HTML z danymi przykładowymi, bez połączenia z API/ROS/CAN. Nowe makiety zapisuj w podfolderze nazwanym ogólnie wg tematu (np. `temp/sterowanie_reczne/`) z własnym `README.md`; zastąpione propozycje usuwaj. Nie są importowane przez `web/` ani wdrażane; wdrożenie wybranej propozycji to osobne zadanie w `web/src`. |

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

## 5. Test jakości ruchu TCP (stała procedura)

- Standardowy test płynności i dokładności planowanego ruchu PTP/LIN: `docs/tcp_motion_quality_procedure.md`,
  skrypt `scripts/tcp_motion_quality.py`, skill `.claude/skills/tcp-motion-quality/SKILL.md`.
- Suchy przebieg (bez ruchu): `python3 scripts/tcp_motion_quality.py`; test: `--execute [--acc <kody>]`.
  12 ruchów (20/400 mm × 5/15/40 mm/s, PTP tam, LIN z powrotem), każdy przebieg od zerowania MKS, ok. 5 min.
- **Stała zgoda operatora (08.11.2026):** dopóki operator nie zgłosi sprzęgnięcia mechanicznego, ten test
  można wykonywać bez osobnej karty (zawsze od zerowania MKS). Po sprzęgnięciu obowiązuje §2 (karta i zgoda).

## Bieżący kontekst

- 10.11.2026 — Programowanie: zmiana rodzaju rozkazu istniejącego bloku (PTP ↔ LIN) z listy po kliknięciu nazwy ruchu, bez usuwania bloku (`ProgrammingPage.tsx` `CommandSwitch`, `programModel.ts` `compatibleCommands`/`changeCommand`: id i parametry o tej samej nazwie zostają, wartość spoza zakresu → domyślna). Test `programModel.test.ts` (8/8), `web/dist` przebudowany (Ctrl+F5, bez restartu usługi); `docs/programming.md` uzupełniony.
