# AGENTS.md — instrukcje operacyjne dla AI

Zakres: całe repozytorium. Źródłem prawdy jest kod. Komunikacja i dokumentacja: język polski (zachowuj angielskie nazwy w kodzie/API).

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

## Bieżący kontekst

- 04.11.2026 — Podgląd 3D: przełącznik „Trace TCP” i przycisk „Kasuj trace” (`RobotPreview.tsx`, `robotPreviewScene.ts`, `previewExecutionTrail.ts`: wspólny `createTrailBuffer`, nowe `createLiveTrail` — błękitna droga TCP z FK pomiarów przy każdym ruchu, ≥ 50 ms i ≥ 0,2 mm, 20 000 punktów, rozcinanie przy niepełnych/nieświeżych pozycjach; złoty ślad PTP/LIN bez zmian); opis `docs/robot_preview.md`. Web: build OK, testy 160/161 (1 błąd w niezmienianym `controlMessages.test.ts`); `web/dist` przebudowany — aktywny po Ctrl+F5, bez restartu usługi. Stan wcześniejszy (bez zmian): KEEPALIVE 40 ms i odczyt parametrów napędów `/robot/can/read_drive_parameters` wdrożone i aktywne od restartu 04.11 16:30 (odczyt: Kp/Ki/Kd/Kv 220/100/270/320 na J1–J6, prąd 1600 mA poza J5 = 200 mA; J4 — silnik NEMA 17 pancake). Wały rozsprzęglone na polecenie operatora.
