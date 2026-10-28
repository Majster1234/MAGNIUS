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

- 26.10.2026: dodano automatyczne wyłączenie ENABLE po 60 min bez zakresu ruchu >2° na dowolnym wale J1–J6 (enkoder silnika, bez przełożenia). Ruch dowolnej osi, aktywne sterowanie/operacja, brak świeżych danych, zmiana ENABLE i przerwa obserwacji >250 ms zerują odliczanie. Po upływie czasu istniejąca sekwencja wyłączania wszystkich wymaga stabilnego postoju i potwierdzeń 3A, z odstępami ≥200 ms; nie odcina aktywnego JOG/programu. Brak samoczynnego ponownego ENABLE; przygotowany program unieważniany. Powód trafia do stanu ręcznego/historii Centrum komunikatów. Funkcja niezależna od reguły awarii krytycznej. Build tylko `mks_servo_can` Release w `install/tcp-jog-fix-ros` OK, 13 celowanych testów na atrapach/zegarze wirtualnym OK. Backend wymaga restartu usługi przy potwierdzonym postoju; bez restartu, resetu CAN i komend do sprzętu w tej zmianie. Szczegóły: `docs/tcp_jog.md`.

- 26.10.2026: dodano domyślnie włączoną regułę `safety.disable_enable_on_critical_fault` w Parametrach robota (boolean, stare snapshoty→true). Krytyczna awaria właściciela/generatora rozsyła F3 ENABLE=0 do CAN ID 1–6 bez oczekiwania na gotowość/ACK/200 ms; błąd jednej wysyłki nie blokuje kolejnych. Kolejka operacji jest anulowana, brak wznowienia; wysłanie nie potwierdza wyłączenia ani postoju. Zwykły STOP operatora, anulowanie programu i planowa dezaktywacja nie uruchamiają reguły; awaryjne przerwanie TCP przez panel/most uruchamia. Przy streamingu F3 ENABLE=0 jest ostatnią komendą po F7/F6(0). Opcja stosowana transakcyjnie z profilem, rollback przywraca poprzednią; nie zmienia kalibracji. Build Release 6 pakietów (interfejsy, mks, diagnostyka, konfiguracja, hardware i planner) w `install/tcp-jog-fix-ros` oraz frontend OK. Celowane testy: 12 C++, 2 Python i 7 TS OK; lokalna serializacja nowego typu ROS OK. Frontend wdrożony, reguła backendu wymaga restartu całej usługi przy potwierdzonym postoju, potem Ctrl+F5. Bez testów fizycznych, resetu CAN, restartu i poleceń do napędów. Szczegóły: `docs/tcp_jog.md`.

- 26.10.2026: rozszerzono komunikaty zatrzymania CAN/JOG/TCP/PTP o J1–J6 i CAN ID, konkretny warunek (pozycja/ENABLE/stall/tryb/uzbrojenie), wiek pomiaru, próg i uchyb. Stan ręczny/TCP przekazuje pierwotny powód właściciela zamiast starszego opisu postoju. Centrum komunikatów pokazuje powód bez rozwijania szczegółów i osie w tytule awarii. Build Release `mks_servo_can` + `robot_can_diagnostics` do `install/tcp-jog-fix-ros` oraz frontend OK; frontend wdrożony do aktywnej nakładki (Ctrl+F5), backend wymaga restartu usługi przy potwierdzonym postoju. Na żądanie operatora bez uruchamiania testów, odczytów sprzętu, resetu CAN i restartu. Instrukcja: `docs/tcp_jog.md`.

- 26.10.2026: dodano „Reset CAN” obok „STOP robota” w Sterowaniu ręcznym, z istniejącym POST `/api/v1/can/reset` i blokadą podczas operacji/aktywnego sterowania. Build frontendu OK; pliki wdrożono do aktywnej nakładki `install/tcp-settings-ros/robot_web_bridge/share/robot_web_bridge/web`, bez restartu usługi. Zgodnie z żądaniem operatora nie uruchamiano testów ani rzeczywistego resetu CAN. Otwarty panel wymaga Ctrl+F5.

- 26.10.2026: **wykorzystano 6/6 prób fizycznych; odbiór dokładności TCP niezaliczony**. Nie wykonywać kolejnych ruchów bez nowego etapu, pełnej karty i osobnej zgody. Ruch fizyczny wyłącznie przyciskami panelu operatorskiego, bez bezpośrednich poleceń skryptami WS/ROS/CAN.
- Próba 5: Baza +X/10 mm, 2 mm/s, pełna referencja i postój pomiarowy (±0,10°, dwa świeże zestawy ≥100 ms, 0 RPM), bez awarii właściciela. FK: ΔX/Y/Z +9,771/−0,107/+0,276 mm; końcowa norma YZ 0,296 mm, maksimum raw 0,927 mm, sync 1,033 mm — cele <0,10/<0,25 mm niespełnione. Inna pozycja początkowa niż w Próbie 4; procentów nie traktować jako izolowanego efektu kodu.
- Próba 6: zatwierdzony powrót Baza −X/10 mm przerwany po 49 ms i 0,008503 mm referencji. Właściciel: „Brak świeżej pozycji/ENABLE lub awaria napędu”; ostatni J3 miał 56,132 ms i nieważną prędkość, rozrzut RX 52,072 ms. Wskazuje to na świeżość, ale predykat obejmuje też ENABLE/stall/tryb. Nie było przekroczenia budżetu IK.
- **Stan końcowy:** awaria CAN zatrzaśnięta, brak świeżych pomiarów prędkości sześciu osi i brak zwalidowanego domknięcia. Operator potwierdził fizyczny postój wszystkich napędów. Nie resetowano awarii ani nie ponawiano ruchu/ENABLE. Komunikat ręczny „Zatrzymanie potwierdzone” nie zastępuje świeżych pomiarów po tej awarii.
- Aktywna nakładka `install/tcp-jog-fix-ros`; operator wykonał restart usługi (MainPID 24929), następnie przygotował napędy przyciskiem „Załącz wszystkie”. Cztery zmienione pakiety zbudowane jawnie Release: `mks_servo_can`, `robot_kinematics`, `robot_motion_planner`, `robot_can_diagnostics`; 204 wyniki colcon, 0 błędów/porażek/pominięć. Bez budowania web w tym etapie.
- Parametry produkcyjne zapisane w `config/local/execution.production.yaml` i sprawdzone w ROS przed próbami: `tcp_jog_enabled=true`, IK 1e-6 m / 1e-5 rad, `tcp_jog_max_jerk=0.5` m/s³, `joint_stop_tolerance_deg=0.10`. Fizyczne limity TCP 100 mm/s, 100 °/s, 100 mm/s², 100 °/s². Nie zmieniano geometrii, kalibracji ani limitów mechanicznych; watchdogi pozostają aktywne.
- Generator TCP ma analityczny profil o ograniczonym jerku i dokładnym końcu drogi; streaming TCP osobno wyznacza FF/korekcję RPM każdej osi. Telemetria 100 Hz zapisuje zatwierdzoną referencję IO/F5, q_raw/q_sync, FK, RX i czas obliczeń. Ekstrapolacja ±20 ms, fallback przy wieku >50 ms/nieważnej prędkości. Domyślne tolerancje biblioteki IK nadal 1e-9 m / 1e-8 rad.
- Offline na modelu i pozycji startowej Próby 4: 514 próbek, dokładne 10 mm, rozbieg 140 ms/14 cykli, maks. czas próbki 0,192075 ms <7 ms. Brak pomiarów referencji/IK/CPU w starym logu 4 ogranicza porównanie przed/po. Próby przy rozsprzęgniętych wałach nie mierzą przestrzennego TCP.
- Trwałe pełne dane 1–6: `docs/tcp_jog_test_data_2026-10-26.json`; raport `docs/tcp_jog_report_2026-10-26.md`; porównanie i ograniczenia `docs/benchmark_results.md`. Próba 6: `stop_confirmed=false`, osobne potwierdzenie operatora, zwalidowane domknięcie `null`.
- Wcześniej poprawiono panel: walidowane `velocity`/`step` jawnie float przed typowanym ROS (JSON `0` powodował AssertionError); opis awarii pokazuje przyczynę właściciela. Build mostu, 142 TS i Chromium WS→typ ROS na atrapach przeszły. Frontend aktywny w `install/tcp-settings-ros/robot_web_bridge/share/robot_web_bridge/web`; szczegóły w raporcie.
- Zbiorcze ENABLE: końcowe F3 J1→J6 co najmniej co 200 ms, kolejny po świeżym 3A poprzedniego; błędy/STOP blokują kontynuację. Jedno IO SocketCAN, wykluczenie sterowań osi/TCP/programu. Konfiguracja panelu udostępnia cztery fizyczne limity TCP; brak wartości blokuje LIVE.
- Historia: Próba 1 +X przerwana przy 3,820 mm przez budżet solvera; 2 +Y/40 mm, 3 +Z/10 mm i 4 +X/10 mm ukończone po osobnych zgodach, z postojem. Poprzednia tolerancja postoju ±0,5°. Naprawa ciągłości złotego śladu 3D PTP/LIN opisana w `docs/robot_preview.md`; nie zmieniano jej w obecnym etapie.
