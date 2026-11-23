# Audyt przed uproszczeniem PTP — 24.10.2026

Poniższy audyt opisuje stan **przed** wdrożeniem podstawowego PTP z ACK i
oczekiwaniem na pozycję. Późniejszy aktualny kontrakt100 Hz bez ACK ruchu: [motion_execution.md](motion_execution.md).
Zniesienie bramek odbioru jest decyzją projektu, nie zaliczeniem historycznych prób.

Przegląd: **2026-10-24**, źródła repozytorium, wyniki sprzętowe 21–22.10
oraz pasywny odczyt API i parametrów ROS o 12:33–12:34 CEST.
Nie wykonano ruchu, restartu, zmian parametrów ani nowych testów sterowania.
Wcześniejsze testy opisane w raportach nie są testami tej sesji.

**Ocena:** ścieżka oprogramowania istnieje, lecz produkcyjny wykonawca nie ma
pełnego odbioru sprzętowego. Osiągnięcie celu jednej osi w wariancie stanowiskowym
nie potwierdza koordynacji, dynamiki ani gotowości sześcioosiowego robota.

## Stan potwierdzony

| Warstwa | Co istnieje | Granica potwierdzenia |
| --- | --- | --- |
| Panel i API | Przygotowanie odrębnego fizycznego planu, podgląd, zatwierdzenie, wykonanie, status i STOP; klucz chroniący przed powtórzeniem | Są testy na atrapach; pełna ścieżka z fizycznym robotem nieodebrana |
| Planner ROS | Plan PTP od pomiarów; q/v/a i wspólny czas; kontrola rewizji modelu/konfiguracji | Model i parametry muszą odpowiadać mechanice |
| Kompilator MKS | Dobór `native_ptp` albo kompletny bufor `streamed`; limity i kwantyzacja | Dopasowanie modelu profilu do firmware wymaga pomiarów |
| Wykonawca CAN | Jeden właściciel magistrali; F5/ACK/wspólny SYNC; nadzór i zatrzymywanie | Testy emulatora nie odwzorowują wszystkich zachowań napędów |
| Integracja ros2_control | Zwolnienie JTC, przejęcie programu, ochrona przed starymi nastawami | Wymaga odbioru przejść trybów i odzyskiwania stanu na sprzęcie |
| Sprzęt | Próba 5: CAN 1, +10° wału, końcowe 9,90967°, RPM=0 | Izolowany wariant, wyłączony nadzór przejściowego błędu; niezaliczona płynność i brak prób 2/4/6 osi |

Stan uruchomionej usługi: `execution_enabled=false`, trzy flagi odbioru `false`,
rewizja wykonania pusta, sześć tolerancji i kodów acc ma wartość `-1`.
Program: `idle`, bez fault/busy. Dostępne J1/J2/J3/J5 mają ENABLE, przygotowanie
i ważną kalibrację; J4/J6 bez odczytu. Włączone są osobno ignorowanie braków
w symulacji i dopuszczenie niepełnego zestawu do ruchu. Nie włączają wykonawcy.

## Blokady w bieżących źródłach

1. **Sukces PTP może wywołać fałszywy fault.** `schedule_program()` kasuje
   `source_`, po czym ta sama iteracja `tick()` porównuje pozycję ze starym
   `committed_positions_` JTC. Poprawka pozostała w narzędziu stanowiskowym.
2. **STOP i bufor nie mają pełnego odbioru.** F7 nie skasowało celu w próbie
   21.10. Sekwencja F7 → F6(0) dała ograniczony wynik na jednym wale 22.10.
   `stop_reply()` uznaje tylko `01`; zaobserwowane `05` nie jest dowodem postoju.
   Trzeba rozdzielić odebranie rozkazu, rzeczywisty postój i usunięcie starego celu.
3. **Pomiar prędkości/postoju wymaga ujednolicenia.** Uzbrajanie wymaga dokładnego
   `32=0`, chociaż na stanowisku odczyt zachowywał −1 RPM przy postoju.
   Należy oprzeć decyzję na zweryfikowanym zestawie świeżych pomiarów, F1
   i stabilności pozycji; nie wystarczy usunąć warunku wartości prędkości.
4. **Dynamika niepotwierdzona.** Różnica pozycji 1,56006°/41,0923 ms odpowiada
   estymacie 6,33 RPM przy nastawach ≤1 RPM. To nie rozstrzyga między ruchem
   a aktualizacją/próbkowaniem danych. Potrzebna jest weryfikacja niezależna.
5. **Wariant czasowy nie trafił do produkcji.** Źródła nadal mają SYNC +1 ms
   i koniec diagnostyki 18 ms; udana próba używała 2,5 ms/19 ms. Jeden przebieg
   nie wyznacza granicznych opóźnień dla sześciu osi i obciążonej usługi ROS.
6. **Lokalny YAML nie jest konfiguracją do pełnego wdrożenia.**
   `config/local/execution.yaml` włącza flagi dla starej rewizji, różnej od aktywnej.
   Parametry `execution_goal_tolerance_rad` i `execution_tracking_delay_ms`
   nie mają obsługi w źródłach. Kod sprawdza śledzenie od początku, a tolerancję
   końcową wylicza jako `tracking_error/4`; przy lokalnym 0,5° daje to 0,125°,
   a nie zadeklarowane w pliku 0,2°. Ten plik nie jest obecnie zastosowany.

## Kolejność dokończenia

| Etap | Praca | Warunek przejścia dalej |
| --- | --- | --- |
| 1. Kryteria i konfiguracja | Ustalić dla osi dopuszczalne błędy w ruchu/na końcu, dynamikę, synchronizację i reakcję STOP. Uzgodnić obsługiwane parametry, aktywną rewizję i zakres odbioru | Jednoznaczne kryteria i brak pozornie działających pól YAML |
| 2. Regresje wykonawcy | Naprawić przejście po sukcesie; dodać scenariusz niezerowego celu oddalonego od starych nastaw JTC, pozostania w postoju i kolejnego PTP. Rozdzielić diagnostykę F6=05/odmowa/timeout | Sukces nie staje się fault; ACK nie udaje pomiaru zatrzymania |
| 3. STOP/bufor na jednej osi | Odbiór przed startem, podczas ruchu i przy częściowym zestawie ACK; późniejszy kontrolowany SYNC tylko w osobnej zatwierdzonej próbie | Brak wykonania starego celu, zmierzony postój i droga zatrzymania; powtarzalny odzysk po błędzie |
| 4. Sprzężenie i dynamika | Wyjaśnić skoki enkodera, świeżość `32`, wpływ kolejnych F5 na rampy; porównać pomiary z niezależną obserwacją wału | Potwierdzone rzeczywiste prędkości/przyspieszenia i śledzenie w całym ruchu |
| 5. Harmonogram produkcyjny | Dobrać okna na podstawie RTT, opóźnienia obsługi RX, jitteru IO i obciążenia. Zmierzyć przed/po te same zmieniane cykle i CPU, również z rejestracją/statusami | Marginesy dla docelowej liczby osi; zachowany nadzór i brak nadrabiania próbek. Wyniki w benchmark_results.md |
| 6. Odbiór 1 → 2 → 4 → 6 osi | Produkcyjny kod i nadzór; różne drogi/kierunki, oś nieruchoma, małe prędkości, dłuższe ruchy, powtórzenia. Odbierać oba dostępne tryby albo jawnie zablokować nieodebrany | Zmierzona koordynacja przebiegu, startu i końca, dokładność i płynność; nie tylko komplet ACK |
| 7. ROS/panel/awarie | Cały przebieg prepare → execute → settling → succeeded; JOG/PTP/JTC, rewizje i powtórzenia żądań; brak ACK, USB/BUS-OFF, utrata osi, watchdog i restart procesów | Brak konkurencyjnego właściciela CAN, niezamierzonego ruchu i automatycznego wznowienia |
| 8. Robot i uruchomienie | Potwierdzić J1–J6, przekładnie/kierunki/offsety/bazowanie, limity, geometrię/TCP i warunki zatrzymania pod obciążeniem; wdrożyć odebrany build i konfigurację | Dopiero wtedy zgodna rewizja, wartości sześciu osi, udokumentowane flagi odbioru i `execution_enabled=true` |

Podczas etapu 7 utrata LAN lub zamknięcie przeglądarki ma **zachować wykonanie
już zatwierdzonego programu** zgodnie z obecnym kontraktem. Osobno sprawdza się
STOP operatora, awarię bridge oraz awarię właściciela CAN. Nie utożsamiać tych zdarzeń.
Standalone i CanSystem są alternatywnymi właścicielami CAN; uruchomienie PTP
nie wymaga dołożenia drugiego nadającego procesu. Powrót do JTC musi być jawny.

Testy wieloosiowe obejmują wspólny postęp trajektorii, nie tylko jednoczesne
wyzwolenie SYNC. PTP odbywa się w przestrzeni przegubów; prosta droga TCP,
fizyczny LIN i blending pozostają odrębnymi funkcjami. Ich implementacja nie jest
warunkiem zakończenia wdrożenia pojedynczego koordynowanego PTP.
Odbiór modelu/TCP nie zastępuje oceny kolizji ani fizycznego zatrzymania robota.

Każda nowa próba ruchu wymaga ustalonego zakresu i zgody użytkownika według
AGENTS.md. Ten przegląd nie zatwierdza prób ani zmiany flag wykonania.

## Źródła

- [Wykonanie PTP](motion_execution.md), [raport sprzętowy](motion_hardware_results.md),
  [progi czasowe](ptp_timing_limits.md), [benchmarki](benchmark_results.md).
- [Program CAN](../ros2_ws/src/mks_servo_can/src/program_bus.cpp),
  [manager i STOP](../ros2_ws/src/mks_servo_can/src/bus_manager.cpp),
  [kompilator](../ros2_ws/src/mks_servo_can/src/motion_executor.cpp),
  [narzędzie wariantów](../scripts/build_ptp_limits.py).
- [API ROS wykonawcy](../ros2_ws/src/robot_can_diagnostics/src/program_api.cpp),
  [mostek](../ros2_ws/src/robot_web_bridge/robot_web_bridge/motion.py),
  [CanSystem](../ros2_ws/src/robot_hardware/src/can_system.cpp),
  [panel](../web/src/MotionExecution.tsx).
