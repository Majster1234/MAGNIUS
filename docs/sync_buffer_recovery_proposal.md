# Odzyskiwanie bufora SYNC po STOP

**Status 29.10.2026: pełny odbiór sprzętowy nadal niezaliczony.** Po oddzieleniu
przejść ENABLE J1–J3, J5 i J6 przeszły po dwa nadpisania znanych buforów oraz
późniejsze SYNC i ponowne ENABLE. J4 nie zaliczyła kontroli dodatniej.
Odczyty: 337059 zakończonych zapytań bez timeoutów; ustawiono 24 ms (+20%).
Nowy właściciel ma nieblokujące odstępy 6 ms między seriami STOP i wymaga
wdrożenia osobnej nakładki. Później, na jawne polecenie operatora, gotowe pakiety
wdrożono do aktywnej nakładki i włączono automatyczne zdjęcie blokady mimo
niezaliczonego J4. Bez nowych testów; nie zmienia to wyniku odbioru sprzętowego.
Po wdrożeniu stan to `idle_disabled`, `fault=false`, ENABLE=0 wszystkich osi.
[Powtórne próby i czasy](sync_recovery_timing_2026-10-29.md),
[pierwszy raport](sync_recovery_hardware_2026-10-29.md).

## Podstawa protokołu

Instrukcja [MKS SERVO42/57D CAN V1.0.9](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf)
w §11.4.2 opisuje F5 z prędkością, przyspieszeniem i pozycją równymi zero jako
natychmiastowy STOP. Biblioteka udostępnia dla niego osobny konstruktor; zwykłe
F5 ruchu nadal odrzuca zerowe RPM. W trybie synchronicznym status `05` oznacza
przyjęcie polecenia, a nie postój ani odczyt zawartości bufora.

## Sekwencja aplikacji

Globalny STOP unieważnia generacje sterowania, program, sesję JOG oraz wszystkie
oczekujące callbacki. Dla każdej skonfigurowanej osi wysyła F7, udokumentowane
F5 STOP i — przy STOP operatora albo włączonej regule awarii krytycznej —
`F3 ENABLE=0`. Niepowodzenie jednej osi nie przerywa wysyłek do pozostałych.
Nie jest wysyłane `4B`.

Nieblokująca maszyna stanów wykonuje:

`stopping → verifying_disabled → overwriting → restoring → verifying → idle_disabled`

Po świeżym potwierdzeniu `ENABLE=0`, F1=postój, 0 RPM i stabilnej pozycji:

1. ustawia i odczytuje `8C=01/00`;
2. odczytuje `4A`; tylko przy `4A=1` wysyła neutralne F5 STOP i wymaga `05`;
3. po sprawdzeniu bramki odbioru sprzętowego ustawia i odczytuje `4A=0`;
4. ustawia i odczytuje `8C=00/00`;
5. ponownie wymaga dwóch zestawów pozycji oddalonych o co najmniej 100 ms,
   zmiany nie większej niż 0,5°, świeżego `ENABLE=0`, F1 i 0 RPM.

Jeżeli podczas sekwencji widziano `4A=1`, końcowe odblokowanie wymaga parametru
`execution_staged_reset_verified=true`. Normalnym warunkiem ustawienia jest
pozytywny odbiór sprzętowy; 29.10 operator jawnie zezwolił na wyjątek mimo J4
i parametr włączono. Sama nazwa flagi nie dowodzi odbioru. Przy fladze false procedura zatrzymuje się
przed zmianą `4A=0`, pozostawiając `SYNC=1` jako widoczny znacznik blokady
również po restarcie procesu. Ponowny STOP i reset komunikacji nie omijają
bramki. Przy `4A=0` zwykłe odzyskanie po STOP może
zakończyć się bez tej bramki. Nigdy nie następuje automatyczne ENABLE ani
wznowienie poprzedniego ruchu.

Gdy reguła odcinania ENABLE przy awarii krytycznej jest wyłączona, procedura
czeka na jawne wyłączenie napędów przez operatora. Samo odzyskiwanie nie odcina
wtedy momentu. Globalny STOP operatora zawsze wysyła ENABLE=0.

Brak lub odrzucenie odpowiedzi przechodzi do `recovery_failed`. Odczyty nadal
działają, a ruch pozostaje zablokowany. Jeśli nie potwierdzono zatrzymania lub
wyłączenia ENABLE, dla niepotwierdzonej osi z odczytanym,
niezerowym watchdogiem stosowana jest jedna przerwa `watchdog + 50 ms`, po czym
diagnostyka wraca. Nieznany, zerowy lub nieaktualny watchdog nie jest traktowany
jako dowód zatrzymania.

## Diagnostyka i reconnect

Awaria ruchu nie zatrzymuje RX. Właściciel nadal odpytuje identyfikację, pozycję,
prędkość 32H, ENABLE, stall, F1, SYNC i watchdog w dotychczasowym limicie 40%
magistrali. Stan online i ważność pomiarów są publikowane niezależnie od blokady
ruchu. Utrata transportu uruchamia ponowne otwieranie po 0,5/1/2/5 s, następnie
co 5 s; powrót łącza rozpoczyna odzyskiwanie od nowa, bez ENABLE i bez ruchu.

Stan ręczny ROS/JSON zawiera `recovery_state`, `recovery_message` i
`recovery_pending_mask`. Panel rozróżnia STOP, wyłączanie ENABLE, potwierdzenie
postoju, niepowodzenie oraz gotowość do jawnego ponownego załączenia. W awarii
operator może wysłać wyłącznie `ENABLE=0`; załączenie i pozostałe operacje są
zablokowane.

## Odbiór sprzętowy

Przed każdą próbą obowiązuje karta osi, kierunku, prędkości, czasu/zakresu i
warunku STOP oraz osobna zgoda operatora zgodnie z `AGENTS.md`. Na rozsprzęgniętych
wałach i firmware 1.0.9 należy dla J1–J6 zarejestrować wszystkie ramki, pozycję,
RPM i ENABLE oraz sprawdzić:

- przyjęcie neutralnego STOP i brak ruchu bez `4B`;
- zachowanie oczekującego rozkazu przy przejściu `4A=0`;
- późniejsze kontrolowane `4B` i ponowne ENABLE, aby wykluczyć stary ruch;
- odrzucenie F5, utratę osi, STOP na każdym etapie i powrót transportu.

Nieoczekiwany ruch, niepełne pomiary albo brak jednoznacznego potwierdzenia
blokują ustawienie `execution_staged_reset_verified=true`.

Rejestrator incydentu ma ograniczone okno i rotację plików. Odbiór wymaga pełnego
zrzutu CAN obejmującego całą próbę, również późniejsze wyzwolenia i ponowne ENABLE.

## Wyniki offline — 29.10.2026

- Release: `robot_control_interfaces`, `mks_servo_can`, `robot_can_diagnostics`,
  `robot_hardware`, `robot_motion_planner`, `robot_web_bridge` — build OK.
- Celowane testy pięciu pakietów ROS/C++: **254 wyniki, 0 błędów, 0 porażek,
  0 pominięć**. Licznik obejmuje opakowania CTest; nie oznacza 254 prób sprzętu.
- 11 testów odzyskiwania obejmuje STOP na każdym etapie, niezależny rozkaz
  aktywny/oczekujący, neutralny STOP bez zmiany pozycji na zero, firmware
  odrzucający F5 przy ENABLE=0, brak/spóźniony/uszkodzony ACK, niezgodny odczyt
  przywracanych ustawień, watchdog, utratę osi, reconnect i stare generacje.
  Dodatkowo przechodzi 25 testów wykonawcy synchronicznego oraz regresja zwykłego
  streamingu, JOG, konfiguracji, kalibracji i wyłączania ENABLE.
- Most HTTP/WS/ROS: **69 testów OK**, w tym przekazywanie etapu odzyskiwania i
  maski osi oraz zachowanie pomiarów przy niezależnej blokadzie ruchu.
- Frontend: build i typecheck OK; **144/145 testów TS**. Zastany błąd
  `controlMessages.test.ts` (nieznany komunikat #42) nie dotyczy odzyskiwania.
  Chromium na atrapach: etapy, maska J3, odczyt pozycji podczas awarii, blokada
  załączenia/JOG, dostępne ENABLE=0 i Reset CAN, gotowość po zakończeniu oraz
  układ 1280/1366/1920 px — OK.

Powyższy etap był wyłącznie offline, bez pomiaru wydajności przed/po. Późniejsze
próby fizyczne opisano w [raporcie sprzętowym](sync_recovery_hardware_2026-10-29.md).
Zgodność modelu bufora i skuteczność całej sekwencji nadal nie mają odbioru.
Lokalne logi: `/tmp/stop-recovery-build-final.log`,
`/tmp/stop-recovery-tests-final.log`, `/tmp/stop-recovery-results-final.log`.
Wyniki przypadków znajdują się także w `build/stop-recovery-ros/*/test_results/`.

## Przygotowanie wdrożenia

Nakładka Release: `install/stop-recovery-ros`. Zawiera nowe typy ROS, bibliotekę
MKS, diagnostykę, adapter hardware, planner i most z frontendem. Przykładowy
drop-in: `system/application/stop-recovery-ros.conf.example`. Przy potwierdzonym
postoju operator może zastosować:

```bash
cd /home/mk/Desktop/Projekty
sudo install -d /etc/systemd/system/robot-web-status.service.d
sudo install -m 0644 system/application/stop-recovery-ros.conf.example /etc/systemd/system/robot-web-status.service.d/tcp-settings.conf
sudo systemctl daemon-reload
sudo systemctl restart robot-web-status.service
```

Następnie Ctrl+F5. `execution_staged_reset_verified: false` jest jawnie zapisane
w `config/local/execution.production.yaml`. Nakładka została już aktywowana
przez operatora przed próbami 29.10. Podczas prób zakończono tymczasowo
właściciela CAN i kontrolowanie odtworzono usługę. Stan końcowy: odczyty sześciu
osi działają, ENABLE=0, SYNC=1 i `recovery_failed`. Nie należy ponawiać ENABLE
w ramach opisanego odbioru ani traktować samego restartu jako usunięcia bufora.
