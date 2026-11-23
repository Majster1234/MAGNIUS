# JOG TCP: siatka F5 i świeżość pozycji — 30.10.2026

## Objaw

Podczas JOG TCP na różnych osiach i w różnych chwilach:
„Zatrzymanie sterowania: Jn (CAN ID n): pozycja (0x31): nieaktualny odczyt, wiek 60.x ms, wymagany < 60 ms”.

## Przyczyna (potwierdzona zapisem `candump` na sprzęcie)

1. W JOG TCP F5 wychodziło w chwili nadejścia paczki generatora (własny zegar 10 ms), a okno odczytów
   liczono od `tcp_started_`. Zapytanie mogło wyjść ~1 ms przed F5 do tej samej osi. MKS odpowiada tym
   samym CAN ID co host; odpowiedź ginęła lub była spóźniona (stara wersja: 71–102 F5 na oś wysłane przy
   oczekującym odczycie, +105 przejść `can0` w error-passive w jednej rundzie).
2. Jedna utracona odpowiedź (także diagnostyczna 0x3A/0x3E/0x32/0xF1) blokowała adres na
   24 ms (timeout) + 24 ms (kwarantanna). Pozycja osi nie była wtedy odpytywana i jej wiek przekraczał 60 ms.
   Przykład: 0x3A do J3 w 120,00 ms, F5 do J3 w 121,02 ms, brak odpowiedzi, awaria J3 przy wieku 60,366 ms.
3. Rozjazd faz generatora i właściciela dawał 0 albo 2 paczki w cyklu; prędkość F5 z różnicy referencji
   dzielonej przez stałe 10 ms była wtedy podwójna.

## Zmiany

- `mks_servo_can`: F5 JOG TCP tylko w pierwszych 2 ms slotu 10 ms od startu sesji i tylko bez oczekujących
  odpowiedzi osi robota; odczyty po F5, cisza ≥ 4 ms przed kolejnym F5 (`kTcpSendWindow`, `kTcpReplyMargin`).
- W slocie TCP termin odpowiedzi kończy się 1 ms przed kolejnym F5, bez kwarantanny; unieważnienie stanu osi
  dopiero po ciszy równej `feedback_timeout` (60 ms = 6 slotów). Limit 60 ms bez zmian.
- Zaległa pozycja ma pierwszeństwo przed diagnostyką tej osi.
- Prędkość F5 TCP liczona z liczby kroków generatora między wysłanymi referencjami (`reference_interval_sec`).
- `robot_can_diagnostics`: generator TCP liczy krok 5 ms przed slotem F5 właściciela
  (`BusRuntime::begin_tcp_jog(..., slot_origin)`).
- Testy: `test_tcp_jog` (siatka F5 i brak oczekujących odczytów przy F5; 2 zgubione odpowiedzi na każdej osi
  bez awarii, 10 z rzędu → awaria „J2 … pozycja (0x31)”; brak odpowiedzi 0x32 nie opóźnia pozycji),
  `test_streaming_motion` (dwa kroki = ta sama prędkość). 258 testów colcon, 0 błędów.

## Próby sprzętowe (zgoda operatora; Baza, ruch ciągły 5 mm/s / 3 °/s, 3 s; X3: 10 s)

Narzędzia: `scripts/tcp_jog_hardware_trial.mjs` (WS panelu), `scripts/record_tcp_jog_telemetry.py`,
`scripts/analyze_tcp_jog_candump.py`. Surowe dane: `log/tcp-grid-20261030/`.

| | Stara wersja (A2) | Nowa, pierwsza (C1) | Nowa, faza generatora (C2+C3) |
| --- | ---: | ---: | ---: |
| Awarie „pozycja (0x31) nieaktualna” | 5 / 9 ruchów | 0 / 17 | 0 / 16 |
| F5 przy oczekującym odczycie (na oś) | 71–102 | 0 | 0 |
| Zgubione odpowiedzi 0x31 | 3–8 na oś | 0 | 0 |
| Odstęp F5 poza 8–12 ms | 63–65 / 3695 (min 3,96 ms) | 330 / 4195 (luki 20–30 ms) | 12 / 1020 (min 8,28 ms) |
| Przejścia `can0` error-warning / error-passive | +93 / +105 | — | +4 / +4 |

## Test J6 i przyczyna awarii napędowych (30.10.2026, zgoda operatora)

Narzędzie Rz (głównie J6), 1 / 3 / 10 °/s, 2 rundy ± (`log/tcp-grid-20261030/J6-1…3`): 12/12 ruchów z awarią
(uchyb J6 +1,25…+1,30° niezależnie od prędkości albo „ruch przeciwny” J6/J1 po ~0,3 s).

**Korekta:** wcześniejsza „mediana 0x31 − cel F5 J6 = +330 impulsów” obejmowała wyprzedzenie celu F5 (85 ms);
przy 10 °/s J6 nadąża za celem z typowym opóźnieniem. Rozstrzygające są starty sesji i ruch wolny:

- Pierwszy cel F5 sesji jest równy pozycji 0x31 (różnica 0–6 impulsów), a mimo to w 150–250 ms napęd odjeżdża
  w stałą stronę, niezależnie od kierunku rozkazu: J6 +146…+1018, J1 −73…−259 impulsów (J3, J4 ujemnie).
  Prędkość odjazdu przekracza zadane 1–2 RPM. To samo w starej wersji (A2: J6 +98…+161, J1 −50…−82).
- Przy 1 °/s cel F5 J6 zmienia się o −11…+54 impulsów, a J6 przejeżdża +200…+580 (4–13° wału) — oddala się od
  bezwzględnego celu.
- Przy ENABLE=0 wał się nie przesuwa (0–1 impuls między STOP a ENABLE) — to nie ruch swobodny.
- Skutek skumulowany przy parach ruchów ±: J1 −84,9° → −123,2°, J6 +33,9° → +139,1° w ciągu dnia prób.

**Wyjaśnienie:** cel F5 (pozycja bezwzględna) jest interpretowany w wewnętrznym układzie zadanej pozycji
napędu MKS, który przy postoju różni się od odczytu enkodera 0x31 o uchyb regulatora pozycji w trzymaniu
(rzędu 100–300 impulsów, 2–7° wału, znak stały dla osi). Właściciel CAN wysyła cel = odczyt enkodera, więc na
starcie każdej sesji napęd „domyka” swój uchyb i przesuwa wał o tę wartość w stałą stronę; dalej regulator
prędkości (korekta Kp względem pomiaru) podąża za oddalającą się osią. Pozycja końcowa po postoju ma ten sam
uchyb — stąd błąd dojazdu J1/J3 i „Nie potwierdzono zatrzymania TCP”. Prawdopodobna przyczyna wielkości uchybu:
mała sztywność pozycji w trzymaniu (prąd holding 1, nastawy regulatora MKS) oraz szum enkodera magnetycznego.
Hipotezy nie potwierdzono odczytem uchybu z napędu — brak instrukcji protokołu SERVO42D w repozytorium.

**Tolerancja dojazdu** zmieniona na `motor_stop_tolerance_deg: 0.3` (wał; 0,03° przegubu przy 10:1) — mniejsza
niż obecne uchyby napędów (J1/J3 1,2–1,8° wału), więc do poprawy regulatora potwierdzenie postoju TCP będzie
częściej odrzucane.

**Blokada po restarcie** naprawiona: postój = |prędkość wału| ≤ 0,5 RPM (szum enkodera przy postoju ±0,03 RPM),
odrzucone limity z `/robot/configuration/active` ponawiane co 50 ms. Sprawdzone na sprzęcie: po restarcie przy
ENABLE=0 i szumie J6 ±0,015 RPM limity TCP wczytane automatycznie.

## PTP: niekończący się „dolot końcowy” (30.10.2026)

Program PTP (plan 8,94 s) wisiał w `waiting_position` „Dolot końcowy: poza tolerancją J1 J6” ponad 560 s. Zapis
CAN: po ostatniej ramce wykonawca nie wysyłał żadnego F5, tylko odczyty; J6 (cel = start = 142,33°) stał na
147,55°, J1 0,76° od celu. Przyczyna: (1) wykonawca nie miał ani korekty, ani terminu dolotu; (2) ten sam
uchyb układu celu F5 napędu MKS co w JOG TCP. Zatrzymano przez `/api/v1/motion/stop` (robot w postoju).

Poprawka (`mks_servo_can`, tryb streamed): aktywny dolot końcowy — 300 ms po ostatniej ramce, co 200 ms, dla
osi poza tolerancją rozkaz F5 = poprzedni rozkaz + (cel − pomiar), ograniczony do zakresu osi, w fazie slotu
programu i bez oczekujących odpowiedzi; po 5 s bez tolerancji awaria z uchybami osi („Dolot końcowy nieudany
… J6=…°”). Testy: kompensacja przesunięcia 600 impulsów (sukces), oś zablokowana (awaria z opisem zamiast
czekania). 260 testów colcon, 0 błędów. Build: `install/tcp-grid-ros`; **wdrożenie wymaga restartu usługi**.

## Nierozwiązane (wymagają decyzji)

- Kalibracja regulatora / prądu trzymania MKS (wymaga instrukcji protokołu SERVO42D w wersji firmware napędów:
  odczyt uchybu kąta wału, prądu holding i nastaw Kp/Ki/Kd przed jakąkolwiek zmianą).
- Jedno przekroczenie KEEPALIVE 200 ms po stronie właściciela przy odstępie klienta ≤ 69 ms (ścieżka most/ROS).

## Wdrożenie

Pakiety `mks_servo_can`, `robot_hardware`, `robot_motion_planner`, `robot_can_diagnostics` z
`install/tcp-grid-ros` skopiowane do aktywnej `install/stop-recovery-ros`; poprzednie wersje:
`install/deployment-backups/tcp-grid-1793389149161849710` (sprzed zmian) i `…-1793389884298187320`.
Usługa odtworzona przez systemd (`Restart=always`); po poprawce blokady/tolerancji MainPID 44648
(kopia: `install/deployment-backups/tcp-grid-1793391048084230765`).
