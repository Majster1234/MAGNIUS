# JOG TCP na mechanizmie wykonania LIN — plan przebudowy

Stan: 02.11.2026. Etap 1 wdrożony w kodzie, czeka na próby sprzętowe operatora.

## Cel

JOG TCP ma się poruszać tak płynnie jak PTP/LIN. Dlatego korzysta z tego samego wykonania: kolejki punktów q/v/a,
interpolacji Hermite'a i tego samego `streaming_setpoint` (lookahead, FF, Kp ze znakiem, dithering RPM, acc).
Jedyna różnica względem LIN: koniec ruchu nie jest znany z góry.

## Dlaczego stary tor był mniej płynny

| | PTP / LIN | Stary JOG TCP |
|---|---|---|
| Referencja | Trajektoria q/v/a, interpolacja Hermite'a w chwili slotu | Jeden punkt na krok generatora, wysyłany na żywo |
| Prędkość F5 | Analityczna v, dithering RPM | Różnica pozycji / czas, zaokrąglenie do całych RPM, min. 1 RPM |
| Korekta Kp | Ze znakiem | Z wartości bezwzględnej uchybu |
| Cel F5 (lookahead) | Ekstrapolacja z v, a | Punkt z drogi hamowania przeliczanej co 10 ms |
| Dolot | Wspólny z programem | Osobna kopia z tolerancją w stopniach wału |

## Etap 1 — wdrożony (do prób)

**Generator (`robot_can_diagnostics/TcpJogApi`)**
- `TcpJog` (bez zmian) liczy punkty co 10 ms. Wątek co 10 ms dopełnia kolejkę do **200 ms** przed wykonaniem
  (`kHorizon`, maks. 25 kroków / 7 ms na wywołanie) i przekazuje ją przez `BusRuntime::append_tcp_points`.
- **STOP z panelu lub brak KEEPALIVE ≥ 150 ms → `cut_tcp_jog`** (bez rampy).
- Koniec drogi (limit, osobliwość, koniec impulsu) zostaje jak dotąd: generator hamuje po sprawdzonej drodze
  i zamyka kolejkę (`closed`). To nie jest rampa po puszczeniu przycisku, tylko ochrona limitów mechanicznych.
- Usunięte: sprawdzanie uchybu i ruchu przeciwnego w API, potwierdzanie postoju w API (strefa martwa, 0 RPM),
  fazowanie wątku względem slotów F5.

**Właściciel CAN (`mks_servo_can`, [tcp_jog.cpp](../ros2_ws/src/mks_servo_can/src/tcp_jog.cpp))**
- Kolejka `tcp_points_` próbkowana w każdym slocie F5 przez `interpolate_points` (ta sama funkcja co `program_reference`)
  i `streaming_setpoint` z parametrami `execution_*` (jak LIN). Oś czasu rusza po zebraniu 50 ms kolejki.
- Walidacja przy dopisywaniu: t=0 z v=0 na starcie, odstępy ≤ 50 ms, limity pozycji/prędkości/przyspieszenia osi,
  horyzont ≤ 1 s.
- **Ucięcie:** koniec kolejki = `t_teraz + lookahead` (85 ms), czyli dokładnie punkt, do którego napęd już jedzie
  (ostatni cel F5). Do tej chwili referencja biegnie z pełną prędkością, potem nie ma nowych celów i napęd
  wyhamowuje własną rampą `acc` F5. Robot nie zawraca do punktu puszczenia przycisku.
- Brak danych generatora (kolejka się skończyła) = ucięcie na końcu kolejki, bez awarii.
- **Dolot i potwierdzenie jak PTP:** po 300 ms co 200 ms F5 += cel − pomiar dla osi poza `execution_position_tolerance_rad`;
  koniec po dwóch pomiarach w tolerancji odległych ≥ 100 ms; awaria po 5 s.
- Zostały zabezpieczenia: KEEPALIVE 200 ms (tylko przy otwartej kolejce), uchyb śledzenia ≤ 1,25°,
  pomiar w limitach mechanicznych, świeżość pozycji, siatka F5 z oknem odczytów.
- Usunięte: `tcp_streaming_setpoint`, `tcp_final_approach`, `brake_tcp_jog`, liczenie kroków generatora,
  przyjmowanie pojedynczych paczek `CommandBatch` od JOG TCP.

**Bez zmian:** protokół panelu (START/KEEPALIVE/STOP), most, GUI.

## Co sprawdzić w próbach (operator)

1. Płynność w ruchu ustalonym: wykres wałów z kanałami „Cel F5” i „RPM F5”; porównać z LIN na podobnej prędkości.
2. Puszczenie przycisku: dobieg po puszczeniu (oczekiwane: ok. 85 ms ruchu + hamowanie napędu `acc`);
   czy tor TCP przy hamowaniu nie odbiega zauważalnie (każda oś hamuje własną rampą, niezależnie).
3. Opóźnienie startu: oś czasu rusza po ~50 ms kolejki + czas pierwszych kroków IK.
4. Dolot końcowy: tolerancja jak w LIN (`execution_position_tolerance_rad`), a nie 0,3° wału.
5. Zachowanie przy limicie osi: hamowanie generatora po sprawdzonej drodze.

## Etap 2 — po informacji zwrotnej z prób

- **Rampa po puszczeniu (opcjonalnie):** zamiast ucinać, podmienić resztę kolejki na profil hamowania od bieżącego
  punktu (`JerkProfile::stop`). Kolejka 200 ms nie wydłuża reakcji, bo się ją zastępuje, a nie dojeżdża do końca.
- **Uproszczenie bezpieczeństwa generatora:** zamiast przeliczać całą drogę hamowania co krok, sprawdzać prostą
  do końca kolejki + droga hamowania (hamowanie odbywa się po tej samej prostej).
- Usunąć martwe już elementy: parametr `motor_stop_tolerance_deg` (GUI/konfiguracja), `lookahead` w `TcpJogRequest`,
  `lookahead_positions` w `CommandBatch`/`TcpJogSample`.
- Rozważyć wspólny dolot PTP/TCP w jednej funkcji (dziś dwie kopie tej samej logiki).
- Po usunięciu przesunięcia napędu J4/J6 (kalibracja) ocenić, czy limit uchybu śledzenia 1,25° jest właściwy.
