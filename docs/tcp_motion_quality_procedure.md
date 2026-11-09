# Procedura testu jakości ruchu TCP (planowany ruch PTP/LIN)

Standardowy test jakości ruchu robota w trybie planowanego ruchu TCP. Służy do porównywania nastaw
wykonania (np. kodu przyspieszenia F5 `motion.f5_acceleration_code`) na tych samych ruchach.
Skrypt: `scripts/tcp_motion_quality.py`.

## Zgoda na ruch

- **Dopóki operator nie zgłosi, że maszyna jest sprzęgnięta mechanicznie** (zgoda z 08.11.2026), test
  można wykonywać bez osobnej karty dla każdego przebiegu, pod warunkiem że **każdy test zaczyna się od
  zerowania pozycji sterowników MKS**. Przed uruchomieniem asystent i tak podaje, co zostanie wykonane
  (wartości acc, liczba przebiegów).
- Po sprzęgnięciu mechanicznym zgoda wygasa: każdy przebieg wymaga karty (oś, kierunek, prędkości, zakres,
  warunki zatrzymania) i wyraźnej zgody operatora (AGENTS.md).

## Przebieg jednego testu

1. **Warunki wstępne:** usługa z nagrywaniem (`--record`, pliki `log/manual-can/can-*.jsonl`), tryb PTP/LIN
   `synchronized`, brak awarii, konfiguracja ROS uzgodniona.
2. **ENABLE=0** wszystkich osi; jeśli test dotyczy acc, zapis `motion.f5_acceleration_code` do ROS.
3. **Zerowanie MKS** przy ENABLE=1 (oś przygotowana; tylko wtedy sterowanie odpytuje prędkość i `F1`
   co 50 ms i potwierdza postój): komenda `92`, J1–J6, `POST /api/v1/manual/axes/{n}/zero`,
   potwierdzenie `31` ≈ 0. Następnie ENABLE=0 i potwierdzenie odniesienia osi z nieważną kalibracją
   (wymaga ENABLE=0). Przy offsetach 0 wszystkie przeguby = 0°.
4. **ENABLE=1** i PTP przygotowawczy (20 °/s, 100 °/s²) do pozy startowej **J1–J6 = 0/0/0/0/45/0°**
   (poza bez osobliwości nadgarstka; TCP ≈ X 399, Y 0, Z 351 mm, pitch 45°). Ten ruch nie jest oceniany.
5. **12 ocenianych ruchów** względem TCP pozy startowej, kierunek w układzie bazy wybierany automatycznie
   (pierwszy wykonalny z listy dla obu dystansów; obecnie (−X, +Y, −Z)/√3, cel 400 mm wymaga m.in.
   J1 ≈ 73°, J4 ≈ −65°, J6 ≈ 123°), orientacja stała:

   | Dystans | Prędkość | Ruchy |
   | --- | --- | --- |
   | 20 mm (bliski) | 5, 15, 40 mm/s | PTP start → cel, LIN cel → start |
   | 400 mm (daleki) | 5, 15, 40 mm/s | PTP start → cel, LIN cel → start |

   Razem 6 PTP i 6 LIN; PTP jedzie w jedną stronę, LIN wraca, więc każda para kończy w pozie startowej.
   Czas jednego testu ≈ 5 min (400 mm przy 5 mm/s to ~80 s na ruch). LIN: posuw TCP jak w tabeli, przyspieszenie
   100 mm/s² (minimum planera), prędkość kątowa 30 °/s. PTP (ruch w przegubach): prędkość i przyspieszenie
   osi dominującej skalowane tak, by czas był zbliżony do LIN, czyli Δq_max·v/d oraz Δq_max·100/d,
   w granicach limitów osi (80 °/s, 500 °/s²). Planer sprawdza limity osi, a suchy przebieg
   planuje wszystkie ruchy przed jakimkolwiek ruchem.
6. **ENABLE=0** na koniec, analiza i zapis wyników.

Przerwanie (awaria, nieudany program, Ctrl+C) wysyła STOP programu i kończy test; wyniki częściowe
są zapisywane.

## Wskaźniki (z nagrania CAN, zdarzenia `sync_feedback`)

Pozycje zmierzone i referencja w chwili pomiaru są interpolowane na siatkę 10 ms i przeliczane na TCP
kinematyką z konfiguracji (zgodność z planerem sprawdzana przed testem, próg 0,5 mm).

| Wskaźnik | Znaczenie |
| --- | --- |
| Tętnienie v [mm/s, %] | RMS różnicy prędkości TCP zmierzonej i zadanej, gdy zadana > 20% szczytu (% szczytu). Główna miara szarpania. |
| Zatrzymania [%] | Udział próbek przelotu (zadana > 50% szczytu), w których zmierzona prędkość < 50% zadanej (stick-slip, pulsowanie). |
| Odchyłka drogi max/RMS [mm] | Odległość TCP zmierzonego od TCP zadanego w tej samej chwili (opóźnienie i błąd toru). |
| Uchyb na końcu trajektorii [mm] | TCP z ostatniego `sync_feedback` (koniec trajektorii, przed dolotem) względem celu — miara opóźnienia. |
| Błąd końcowy po dolocie [mm] | TCP z pozycji po zakończeniu programu i 1 s postoju względem celu. |
| Plan / wykonanie [s], korekty dolotu | Czas planowany, rzeczywisty (z dolotem) i liczba korekt dolotu końcowego SYNC. |
| Maks. uchyb osi [°] | Na oś, w `results.json`. |

## Uruchomienie

```bash
python3 scripts/tcp_motion_quality.py                         # suchy przebieg: IK + plan 12 ruchów, bez ruchu
python3 scripts/tcp_motion_quality.py --execute               # test przy bieżącym acc
python3 scripts/tcp_motion_quality.py --execute --acc 50 120 241   # seria testów, każdy od zerowania
python3 scripts/tcp_motion_quality.py --execute --acc exec    # acc z parametrów wykonania (pole puste)
```

### Serie nastaw regulatora wykonania (`--tune`)

Kp, FF, wyprzedzenie celu F5 i człon całkujący zmienia się **poza konfiguracją robota**. Skrypt ustawia je przez
`POST /api/v1/motion/execution-tuning` jako parametry ROS węzła `robot_can_diagnostics`. Zmiana działa w locie,
wyłącznie w postoju, i unieważnia przygotowany program. Każdy przebieg zaczyna od nastaw z pliku parametrów
i zmienia jedną nastawę; po serii (także po przerwaniu) skrypt przywraca nastawy z pliku. Dopóki działają nastawy
testowe, GUI pokazuje w Centrum komunikatów ostrzeżenie „Testowe nastawy regulatora”, a status ruchu ma pole
`execution_tuning` (`test`, `active`, `baseline`). Restart usługi również przywraca nastawy z pliku.

```bash
python3 scripts/tcp_motion_quality.py --execute --baseline --tune lookahead_ms=120,160 --tune ff=1.1,1.2 --tune kp=1.5,4
```

Klucze: `kp` [1/s], `ff`, `lookahead_ms` (0–200), `ki` [1/s], `ki_limit_deg`. Ręcznie:
`curl -X POST $ROBOT_API/api/v1/motion/execution-tuning -d '{"position_kp": 4}'`, powrót: `-d '{"reset": true}'`.

Każdy przebieg zapisuje też czas wysyłki F5/SYNC z nagrania CAN (odstęp SYNC, spóźnienie względem terminu,
przygotowanie → ACK F5) — druga tabela w `summary.md`.

Wyniki: `log/tcp_quality/<czas>/` (`events.jsonl`, `results.json`, `summary.md`). Wnioski z serii
dopisywać do `docs/benchmark_results.md`.
