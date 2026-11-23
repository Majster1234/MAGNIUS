# Dokładność strumieniowania MKS — 25.10.2026

Późniejsze wdrożenie: poprawkę włączono do `install/gear-ratio-continuity-ros`
razem z [zachowaniem kątów po zmianie przełożenia](gear_ratio_continuity.md).
Punkt startowy usługi został przełączony na ten build, bez restartu.
Poniższy raport opisuje wcześniejszą turę izolowanego sprawdzenia.

Wprowadzono predykcję pozycji drugiego rzędu, ograniczenie horyzontu do końca programu,
kompensację wieku pomiaru i korekcję pozycji przy zerowej prędkości referencji.
Build i testy dotyczą izolowanego kandydata `install/streaming-accuracy-ros`.
Zgodnie z instrukcją zlecenia nie przełączano aktywnej instalacji, nie restartowano
usługi i nie otwierano fizycznego SocketCAN. Brak odbioru sprzętowego tej poprawki.

## Implementacja

- `streaming_setpoint`: nowe q/v/a, pozostały czas i wiek pomiaru; zachowane stare przeciążenia.
- Horyzont: min(85 ms, pozostały czas, czas do zatrzymania przy hamowaniu); pozycja q + v·t + ½a·t².
- Pomiar porównywany z q − v·τ + ½a·τ², gdzie τ jest ograniczone do 50 ms.
  Wartość i znacznik czasu są pobierane razem. Znacznik RX mierzy starzenie w procesie;
  nie określa nieznanego opóźnienia wewnątrz napędu ani czasu próbkowania enkodera.
- Przy |v| ≤ 1e−6 rad/s znak korekcji wynika z uchybu poza pasmem 0,0005 rad.
- PTP i LIN korzystają z tej samej ścieżki programu. JTC przekazuje a=0 i swój pełny lookahead,
  ponieważ ten interfejs nie zawiera końca trajektorii. Limity, dithering i STOP zachowano.
- Cykl 10 ms, brak ACK/SYNC i bramka odczytów 4 ms bez zmian. Wyliczony budżet sześciu osi
  przy 1 Mbit/s nadal wynosi 26,58% (pula robota 40%); to budżet ramek, nie pomiar przewodu.

## Symulacja pętli zamkniętej

`streaming_jtc --accuracy` porównuje zamrożony stary algorytm i aktualny kod biblioteki.
Trajektoria 0° → 20° → 0°, po 2 s na odcinek i po 0,5 s obserwacji, krok modelu 1 ms,
komendy 100 Hz, opóźnienie sprzężenia 11,2 ms, enkoder 16384 i kwantyzacja 1 RPM.
Przełożenia syntetyczne 3–6 dla czterech osi oraz 3–8 dla sześciu, naprzemienne kierunki,
offset 0, limity 1 rad/s i 2 rad/s². Nie są to parametry fizycznego robota.
Model jest idealnym pozycjonerem ograniczonym RPM; nie modeluje dynamiki ani firmware.
W przerwach utrzymuje ostatnią komendę; nie uruchamia dodatkowego regulatora.

| Osie | Wariant | RMSE [°] | Maks. błąd [°] | Settling po T planu [ms] |
| --- | --- | ---: | ---: | ---: |
| 4 | baseline | 0.105593 | 0.195309 | 0 |
| 4 | patched | 0.030785 | 0.138626 | 0 |
| 6 | baseline | 0.104686 | 0.195309 | 0 |
| 6 | patched | 0.028231 | 0.138626 | 0 |

RMSE maleje o 70,85% (4 osie) i 73,03% (6 osi); maksimum o 29,02%.
Settling 0 ms oznacza, że przy końcu nominalnego odcinka obie wersje są już w ±0,5°
i pozostają w nim co najmniej 100 ms. Nie wykazano poprawy czasu ustalania.
Jest to nowy benchmark dokładności: baza jest jawnie odtworzonym starym algorytmem,
nie dawnym pomiarem dokładności sprzętu. Pełny JSON: `log/streaming-accuracy-20261025/simulation.json`.

## Analiza istniejących nagrań sprzętowych

Źródło: `log/hardware-ptp/streaming100-20261024` (24.10.2026). Wszystkie pięć prób
przeliczono offline, zachowując oryginalne nagrania i dawne raporty. Poniższe tabele
dotyczą ukończonej próby `axes-1-2-3-5-attempt-1`, 4715 zaakceptowanych pozycji.
**To wyniki wcześniejszego kodu na sprzęcie; nie potwierdzają zachowania kandydata.**

## trajectory_accuracy

| Miara | overall | J1 | J2 | J3 | J5 |
| --- | ---: | ---: | ---: | ---: | ---: |
| rmse_deg | 0.349175 | 0.445058 | 0.357267 | 0.315871 | 0.247034 |
| mae_deg | 0.274376 | 0.353581 | 0.289052 | 0.273049 | 0.180276 |
| p95_error_deg | 0.710886 | 0.862701 | 0.668578 | 0.587177 | 0.467344 |
| max_abs_error_deg | 1.31876 | 1.31876 | 1.15154 | 1.03105 | 1.02472 |
| max_error_time_sec | 2.87503 | 2.87503 | 3.5237 | 3.47707 | 3.90507 |
| within_0_5_deg_pct | 86.3839 | 72.8728 | 86.0524 | 91.0866 | 95.7155 |
| within_0_1_deg_pct | 23.1177 | 21.23 | 20.1183 | 12.0543 | 39.2459 |
| final_error_deg | 0.351562 | 0.351562 | 0.131836 | 0.153809 | -0.109863 |
| overshoot_deg | 0.324707 | 0.192871 | 0.324707 | 0.324707 | 0.170898 |
| max_inter_axis_desync_pct | 8.06375 | 8.06375 | 7.4086 | 7.39934 | 8.06375 |
| max_inter_axis_desync_deg | 1.62161 | 1.5844 | 1.50578 | 1.47174 | 1.62161 |

## schedule_accuracy

| Miara | Wartość |
| --- | ---: |
| tx_period_mean_ms | 10.0002 |
| tx_jitter_rms_ms | 0.415808 |
| tx_period_p99_ms | 11.0422 |
| tx_period_max_ms | 12.1397 |
| batch_tx_skew_mean_ms | 0.0379118 |
| batch_tx_skew_max_ms | 0.123403 |
| rx_feedback_period_mean_ms | 11.1799 |
| rx_feedback_period_p95_ms | 16.8037 |
| rx_feedback_period_max_ms | 19.98 |
| skipped_frames_count | 0 |
| incomplete_batches_count | 0 |
| quiet_window_violations | 25 |
| planned_duration_sec | 12.95 |
| actual_settling_time_sec | 11.5808 |
| settling_delay_ms | -1369.19 |

Czas maksimum: J1/J2/J3 i overall względem startu `plus20`, J5 względem `return0`.
Uchyb końcowy każdej osi jest ze znakiem z ostatniego odcinka; overall to największy moduł.
Przeregulowanie liczone osobno w kierunku każdego odcinka, następnie maksimum odcinków.
Procenty dokładności są liczone z próbek, bez ważenia czasem. Percentyle są interpolowane liniowo.
Nominalna referencja pochodzi z zapisanych q/v/a Hermite’a; stare nagrania mają tylko
start/cel/T, więc odtwarzany jest ich kwintyczny profil z zerowym v/a końców.
Weryfikowane są CRC, znak 48-bitowego pomiaru/24-bitowego celu i profil przekładni/kierunku/offsetu.
Obracane pliki trace są scalane chronologicznie; błędne i powtórzone pozycje pomijane.
Dla tej próby: 0 błędnych pozycji, `dropped_total` maksymalnie 0.

Desynchronizacja: interpolacja liniowa pomiarów we wspólnym zakresie czasu, bez ekstrapolacji.
Osie stojące są wyłączane. Dla osi wynik w stopniach to największa różnica postępu względem
pozostałych osi pomnożona przez jej zakres; overall używa największego zakresu uczestników.
Czasy i rozrzut paczki są ze śladu procesu, bez potwierdzenia chwili transmisji na przewodzie.
Okres RX 11,18 ms jest okresem odświeżania, nie zmierzonym opóźnieniem enkodera.
Założenie 11,2 ms w symulacji pochodzi ze specyfikacji, nie z tego pomiaru okresu.

`quiet_window_violations` estymuje fazę zegara od pierwszej ramki paczki, obejmuje zapytania
uczestniczących osi przed ostatnim F5, w ostatnich 4 ms slotu. Brak zapisu wewnętrznego
deadline i jitter granic ograniczają ocenę przy krawędzi okna. W tym nagraniu wynik 25
nie spełnia oczekiwanego 0; nie zerowano go i nie uznano za odbiór aktualnej bramki.
Pominięte sloty estymowane z czasu między pierwszą i ostatnią paczką oraz ich liczby;
niepełne paczki liczone oddzielnie. Stary tryb ACK/SYNC ma ocenę okna i skew jako brak danych.

Settling oznacza początek ostatniego ciągłego pasma ±0,5°, potwierdzonego co najmniej
dwoma pomiarami obejmującymi ≥100 ms na każdej osi. Może nastąpić przed T planu;
ujemne opóźnienie nie oznacza wcześniejszego zakończenia wykonawcy. W agregacji sumowane
są czasy odcinków bez przerwy między nimi. `plus20`: 5,436784 s przy planie 6,44 s;
`return0`: 6,144025 s przy planie 6,51 s. Rzeczywisty czas wykonania programu pozostaje
oddzielnie w `legs[].actual_seconds`. Brak pomiaru lub potwierdzenia daje null.

| Nagranie | Ukończone | RMSE [°] | Zapytania w oknie | Pominięte sloty | Settling [s] |
| --- | --- | ---: | ---: | ---: | ---: |
| axes-1-2-3-5-attempt-1 | tak | 0.349175 | 25 | 0 | 11.580808 |
| axes-1-2-attempt-1 | tak | 0.319143 | 1620 | 0 | 11.392265 |
| axes-1-2-attempt-2 | nie | 0.374053 | 74 | 0 | — |
| axes-1-2-attempt-3 | tak | 0.325443 | 8 | 0 | 11.437779 |
| axes-1-attempt-1 | tak | 0.356184 | 801 | 0 | 11.316644 |

Próba przerwana zawiera metryki tylko zmierzonego fragmentu, bez deklaracji uchybu po ustaleniu.
Pełne JSON, Markdown i CSV każdej próby: `log/streaming-accuracy-20261025/hardware`.

## Sprawdzenia i odtworzenie

25.10.2026: build 11 pakietów ROS zakończony poprawnie; `colcon test-result`:
**411 tests, 0 errors, 0 failures, 0 skipped**. `pytest scripts/ benchmarks/`: **41 passed**.
Dodano testy hamowania, wieku pomiaru, korekcji końcowej, walidacji i zgodności wrapperów.
Zaktualizowano test wysyłki w rzeczywistym t=17 ms do predykcji drugiego rzędu.
12 testów analizatora obejmuje statystyki, Hermite’a, wyrównanie czasu, przerwanie/brak osi,
pominięty slot, okno ochronne, CRC/ujemne pozycje/profile, obrót plików i przerwy między odcinkami.
Pierwszy build wymagał ujednolicenia typów NAN/INFINITY w nowym teście; pierwsza pełna próba
testów wykryła stare oczekiwanie liniowego lookahead w teście 17 ms. Obie poprawiono,
a końcowy pełny build/test zakończył się bez błędów. Nie zatajono wstępnych nieudanych prób.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --build-base build/streaming-accuracy-ros \
  --install-base install/streaming-accuracy-ros --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
source install/streaming-accuracy-ros/local_setup.bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 colcon test --base-paths ros2_ws/src \
  --build-base build/streaming-accuracy-ros --install-base install/streaming-accuracy-ros \
  --return-code-on-test-failure
colcon test-result --test-result-base build/streaming-accuracy-ros --verbose
ROBOT_BENCH_BUILD_DIR="$PWD/build/streaming-accuracy-ros/benchmarks" python3 -m pytest -q scripts/ benchmarks/
python3 scripts/analyze_ptp_tracking.py --directory log/hardware-ptp/streaming100-20261024 \
  --output-dir log/streaming-accuracy-20261025/hardware
build/streaming-accuracy-ros/benchmarks/streaming_jtc --accuracy
```

Koszt wykonania i CPU przed/po: [benchmark_results.md](benchmark_results.md#25102026--dokładność-streamingu-izolowany-kandydat).
Nie budowano frontendu, bo ta zmiana go nie obejmuje. Nie ma sprzętowej bazy po poprawce.

## Stan produkcji

Odczyt przed/po: MainPID=24304, NRestarts=0, ActiveState=active i czas aktywacji bez zmian.
Konfiguracja `can0` identyczna: UP, ERROR-ACTIVE, 1 Mbit/s, TX1000. Hash punktu startowego
`install/streaming-smooth-ros/local_setup.bash` nie zmienił się; nadal przekierowuje do `install/lin-ros`.
Potwierdzenia: `production-before.json`, `production-after.json`, `production-comparison.json`
w `log/streaming-accuracy-20261025`. Testy używały atrap i lokalnego ROS/DDS, bez fizycznego CAN.
Kandydat nie został wdrożony ani uruchomiony przez usługę. Dalszy krok to osobno uzgodniony
odbiór sprzętowy: osie, kierunek, prędkość, czas/zakres i zatrzymanie.

Po osobnym zatwierdzeniu i przełączeniu wdrożenia operator może użyć poniższej komendy.
W bieżącym zadaniu restart jest zabroniony; nie wykonano go.

```bash
sudo systemctl restart robot-web-status.service
```
