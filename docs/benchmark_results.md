# Benchmarki ROS i backendu — instrukcja i wyniki

Narzędzie deweloperskie działa poza aplikacją. Mierzy tylko podprogramy wskazane
w bieżącym zadaniu, bez zmian GUI, usług ROS, harmonogramu sterowania i CI.
Nie uruchamia napędów. `run` wykonuje wskazany adapter, dlatego adaptery muszą
używać atrap wejść/wyjść, bez otwierania fizycznego CAN i wydawania poleceń robotowi.

## Procedura przy zmianie logiki

1. Przed edycją ustal listę zmienianych funkcji/callbacków/cykli. Zachowaj punkt
   odniesienia dla bieżącego promptu; wcześniejsze zmiany użytkownika nie należą
   do jego zakresu. Nie wybieraj benchmarków wyłącznie z całego `git diff`.
2. Wybierz powiązane scenariusze z `benchmarks/cases.json`. Jeśli ich brakuje,
   najpierw dodaj adapter rzeczywistego kodu i dane referencyjne, następnie zmierz
   wersję przed zmianą. Nie wykonuj całej kolekcji. Zmiana komentarzy lub dokumentacji
   nie wymaga pomiaru czasu wykonania.
3. Uruchom pomiar bazowy, zmień logikę, wykonaj jej testy poprawności i ponów pomiar
   z `--baseline`. Ten sam adapter, wejścia, liczba cykli, sprzęt i konfiguracja
   budowania muszą obowiązywać po obu stronach. Jeśli trzeba poprawić adapter,
   ponów obie strony z poprawionym adapterem.
4. Przy zmianie cyklu procesu wykonaj dodatkowo `watch` przed i po, jeśli możesz
   zapewnić to samo obciążenie. Opisz aktywnych klientów, częstotliwości wejść,
   stan robota i sposób budowania. Gdy nie ma takich warunków, zapisz ograniczenie.
5. Wykonaj `compare`. Uzupełnij tabelę historii dla zmienionych podprogramów,
   także przy pogorszeniu, niejednoznacznym wyniku lub braku możliwości pomiaru.
   Narzędzie zapisuje JSON; opis celu zmiany i wniosek dopisuje agent do tego pliku.

Wszystkie polecenia uruchamiaj z katalogu głównego repozytorium. Domyślnie wyniki
otrzymują unikalne nazwy w ignorowanym `log/benchmarks`. `--output` pozwala wskazać
konkretny plik; istniejący plik nigdy nie jest nadpisywany.

### Powtarzalny pomiar podprogramu

Przykład **techniczny**, sprawdzający narzędzie, bez kodu produkcyjnego ROS:

```bash
python3 scripts/benchmark_ros.py run --case technical.python --output log/benchmarks/python-before.json
# W zadaniu optymalizacyjnym tutaj następuje zmiana właściwego podprogramu.
python3 scripts/benchmark_ros.py run --case technical.python --baseline log/benchmarks/python-before.json --output log/benchmarks/python-after.json
python3 scripts/benchmark_ros.py compare --before log/benchmarks/python-before.json --after log/benchmarks/python-after.json
```

`technical.cpp` działa analogicznie. Runner konfiguruje i buduje wybrany target
osobnym CMake w `build/benchmarks`, z `CMAKE_BUILD_TYPE=Release`. Kompilacja pozostaje
poza pomiarem. Narzędzia wymagane: Linux, Python 3.10+, a dla C++ CMake i kompilator
C++17. Nie zmienia się standardowe budowanie workspace ani manifesty pakietów ROS.

Pomocniki wykonują rozgrzewkę (minimum 1 s), kalibrują liczbę cykli do co najmniej
200 ms w serii kalibracyjnej, a następnie mierzą siedem serii. Po zmianie liczba
cykli jest odtwarzana z bazowego JSON; szybsze serie mogą wtedy trwać poniżej 200 ms.
Pomiar CPU wykorzystuje czas całego procesu, obejmujący jego wątki. Koszt samej pętli
i wywołań pomocnika jest częścią wyniku; nie odejmujemy oszacowanego narzutu.
Przy bardzo małych funkcjach cykl powinien obejmować stały, opisany pakiet operacji.

Raport zawiera medianę i min–max **średniego czasu na cykl w seriach**, czas CPU na
cykl oraz przepustowość. Nie są to percentyle opóźnienia pojedynczych wywołań ani
gwarancje czasu rzeczywistego. Przygotowanie i sprawdzenie wyników są poza pomiarem,
ale razem z rozgrzewką podlegają limitowi 30 s dla procesu adaptera. Błąd adaptera,
błędne dane i timeout dają status `error`, bez wiarygodnego wyniku wydajności.
Konfiguracja i budowanie C++ mają osobne limity po 120 s.

### Dodanie scenariusza

W `benchmarks/cases.json` dodaj tylko scenariusz potrzebny do zadania:

- `language`: `python` lub `cpp`; `module` dla modułu Python lub `target` dla targetu CMake.
- `sources`: lista obiektów `path` (względem repo) i `symbols` (nazwy badanych funkcji).
- `adapter_sources`: pliki adaptera, fixture'ów i dodatkowych ustawień budowania,
  których treść musi pozostać identyczna przed/po. **Nie umieszczaj tu optymalizowanej
  implementacji** — jej hashe są zapisywane osobno w `code.sources_sha256`.
- `workload`: jednoznaczny opis danych i pracy jednego cyklu; `technical_only: false`
  dla rzeczywistych benchmarków. Nie wymyślaj parametrów fizycznego robota.

Python: moduł udostępnia fabrykę stanu z metodami `cycle(index)` i `verify()` oraz
wywołuje `benchmarks.harness.main(fabryka)`. Fabryka odtwarza stan przed każdą serią.
`verify()` sprawdza wyjście i zwraca dokładnie `True`; wyjątek oznacza błąd.
Dla asynchronicznego backendu utwórz pętlę zdarzeń w przygotowaniu, a w cyklu
czekaj na zakończenie mierzonej operacji. Nie mierz jedynie utworzenia coroutine.

C++: dodaj wybrany target do `benchmarks/CMakeLists.txt`, połącz go z potrzebną
implementacją i użyj `robot_benchmarks::main(argc, argv, fabryka)` z `harness.hpp`.
Obiekt ma `cycle(uint64_t)` i `verify()`. Wyniki muszą być obserwowalne oraz
sprawdzane; adapter ma zapobiegać usunięciu pracy przez optymalizator. Przykład używa
`volatile` na akumulatorze. Nie dodawaj `volatile` do kodu produkcyjnego.
Zależności ROS rozwiązuj w osobnym projekcie benchmarków po załadowaniu Humble;
implementację przed/po zawsze przebudowuj z tych samych ustawień Release.
Adapter wypisuje wyłącznie JSON na stdout; informacje diagnostyczne kieruj na stderr.

Pomocniki zwracają `iterations` i siedem `samples` z czasami `wall_ns` oraz `cpu_ns`.
Runner uruchamia adapter w osobnym procesie; start Pythona/C++ nie jest mierzony.
Nie dodawaj pomiarów ani zależności od benchmarków do ścieżki wykonywania aplikacji.

### CPU działających procesów

Po samodzielnym ustaleniu właściwych PID-ów podaj je jawnie; np. dla PID 1234:

```bash
python3 scripts/benchmark_ros.py watch --pid bridge=1234 --context "Postój; jeden klient WS; ten sam profil i build Release" --output log/benchmarks/process-before.json
```

Po wdrożeniu zmiany powtórz pomiar, wskazując aktualny PID, identyczną etykietę
`bridge` i identyczny opis warunków. Można powtórzyć `--pid` dla innych zmienionych
procesów. Nie obserwuj wszystkich procesów ROS, jeśli zmiana ich nie dotyczy.
`--duration` (domyślnie 30 s) i `--interval` (1 s) muszą być takie same przed/po.

Odczyt `/proc/PID/stat` obejmuje wątki procesu, bez procesów potomnych. Raportuje
czas CPU, średnie obciążenie ważone rzeczywistym czasem i maksimum w próbkach.
100% to jeden rdzeń, więc proces wielowątkowy może przekraczać 100%. Nie wolno
przypisywać całego CPU procesu pojedynczej zmienionej funkcji. Nie mierzymy tym
opóźnienia callbacków ani cyklu komunikacji z napędami.

Zakończenie procesu, zombie, ponowne użycie PID-u, zmiana executable/affinity lub
brak dostępu kończą obserwację statusem `incomplete`, zachowując zebrane próbki.
Narzędzie nie restartuje usług i nie wykonuje ruchu. Obserwacja istniejącego ruchu
nie stanowi zgody na jego wywołanie; obowiązują zasady testów ruchu z AGENTS.md.

### Interpretacja porównania

`compare` sprawdza kompletność pomiarów oraz zgodność scenariusza, pomocników,
danych, środowiska wykonawczego, sprzętu i ustawień. Dla `watch` dodatkowo sprawdza
etykiety/executable/affinity procesów i parametry próbkowania, dopuszczając nowe PID-y.
Zmiana optymalizowanego kodu jest oczekiwana i nie blokuje porównania.

Spadek kosztu: `(przed − po) / przed × 100%`. Dodatni wynik oznacza oszczędność,
ujemny pogorszenie; dla zerowego kosztu bazowego procent wynosi `null`.
Brak `--before` daje `no_baseline`, różne warunki `not_comparable`; oba stany
nie zawierają różnic procentowych. Kod wyjścia 0 oznacza pomiar kompletny lub
porównywalny, 2 — błąd, niepełność, brak bazy lub nieporównywalność. Nie ma progów
blokujących zmiany ani automatycznej oceny istotności statystycznej. Wniosek musi
uwzględniać rozrzut; w razie wątpliwości powtórz obie strony w stabilnych warunkach.

JSON zawiera czas UTC, stan Git i hash zmian śledzonych, hashe badanych plików,
CPU/system/affinity/governor, obciążenie hosta oraz wersję Pythona lub kompilatora
i flagi Release. Nie zapisuje argumentów procesów, środowiska, danych kont ani
surowych logów aplikacji. Dla `watch` stan Git opisuje repozytorium obserwatora,
nie dowodzi wersji załadowanej do usługi. W historii podaj osobno potwierdzoną
wersję działającego procesu i jego ustawienia budowania; brak wiedzy zaznacz jawnie.
Automatyczne sprawdzenie warunków nie potwierdza identyczności zewnętrznego ruchu.

## Historia optymalizacji

Poniżej pomiary rzeczywistego kodu na atrapach wejść/wyjść. Wdrożenie narzędzia
i jego testy techniczne nie są optymalizacją ROS ani pomiarem osiągów robota.
Pomiary dodatku PTP dokumentują również narzut nowej funkcji; nie dowodzą poprawy płynności.

| Data | Podprogram / plik | Cel zmiany | Scenariusz i warunki | Przed: wall / CPU na cykl lub CPU procesu | Po | Spadek kosztu [%] | Rozrzut przed / po | Wniosek i ograniczenia |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-10-21 | CanBusManager::submit/tick i harmonogram | Źródło PROGRAM i lokalny wykonawca; regresja istniejącej ścieżki trajektorii | `motion.bus`, 900 cykli × 7 serii | 240.331 / 240.096 µs | 267.548 / 267.234 µs | -11.32 wall; -11.30 CPU | 238.989–244.379 / 263.252–274.545 µs wall | Większy koszt; bez dowodu poprawy czasu reakcji lub fizycznego ruchu. |
| 2026-10-21 | Storage.system_settings/set_system_settings | Osobna trwała polityka niepełnego zestawu i migracja | `motion.settings`, 20000 cykli × 7 serii | 17.074 / 17.064 µs | 19.534 / 19.531 µs | -14.40 wall; -14.46 CPU | 16.913–17.818 / 19.028–19.865 µs wall | Większy koszt; bez dowodu poprawy czasu reakcji lub fizycznego ruchu. |
| 2026-10-21 | CanBusManager: pomiar końcowy F5/ACK | Obsługa spóźnionego completion bez utraty slotu ACK; pełny koszt programu uwzględniony osobno | `motion.bus`, 900 cykli × 7 serii; ta sama baza | 240.331 / 240.096 µs | 268.537 / 268.500 µs | -11.74 wall; -11.83 CPU | 238.989–244.379 / 262.101–276.411 µs wall | Narzut około 28.2 µs na cykl; brak dowodu przyspieszenia. |

Przy wpisie podaj jednostki, rewizje/hashe obu wersji, sprzęt, wersję narzędzi,
ustawienia budowania i nazwy lokalnych JSON. Lokalne JSON nie są śledzone przez Git,
więc istotne wartości i metadane muszą znaleźć się także w tym pliku. Przy braku
pomiaru wpisz przyczynę zamiast zera lub oszacowanej poprawy.

## Sprawdzenie samego narzędzia

```bash
python3 -m unittest discover -s benchmarks/tests -v
```

Testy obejmują pomocniki Python/C++, statystyki, weryfikację wyników i danych,
zgodność pomiarów, błędy/timeouty oraz odczyt CPU kontrolowanych procesów.
Nie wymagają robota ani zmiany stanu usług. CMake buduje cel testowy osobno w
`build/benchmarks`; żaden wynik tych testów nie trafia do historii optymalizacji.

## PTP i niepełny zestaw — zakres pomiaru 2026-10-21

Przed zmianą kodu utworzono `motion.bus` i `motion.settings` oraz zapisano bazę.
Scenariusze mają identyczne adaptery, dane i cykle po obu stronach. Adapter CAN
korzysta z emulowanego transportu i wstrzykiwanego zegara, bez `can0` i bez ruchu.
Jeden cykl to submit i 20 ticków po 1 ms (50 Hz); odtwarza zerową trajektorię sześciu
osi. Ustawienia: odczyt i zapis bez zmiany wartości w prywatnym katalogu tymczasowym.
To regresja ścieżki istniejącej, nie pomiar kosztu fizycznego programu PTP.

Zmienione podprogramy/cykle tego zadania:

- CAN: `submit`, `activate`, `renew_source_lease`, `tick`, `decode/expire`, harmonogram
  ruchu/odczytów i `control_state`; cykl `BusRuntime::start` ma bezwzględne pobudki.
  Benchmark obejmuje wywołania istniejącej ścieżki w cyklu; nie mierzy wszystkich
  gałęzi błędów ani faktycznych pobudek wątku IO.
- Nowe: `compile_motion_program`, `program_reference`, prepare/execute/schedule
  programu i bariera uzbrojenia; dla tych funkcji nie istniała wersja bazowa.
  Sprawdzono poprawność na emulatorze; nie przypisujemy im różnicy procentowej.
- Backend/ROS: `Storage.system_settings/set_system_settings` (zmierzone), publikacja
  ustawień węzła konfiguracji, metadane `PlanningClient.move/watch`, guard/akcja
  fizycznego planera, `ProgramApi`, `MotionClient` i ochrona `CanSystem.read/write`.
  Osobnych baz czasowych callbacków ROS i metadanych nie uzyskano; brak pomiaru nie
  oznacza zerowego kosztu. Zmiany launch/respawn sprawdzono składniowo, bez startu stosu.

Metadane obu porównań: Intel i5-7500T 2.70 GHz, 4 logiczne CPU, affinity 0–3,
Ubuntu/Linux 6.8.0-138-generic, governor powersave. C++ GCC 11.4.0, Release z jawnym
`-O1 -DNDEBUG` po obu stronach (pierwsza kompilacja O3 przekroczyła limit; nie jest
bazą tabeli). Python CPython 3.10.12, optimize=0. Baza Git
`6d18ae0aa5d6a27ce01a38f4513d8519c1f56548`, zmiany robocze przed/po; bez commitu.
Porównania narzędzia: `comparable`. Inne procesy hosta i powersave ograniczają
interpretację wyników; nie jest to pomiar jitteru pojedynczego terminu SYNC.

Pliki lokalne, pomijane przez Git:
`log/benchmarks/ptp-bus-before.json`, `ptp-bus-after.json`,
`ptp-settings-before.json`, `ptp-settings-after.json` (ten sam katalog).
SHA-256 źródeł (skróty przed → po, pełne w JSON):

- `bus_manager.cpp`: `991e9616d6fd548e` → `c2b41eab000c2c5c`;
- `bus_runtime.cpp`: `c7da9b79187a8f79` → `6e58c7eb4c03dce1`;
- `storage.py`: `4b0fa82a41a6b1ed` → `2e5a0c5419e480f7`.

**CPU działających procesów: brak porównywalnego pomiaru przed/po.** Nie zachowano
bazy procesu stosu przy tym samym obciążeniu; nie restartowano usług ani nie
uruchamiano robota. Kolumna CPU tabeli to czas procesu adaptera na cykl, nie procent
CPU bridge/CAN/planera. Nie potwierdzono optymalizacji obciążenia procesów ani
poprawy płynności. Narzut ścieżki CAN wynosi około 27 µs na symulowany cykl 20 ms,
ustawień około 2.46 µs na parę operacji. Odbiór terminów USB/CAN i dynamiki pozostaje
osobnym, niewykonanym sprawdzeniem sprzętowym.

Pomiar końcowy po poprawce odpowiedzi F5: `log/benchmarks/ptp-bus-after-final.json`;
`bus_manager.cpp` SHA-256: `c26e7d60870204027689796fb8f3ccd5fbb074ec8fca5695e3cf1f89fd18dde7`.
Poprzedni pomiar CAN w tabeli zachowano; końcowy narzut to około 11.74% wall / 11.83% CPU.


## 2026-10-22 — harmonogram PTP/CAN 50/25 Hz

Cel: zsynchronizować próbki i SYNC, odczytywać każdą oś co 40 ms, rozłożyć
transakcje w fazach i ograniczyć planowane pasmo do 40%. Zmniejszyć koszt IO
przez listę wykrytych adresów, bufory zapytań/SYNC i CRC bez kopii payloadu.
Pozostałe zmiany: STOP F7 → F6(0), reset zachowujący znane adresy, skan 1–20.

Podprogramy tego zadania: `CanBusManager::tick`, `start_cycle`, `reserve`,
`schedule_motion/program/feedback/background`, `send_query`, `receive/decode/expire`,
`start_scan`, `reset_session` i procedura STOP; `compile_motion_program`,
`program_reference/position_bounds`, `request/validate_response` oraz adapter
`BusRuntime`. Planner korzysta ze wspólnego okresu. Callback resetu diagnostyki
ma test ROS sukcesu/błędu; nie mierzymy restartu systemd ani fizycznego CAN.
Nie ponawiano benchmarków ustawień, IK ani pozostałych wcześniejszych zmian.

Nowy adapter `benchmarks/can_schedule.cpp` i pięć scenariuszy powstały przed
zmianą logiki. Obie wersje mają identyczny adapter, dane i liczbę ticków;
`compare` potwierdził zgodność metadanych wszystkich sześciu porównań.
PTP ma syntetyczne q/v/a: 0→0,1→0 rad w 2 s, pełny lub czteroosiowy zestaw.
Programowy cykl benchmarku obejmuje prepare/arming i całe wykonanie, nie jeden
okres 20 ms. HOLD trwa 0,2 s. Pomiar idle wyłącza skan po wykryciu urządzeń
przez zamknięcie/otwarcie transportu; dzięki temu mierzymy samą diagnostykę.
Scenariusz reset bada pełne czyszczenie i ponowny skan po obu stronach;
zachowanie adresów przez reset operatora jest odrębnie sprawdzone w testach.

| Scenariusz i zakres jednego cyklu | Cykle × serie | Przed wall / CPU [µs] | Po wall / CPU [µs] | Spadek wall / CPU [%] | Rozrzut wall przed / po [µs] |
| --- | --- | --- | --- | --- | --- |
| `can.stream6` — PTP strumieniowany 6 osi, 2500 ticków/program | 6 × 7 | 37255.288 / 37252.939 | 2054.017 / 2054.422 | 94.49 / 94.49 | 36905.522–37621.128 / 1977.816–2087.366 |
| `can.stream4` — PTP strumieniowany CAN 1,2,3,5, 2500 ticków/program | 12 × 7 | 33110.435 / 33109.777 | 1556.972 / 1557.107 | 95.30 / 95.30 | 32996.394–33732.851 / 1536.386–1588.926 |
| `can.native` — Natywny HOLD 6 osi, 800 ticków/program | 20 × 7 | 10297.339 / 10297.028 | 456.323 / 456.394 | 95.57 / 95.57 | 10234.763–10626.601 / 435.506–463.195 |
| `can.idle20` — Diagnostyka 20 urządzeń, 20 ticków/cykl | 700 × 7 | 301.438 / 301.404 | 21.164 / 21.166 | 92.98 / 92.98 | 298.436–309.767 / 20.326–22.092 |
| `can.reset` — STOP, pełny reset, skan i 45000 ticków/cykl | 1 × 7 | 582038.235 / 582008.784 | 17743.608 / 17745.147 | 96.95 / 96.95 | 576413.721–588387.715 / 17365.822–18146.449 |

Rozrzut to min–max średnich siedmiu serii, nie percentyle pojedynczych wywołań.
Metadane: Intel i5-7500T, 4 CPU, affinity 0–3, governor powersave,
Linux 6.8.0-138-generic, GCC 11.4.0, Release `-O1 -DNDEBUG`, rozgrzewka ≥1 s.
Liczby cykli po zmianie odtworzono z bazy; krótsze serie po optymalizacji
mogą trwać mniej niż 200 ms. Żadna z pięciu zmierzonych ścieżek nie wykazała regresji.

**CPU procesu adaptera:** ten sam emulator 20 urządzeń, 50 cykli czasu ściennego/s,
każdy po 20 ticków 1 ms; 2 s oczekiwania przed odczytem i 15 s pomiaru co 1 s.
Średnio **5,266% → 0,667% jednego rdzenia**, spadek względny **87,34%**
(4,600 pp); maksima próbek **6,000% → 2,000%**. CPU procesu w oknie:
0,790 → 0,100 s. To po jednym oknie przed/po, z rozdzielczością liczników `/proc`;
nie ma rozrzutu między powtórzonymi oknami. Nie jest to CPU działającej usługi
ROS ani pomiar latencji SocketCAN/USB. Brak porównywalnego pomiaru produkcyjnego
procesu z fizycznymi napędami pozostaje jawnie otwartym ograniczeniem.

Wniosek: koszt idle spadł o 92,98%, całych syntetycznych programów o około
94–96%, a pełnego scenariusza STOP/reset o 96,95%. Poprawa wynika głównie
z usunięcia przeglądania 2047 adresów w każdym ticku i rzadszych odczytów.
Nie dowodzi płynności, braku retransmisji ani skuteczności kasowania bufora
F7 → F6(0) na napędach. Model budżetu dla sześciu osi: średnio 37,11%, szczyt
zaplanowanych transakcji ≤38,95%; testy zegara sprawdzają limit 40% i fazy.

Wyniki lokalne: `log/benchmarks/can-20261022-*-baseline.json`,
`can-20261022-*-after.json`, `can-20261022-cpu-before.json` i porównania
`20261022T211017.*-compare.json`. Dane i logi pozostają poza Git.
Przykład odtworzenia po zapisaniu bazy (bez GUI/CI):

```bash
cmake -S benchmarks -B build/benchmarks -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
python3 scripts/benchmark_ros.py run --case can.idle20 --baseline log/benchmarks/can-20261022-idle20-baseline.json --output log/benchmarks/can-idle20-new.json
python3 scripts/benchmark_ros.py compare --before log/benchmarks/can-20261022-idle20-baseline.json --after log/benchmarks/can-idle20-new.json
```

`CAN_BENCH_REALTIME_SECONDS=22 build/benchmarks/can_idle20` udostępnia obciążenie
procesu do polecenia `watch` opisanego wyżej. Przed każdym kolejnym porównaniem
należy zachować zgodne metadane i odczekać na koniec kompilacji/testów innych procesów.

Sprawdzenia końcowe 2026-10-22: izolowany build czterech pakietów ROS w Release
`-O1 -DNDEBUG`, `colcon test-result`: **128 testów, 0 błędów, 0 niepowodzeń,
0 pominięć**; frontend build i 75 testów oraz test przeglądarkowy CAN na atrapach;
3 testy narzędzia STOP i kontrola składni Pythona. Testy JOG dostosowują czas
obserwacji do 40 ms, zachowując progi postoju i limit świeżości 60 ms.
Pierwsze uruchomienia wykryły niezgodny install-base testów i stary obiekt
z równoległej, przerwanej kompilacji; końcowa pełna odbudowa i poprawny prefix
usunęły te problemy. Build/testy ROS i Chromium wykonano poza ograniczeniami
sandboxa, w izolowanej instalacji i domenach testowych. Działających usług
robota nie przeładowywano; flag odbioru i konfiguracji sprzętu nie zapisywano.

## 23.10.2026 — trace wałów J1–J6

Zakres tego zadania: przejęcie przyjętego `31H` w `CanBusManager::decode`,
`record_shaft_position`, kolejka SPSC/drain, timer `publish_shaft_trace`,
`ShaftTraceStore.ingest/_write/live/history/export` oraz odbiór nowej paczki
w bridge. Cel: ciągły trace wszystkich pomiarów, poza IO, bez dodatkowego CAN.
Nie zmieniano w tym zadaniu faz PTP, progów, flag odbioru ani programu próby 5.

Bazę zabezpieczono **przed zmianą logiki**: nagłówki i bibliotekę CAN w
`/tmp/shaft-trace-baseline`, JSON w ignorowanym
`log/benchmarks/shaft-trace-20261023/`. Obejmuje wcześniejsze niezapisane zmiany
użytkownika — porównanie z HEAD nie byłoby właściwą bazą. Adaptery:
`benchmarks/shaft_capture.cpp` i `benchmarks/shaft_backend.py`. Są niezależne od
GUI/CI; ich własne JSON zawierają pojedyncze serie/cykle, nie format ogólnego
runnera `benchmark_ros.py`.

Metadane: ten sam komputer i5-7500T, Linux 6.8.0-138-generic, GCC 11 / C++17,
Release **-O1 -DNDEBUG** po obu stronach, Python 3.10.12. Brak przypięcia CPU,
stałej częstotliwości oraz wyłączności hosta. Wyniki opisują narzut funkcji
i procesu adaptera; nie są gwarancją czasów na sprzęcie ani wykorzystania CAN.

### Odbiór CAN i opróżnianie kolejki

Ten sam emulator sześciu osi, 45000 ticków startu, rozgrzewka 1000 cykli,
7 serii × 10000 cykli. Cykl to 20 ticków po 1 ms, bez czekania na zegar ścienny.
Po zmianie dodatkowo opróżnia kolejkę do stałego bufora 256 próbek.

| Miara / cykl 20 ms emulatora | Przed, mediana [min–max] µs | Po, mediana [min–max] µs | Zmiana |
| --- | ---: | ---: | ---: |
| Czas ścienny | 8,208 [8,147–8,344] | 8,824 [8,639–8,994] | +7,51% |
| Czas CPU | 8,209 [8,148–8,344] | 8,823 [8,632–8,994] | +7,48% |

W obu wynikach **TX=414908, RX=414894, 36625 zapytań pozycji na każdą oś**.
Narzut wynosi około 0,617 µs na cykl adaptera, przy identycznym ruchu CAN.
Regresja czasu jest kosztem nowego przechwycenia/znaczników/kolejki, bez
przenoszenia dysku i ROS do IO. Test przyjętych pomiarów sprawdza 60 próbek
w 400 ms / sześć osi (25 Hz każdej), bez ponownego emitowania cache.
Regresje istniejących testów harmonogramu zachowują cykl komend 20 ms i pomiarów
40 ms. Nie wykonywano nowej próby ruchu, która potwierdzałaby te terminy fizycznie.

Odtworzenie wersji bieżącej, po buildzie pakietu CAN:

```bash
g++ -std=c++17 -O1 -DNDEBUG -Iros2_ws/src/mks_servo_can/include benchmarks/shaft_capture.cpp -Linstall/shaft-trace-ros/mks_servo_can/lib -Wl,-rpath,"$PWD/install/shaft-trace-ros/mks_servo_can/lib" -lmks_servo_can -lpthread -o /tmp/shaft-capture
/tmp/shaft-capture > log/benchmarks/shaft-trace-current.json
```

### Rejestrator i odbiorcy backendu

Na wariant 375 cykli / 15 s: **2250 pomiarów** sześciu osi (150/s), 300 aktualizacji
dotychczasowego `ManualStore` (20 Hz). Klienci są emulowani: każdy wykonuje
snapshot i po zmianie odbiór/serializację nowego trace przy 25 Hz, bez HTTP,
przeglądarki i narzutu renderowania. Porównanie zachowuje te same dane i liczby
cykli. Pliki tymczasowe mają domyślny limit 20 MB/oś, `fsync` co sekundę.
CPU to czas wszystkich wątków procesu / czas ścienny, **100% = jeden rdzeń**;
15 odczytów co około sekundę. Callback obejmuje wejście oraz emulowanych klientów;
praca writera jest uwzględniona w CPU procesu, poza czasem callbacku.

| Odbiorcy | CPU przed %, mediana [min–max] | CPU po %, mediana [min–max] | Zmiana względna / punkty procentowe |
| --- | ---: | ---: | ---: |
| 0 | 0,513 [0,212–0,536] | 1,360 [0,933–2,563] | +165,09% / +0,847 pp |
| 1 | 1,906 [1,617–2,006] | 7,220 [5,155–7,578] | +278,79% / +5,314 pp |
| 3 | 4,537 [3,902–4,709] | 13,081 [8,938–13,944] | +188,31% / +8,544 pp |

| Odbiorcy | Callback przed, mediana / p95 µs | Callback po, mediana / p95 µs | Zmiana mediany |
| --- | ---: | ---: | ---: |
| 0 | 181,971 / 199,157 | 185,132 / 477,251 | +1,74% |
| 1 | 770,287 / 795,807 | 2348,813 / 2944,575 | +204,93% |
| 3 | 1811,928 / 1889,463 | 5248,321 / 6724,621 | +189,65% |

W każdym wariancie po zmianie zapisano **2250/2250 próbek**, brak utrat źródła,
sekwencji i writera. To przyrost funkcjonalności i koszt CPU, nie przyspieszenie.
Ograniczono odczyt bufora live: iteracja od końca kończy się przy kursorze
klienta, zamiast przeglądać wszystkie 4096 wpisów przy każdym zapytaniu.
Pierwszy wariant „po” miał CPU mediany 2,095 / 7,506 / 14,321%; końcowy
1,360 / 7,220 / 13,081%. Rozrzut zachodzi na siebie, więc nie przypisujemy całej
różnicy tej optymalizacji. Złożoność zwykłego odczytu zależy od nowych próbek.

Odtworzenie, na tych samych wejściach, poza GUI i działającym robotem:

```bash
source /opt/ros/humble/setup.bash
python3 benchmarks/shaft_backend.py --legacy --seconds 15 --output log/benchmarks/shaft-backend-legacy.json
python3 benchmarks/shaft_backend.py --seconds 15 --output log/benchmarks/shaft-backend-current.json
```

`--legacy` odtwarza brak sinka trace, jak w bazie sprzed dodania modułu. Sam
publisher trace nie istniał wcześniej. Jego okres i kolejność sprawdzono
funkcjonalnie przez ROS z transportem pamięciowym, również po wstrzymaniu
executora na 180 ms. **Nie wyodrębniono porównania CPU samego wywołania ROS
publish, ani pomiaru procesu całej aplikacji z 0/1/3 fizycznymi przeglądarkami**.
Emulowane odbiorniki mierzą inny zakres; te pomiary pozostają ograniczeniem.

### Testy i stan odbioru

2026-10-23: ukierunkowany build pięciu pakietów (`robot_control_interfaces`,
`mks_servo_can`, `robot_can_diagnostics`, `robot_hardware`, `robot_web_bridge`)
w osobnym `build/shaft-trace-ros` / `install/shaft-trace-ros`, następnie spójna
instalacja w `ros2_ws/install` z `build/shaft-trace-deploy`. Backend: 133 testy,
w tym 19 trace; `colcon test-result`: 117 testów, 0 błędów i pominięć.
Frontend: build TypeScript/Vite i 78 testów. Chromium: dziewięć
scenariuszy na atrapach, w tym sześć wykresów, skale, historia, kursor, utrata
strumienia, zapamiętanie widoku, CSV i panele brakujących osi. Rotacja pełnych
20 MB ponad dwa razy, restart, równoległy eksport, CRC, ENOSPC i przepełnienie
kolejek przeszły bez przekroczenia limitu plików.

Pierwsze próby testów napotkały ograniczenie zapisu ROS_LOG_DIR/wybudzania
asyncio w sandboxie, błędny unsigned w nowym wejściu testowym i brak podkładu
planera w jednej komendzie. Naprawiono środowisko/fixture i powtórzono testy.
Przeglądarkowy TCP przekroczył raz limit oczekiwania na programowy WebGL i
przeszedł osobno; konfiguracja trafiła na równoległą przebudowę dist i przeszła
po jej zakończeniu. Nie zmieniano tolerancji produkcyjnych, aby zaliczyć testy.
Vite zachowuje wcześniejsze ostrzeżenie rozmiaru pakietu Three.js >500 kB.

Pasywny **punkt odniesienia** istniejącej aplikacji: 10 s, 2000 odpowiedzi
enkoderów na CAN 1/2/3/5, po 500/oś, mediany 19,949–19,977 ms. Brak ramek ruchu.
To starsze uruchomione binaria (50 Hz w postoju), a nie źródła użyte w porównaniu
CAN przed/po trace (25 Hz). Odczyt HTTP zgłaszał idle i ENABLE=1 na tych osiach;
nie zmieniano tego stanu. CAN 4/6 nie dostarczyły pomiarów.

**Nowego odbioru sprzętowego trace nie zaliczono.** Automatyczna kontrola
uprawnień odrzuciła `sudo systemctl restart robot-web-status.service`: restart
działającego stosu może przerwać obsługę robota i uruchamia komunikację startową
CAN, czego nie obejmuje zgoda na sam pasywny nasłuch. Nie omijano blokady.
Usługa nadal wymaga zatwierdzonego restartu, a następnie pasywnej obserwacji
`scripts/observe_shaft_trace.py` i sprawdzenia HTTP/CSV przy zamkniętym GUI.
W tej sesji nie wysłano ruchu, ENABLE, zerowania ani SYNC i nie zmieniono flag PTP.

## 23.10.2026 — doczytywanie historii podczas przeciągania

Zakres: wyłącznie frontend modułu Diagnostyka. Zastąpiono trailing debounce
120 ms i anulowanie pobrania przy każdym ruchu myszy przez `ShaftHistoryCache`:
stałe przedziały szerokości okna, margines po obu stronach, LRU do ośmiu
przedziałów i najwyżej dwa zapytania równoległe. Priorytety wynikają z bieżącego
widoku, bez kolejki dawnych ruchów. Bieżące próbki WS mają osobny ograniczony
bufor. Renderer wybiera próbki widocznego zakresu; nie łączy niepobranych luk.
API, rejestrator, ROS, CAN i PTP pozostają bez zmian.

Przed zmianą kodu wykonano bazę na dotychczasowym zbudowanym panelu. Porównanie
wykonuje zewnętrzny skrypt `web/test/diagnostics-prefetch.browser.cjs`, bez
narzędzi pomiarowych w interfejsie użytkownika i bez integracji CI. Chromium
działa headless na atrapach HTTP/WS, bez otwierania fizycznego CAN. Każda strona
pomiaru: siedem nowych kontekstów, sześć osi × 3001 próbek (120 s / 25 Hz),
okno 10 s, 600 ms na rozruch/bufor, opóźnienie HTTP 80 ms, 24 ruchy myszy
z odstępem 25 ms. Ostatnie 12 ruchów każdej próby obejmuje dane całkowicie
poza pierwotnym widokiem. Mysz pozostaje wciśnięta i porusza się bez przerwy
120 ms, która wcześniej dopiero uruchamiała HTTP.

Metadane: ten sam komputer i5-7500T / Linux 6.8, Node 22.14, Playwright 1.64.1,
Vite 7.3.7, viewport 1920×1080. Bez wyłączności hosta i przypięcia CPU.
Pliki wynikowe: `log/benchmarks/shaft-trace-prefetch-20261023/{before,after}.json`.

| Miara | Przed | Po |
| --- | ---: | ---: |
| Klatki z linią podczas ciągłego przeciągania poza pierwotny zakres | 0/84 | 84/84 |
| Dane narysowane jeszcze przed puszczeniem myszy | 0/7 prób | 7/7 prób |
| Oczekiwanie od puszczenia do kontrolnego odczytu linii, mediana [min–max] ms | 222,13 [214,31–223,55] | 4,78 [3,84–9,75] |
| Zapytania historii na próbę | 2 | 5 |
| W tym zapytania rozpoczęte przy wciśniętej myszy | 0 | 2 |
| CPU zadań głównego wątku renderera, mediana [min–max] % | 24,82 [22,67–25,18] | 33,22 [32,09–42,19] |

Miara adaptera od puszczenia myszy zmalała o **97,85%**, ale po zmianie próbki
były już wcześniej narysowane: około 4,8 ms obejmuje obsługę mouseup i sprawdzenie
testowe, a nie opóźnienie pobrania danych. Dodatkowe zapytania są ceną pobrania
marginesu; nie powstają przy każdym przesunięciu kursora. W każdej próbie limit
wynosił pięć żądań, przy asercji najwyżej dziesięciu. Skrajnie szybki skok poza
zachowany margines nadal zależy od czasu HTTP, lecz pobieranie nie czeka na
zakończenie gestu.

CPU renderera wzrosło o **8,40 pp / 33,84% względnie**. Przed zmianą wykresy
przez większą część tej fazy były puste; po zmianie rysowały sześć przebiegów.
Pomiar używa przyrostu `TaskDuration` z DevTools / czasu ściennego i obejmuje
główny wątek renderera. **Nie jest CPU całego procesu przeglądarki ani aplikacji
ROS** — tych wartości w tym zadaniu nie mierzono. Nie przypisujemy całego
narzutu samej klasie cache. Wynik potwierdza poprawę dostępności danych przy
większym koszcie rzeczywistego rysowania.

Odtworzenie na zbudowanym panelu, spoza GUI i bez sprzętu:

```bash
node web/test/diagnostics-prefetch.browser.cjs --baseline > log/benchmarks/trace-before.json
# Po zmianie i ponownym buildzie tego samego panelu:
node web/test/diagnostics-prefetch.browser.cjs > log/benchmarks/trace-after.json
```

`--baseline` zbiera te same miary, bez wymagania wstępnego buforowania. Wariant
końcowy wymaga gotowych danych we wszystkich 84 klatkach i kontroluje liczbę
zapytań. Skrypt podsłuchuje rysowanie canvas wyłącznie w teście; nie dodaje
telemetrii do kodu produktu.

Końcowe sprawdzenia ograniczono do zmienianego modułu: build TypeScript/Vite,
**11 testów** `shaftTrace.test.ts` / `shaftHistoryCache.test.ts`,
`diagnostics.browser.cjs` oraz siedem prób `diagnostics-prefetch.browser.cjs`.
Testy obejmują margines, zmianę kierunku, limit pamięci/równoległości, brak
anulowania przy ruchu, odrzucanie spóźnionych odpowiedzi po zmianie skali,
odświeżenie niepełnego przedziału po live, luki, szczyty, błąd HTTP i zamknięcie.
Nie uruchamiano testów pozostałych modułów ani prób sprzętowych.

## 2026-10-24 — podstawowy PTP: ACK zestawu i oczekiwanie grupy

Cel: zachować lokalną trajektorię i regulację w MKS, zastąpić natychmiastowy fault
odchyłki oczekiwaniem, odczytywać kąt 10 Hz oraz zebrać sześć ACK F5 we wspólnych
5 ms. Nie jest to pomiar fizycznej płynności ani odbiór magistrali.

Zakres zmienionych podprogramów: `configure_program_authorization`,
`prepare_program/execute_program/program_setup_reply`, `schedule_program`,
`begin_program_check/check_program_positions/program_batch_window`;
`CanBusManager::tick/can_send/expire/send_query/schedule_feedback/schedule_background`,
rezerwacja budżetu i `mandatory_cycle_bits`, `ManualMotorController::observe_positions`.
Kompilacja programu jest mierzona w prepare, z odrębną tolerancją PTP; kod JTC
nie otrzymuje tej tolerancji. Nowe parametry API ROS i metadana okresu publikowana
przez diagnostykę są sprawdzone funkcjonalnie, bez osobnego pomiaru ich narzutu.

Przed zmianą logiki zapisano bazę istniejących `can.stream4`/`can.stream6`,
rozszerzając wcześniej adapter o tryb pomiaru CPU PTP w czasie rzeczywistym.
Przed/po mają identyczny adapter, punkty q/v/a 0→0,1→0 rad w 2 s oraz 2500
syntetycznych ticków 1 ms na program. Cykl pomiarowy obejmuje przygotowanie,
uzbrojenie i cały program, nie pojedynczy okres CAN. Zmiana zachowania może
wydłużyć wykonanie; scenariusz nadal mieści cały program w tej samej liczbie ticków.
Nie uruchamiano reszty rejestru benchmarków ani GUI/CI do pomiaru wydajności.

| Scenariusz | Cykle × serie | Przed wall / CPU [µs/program] | Po wall / CPU [µs/program] | Spadek wall / CPU [%] | Rozrzut wall przed / po [µs] |
| --- | --- | --- | --- | --- | --- |
| `can.stream4`, CAN 1/2/3/5 | 200 × 7 | 1587,439 / 1587,358 | 1437,384 / 1437,296 | 9,45 / 9,45 | 1573,080–1614,363 / 1430,827–1442,558 |
| `can.stream6` | 100 × 7 | 2116,678 / 2116,594 | 1848,832 / 1848,620 | 12,65 / 12,66 | 2067,193–2126,545 / 1843,848–1854,553 |

Wartości centralne: mediany średnich siedmiu serii; rozrzut: min–max, nie percentyle
pojedynczych ticków. Metadane: Intel i5-7500T, 4 CPU, affinity 0–3, powersave,
Linux 6.8.0-138-generic, GCC 11.4.0, Release `-O1 -DNDEBUG`, rozgrzewka >=1 s.
Oba porównania narzędzia potwierdziły zgodność metadanych, danych i liczby cykli.

**CPU procesu:** dwa odrębne emulatory (4 i 6 osi) uruchomione równolegle,
1000 ticków/s czasu ściennego, program co 3 s. Każdy przebieg trwał 19 s:
7 programów i 19000 ticków przed i po. Po 2 s rozgrzewki obserwowano 15 s,
co 1 s, przez `/proc`. Oba procesy zużyły **0,420 → 0,420 s CPU**, średnio
około **2,800% → 2,800% jednego rdzenia** każdy, maksima około 4%.
Obliczone różnice średnich poniżej 0,01% względnie wynikają z różnic długości
okna; **brak mierzalnej poprawy CPU procesu** przy rozdzielczości liczników.
To po jednym oknie przed/po; brak rozrzutu między powtórzonymi oknami.
Nie mierzono CPU nowej usługi ROS ze sprzętem — nie była uruchamiana.

Procedura odtworzenia bazy i pomiaru po zmianie (nowe nazwy plików przy każdym
uruchomieniu; narzędzie nie nadpisuje wyników):

```bash
cmake -S benchmarks -B build/benchmarks -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
python3 scripts/benchmark_ros.py run --case can.stream4 --context ptp-ack-wait-offline --output log/benchmarks/ptp-stream4-base.json
python3 scripts/benchmark_ros.py run --case can.stream6 --context ptp-ack-wait-offline --output log/benchmarks/ptp-stream6-base.json
# Po zmianie: ten sam --case i --context, dodatkowo --baseline <plik-bazy>.
# Następnie compare --before <baza> --after <nowy-plik>.
CAN_BENCH_PTP_REALTIME_SECONDS=19 build/benchmarks/can_stream4 > /tmp/ptp-cpu4.json & ptp_pid4=$!
CAN_BENCH_PTP_REALTIME_SECONDS=19 build/benchmarks/can_stream6 > /tmp/ptp-cpu6.json & ptp_pid6=$!
sleep 2
python3 scripts/benchmark_ros.py watch --pid stream4="$ptp_pid4" --pid stream6="$ptp_pid6" --duration 15 --interval 1 --context 'PTP 4/6: 1000 tickow/s, jeden program co 3 s, emulatory' --output log/benchmarks/ptp-cpu-new.json
wait "$ptp_pid4" "$ptp_pid6"
```

Lokalne wyniki: `log/benchmarks/ptp-ack-wait-20261024/`: `stream{4,6}-before.json`,
`stream{4,6}-after-final.json`, porównania i `cpu-{before,after,compare}.json`.
Pliki `*-after.json` są wcześniejszym pomiarem roboczym; tabela używa wyłącznie
`*-after-final.json`, po skończonych kompilacjach i testach.

Scenariusze regresji wykazały przed zmianą: wymóg flag odbioru, STOP przy utracie
odczytu, brak przyjęcia sześciu ACK blisko 5 ms oraz fałszywy fault po niezerowym
celu >starej tolerancji JTC. Nie mają porównywalnego czasu wykonania sukcesu przed
zmianą; są oceniane funkcjonalnie. Po zmianie: **40 testów wykonawcy/kompilatora,
7 wybranych JTC, 5 JOG, 3 parametrów ROS i 4 CanSystem — bez błędów**.
Build trzech pakietów w izolowanym overlay Release -O1, frontend typecheck/build
i przeglądarka na atrapach statusów PTP przeszły. Nie wykonano ruchu ani restartu.

Wniosek: koszt syntetycznych programów spadł o 9–13%, CPU procesów bez mierzalnej
poprawy. Model zajętości CAN wynosi 22,47% dla czterech i 33,33% dla sześciu osi,
poniżej 40%; zachowanie budżetu jest sprawdzane także przy limicie 26%.
Osiągalność wspólnego ACK w 5 ms, realna dynamika i skuteczność STOP wymagają
osobnego odbioru. Historyczne wyniki sprzętowe nie zostały podniesione do zaliczenia.

## Próby stanowiskowe PTP — 2026-10-24

Seria sprzętowa z [raportem dziewięciu prób](motion_hardware_results.md#podstawowy-ptp--2026-10-24)
nie zmieniła logiki ani optymalizacji produkcyjnej biblioteki. Badano rzeczywisty
`CanBusManager::tick`, `schedule_program` i obsługę ACK/pozycji w nowym procesie
stanowiskowym; wejście: te same osie, +20° i powrót, quintic nominalnie 20 ms,
ACK zestawu 5 ms, pozycja 100 ms, tolerancja 0,5°, rozkazy do 1 RPM wału.

Interwencje dotyczyły przygotowania stanowiska i częstości wywołania `tick`:
1 ms → 0,5 ms → 0,25 ms → 0,2 ms; ostatnia próba miała affinity CPU2.
Dla czterech osi pełny ruch przy 0,5 i 0,25 ms osiągał oba cele, ale inne próby
z tymi samymi okresami przerwał ACK. Także 0,2 ms z affinity nie wystarczyło.
Maksymalny czas pełnego ACK w dwóch ukończonych parach czterech osi wyniósł
4,818 i 4,090 ms; nie są to maksima całej serii, która zawierała timeouty.

Nie wykonano porównywalnej bazy czasów samych podprogramów ani pomiaru CPU
przy tych czterech częstotliwościach. Nie podajemy zmiany procentowej ani
rozrzutu wydajności: różne momenty przerwań, pozycje startowe i liczba
ukończonych punktów uniemożliwiają porównanie czasu całych prób jako benchmarku.
Wniosek: **brak podstaw do uznania tej interwencji za optymalizację**; krótszy
okres IO nie gwarantuje 5 ms i wymaga pomiaru CPU przed zmianą produkcyjnego
`BusRuntime`. Poprzednie emulowane `can.stream4`/`can.stream6` i CPU pozostają
wynikami osobnego porównania powyżej; nie przypisujemy ich sprzętowi.

Metadane: ta sama biblioteka Release `-O1 -DNDEBUG`, Linux SocketCAN `can0`
500 kbit/s, rozsprzęglone MKS 42D, mikrokrok 16. Źródła, SHA, znaczniki
monotoniczne i ślady: `log/hardware-ptp/group-20261024/`. Bez pomiarów CPU,
bez testu sześciu osi i bez powtórzenia całej kolekcji benchmarków.

## ACK 10 ms i rezerwa CAN — 2026-10-24

Zakres tego zadania: domyślny `program_ack_timeout` oraz parametr ROS
`execution_ack_timeout_ms` 5→10 ms. Wpływa to na cykle
`prepare_program/execute_program`, `schedule_program/program_batch_window`,
`schedule_feedback/schedule_background` i `tick`, przez długość okna bez
rozpoczynania diagnostyki. Nie zmieniano implementacji tych metod ani okresu
pobudzania IO, JOG lub JTC. Diagnostyka zyskała metadane limitu łącznego CAN
60% i rezerwy przyszłego modułu 20%; bieżący robot nadal planuje najwyżej 40%.

Przed zmianą zachowano bazę can.stream4/6; po zmianie te same punkty q/v/a,
200 programów × 7 serii i 2500 syntetycznych ticków 1 ms na program.
Sprawdzono identyczność ustawień i SHA adaptera. Scenariusze obejmują
przygotowanie i wykonanie, z tą samą liczbą cykli, nie pojedynczy tick.
Biblioteka managera przyjmuje deadline jako parametr; jej SHA pozostał ten sam,
a nowa kompilacja klienta/API przekazuje nowy domyślny parametr 10 ms.

| Scenariusz | Programy × serie | Przed wall / CPU [µs/program] | Po wall / CPU [µs/program] | Spadek wall / CPU [%] | Rozrzut wall przed / po [µs] |
| --- | --- | --- | --- | --- | --- |
| can.stream4 | 200 × 7 | 1446.737 / 1446.751 | 1411.055 / 1408.808 | 2.47 / 2.62 | 1437.840–1455.237 / 1384.046–1436.044 |
| can.stream6 | 200 × 7 | 1851.687 / 1850.932 | 1784.245 / 1784.038 | 3.64 / 3.61 | 1840.486–1871.638 / 1758.940–1831.656 |

Mediany średnich serii; rozrzut min–max. Intel i5-7500T, 4 CPU, affinity 0–3,
Linux 6.8.0-138-generic, GCC 11.4.0, Release `-O1 -DNDEBUG`, jak w metadanych
plików. Wynik około 2–4% niższy nie oznacza skrócenia fizycznego ruchu:
zmiana okna ACK zmienia też harmonogram i liczbę odczytów w stałym cyklu.
Nie jest to optymalizacja implementacji ani gwarancja deadline na sprzęcie.

CPU procesu w emulacji: przed/po dwa procesy 4 i 6 osi równolegle, 19 s,
po 19000 ticków i 7 programów; 2 s rozgrzewki, obserwacja 15 s co 1 s.
W obu przypadkach zakres 0–3, tło hosta nie było izolowane.

| Scenariusz | CPU przed → po [s w oknie] | Średnio przed → po [% jednego rdzenia] | Maks. próbek przed → po [%] |
| --- | --- | --- | --- |
| can.stream4 | 0,390 → 0,400 | 2,600 → 2,667 | około 4 → 4 |
| can.stream6 | 0,410 → 0,340 | 2,733 → 2,267 | około 4 → 4 |

Dla 4 osi CPU wzrósł około 2,56% względnie, dla 6 spadł około 17,07%.
To pojedyncze okno na wariant, z rozdzielczością liczników /proc, bez rozrzutu
między powtórzeniami; nie traktujemy go jako dowodu poprawy CPU. W serii
sprzętowej CPU nowego procesu stanowiskowego wyniósł 6,74–8,59% rdzenia,
z aktywnym trace i przygotowaniem; nie ma porównywalnego pomiaru sprzętu ACK5
z poprzedniej serii. Nie porównujemy tych wartości z emulacją ani usługą ROS.

Wyniki, komendy i metadane: `log/hardware-ptp/ack10-tolerance-20261024/`,
`stream{4,6}-{before,after}.json`, `cpu-{before,after}.json`,
`cpu{4,6}-{before,after}-run.json`. Użyto tej samej procedury `benchmark_ros.py
run --case can.stream4|can.stream6 --context ack10-reserve20`, po zmianie
`--baseline` wskazuje odpowiednią bazę. CPU: ta sama procedura realtime/watch
opisana wyżej. Nie uruchamiano reszty rejestru benchmarków ani GUI.

Sprawdzenia funkcjonalne: build trzech pakietów; 40 testów PTP/kompilatora,
3 parametrów ROS OK; sześć prób sprzętu z ACK10/zmianą tolerancji.
[Raport i procent zgodności](motion_hardware_results.md#ack-10-ms-i-porównanie-tolerancji--2026-10-24):
jedna pełna próba, 21,02% wobec czasu nominalnego i 85,76% wobec uruchomionych
celów w paśmie ±0,5°. ACK10 nie usunął braków ACK i błędów CAN; odbiór
niezaliczony. Model 6 osi 33,33% + przyszła rezerwa 20% = 53,33% <60%,
bez fizycznego testu sześciu osi i bez wygenerowanego ruchu przyszłego modułu.

## 24.10.2026 — ciągłe F5 100 Hz, lookahead i feed-forward

Zakres: `prepare_program/execute_program/schedule_program`, kompilacja strumienia,
`streaming_setpoint`, `tick/schedule_feedback/schedule_background/reserve`,
`SocketCan::receive_loop/position_feedback`, skrzynki `BusRuntime` oraz
`CanSystem::read/write/on_activate`; dodatkowo F6 bez ACK w module ręcznym.
Cel: usunięcie pauz ACK/tolerancji pośredniej, aktualna referencja co10 ms,
asynchroniczny RX i brak mutexów/odbioru w read/write.

Przed edycją zapisano źródła, zakres/hash i bazę `can.stream4/6` przy500k:
mediany1390,796/1774,217 µs za program (200 iteracji ×7). Nie porównujemy tych
wartości procentowo do profilu1M, ponieważ zmieniłyby się warunki magistrali.
Do porównania kontrolowanego odtworzono stary wykonawca ACK10/20 ms z zapisanych
źródeł i wcześniejszej biblioteki; **jedyna adaptacja starego managera to
zezwolenie na 1 Mbit/s**. Stara biblioteka zachowała SHA256
`8cca3bef20c85118131a46de7ebb24a1c42383e876683cee1227513a2039978a`.
Nagłówki bazy odtworzono w osobnym katalogu zgodnie ze starym ABI (instalowane
nagłówki symlink-install wskazują już nowe źródła). Smoke test bazy zakończył
program bez fault. Jest to kontrolowana baza odtworzona, nie ponowny test sprzętu.

Obie strony mają **ten sam adapter emulatora**, SHA256
`4c210c9ebcde9b3291f712accc87540782a67c3439db3c33deff05e2cbde3ce9`.
Dane: cztery/sześć osi, przekładnia10, q=0→0,1→0 rad/2 s, 2500 ticków1 ms
na program, ten sam profil i zachowanie odczytów/konfiguracji w emulatorze.
Różnią się zamierzone algorytmy: stary50 Hz+ACK10+kontrola pośrednia kontra
nowy100 Hz bez ACK/SYNC z lookahead25 ms i FF1,15. Mierzymy cały program
z kompilacją i uzbrajaniem, nie czas samej ramki ani opóźnienie na przewodzie.

Host: Ubuntu22.04, kernel6.8.0-138, i5-7500T, affinity0–3; GCC11.4,
C++17 Release `-O1 -DNDEBUG`. Warmup1 s, 7 serii po 200 programów, bez równoległego
builda i testów przeglądarki. Narzędzie: `benchmarks/harness.hpp`, osobne binaria
z tym samym adapterem; `--iterations 200`. Jednostka poniżej: µs/program.

| Scenariusz | Wall przed (min–max) | Wall po (min–max) | Oszczędność wall | CPU przed→po | Oszczędność CPU |
| --- | ---: | ---: | ---: | ---: | ---: |
| can.stream4 | 1401.952 (1390.288–1415.335) | 1273.744 (1249.636–1301.488) | 9.14% | 1401.826→1273.701 | 9.14% |
| can.stream6 | 1780.383 (1755.912–1924.953) | 1768.724 (1745.983–1801.683) | 0.65% | 1780.104→1768.650 | 0.64% |

CPU działających procesów: uruchomiono równolegle jeden emulator4 osi i jeden6,
każdy przez 19 s z tickiem1 ms i programem co 3 s (7 programów/19000 ticków).
`benchmark_ros.py watch` zbierał `/proc` przez15 s od2 s po starcie, interwał 1 s,
ten sam kontekst na obu stronach. 100% oznacza jeden rdzeń.

| Proces | CPU przed→po | Różnica względna | Maksimum próbki przed→po |
| --- | ---: | ---: | ---: |
| stream4 | 2.466%→2.600% | +5.41% | 4.000%→4.003% |
| stream6 | 2.333%→2.200% | -5.71% | 4.000%→3.000% |

Wniosek: koszt całego programu4 osi spadł o9,14%; dla6 osi zmiana0,65% mieści
się w rozrzucie, więc brak dowodu przyspieszenia. CPU procesu4 osi nieznacznie
wzrósł,6 osi spadł; przy małych czasach i kwantyzacji `/proc` nie traktujemy
tego jako istotnego zysku. Główny efekt to dwukrotnie częstszy, nieblokowany
strumień. To nie pomiar płynności ani zgodności fizycznej trajektorii.

Dla nowego wątku SocketCAN RX, atomowych read/write i JOG **nie ma porównywalnej
bazy CPU z fizycznym500k ani nowej sesji sprzętowej**. Emulator nie otwiera CAN
ani nie uruchamia SocketCan RX. Test UNIX socketpair sprawdza prawdziwą pętlę
RX/cache i kolejkę, ale nie jej fizyczną latencję/CPU. Nie przypisujemy kosztów
całego managera tym podprogramom. Odbiór czasowy100 Hz/1M pozostaje do wykonania.

Lokalne artefakty (ignorowane przez Git): `log/streaming100-20261024/`,
`scope.json`, `before/`, `baseline1M/metadata.json`, `comparison-metadata.json`,
`stream*-1M-{before,after}.json`, `cpu-1M-{before,after}.json`,
`comparison-summary.json`, `after-source-sha256.json`. Powtórzenie surowego
pomiaru: `baseline1M/can_stream4 --iterations 200` i
`build/streaming100-dev/can_stream4 --iterations 200`, analogicznie stream6.
Baza zachowuje stary zegar i ACK; nie należy jej używać do sterowania sprzętem.

Sprawdzenia tej wersji: cały workspace11 pakietów (bez ostrzeżeń kompilatora),
327 testów ROS bez błędów,91 testów panelu,9 scenariuszy Chromium na atrapach,
rosdep, bash/AST/XML/YAML, dry-run konfiguracji1M/txqueuelen1000. Vite nadal
zgłasza istniejący rozmiar chunku>500 kB; testy modeli mogą logować brak geometrii
kolizyjnej przykładowego URDF. Systemd verify zgłasza niezwiązane ostrzeżenia
hosta (netplan/snapd); szablon usługi przechodzi kontrolę. Początkowy test ROS
załadował stary prefiks bibliotek; końcowy używa jawnego `--install-base` nowego
workspace. Test nawigacji czeka teraz na aria-current; przejściową porażkę testu
JOG powtórzono bez zmiany progów i uzyskano sukces. Ruchu/restartu nie wykonano.

Końcowa poprawka CanSystem pobiera czas po atomowym odczycie cache, aby
równoległy RX nie udawał pomiaru z przyszłości. Adres bierze ze snapshotu
CanSystem, bez odczytu mutowalnej konfiguracji managera. Po poprawce ponownie
327 testów ROS bez błędów. Parser bitrate Netlink potwierdził 500000 bez
otwierania SocketCAN; stan usługi/PID bez zmiany. Nowy panel pokazuje tryb
bez ACK zamiast pustych metryk ACK/SYNC; 91 testów i scenariusz shell ponownie OK.

## 24.10.2026 — poprawki ujawnione na rzeczywistych MKS po wdrożeniu

Zakres: send_manual i ManualMotorController::tick (F7), następnie
schedule_feedback/schedule_background oraz pierwszy zestaw schedule_program.
Przy CanRSP=Disable napęd nie odpowiada na F7: sekwencja ENABLE czekała
niepotrzebnie 20 ms. Ramki i stan przed poprawką: log/streaming100-deployment/
f7-before oraz log/hardware-ptp/streaming100-20261024/enable-diagnostic-*.
Test tego samego braku odpowiedzi przed zmianą: FAIL; po: PASS;
zgubione F7 nadal blokuje ENABLE przez F1. Test trwał 1 ms przed i po;
/usr/bin/time miał 0,00 s CPU obu procesów, więc brak rozdzielczości i brak
podstaw do procentu oszczędności. To poprawka poprawności, nie optymalizacja CPU.
107 testów pakietu CAN po zmianie OK, fizyczne ENABLE/zero/disable 1/2/3/5 OK.

Drugi problem: przed pierwszym F5 wykonawca uruchamiał 31 około 1 ms wcześniej.
Dwie osie osiągnęły cel, lecz zapis zawierał 221 ramek błędów BIT1/controller.
Test kolejności odtworzył 4 przedwczesne 31 przed pierwszym F5. Poprawka
wyrównuje początek odczytów do pierwszego F5; nie czeka na encoder/ACK.
Po zmianie 108 testów pakietu CAN OK. Narzędzie prób przerywa przy błędach CAN.

Pomiar: ten sam benchmarks/can_schedule.cpp i binarium dynamiczne,
1 Mbit/s w emulatorze, q=0→0,1→0/2 s, 2500 ticków 1 ms, 200 iteracji×7,
warmup 1 s, GCC11.4/O1/NDEBUG, i5-7500T/affinity0–3. Aplikacja sprzętowa
była zatrzymana. Zakres pomiaru to cały program 4/6 osi, wraz z uzbrajaniem.

| Przypadek | Wall przed µs/program (min–max) | Wall po µs/program (min–max) | Oszczędność | CPU mediana przed→po µs |
| --- | ---: | ---: | ---: | ---: |
| can.stream4 | 1296.218 (1285.688–1302.662) | 1311.765 (1288.521–1339.772) | -1.20% | 1296.120→1310.224 |
| can.stream6 | 1783.266 (1757.874–1798.648) | 1771.875 (1739.196–1826.933) | 0.64% | 1783.229→1771.891 |

CPU działających emulatorów: 4 i 6 osi równolegle, 19 s, 7 programów co 3 s,
tick 1 ms. Watch 15 s po 2 s, interwał 1 s. 100% oznacza jeden rdzeń.

| Proces | CPU przed→po | Maksimum próbki przed→po |
| --- | ---: | ---: |
| stream4 | 3.133%→2.466% | 4.000%→3.001% |
| stream6 | 2.533%→2.800% | 4.001%→4.998% |

Wniosek: wall 4 osi pogorszył się o 1,20%, 6 osi poprawił o 0,64%; zakresy
pokrywają się, brak istotnego zysku. Celem była poprawna kolejność transmisji.
CPU nie odwzorowuje RX sprzętu ani skutków retransmisji; nie przypisujemy mu
poprawy komunikacji. Artefakty: log/streaming100-deployment/phase-before,
phase-after, phase-{before,after}-cpu.json; snapshoty źródeł przed zmianą zachowano.

### Ostatnie 4 ms cyklu bez nowych zapytań — interwencja po drugiej próbie

Pierwsza poprawka kolejności nie usunęła kolizji F1/F5 w kolejnych cyklach.
Ślad drugiej próby wykazał F1~1,5 ms przed F5; test sprzętowy zatrzymał się.
Dodano `program_query_window` dla schedule_feedback/schedule_background:
nowe odczyty PTP rozpoczynamy tylko w fazie 0–6 ms względem zegara F5.
Ruch nie czeka na pomiar. Test na emulatorze z opóźnieniem 2 ms odtwarzał
odczyty w końcu cyklu przed zmianą, po zmianie przechodzi. 109 testów CAN OK.

Bazą są bezpośrednio poprzednie pomiary phase-after (zebrane przed edycją),
identyczny adapter i binarium, 1M/200 iteracji×7/2500 ticków, konfiguracja jak wyżej.
Nowe pliki: guard-before/, guard-after/, phase-guard-after-cpu.json.

| Przypadek | Wall przed µs/program (min–max) | Wall po µs/program (min–max) | Oszczędność | CPU mediana przed→po µs |
| --- | ---: | ---: | ---: | ---: |
| can.stream4 | 1311.765 (1288.521–1339.772) | 1226.477 (1196.302–1261.038) | 6.50% | 1310.224→1224.988 |
| can.stream6 | 1771.875 (1739.196–1826.933) | 1672.640 (1663.275–1720.920) | 5.60% | 1771.891→1672.551 |

CPU procesu: ten sam watch 15 s po 2 s, 4+6 osi równolegle przez 19 s.

| Proces | CPU przed→po | Maksimum przed→po |
| --- | ---: | ---: |
| stream4 | 2.466%→3.600% | 3.001%→5.001% |
| stream6 | 2.800%→2.400% | 4.998%→3.996% |

Wniosek: wall zmalał o6,50%/5,60%, ale CPU procesu4 osi wzrósł
2,466→3,600% rdzenia (około46% względnie); dla6 osi spadł2,800→2,400%.
Nie jest to jednolita poprawa wydajności. CPU dotyczy całego procesu; nie
przypisujemy tej zmiany jednej funkcji. Celem była ochrona kolejności CAN.
Po poprawce dwie i cztery osie osiągnęły cele bez błędów CAN; to efekt
potwierdzony na sprzęcie w ograniczonym profilu 1 RPM/acc 1, nie pełny odbiór RT.
Rzeczywisty proces C++ z trace: 7,78% rdzenia (J1), 9,37% (J1+J2 po poprawce),
10,72% (cztery osie). Brak porównywalnej bazy sprzętowej przed/po dla tego CPU:
wcześniejsze błędy CAN zmieniały liczbę retransmisji i długość próby. Te liczby
nie są procentem optymalizacji. Pełne dane i ograniczenia: [raport](motion_hardware_results.md).

Końcowa biblioteka jest rzeczywiście załadowana w produkcyjnym can_diagnostics;
SHA256 4e8cd2a594d43a1ff71684fa308bf618abbfd522f818fe7c02e559bbe433d586.
Weryfikacja: 109 testów CAN po ostatniej poprawce i testy 2 konsumentów w domenie
ROS 87, odseparowanej od działającego ROS 0. Suma zapisanych wyników 331 bez błędów
zawiera też wcześniejsze wyniki niezmienianych pakietów; nie przypisujemy ich
nowej sesji. Produkcja po testach active/fresh, 500k/TX1000, bez ruchu/fault.
