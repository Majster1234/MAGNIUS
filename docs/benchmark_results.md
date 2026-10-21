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
