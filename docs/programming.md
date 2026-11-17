# Programowanie — edytor programów ruchowych (bloki)

Strona `#programming` („⌖ Programowanie”), wdrożona 09.11.2026 na podstawie szkicu
`temp/programowanie_trajektorii/v2b_bloki_stal.html`. Zakres: tworzenie i zapis programów (bloki, zmienne, pozycje,
biblioteka) oraz — od 09.11.2026 — **logika wykonania**: wykonawca programów w moście z dwoma kursorami i ciągami ruchów
PTP/LIN z promieniem pominięcia punktu (sekcja „Logika wykonania”). Od 10.11.2026 biblioteka ma kategorie (ruchowe,
czasowe, logiczne, matematyczne, pętle i skoki), a płótno jest **schematem blokowym** (rozgałęzienia „jeśli”/„wybierz”).
Widok tekstowy jest podglądem generowanym z bloków.

## Pliki

| Ścieżka | Zawartość |
| --- | --- |
| `$XDG_STATE_HOME/robot/programs/` (domyślnie `~/.local/state/robot/programs`, parametr mostu `programs_directory`) | katalog danych |
| `…/library/commands.json` | **biblioteka rozkazów** (definicje bloków); przy pierwszym starcie kopia domyślnej z `robot_web_bridge/program_library/commands.json`; można edytować ręcznie (zła składnia → użyta domyślna + ostrzeżenie w GUI) |
| `…/library/variables.json` | biblioteka zmiennych wielokrotnego użytku (`{"schema_version":1,"items":[…]}`) |
| `…/library/positions.json` | biblioteka pozycji wielokrotnego użytku |
| `…/programs/<id>.json` | programy: zmienne, pozycje, drzewo bloków |
| `…/programs/.trash/` | usunięte programy (`<id>-RRRRMMDD-HHMMSS.json`) |

Zapis atomowy (plik `.tmp` + `fsync` + `rename`). Rewizja = pierwsze 16 znaków SHA-256 treści; zapis z nieaktualną
rewizją → HTTP 409 (np. program zmieniony w drugim oknie).

## Format programu (schema 1)

```json
{"schema_version": 1, "id": "paletyzacja_3", "name": "Paletyzacja 3×", "description": "",
 "created_at": "…", "updated_at": "…", "revision": "…",
 "variables": [{"name": "licznik", "type": "INT", "init": 0, "note": "…"},
               {"name": "v_lin", "type": "REAL", "init": 50, "unit": "mm/s", "min": 1, "max": 100}],
 "positions": [{"name": "Home", "joints_deg": [0, 0, 0, 0, 45, 0], "source": "pomiar", "created_at": "…"}],
 "blocks": [{"id": "b1", "command": "move_ptp", "params": {"position": {"position": "Home"}, "speed": 30}},
            {"id": "b2", "command": "for", "params": {"counter": {"var": "i"}, "from": 1, "to": 3},
             "body": [{"id": "b3", "command": "move_lin", "params": {"position": {"position": "Odbiór"}, "speed": {"var": "v_lin"}}}]}]}
```

- Typy zmiennych: `INT`, `REAL`, `BOOL`, `POZYCJA` (nazwa pozycji), `TEKST`.
- Nazwy zmiennych i pozycji: litera na początku, dalej litery (także polskie), cyfry, `_`; do 40 znaków; unikalne.
- Pozycje: 6 kątów przegubów w stopniach (`joints_deg`); „Zapamiętaj bieżącą pozycję robota” czyta
  `/api/v1/manual/status` (wymaga znanej pozycji przegubu na wszystkich 6 osiach).
- Parametr bloku: literał albo odwołanie `{"var": …}` / `{"position": …}`; `null` = nieuzupełniony.
- Bloki z gniazdami (pętle: `body`; `if`: `body` + `else`; `switch`: `body` = wyłącznie bloki `case` + `else`;
  `case`: `body`) — klucze gniazd zgodne z `slots` rozkazu. Gniazdo z `accepts` przyjmuje tylko wymienione rozkazy,
  a rozkaz z `parents` może stać tylko w takim gnieździe (naruszenie → HTTP 422).
- Limity: 1 MiB, 2000 bloków, zagnieżdżenie 16, 500 zmiennych/pozycji. Brakujące odwołania nie blokują zapisu —
  wracają jako `problems`.

## Biblioteka rozkazów (`commands.json`)

Biblioteka budowana od nowa (09.11.2026, kopia dawnej: `library/commands.json.bak-20261109`); 10.11.2026 rozszerzona
o kategorie i logikę (kopia poprzedniej: `library/commands.json.bak-20261110`). Każdy rozkaz ma obsługę w interpreterze
(`Interpreter.HANDLERS`, test `test_default_library_has_categories_handlers_and_blend_radius`).

| Kategoria | Rozkaz | Blok | Działanie |
| --- | --- | --- | --- |
| Ruchowe | `move_ptp` | `PTP do {position} v {speed} ⌀ {blend_mm}` | prędkość 1–100 % (najwolniejszej osi z konfiguracji); promień pominięcia 0–200 mm |
| | `move_lin` | `LIN do {position} v {speed} ⌀ {blend_mm}` | prędkość TCP mm/s (≤ `tcp.max_linear_velocity`); promień 0–200 mm |
| | `move_rel` | `LIN o ΔX {dx} ΔY {dy} ΔZ {dz} układ {frame} v {speed} ⌀ {blend_mm}` | ruch względny pozycyjny: przesunięcie TCP po prostej (±2000 mm), orientacja bez zmian; mm/s |
| | `rotate_rel` | `obrót o RX {rx} RY {ry} RZ {rz} układ {frame} v {speed} ⌀ {blend_mm}` | ruch względny rotacyjny: obrót narzędzia wokół początku TCP (±360°, kolejno X, Y, Z), położenie bez zmian; °/s ≤ `tcp.max_angular_velocity` (10–360) |
| Czasowe | `wait_time` | `czekaj {seconds}` | postój 0–3600 s po dojeździe; kończy ciąg ruchów (dolot ±0,5°); 0 s = tylko dokładny postój |
| | `wait_operator` | `czekaj na operatora {message}` | robot stoi do „⏵ Kontynuuj” w panelu Wykonanie; kończy ciąg |
| Logiczne | `if` | `jeśli {condition}` | gałęzie TAK (`body`) / NIE (`else`) |
| | `switch` + `case` | `wybierz wg {value}` / `przypadek {match}` | pierwszy włączony przypadek o wartości równej `value`, inaczej `else`; nowy blok ma 2 przypadki |
| | `logic_set` | `ustaw {target} := {condition}` | zmienna BOOL := warunek |
| | `logic_toggle` | `zaneguj {target}` | zmienna BOOL := NOT zmienna |
| Matematyczne | `assign` | `przypisz {target} := {expression}` | dowolna zmienna := wyrażenie / zmienna / pozycja (POZYCJA) |
| | `increment`, `decrement` | `zwiększ/zmniejsz {target} o {step}` | INT/REAL ± krok |
| | `calc` | `oblicz {target} := {a} {op} {b}` | `+ - * / MOD ^ MIN MAX` |
| | `math_func` | `oblicz {target} := {func} ( {arg} )` | ABS, SQRT, SIN, COS, TAN, ASIN, ACOS, ATAN (°), ROUND, TRUNC, FLOOR, CEIL, EXP, LN, LOG |
| | `limit` | `ogranicz {target} do {min} … {max}` | przycięcie do zakresu |
| Pętle i skoki | `for` | `dla {counter} od {from} do {to} krok {step}` | licznik INT, krok ≠ 0 (ujemny = w dół); po pętli licznik ma ostatnią wartość z zakresu |
| | `repeat` | `powtórz {count} razy` | 0–100 000 |
| | `while` | `dopóki {condition}` | warunek sprawdzany przed obiegiem |
| | `repeat_until` | `powtarzaj aż {condition}` | warunek po obiegu (co najmniej 1 obieg) |
| | `break`, `continue` | `przerwij pętlę`, `następny obieg pętli` | najbliższa pętla; poza pętlą → uwaga |
| | `label`, `goto` | `etykieta {name}`, `skocz do {target}` | skok tylko do etykiety w tej samej lub nadrzędnej liście bloków (wyjście z pętli/gałęzi dozwolone, wejście do wnętrza nie) |
| | `stop_program` | `zakończ program` | koniec programu (ostatni ruch się zatrzymuje) |

**Ruch względny** (`move_rel`, `rotate_rel`; układ `baza` albo `narzędzie`) liczy się od celu poprzedniego ruchu
programu (na początku programu lub po czekaniu — od celu poprzedniego ciągu, w pierwszym ciągu od pozycji startowej).
Interpreter wydaje ruch bez kątów (`Move.relative`), a wykonawca przed planowaniem ciągu wyznacza cel
(`ProgramRunner.resolve` → `PlanningClient.relative_target`): FK z poprzedniego celu, przyrost pozy jak w JOG TCP
(`tcp_jog.increment_pose`: przesunięcie, potem obroty wokół X, Y, Z — w bazie zewnętrzne, w narzędziu wewnętrzne), IK
z ziarnem poprzedniego celu (tolerancja 1 µm / 1e-5 rad). Wykonywany jest jako LIN (planer sprawdza całą prostą
i gałąź IK); brak rozwiązania → błąd przebiegu przy bloku z przyczyną (`planning.ik_failure_text`: oś zatrzymana na
granicy zakresu, pozostały błąd, kąty startu). Uwaga: punkt startowy na granicy zakresu osi (np. J2 = górny limit)
zwykle blokuje ruch względny — nawet przesunięcie w bok wymaga minimalnego wyjścia poza limit. Obrót: posuw liniowy z limitu konfiguracji, prędkość kątowa
z bloku. Zmiana wcześniejszej pozycji zmienia też cele ruchów względnych za nią.

**Semantyka.** Bez wejść/wyjść logika jest deterministyczna: interpreter liczy ją z wyprzedzeniem, więc „Sprawdź”
symuluje cały program (wszystkie gałęzie i obiegi, oś czasu). Przypisanie konwertuje do typu zmiennej (INT —
zaokrąglenie ½ w górę; TEKST — zapis wartości; POZYCJA — nazwa istniejącej pozycji), `min`/`max` zmiennej są twardym
zakresem (przekroczenie = błąd przebiegu przy bloku). Warunek musi mieć wartość BOOL. Ochrona: 100 000 kroków logiki,
2000 ruchów na przebieg.

**Wyrażenia** (`condition`, `expression`; własny parser `compile_expression` w `program_interpreter.py`, ta sama
gramatyka w `programModel.ts` `checkExpression`): liczby, `'tekst'`, `TRUE`/`FALSE`, `PI`, zmienne; precedencja od
najsłabszej: `OR`, `XOR`, `AND`, `NOT`, porównanie (`= <> < <= > >=`, bez łańcuchów — łącz przez `AND`), `+ -`,
`* / MOD`, minus jednoargumentowy, `^` (potęga, prawostronna, wykładnik ≤ 100); funkcje jak wyżej oraz `ATAN2(y, x)`,
`MIN(…)`, `MAX(…)`, `LIMIT(min, x, max)`. Słowa kluczowe/funkcje bez względu na wielkość liter; zakres wartości ±1e12.
Błędy składni i nieznane nazwy są uwagami (blokują zatwierdzenie).

Pozycje (punkty) definiuje się w zakładce „Pozycje” (pomiar robota lub ręcznie J1–J6). Literały spoza `min`/`max`
(`integer`) i nieuzupełnione pola są uwagami (`problems`) — blokują zatwierdzenie, nie zapis. Uwagi przepływu
(`flow_problems`): powtórzone/błędne etykiety, skoki do etykiet nieistniejących/niedostępnych, `break`/`continue` poza
pętlą, wyrażenia, typ zmiennej niezgodny z `types`, wartość spoza `options`, krok pętli 0. Przy aktualizacji domyślnej
biblioteki kopię w katalogu danych trzeba podmienić ręcznie (magazyn kopiuje ją tylko, gdy pliku brak).

Format definicji rozkazu: `id`, `category`, `icon`, `name` (krótka nazwa w uwagach/menu), `shape` (kształt w schemacie:
`process`, `decision`, `switch`, `case`, `loop`, `jump`, `label`, `terminator`, `wait`), `help` (podpowiedź), `label`
(tekst bloku z polami `{param}`), `text` (linia podglądu tekstowego), `end` (linia zamykająca, może mieć pola),
`slots` (`name`, `label`, `text`, `accepts`), `parents`, `initial` (bloki początkowe gniazd), `params` (`name`, `kind`,
`label`, `unit`, `default`, `min`, `max`, `integer`, `types`, `options`). Rodzaje parametrów (`kind`): `position`,
`number`, `variable`, `condition`, `expression`, `text`, `bool`, `choice` (`options`), `label`, `output`, `input`,
`value`. Przeciąganie zmiennej/pozycji na pole działa wg zgodności typu (`position` — pozycja albo zmienna `POZYCJA`,
`number` — `INT`/`REAL`, `condition` — `BOOL`, `expression` — dowolna zmienna lub pozycja).

## API (`robot_web_bridge/programs.py`)

| Metoda i ścieżka | Działanie |
| --- | --- |
| `GET /api/v1/programs` | lista (id, nazwa, czas, rewizja, liczba bloków/zmiennych/pozycji) |
| `POST /api/v1/programs` `{"program": {...}}` | nowy program (id z nazwy, unikalny) → 201 |
| `GET /api/v1/programs/{id}` | `{"program", "problems"}` |
| `PUT /api/v1/programs/{id}` `{"program", "expected_revision"}` | zapis; 409 przy konflikcie, 422 przy błędzie struktury |
| `DELETE /api/v1/programs/{id}` | przeniesienie do `.trash/` |
| `GET /api/v1/program-library` | rozkazy, biblioteki zmiennych i pozycji (z rewizjami), katalog |
| `PUT /api/v1/program-library/{variables,positions}` `{"items", "expected_revision"}` | zapis biblioteki |

## Edytor (frontend)

`web/src/ProgrammingPage.tsx`, logika `programModel.ts` (testy `programModel.test.ts`), API `programApi.ts`,
podgląd 3D `programPreviewScene.ts` (robot z geometrii aktywnej konfiguracji w kątach wybranej pozycji, znaczniki
TCP pozycji programu).

- Nagłówek: wybór programu, nazwa, Nowy / Duplikuj / Usuń, przełącznik **Bloki / Tekst**, cofnij/ponów, Zapisz
  (`Ctrl+S`), stan zapisu; ostrzeżenie przy opuszczaniu strony z niezapisanymi zmianami.
- Lewy panel: **Bloki** (paleta z biblioteki rozkazów), **Zmienne** (tworzenie/edycja w oknie z wyborem typu,
  usuwanie z ostrzeżeniem o użyciach, zmiana nazwy aktualizuje odwołania, ⇪ do biblioteki, import z biblioteki),
  **Pozycje** (z pomiaru robota, ręcznie J1–J6, ⟲ nadpisanie z pomiaru, edycja, biblioteka).
- Płótno = **schemat blokowy**: Start → bloki połączone strzałkami → Koniec. „Jeśli” (sześciokąt) rozgałęzia przepływ
  na kolumny TAK / NIE, „wybierz” — na kolumny przypadków (nagłówek przypadku jest blokiem: przeciąganie, edycja
  wartości, „＋ przypadek” na bloku) i „inaczej”; kolumny scalają się pod blokiem. Pętle: ramka z linią powrotu
  i stopką „↺”. Skok i etykieta: zaznaczenie jednego podświetla drugie. Powiększenie − / % / + / ⤢ (dopasuj do
  panelu; zapamiętane w przeglądarce) oraz Ctrl+kółko / szczypanie nad schematem (wokół punktu pod kursorem; samo
  kółko przewija). Na ekranie Programowanie Ctrl+kółko nie powiększa całej strony — podgląd 3D przybliża kamerę. Łączniki (strzałki) są miejscami upuszczenia — podczas przeciągania pojawiają
  się tylko tam, gdzie blok jest dozwolony (`canDrop`: przypadek tylko w „wybierz”, między kolumnami przypadków).
- Upuszczanie: podczas przeciągania układ się nie zmienia; nad wskazanym łącznikiem pojawia się **zielona linia**
  (między kolumnami przypadków — pionowa). Po upuszczeniu program się rozsuwa, a blok „ląduje” z góry z zielonym
  błyskiem (CSS `prg-expand`/`prg-land`; brak przy wyłączonych animacjach).
- Edycja: przeciąganie z palety na łączniki i do gałęzi/pętli, przenoszenie bloków (bez wstawienia
  bloku do własnego wnętrza), kosz, przeciąganie zmiennych (z panelu lub paska nad Start) i pozycji na pola,
  edycja pola po kliknięciu, zaznaczenie + `Delete`, `Ctrl+D` duplikat, wyłączanie bloku, oznaczenie `!` przy
  nieuzupełnionych polach i brakujących odwołaniach.
- Zmiana rodzaju rozkazu bez usuwania bloku: kliknięcie nazwy (np. **PTP ▾**) → lista rozkazów zgodnych (ta sama
  kategoria i gniazda, wspólne parametry; np. PTP ↔ LIN). Id bloku i parametry o tej samej nazwie zostają (wartość
  liczbowa bez przeliczania jednostek — np. 50 mm/s → 50 %); wartość spoza zakresu nowego rozkazu wraca do domyślnej
  (komunikat przy bloku). `programModel.ts`: `compatibleCommands`, `changeCommand`.
- Zakresy osi: pozycja poza zakresem osi (z aktywnej konfiguracji) jest uwagą, jeśli program jej używa
  (`programLimits.ts`), w zakładce „Pozycje” ma czerwony kąt i opis; podgląd 3D pokazuje wybraną pozycję (także po
  zaznaczeniu bloku ruchu) ze wskaźnikami limitów na przegubach jak w sterowaniu ręcznym (łuk + poświata + etykieta
  „J2 + LIMIT!”, ostrzeżenie ≤ 15°, niebezpiecznie ≤ 5°, przekroczenie), czerwonym znacznikiem pozycji i plakietką.
- Pola liczbowe okienek bloków: tekst roboczy (można skasować, wpisać 0, „-”, „0,5”); skasowane pole przyjmuje 0
  (zaznaczone — kolejna cyfra je zastępuje). Tak samo pola wyrażenia i wartości przypadku.
- Prawa kolumna: podgląd 3D, **Podgląd zmiennych** (wartości początkowe; kolumna „Bieżąca” — po wdrożeniu
  symulacji), lista uwag (klik → blok), ścieżka pliku programu.

## Logika wykonania (wykonawca programów w moście)

Moduły: `robot_web_bridge/program_interpreter.py` (interpreter bloków → zdarzenia ruchu), `motion_chain.py` (limity,
podział na ciągi, kursor robota), `program_runner.py` (przebieg, API); planer: akcja
`/robot/planning/compute_motion_chain` (`robot_motion_planner/src/chain.cpp`), wykonawca C++ przyjmuje plan `CHAIN`.

- **Dwa kursory.** *Kursor programu* (▷, niebieska przerywana ramka) — blok, do którego interpreter policzył logikę
  i trajektorię (liczy / czeka na robota). *Kursor robota* (◆, zielony pasek) — blok wykonywany fizycznie, wyznaczany
  z czasu wykonawcy względem czasów ruchów planu (`chain_segments`); w strefie przejścia pasek dzieli się między blok
  „z” i „do” według postępu.
- **Promień pominięcia punktu (`blend_mm`).** 0 = dojazd do punktu i postój (v = 0). > 0 i istnieje następny ruch =
  płynne wyjście z ruchu w odległości r od punktu (na torze TCP, także dla PTP) i przejście do następnego ruchu
  z pominięciem punktu. Planer ogranicza r do połowy krótszego z sąsiednich odcinków (uwaga `blend_reduced`); przy
  zbyt krótkim torze TCP (np. sam obrót narzędzia), zawróceniu drogi albo wyjściu krzywej poza promień — postój
  (`blend_not_possible`). Ostatni ruch ciągu zawsze się zatrzymuje (`chain_end` / `chain_split`).
- **Liczenie: hybryda.** Logika liczona w locie, z wyprzedzeniem; ruchy składane w **ciągi** liczone z góry jako jedna
  trajektoria (postoje v = 0 wewnątrz ciągu — bez przerwy na uzbrojenie). Ciąg kończy się na końcu programu, punkcie
  synchronizacji (przyszłe rozkazy czekania) albo limicie (~80 s szacunku / 64 ruchy; planer ucina na postoju przy
  95 s). Ciąg k+1 jest planowany fizycznie w trakcie wykonywania ciągu k od przewidzianego punktu końcowego; po
  zakończeniu ciągu rozjazd > 0,25° → przeliczenie od pomiaru. Między ciągami ok. 2 s (dolot ±0,5°, przywrócenie trybu,
  uzbrojenie).
- **Geometria przejścia.** Punkt wyjścia/wejścia: przecięcie toru TCP ze sferą o promieniu r wokół punktu; krzywa
  przejścia: wielomian Hermite'a 5. stopnia w przestrzeni przegubów, styczny do obu ruchów (bez IK). Czas: TOTG
  + profil prędkości per próbka (odcinek: limity swojego ruchu; przejście: minimum obu, limity TCP gdy któryś to LIN).
  Walidacja co 5 ms: limity osi i TCP, LIN poza strefami na prostej, przejście w promieniu.
- **Uwaga sprzętowa:** postoje wewnątrz ciągu nie mają potwierdzenia dolotu (±0,5° tylko na końcu ciągu).
- **Zgodność z wykonawcą streaming (10.11.2026).** Wykonawca (`compile_streaming_program`) sprawdza każdy odcinek
  otoczką Bernsteina wielomianu 5. stopnia względem limitów osi z konfiguracji. Na przełączeniach przyspieszenia TOTG
  (start z postoju, koniec rozpędzania, początek hamowania) otoczka zawyża ~2× — PTP od ok. 50% był odrzucany
  („Trajektoria streaming przekracza limity osi”), choć rzeczywisty przebieg mieścił się w limicie ruchu. Planer
  (`refine_for_streaming` w `compile_mks`) dzieli takie odcinki na 2…16 części tego samego wielomianu (bez zmiany
  trajektorii i ramek 10 ms), aż otoczka ≤ 98% limitów; kontrola wykonawcy bez zmian. Program „pati” (PTP 80/100%):
  otoczka 1,59/1,97 → 0,97/0,94 limitu, +8/+31 punktów. Test: `test_chain` `RefineForStreamingSatisfiesExecutorEnvelope`
  (z prawdziwym `compile_streaming_program`).

### Panel „Wykonanie” (GUI)

⟲ **Sprawdź** (zapisany program bez uwag; symulacja od bieżącej pozycji — bez ruchu), ▶ **Uruchom**, ■ **STOP**.

**▶ Uruchom doprowadza program i robota do stanu startu** (lista kroków „Przygotowanie do startu” w panelu; błąd
zatrzymuje proces i wskazuje przyczynę; STOP przerywa):
1. zapis niezapisanych zmian;
2. program bez uwag (lokalnie i w magazynie);
3. analiza `POST /api/v1/programs/{id}/preflight` (`ProgramRunner.preflight`, bez ruchu): **pozycje w zakresie osi**
   (`joint.N.lower/upper` aktywnej konfiguracji; używana pozycja poza zakresem = błąd, nieużywana = ostrzeżenie; klik →
   pozycja w podglądzie 3D), **stan robota** (świeży stan osi, brak awarii i odzyskiwania, kalibracja i znana pozycja
   J1–J6, bieżąca pozycja w zakresie, wykonawca i JOG wolne), **napędy** (ENABLE; wyłączone = do załączenia);
4. symulacja całego programu (sprawdzenie; cele ruchów — także względnych po IK — muszą mieścić się w zakresie osi:
   `MotionLimits.segment`);
5. okno potwierdzenia operatora (liczba ruchów, czas, uwagi, zatwierdzenie rewizji, napędy do załączenia);
6. zatwierdzenie rewizji (jeśli trzeba — klik operatora jest zatwierdzeniem);
7. załączenie napędów `POST /api/v1/manual/enable-all` i oczekiwanie na ENABLE=1 J1–J6 (≤ 3 s; wykonawca C++ wymaga
   ENABLE, kalibracji i świeżego stanu na wszystkich osiach);
8. start (`run` execute; backend ponownie blokuje pozycje poza zakresem).
Skrypt `run_program.py --execute` nie załącza napędów ani nie zatwierdza — działa jak dotąd. Oś czasu ciągów (ruch / przejście / postój / przerwa),
znaczniki promienia na blokach po sprawdzeniu (`⌀ 10→4 mm`, `⌀ ✕ stop` — bursztynowe), lista uwag (klik → blok).
Czekanie: pasek „⏱ … pozostało ~x s”; czekanie na operatora: komunikat bloku i przycisk **⏵ Kontynuuj** (operator).
Podczas wykonania płótno jest tylko do odczytu; ręczne `/api/v1/motion/prepare|execute` zwracają 409.

### API przebiegu (`program_runner.py`)

| Metoda i ścieżka | Działanie |
| --- | --- |
| `POST /api/v1/programs/{id}/run` `{"mode": "check"\|"execute", "revision"}` | start (202); 409 inna rewizja / trwa przebieg lub ruch; 422 uwagi lub brak zatwierdzenia (execute) |
| `POST /api/v1/program-run/check-document` `{"program"}` | sprawdzenie niezapisanego programu (bez wykonania) |
| `GET /api/v1/program-run`, `WS /api/v1/program-run/ws` (10 Hz) | stan: `state`, `program_cursor`, `robot_cursor`, `chain`, `next_chain`, `warnings`, `error`, `timeline`, `total_s` |
| `POST /api/v1/program-run/resume` | kontynuacja po „czekaj na operatora” (stan `waiting_operator`; inaczej 409) — przycisk operatora w GUI |
| `POST /api/v1/program-run/stop` | STOP przebiegu i ruchu |

## Sprawdzanie i wykonanie z terminala (`scripts/run_program.py`)

```bash
python3 scripts/run_program.py <id>                # sprawdzenie na sucho: interpretacja + plan każdego ruchu (bez ruchu)
python3 scripts/run_program.py --file prog.json    # sprawdzenie pliku przed zapisem (walidacja POST /api/v1/programs/check)
python3 scripts/run_program.py <id> --execute      # wykonanie — tylko rewizja zatwierdzona przez operatora
```

- Skrypt jest klientem API przebiegu (powyżej): wypisuje oś czasu per blok, promienie żądane → zastosowane, uwagi;
  przy `--execute` — zmiany kursorów robota i programu. Ctrl+C → `POST /api/v1/program-run/stop`.
- Interpreter (`program_interpreter.py`): rozkazy z tablicy `HANDLERS` (wszystkie z biblioteki); bloki wyłączone
  (z wnętrzem) są pomijane; limity 2000 ruchów i 100 000 kroków logiki; własny parser wyrażeń (bez `eval`).
- Czekanie i czekanie na operatora to punkty synchronizacji (`Sync`): kończą ciąg ruchów (poprzedni ruch z promieniem
  > 0 dostaje postój — uwaga `blend_not_possible`), a wykonawca obsługuje je między ciągami (`ChainBuilder.leading`,
  `ProgramRunner.hold`): stan `waiting` (sen) albo `waiting_operator` (do `resume`), potem kontrola rewizji
  i zatwierdzenia oraz pozycji (> 0,25° → przeliczenie). Na osi czasu sprawdzenia: wpis `kind: "wait"` (czas)
  / `"operator"` (0 s, uwaga `operator`); po czekaniu doliczana jest przerwa między ciągami (~2 s).
- Parametry ruchu z konfiguracji robota: PTP `speed` % × najmniejsza `joint.N.velocity` i `joint.N.acceleration`
  (nie więcej niż limit danej osi); LIN `speed` mm/s ≤ `tcp.max_linear_velocity`, przyspieszenie
  `tcp.linear_acceleration` (≥ 100), prędkość kątowa `tcp.max_angular_velocity` (10–360).
- Wykonanie zawsze poprzedza sprawdzenie (ponownie, jeśli zmieniła się rewizja, konfiguracja lub pozycja > 0,5°);
  zmiana rewizji lub cofnięcie zatwierdzenia w trakcie → zatrzymanie na granicy ciągu. Dziennik
  w `log/programs/<czas>/events.jsonl`.

### Zatwierdzenie operatora

`POST /api/v1/programs/{id}/approve {"revision": …}` (przycisk „✔ Zatwierdź do wykonania” w GUI; tylko zapisany
program bez uwag) zapisuje `approval: {revision, approved_at, client}`. Pole nie wchodzi do skrótu rewizji i nie da się
go ustawić zapisem programu; każda zmiana treści daje nową rewizję, więc zatwierdzenie wygasa. `DELETE …/approve` —
cofnięcie. `--execute` odmawia, gdy `approval.revision ≠ revision`. Agent AI nie zatwierdza programów i przed każdym
`--execute` prosi operatora o zgodę (AGENTS.md §0).
