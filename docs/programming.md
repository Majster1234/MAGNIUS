# Programowanie — edytor programów ruchowych (bloki)

Strona `#programming` („⌖ Programowanie”), wdrożona 09.11.2026 na podstawie szkicu
`temp/programowanie_trajektorii/v2b_bloki_stal.html`. Zakres: tworzenie i zapis programów (bloki, zmienne, pozycje,
biblioteka) oraz — od 09.11.2026 — **logika wykonania**: wykonawca programów w moście z dwoma kursorami i ciągami ruchów
PTP/LIN z promieniem pominięcia punktu (sekcja „Logika wykonania”). Widok tekstowy jest podglądem generowanym z bloków.

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
- Bloki z gniazdami (`for`, `while`: `body`; `if`: `body` + `else`) — klucze gniazd zgodne z `slots` rozkazu.
- Limity: 1 MiB, 2000 bloków, zagnieżdżenie 16, 500 zmiennych/pozycji. Brakujące odwołania nie blokują zapisu —
  wracają jako `problems`.

## Biblioteka rozkazów (`commands.json`)

Biblioteka budowana od nowa (09.11.2026; kopia dawnej: `library/commands.json.bak-20261109`). Obecnie:

| Rozkaz | Blok | Parametry |
| --- | --- | --- |
| `move_ptp` | `PTP do {position} v {speed} ⌀ {blend_mm}` | pozycja; prędkość 1–100 % (najwolniejszej osi z konfiguracji); promień pominięcia 0–200 mm |
| `move_lin` | `LIN do {position} v {speed} ⌀ {blend_mm}` | pozycja; prędkość TCP mm/s (≤ `tcp.max_linear_velocity`); promień pominięcia 0–200 mm |

Pozycje (punkty) definiuje się w zakładce „Pozycje” (pomiar robota lub ręcznie J1–J6). Literały spoza `min`/`max`
(`integer`) i nieuzupełnione pola są uwagami (`problems`) — blokują zatwierdzenie, nie zapis. Przy aktualizacji
domyślnej biblioteki kopię w katalogu danych trzeba podmienić ręcznie (magazyn kopiuje ją tylko, gdy pliku brak).

Format definicji rozkazu:

Rozkaz: `id`, `category`, `icon`, `label` (tekst bloku z polami `{param}`), `text` (linia podglądu tekstowego),
`end` (linia zamykająca), `slots`, `params` (`name`, `kind`, `label`, `unit`, `default`, `min`, `max`, `integer`,
`types`). Rodzaje parametrów (`kind`): `position`, `number`, `variable`, `condition`, `text`, `bool`, `output`,
`input`, `value`. Przeciąganie zmiennej/pozycji na pole działa wg zgodności typu (np. `position` przyjmuje
pozycję albo zmienną `POZYCJA`, `number` — zmienne `INT`/`REAL`). Nazwy `io.outputs`/`io.inputs` są przykładowe
(do zdefiniowania sprzętowo).

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
- Płótno: przeciąganie z palety między bloki i do gniazd pętli/warunków, przenoszenie bloków (bez wstawienia
  bloku do własnego wnętrza), kosz, przeciąganie zmiennych (z panelu lub paska nad Start) i pozycji na pola,
  edycja pola po kliknięciu, zaznaczenie + `Delete`, `Ctrl+D` duplikat, wyłączanie bloku, oznaczenie `!` przy
  nieuzupełnionych polach i brakujących odwołaniach.
- Zmiana rodzaju rozkazu bez usuwania bloku: kliknięcie nazwy (np. **PTP ▾**) → lista rozkazów zgodnych (ta sama
  kategoria i gniazda, wspólne parametry; np. PTP ↔ LIN). Id bloku i parametry o tej samej nazwie zostają (wartość
  liczbowa bez przeliczania jednostek — np. 50 mm/s → 50 %); wartość spoza zakresu nowego rozkazu wraca do domyślnej
  (komunikat przy bloku). `programModel.ts`: `compatibleCommands`, `changeCommand`.
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

### Panel „Wykonanie” (GUI)

⟲ **Sprawdź** (zapisany program bez uwag; symulacja od bieżącej pozycji — bez ruchu), ▶ **Uruchom** (tylko rewizja
zatwierdzona przez operatora; okno potwierdzenia), ■ **STOP**. Oś czasu ciągów (ruch / przejście / postój / przerwa),
znaczniki promienia na blokach po sprawdzeniu (`⌀ 10→4 mm`, `⌀ ✕ stop` — bursztynowe), lista uwag (klik → blok).
Podczas wykonania płótno jest tylko do odczytu; ręczne `/api/v1/motion/prepare|execute` zwracają 409.

### API przebiegu (`program_runner.py`)

| Metoda i ścieżka | Działanie |
| --- | --- |
| `POST /api/v1/programs/{id}/run` `{"mode": "check"\|"execute", "revision"}` | start (202); 409 inna rewizja / trwa przebieg lub ruch; 422 uwagi lub brak zatwierdzenia (execute) |
| `POST /api/v1/program-run/check-document` `{"program"}` | sprawdzenie niezapisanego programu (bez wykonania) |
| `GET /api/v1/program-run`, `WS /api/v1/program-run/ws` (10 Hz) | stan: `state`, `program_cursor`, `robot_cursor`, `chain`, `next_chain`, `warnings`, `error`, `timeline`, `total_s` |
| `POST /api/v1/program-run/stop` | STOP przebiegu i ruchu |

## Sprawdzanie i wykonanie z terminala (`scripts/run_program.py`)

```bash
python3 scripts/run_program.py <id>                # sprawdzenie na sucho: interpretacja + plan każdego ruchu (bez ruchu)
python3 scripts/run_program.py --file prog.json    # sprawdzenie pliku przed zapisem (walidacja POST /api/v1/programs/check)
python3 scripts/run_program.py <id> --execute      # wykonanie — tylko rewizja zatwierdzona przez operatora
```

- Skrypt jest klientem API przebiegu (powyżej): wypisuje oś czasu per blok, promienie żądane → zastosowane, uwagi;
  przy `--execute` — zmiany kursorów robota i programu. Ctrl+C → `POST /api/v1/program-run/stop`.
- Interpreter (`program_interpreter.py`): rozkazy z tablicy `HANDLERS` (obecnie `move_ptp`, `move_lin`); bloki
  wyłączone są pomijane; limit 2000 ruchów; bezpieczny ewaluator wyrażeń (pod przyszłe IF/WHILE).
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
