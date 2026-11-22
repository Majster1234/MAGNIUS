# temp/komunikaty — pasek komunikatów i czynności robota

Makieta HTML z wbudowanym skryptem (interakcje i animacje), bez zasobów zewnętrznych i bez połączeń z API/ROS/CAN.
Otwórz `v2d_pasek_awaria.html` w przeglądarce. Scenariusz zmienia się przyciskami w ramce „Makieta” albo adresem:
`#move`, `#plan`, `#warn`, `#error`, opcjonalnie z `,open`, zakładką `,now` / `,act` / `,arch`, `,fa` (przewinięcie do
karty awarii) i `,can` (szczegółowa analiza). Przykład: `v2d_pasek_awaria.html#error,open,fa`.

## Historia wyboru

- 19.11.2026: pięć wariantów miejsca na komunikaty. Wybrany **wariant 2 — pasek nad stroną**.
- 20.11.2026: v2b (panel z góry, archiwum, animacje), potem v2c (bez stanu programu, kwitowanie, analiza awarii).
- 20.11.2026: **v2d** (zastępuje poprzednie). Uwagi operatora do v2c:
  - analiza awarii była za rozbudowana: zamiast niej opis tekstowy i wyciąg migawki CAN, który wywołał błąd;
  - zamiast dużego wykresu: grafika robota z podświetloną osią, na której wystąpił błąd;
  - w komunikacie uproszczony wykres z kilkoma próbkami przed i po awarii (pomiar i plan), pokazywany tylko wtedy,
    gdy da się go zbudować dla konkretnego błędu;
  - szczegółowa analiza zostaje jako łącze (okno migawki: przebieg pomiaru i planu, wszystkie zdarzenia);
  - w czynnościach robota nad kafelkiem każdej osi: czy była w ruchu, czekała, rozpędzała się albo hamowała.

## v3 — wyróżnienie otwartego panelu (propozycja 20.11.2026)

`v3_wyroznienie_panelu.html` (warianty adresem `#now`, `#a`, `#b`, `#c`, `#d`, opcjonalnie `,warn`, `,closed`).
Uwaga operatora: otwarty panel nie odróżnia się od strony, nie wiadomo, że to okno do zamknięcia. Przyczyna: tło panelu
`#0b0e11` na stronie `#07090b`/`#0e1114`, cienka szara ramka, przyciemnienie 55% bez rozmycia.
- **A · Szuflada** — jaśniejsza powierzchnia panelu i kart, akcent koloru najgorszego komunikatu na górnej krawędzi,
  pasek przy otwartym panelu w kolorze nagłówka panelu z linią akcentu.
- **B · Strona w tle** — przyciemnienie 72% + rozmycie i odbarwienie strony, podpowiedź „kliknij tutaj lub Esc”.
- **C · Okno** — belka „KOMUNIKATY” w kolorze poziomu z podsumowaniem i podpisanym przyciskiem ZAMKNIJ (Esc); pasek
  pokazuje „PANEL KOMUNIKATÓW OTWARTY”, strzałka staje się przyciskiem ZAMKNIJ.
- **Wybrany i wdrożony 20.11.2026: wariant A** (`web/src/messageBar.css`, klasa poziomu na panelu w `MessageBar.tsx`).
- **D · Zalecane** — A + B + C. Nagłówek strony ze STOP zawsze ostry i aktywny. Wdrożenie: CSS i kilka elementów
  w `MessageBar.tsx`.

## Wdrożenie

- **Etap 1 — wdrożony i uruchomiony w panelu 20.11.2026** (`web/src`, build `web/dist`, dowiązania w `install/stop-recovery-ros`, kopia poprzedniego `dist`: `install/deployment-backups/message-bar-1795134846`). Do paska trafiają też Konfiguracja (dawna konsola zdarzeń usunięta), Programowanie, start napędów i Diagnostyka. `MessageBar.tsx` + `messageBar.css` (pasek nad stroną na każdej
  stronie, panel spod nagłówka), `messageCenter.ts` (wspólny magazyn, kwitowanie, moduły), `robotActivity.ts`
  (czynność robota i stan osi z `GET /api/v1/manual/status` + `/api/v1/motion/status` co 1 s), `systemMessages.ts`
  (problemy podzespołów z `/api/v1/ws`). Sterowanie ręczne zgłasza do wspólnego magazynu; lokalna sekcja
  „Komunikaty” usunięta (podgląd 3D zyskał wysokość). Różnice względem makiety: zakładka „Aktualne” to lista pełnych
  kart (opis, „Co zrobić”, link, Kwituj) zamiast układu lista/szczegóły; stan osi bez rozpędzania/hamowania (etap 3).
- **Etap 2 — wdrożony 20.11.2026:** `robot_web_bridge/fault_api.py` (API migawek i analiza), przebiegi wałów w migawkach
  (`shaft_trace.window_rows`, `fault_snapshot --shaft-trace-dir`), frontend `FaultCard.tsx` + `faultSnapshot.ts` (karta,
  okno szczegółowej analizy, migawka w archiwum), podświetlenie osi w `programPreviewScene.ts`. Backend działa po restarcie
  `robot-web-status.service`.
- **Etap 3 — do zrobienia:** archiwum i kwitowanie w backendzie (dziś pamięć karty, 100 wpisów), stan F1 osi
  (rozpędza się / hamuje) w strumieniu sterowania, przebieg w migawce.

## v2d — zawartość

**Pasek:**
- po lewej czynność robota: co robi teraz albo co robił, gdy się zatrzymał;
- w środku przewijane komunikaty (NOWY = nie pokwitowany);
- po prawej „Kwituj n”, liczniki i zegar. Kliknięcie paska rozwija panel z góry; STOP zostaje dostępny.

**Kwitowanie:**
- przy komunikacie albo zbiorczo („Kwituj wszystkie” w panelu i na pasku);
- pokwitowany wynik operacji trafia do archiwum;
- pokwitowany warunek zostaje na liście przygaszony, dopóki przyczyna nie zniknie. Kwitowanie nie kasuje awarii.

**Szczegóły błędu (czerwonego):**
1. Co się stało: opis tekstowy, w tym jedno zdanie z nagrania CAN (np. „tuż przed awarią J1 nie odpowiedział na dwa
   zapytania, licznik skoczył o 742 imp.”), a pod nim surowy powód.
2. Co zrobić: numerowane kroki.
3. **Awaria w skrócie:**
   - **robot z podświetloną osią**: pulsująca poświata przegubu i etykieta „J1 · CAN ID 1”. Pokazywany tylko,
     gdy błąd wskazuje oś;
   - **krótki przebieg**: kilka próbek przed i po awarii; pomiar (kropki, czerwone po awarii), plan (przerywana
     linia: w postoju pozycja trzymania, w ruchu cel planowany), limit postoju i zaznaczone przerwy w odpowiedziach.
     **Pokazywany tylko, gdy da się go zbudować**, czyli gdy błąd wskazuje oś, a przebieg tej osi jest jeszcze
     w buforze albo w migawce. Przy błędzie bez osi (np. „napęd nie potwierdził warunku operacji”) wykresu nie ma,
     a nagłówek karty mówi dlaczego;
   - **wyciąg migawki CAN**: 3–8 zdarzeń, które wywołały błąd (czas względem awarii, zdarzenie, oś, komenda, opis);
   - łącza „Szczegółowa analiza” (okno migawki) i „Pobierz migawkę .jsonl”.
4. Przejdź do, Szczegóły techniczne (zwinięte).

**Szczegółowa analiza (okno migawki):** pełne okno −10 s … +5 s, pomiar i plan, znaczniki zdarzeń, filtry
(zdarzenia kluczowe / CAN · oś / cały CAN / mostek), tabela zdarzeń, pobranie `.jsonl`, przejście do Diagnostyka → Przebiegi.

**Czynności robota:**
- karta „teraz” (co robi lub co robił przed zatrzymaniem);
- **kafelki J1–J6 ze stanem nad każdym**: ROZPĘDZA SIĘ ↗, W RUCHU →, HAMUJE ↘, CZEKA, WYŁĄCZONA, BEZ KOMENDY (awaria).
  W scenariuszu „Ruch” stany zmieniają się na żywo (tylko przy otwartej zakładce). Przy awarii kafelki pokazują
  stan z chwili zatrzymania („CZEKAŁA”, a przy osi z błędem „przed awarią: CZEKAŁA (trzymanie)”);
- oś czasu ostatnich czynności. Stanu programu tu nie ma: program ma własny ekran, a praca automatyczna dostanie
  osobny dashboard.

**Archiwum:** kolumna nagrań (`CAN` = migawka, `PRZEBIEG` = przebieg w buforze, przekreślony = nadpisany),
filtr „Z nagraniem”, doczytywanie stronami.

## Skąd dane (sprawdzone 20.11.2026)

| Element | Źródło w systemie | Stan |
| --- | --- | --- |
| Stan osi nad kafelkiem | MKS F1 „stan silnika”: Zatrzymany / Przyspieszanie / Hamowanie / Pełna prędkość (`motor_state_name`, `bus_manager.cpp`), odpytywany co 50 ms w programie i przy szybkim JOG, co 500 ms poza nimi; tylko tryby SR_* | odczyt jest; trzeba go przekazać do panelu. Przy strumieniu F5 (100 Hz) stan może migać, więc warto łączyć go z prędkością i przyspieszeniem z trajektorii |
| Robot z podświetloną osią | istniejąca scena 3D (`RobotPreview`) i poświata przegubu (`previewLimitGlow.ts`) | w aplikacji: jedna wyrenderowana klatka albo lekka scena tylko przy otwartej karcie, bez drugiego ciągłego kontekstu WebGL |
| Krótki przebieg — pomiar | bufor `shaft-trace` (pozycja każdej osi, ok. 4,3 h) albo ramki 31h z migawki | pomiar jest zawsze w migawce |
| Krótki przebieg — plan | kanały `reference` (cel planowany) i `command` (cel F5) w `shaft-trace`, zapisywane w ruchu; w postoju plan = pozycja trzymania z nadzoru postoju | plan w ruchu jest tylko w buforze (4,3 h), migawka go nie zawiera |
| Wyciąg migawki CAN | `fault_snapshot.py` → `faults.rfs` (10 s przed, 5 s po; CAN, mostek, `/rosout`, dziennik) | dziś tylko CLI |

## Do zrobienia przy wdrożeniu

1. Powiązanie komunikatu z migawką (`snapshot_id` albo czas wyzwalacza) i z osią (wyzwalacz ma oś w `detail`/`axis`).
2. API migawek: lista, metadane, „zdarzenia kluczowe” (filtr: timeouty, naruszenia, awarie i odzyskiwanie z osi
   błędu ±2 s), strona zdarzeń, pobranie `.jsonl`.
3. **Dopisywanie do migawki okna `shaft-trace`** (pomiar, `reference`, `command`, prędkość), żeby krótki przebieg
   z planem był dostępny tak długo jak migawka, a nie tylko 4,3 h.
4. Reguła „czy wykres jest możliwy”: jest oś, a w migawce albo w buforze są próbki z okna. Bez tego tylko robot
   z osią (gdy jest oś) i wyciąg CAN.
5. Stan osi (F1 + trajektoria) w strumieniu stanu sterowania; zapis stanu z chwili awarii w migawce.
6. Archiwum komunikatów i kwitowanie po stronie backendu.

## Lekkość

- Zamknięty panel nie ma treści w DOM. Zostaje sam pasek z jednym timerem karuzeli co 4 s, wstrzymanym w tle.
- Karta awarii powstaje dopiero po wybraniu błędu; krótki przebieg to kilkanaście próbek z pamięci podręcznej.
- Szczegółowa analiza (pełny przebieg i tysiące zdarzeń) ładuje się wyłącznie po kliknięciu, stronami.
- Animacje tylko na `transform`/`opacity`; pulsowanie tylko do pokwitowania; `prefers-reduced-motion` wyłącza wszystko.
