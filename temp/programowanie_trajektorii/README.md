# temp/programowanie_trajektorii — propozycje modułu „Programowanie trajektorii”

Szkice nowego trybu do tworzenia programów robota: nazwane pozycje, program blokowy z przełączaniem na tekst,
zmienne programu, model 3D z symulacją. Dane są przykładowe, brak połączenia z API/ROS/CAN; przyciski ruchu fizycznego
są wyłączone.

**Odstępstwo od reguły „bez skryptów”** (`temp/README.md`): widok 3D i symulacja wymagają JavaScriptu. Pliki ładują
three.js 0.164.1 (ta sama wersja co `web/`) z `unpkg.com` przez importmap. Bez internetu widok 3D pokazuje komunikat,
a symulacja programu (podświetlanie instrukcji, podgląd zmiennych) działa dalej. Każdy plik jest samodzielny.

## Historia wyboru

- 08.11.2026: 5 wariantów (studio, bloki, telefon, scena z osią czasu, wirtualny pendant). Operator wybrał
  **`v2_bloki.html`**; pozostałe usunięte.
- 09.11.2026: trzy rozwinięcia v2 według uwag operatora:
  - bloki zostają, małe okno podglądu kodu usunięte;
  - przełącznik **Bloki / Tekst** w nagłówku (ten sam program; tekst w stylu ST: `VAR … END_VAR`, `:=`, `END_FOR`);
  - w miejscu okna kodu — **podgląd zmiennych** odświeżany podczas symulacji;
  - **zmienne programu deklarowane przed kodem** (typy INT, REAL, BOOL, POZYCJA, TEKST; wartość początkowa, opis);
  - stonowana kolorystyka zamiast fioletu (akcent i kategorie bloków przygaszone).

## Program przykładowy „Paletyzacja 3×” (wspólny)

Zmienne: `licznik : INT := 0`, `gniazd : INT := 3`, `v_lin : REAL := 50 mm/s`, `chwyt_ok : BOOL := FALSE`,
`cel : POZYCJA := Odkładanie_1`. Kod: `FOR i := 1 TO gniazd`, ruchy `MOVE PTP/LIN` (prędkość LIN z `v_lin`, cel ruchu
z `cel`), `WAIT`, `SET OUT1`, `chwyt_ok := IN2`, `cel := Odkładanie_[i]`, `licznik := licznik + 1`,
`IF licznik == gniazd THEN … ELSE … END_IF`. 7 pozycji (`Home`, `Nad_odbiorem`, `Odbiór`, `Nad_odkładaniem`,
`Odkładanie_1…3`). Symulacja: ▶/⏸, krok, reset, 0,5–4×, przewijanie, podświetlenie bieżącego bloku/linii w obu widokach.

| Plik | Akcent | Tworzenie zmiennych | Podgląd zmiennych |
| --- | --- | --- | --- |
| `v2_bloki.html` | fiolet (pierwotny) | — | okno kodu (usuwane w kolejnych wersjach) |
| `v2a_bloki_grafit.html` | grafitowy `#8b9bb0` | sekcja „Zmienne programu” nad blokiem Start: deklaracje jako płaskie bloki, „＋ Nowa zmienna” otwiera formularz w miejscu (nazwa, typ, wartość, jednostka, opis); zmienne także w palecie | tabela: zmienna, typ, wartość, poprzednia wartość; wiersz zmieniony w kroku miga; sekcja systemowa (i, OUT1, IN2, TCP) |
| `v2b_bloki_stal.html` | stalowy `#6f93b3` | lewy panel z zakładkami Bloki · **Zmienne** · Pozycje; karty zmiennych (edycja/usuwanie) i okno „Nowa zmienna” z wyborem typu kafelkami i zakresem min/maks; pasek zmiennych nad Start | karty z dużą wartością i wykresem przebiegu (licznik, i); **pułapka**: zatrzymaj symulację, gdy zmienna = wartość |
| `v2c_bloki_monochrom.html` | ciepła szarość `#a39e93` | blok **VAR** na początku programu edytowany jak tabela (nazwa, typ, wartość, opis, ×, „＋ dodaj zmienną”); bloki neutralne, kolor kategorii tylko na lewej krawędzi | zwarta lista wartości + **dziennik zmian** (czas, linia, zmienna: stara → nowa) |

Do decyzji przed wdrożeniem (osobne zadanie w `web/src` i backendzie): akcent i intensywność kolorów bloków,
miejsce tworzenia zmiennych (nad programem / zakładka / blok VAR), forma podglądu (tabela / karty z przebiegiem /
dziennik), jeden format zapisu programu (JSON/AST) dla obu widoków. Wykonanie na robocie — poza zakresem szkicu.
