# temp/ujednolicenie_wygladu — wspólny system wizualny: wybrany wariant C2

Makieta wyglądu całej aplikacji w stylu C „Sterownia”. Dane są przykładowe, brak połączenia z API/ROS/CAN.
Przyciski ruchu niczego nie wysyłają, a STOP pokazuje tylko komunikat. Otwórz `wariant_c2_drzewo.html` w przeglądarce.

## Historia wyboru

- 17.11.2026: etap 1 — trzy warianty (A Grafit, B Warsztat, C Sterownia). Wybrany **C**.
- 17.11.2026: trzy rozwinięcia C (C1 Pulpit, C2 Drzewo, C3 Oscyloskop). Diagnostyka obejmuje Magistralę CAN
  i Architekturę, a ekrany mieszczą się w oknie. Wybrany **C2**; pozostałe usunięte.
- 17.11.2026: **statusy systemu przeniesione do menu bocznego** (wcześniej wskaźnik SYS/ROS/API/WEB/napędy był w pasku stanu):
  - **menu zwinięte** (pasek ikon 100 px): ten sam wskaźnik w pionie, czyli SYS, ROS, API, WEB jeden pod drugim,
    6 kresek napędów i „6/6”;
  - **menu rozwinięte** (236 px): statusy jako 4 pola, kreski napędów i trzy mierniki: **CPU** (średnia + osobny
    słupek dla każdego z 4 wątków i5-7500T, W0–W3), **RAM** (% i GB), **CAN** (obciążenie % ze znacznikiem limitu 60 %);
  - menu zwija się samo na Sterowaniu ręcznym i Programowaniu, a na pozostałych ekranach jest rozwinięte, tak jak
    w obecnej aplikacji. Przycisk „Zwiń menu” zmienia stan do następnej zmiany ekranu;
  - usunięte powtórzenia: zasoby (CPU/RAM/CAN/RTT) zniknęły z mapy systemu w Diagnostyce (RTT jest teraz przy
    „Magistrala can0”), a CPU/RAM z kafelka „Komputer robota” w Przeglądzie (zamiast nich procesor i system).

- 17.11.2026: **wdrożono w `web/src`** (osobne zadanie): `theme.css`, nowa rama z menu i panelem stanu, Diagnostyka
  z mapą i zakładkami, Przegląd, układ Sterowania. Zmiana względem makiety: stan zwinięcia menu nie zależy od strony
  (zmienia go tylko operator; pamiętany w przeglądarce). Przegląd ma kafelki i listę problemów (bez 3D i pozycji osi —
  wymagałyby strumienia sterowania na stronie Przeglądu).

## Zawartość wariantu C2

- **Menu bez Magistrali CAN i Architektury**: Przegląd, Diagnostyka, Sterowanie ręczne, Programowanie, Konfiguracja
  (+ System wizualny, tylko w makiecie). Pasek stanu na dole: ścieżka, połączenie, Teach pendant.
- **Diagnostyka**: po lewej stała mapa systemu (can0 i napędy J1–J6 z RTT, procesy w grupach System / ROS 2 /
  Backend / Frontend). Po prawej zakładki **Przebiegi** (6 osi 2×3, wspólny pasek narzędzi) · **Magistrala CAN**
  (wskaźniki, urządzenia, szczegóły komunikacji) · **Procesy** · **Zdarzenia**. Kliknięcie osi w mapie otwiera jej
  wykres, procesu jego wiersz, can0 zakładkę magistrali. Reset CAN i Skanuj są w nagłówku.
- **Sterowanie ręczne**: 3D nad komunikatami po lewej; osie (JOG, ENABLE, zero, prędkość) nad JOG TCP po prawej.
- **Przegląd**: 6 kafelków stanu; pod nimi robot 3D, pozycje osi na żywo, ostatnie zdarzenia.
- **Bez przewijania strony** od 1600×900 (sprawdzone też 1920×1080): przewijają się tylko listy w panelach.
  Poniżej 1250 px szerokości lub 740 px wysokości wraca zwykłe przewijanie.
- Z etapu 1:
  - kolory znaczeniowe (zielony OK/LIVE, niebieski planowanie, bursztynowy ostrzeżenie, czerwony awaria/STOP);
  - kolory osi J1–J6 poza barwami stanów;
  - jeden wzór kontrolek, ikony SVG;
  - nagłówek strony w jednej linii, STOP w prawym górnym rogu ekranów ruchu.

Lista zdarzeń jest przykładowa. W aplikacji wymaga rejestru problemów w backendzie (opis w `temp/README.md`,
sekcja Architektura). Zużycie CPU per wątek, RAM i obciążenie CAN są już w danych monitora systemu
(`/robot/system_status`). Wartości per wątek trzeba sprawdzić przy wdrożeniu.

## Źródła

Plik HTML jest generowany, więc nie edytuj go ręcznie. Przebudowa: `python3 temp/ujednolicenie_wygladu/zrodla/zbuduj.py`.

| Plik w `zrodla/` | Rola |
| --- | --- |
| `szkielet_gesty.html` | wspólna treść ekranów, menu z panelem stanu; `{{DIAG}}` = ekran Diagnostyki |
| `diagnostyka/c2_drzewo.html`, `diagnostyka/pasek_przebiegow.html` | Diagnostyka (mapa + zakładki) i pasek narzędzi przebiegów |
| `wspolne_gesty.js` | dane przykładowe, wypełnianie tabel i mapy, wykresy, nawigacja, mierniki w menu |
| `baza.css` | komponenty na tokenach |
| `wariant_c_sterownia.css` | tokeny i rama stylu C |
| `gesty.css` | gęstsza skala i dopasowanie ekranów do okna |
| `wariant_c2_drzewo.css` | układ Diagnostyki i Sterowania, menu rozwinięte/zwinięte z panelem stanu |

Wyjątek od reguły „bez skryptów” z `temp/README.md`: nawigacja, zakładki, mapa i wartości na żywo wymagają
JavaScriptu (wbudowanego w plik, bez zasobów zewnętrznych). Czcionki systemowe (Ubuntu Condensed, DejaVu Sans Mono).
Animacje respektują `prefers-reduced-motion`.

## Do decyzji przed wdrożeniem

- Czy Przegląd zostaje osobnym modułem (część danych jest w Diagnostyce i w panelu stanu menu).
- Zakres wdrożenia w `web/src` (osobne zadanie): tokeny i komponenty, rama z menu, potem moduły po kolei.
