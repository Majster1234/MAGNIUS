# temp/model_3d_kreator — kreator budowy modelu 3D robota (tryb modułowy)

Samodzielne makiety HTML (dane przykładowe, bez API/ROS/CAN i zasobów zewnętrznych). Otwierać bezpośrednio w
przeglądarce; są klikalne: kreator dodaje elementy, Cofnij/Ponów działa, tabela odległości osi przelicza się na bieżąco.
Podgląd 3D zastąpiono schematycznym widokiem z boku (SVG): ⊙ = oś prostopadła do widoku, linia przerywana = oś
w płaszczyźnie widoku (strzałka = zwrot), kropki = miejsca spasowania, przerywany zielony = element w trakcie dodawania.

Założenia od operatora (23.11.2026):
- budowa od podstawy; przy każdym kolejnym elemencie wybór: **element konstrukcyjny** albo **miejsce łożyskowania osi**;
- element konstrukcyjny liczy się do kinematyki, gdy za nim w łańcuchu jest oś (za ostatnią osią przesuwa TCP);
- każdy element wgrywany osobno (własny układ pliku), położenie ze spasowania (walec ↔ walec, styk płaszczyzn);
- po kreatorze kroki można edytować, cofać i dodawać/wstawiać części bez kreatora.

Stan przykładowy we wszystkich wariantach: podstawa → J1 → kolumna_120 → J2 → ramie_250, kreator czeka na kolejny element.

| Plik | Kreator | Edycja po kreatorze | Mocne strony | Słabsze strony |
| --- | --- | --- | --- | --- |
| `v1_kreator_krokowy.html` | Panel obok podglądu z podkrokami: Plik → Rodzaj → Dołączenie → Oś przegubu → Podsumowanie; pasek kroków łańcucha u góry | Przełącznik „Kreator / Edycja”; klik w krok lub element → inspektor (rodzaj, przesunięcie/obrót, zwrot, Wstaw przed/za, Zamień, Usuń) | Najbardziej prowadzący; podsumowanie pokazuje wpływ na kinematykę przed dodaniem | Dwa tryby do zrozumienia; dużo kliknięć „Dalej” przy prostych elementach |
| `v2_lancuch_elementow.html` | Brak osobnego trybu: ostatnie pole łańcucha pyta „Co jest za …?” [Konstrukcyjny] [Oś Jn]; spasowanie w inspektorze z podpowiedziami na podglądzie | Ten sam widok: karty łańcucha, „+” między kartami (wstawianie), przesuwanie ←/→, historia zmian | Najprostszy model myślowy (łańcuch = robot); przełącznik „Prowadź krok po kroku” wyłącza podpowiedzi | Przy 15+ elementach pasek kart wymaga przewijania; mniej miejsca na opisy |
| `v3_drzewo_przewodnik.html` | Okno przewodnika: jedno pytanie na ekran (Co dalej? → Plik → Spasowanie → Oś), postęp Podstawa…J6…Narzędzie | Drzewo członów (człon = oś + elementy do następnej osi), menu ⋯ przy elemencie, panel „Kompletność” | Drzewo pokazuje przynależność do członów (jak przeglądarka Fusion); kreator nie zasłania pracy po zamknięciu | Okno zasłania główny podgląd (ma własny mały); dwa miejsca, w których robi się to samo |

Wspólne dla wszystkich: Cofnij/Ponów obejmuje kroki kreatora; wstawienie w środek łańcucha przesuwa elementy za nim
(i przenumerowuje osie); usunięcie dołącza następny element do poprzednika; „Zapisz model 3D” (wygląd) osobno od
„Zapisz do ROS…” (kinematyka z łańcucha).

**Wdrożono 23.11.2026: `v2_lancuch_elementow.html`** (`web/src/ModelChainBuilder.tsx`, `chainScene.ts`, `modelChain.ts`;
opis `docs/robot_model_3d.md`, etap 2). Pozostałe pliki zostają jako archiwum propozycji.

Wdrożenie wybranego wariantu było osobnym zadaniem w `web/src` (`RobotModelPanel.tsx`, `ElementEditor.tsx`,
`elementScene.ts`) wraz z etapem 2 trybu modułowego (`docs/robot_model_3d.md`).
