# temp/sterowanie_reczne — propozycje modułu „Sterowanie ręczne”

Samodzielne makiety HTML (bez skryptów i połączeń z API/ROS/CAN, dane przykładowe). Przełącznik „Podgląd trybu: LIVE / Planowanie” na górze każdej makiety jest tylko do oglądania; w aplikacji tryb zmienia przycisk „Planowanie” w wizualizacji.

Założenia od operatora:
- **Bez zmian:** nagłówek modułu, tabela JOG i Wizualizacja robota (odtworzone ze stylów `web/src/style.css`, ramka „bez zmian”).
- **TCP live:** bardziej kompaktowy niż obecne dwa krzyżaki 44 px z osobnymi polami prędkości.
- **Ruch PTP/LIN:** aktywny tylko w trybie Planowania; mniej informacji. Zostają procent wykonania i czas do końca ruchu (`progress`, `duration − elapsed`). Znikają: tryb wysyłania, cykl/spóźnienie SYNC, tabela start → cel, opisy etapów.
- **Centrum komunikatów:** przebudowa (warianty poniżej).

| Plik | TCP | PTP/LIN | Komunikaty |
| --- | --- | --- | --- |
| `v1_pasek_stanu.html` | 6 wierszy X/Y/Z/Rx/Ry/Rz w stylu JOG: − wartość + (od razu widać pozycję TCP) | karta pod TCP: PTP/LIN, 2 parametry, Podgląd/Przygotuj/Wykonaj, pasek % + czas; w LIVE jedna linia „dostępny w Planowaniu” | pasek stanu (bieżący stan + licznik uwag) i rozwijana szuflada: problemy z „Co zrobić” + krótka historia |
| `v2_skrzynka_zakladek.html` | jeden krzyżak XY + Z z przełącznikiem Przesuw/Obrót | ścieżka Podgląd → Przygotuj → Wykonaj, pierścień % + czas, STOP przy postępie | skrzynka z zakładkami Teraz / Do sprawdzenia / Historia |
| `v3_konsola_nakladka.html` | pasek 2×3 par „− X +” (przesuw, obrót) z prędkością w wierszu | pływająca karta na wizualizacji (tylko w Planowaniu), parametry pod ⚙ | konsola: przypięty problem z przyciskiem „Rozwiąż”, jednoliniowy log z filtrami |
| `v4_toasty_banner.html` | dwa mini-krzyżaki 32 px (XY+Z, Rx/Ry+Rz) | duży procent + czas, parametry zwinięte w „Parametry ruchu”; w LIVE ukryty | banner problemu nad JOG, toasty informacyjne na wizualizacji, historia pod dzwonkiem |
| `v5_dok_zakladek.html` | numpad 3×3 (Z w rogach), Przesuw/Obrót przełączane (Shift) | zakładka „odtwarzacz” ▶ / ■ z paskiem % i czasem; zakładka zablokowana w LIVE | zakładka z osią czasu (Do sprawdzenia / Historia) + linia stanu pod wizualizacją |
| `v5b_dok_zakladek_pasek_pozycji.html` | **wybrany kierunek:** v5 + zmiana w tabeli JOG — cienki pasek pozycji (3 px) na całej szerokości kolumny „Pozycja” przy dolnej krawędzi wiersza; aktualna wartość nad paskiem w centrum, zakres (dolny/górny limit) po bokach; suwak przesuwa się po pasku, w Planowaniu dodatkowo niebieski znacznik celu i „plan …°” pod wartością | jak v5 | jak v5 |
| `v5c_dok_zakladek_jog_pasek_strzalki.html` | v5b + tabela JOG: kolumny Oś/CAN · JOG·Pozycja · Stan ruchu · Napęd/ENABLE · Zero wału · Prędkość. Strzałki JOG na końcach paska pozycji; między strzałkami: wyśrodkowana wartość, pod nią pasek z suwakiem, pod paskiem limity; stan sterownika (postój / w prędkości) w osobnej kolumnie; w Planowaniu pozycja `rzeczywista / plan` po ukośniku, wyśrodkowana razem; „Zeruj wał” jako przycisk z krawędzią | jak v5 | jak v5 |

**Wdrożono 30.10.2026: `v5c_dok_zakladek_jog_pasek_strzalki.html`** (`ManualPage.tsx`, `TcpControls.tsx`, `MotionExecution.tsx`, `CartesianPlanning.tsx`, `ControlMessageCenter.tsx`, `style.css`). Pozostałe pliki zostają jako archiwum propozycji.

Wdrożenie wybranego wariantu to osobne zadanie w `web/src` (`TcpControls.tsx`, `MotionExecution.tsx`, `CartesianPlanning.tsx`, `ControlMessageCenter.tsx`, `ManualPage.tsx`).
