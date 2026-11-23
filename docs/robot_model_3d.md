# Model 3D robota z CAD i kinematyka z łańcucha elementów

Stan: 23.11.2026 (tryb modułowy — etapy 1–3; dawny tryb „całe złożenie z Fusion” usunięty z interfejsu). Kolizje
(sprawdzanie odległości między członami) — osobny etap; siatki bezpieczeństwa są już generowane i przechowywane.

## Eksport z Fusion 360

- Każdy element (część obracająca się z osią, wspornik, rura, przedłużenie, łożysko…) osobno: komponent → „Zapisz jako
  siatkę” (STL binarny/ASCII albo OBJ). Układ pliku dowolny — położenie ustala spasowanie w aplikacji; pliki z jednego
  złożenia w pozycji zerowej spasowują się w swoich miejscach. Jednostki przy wgrywaniu (domyślnie mm, jak w Fusion).
- Fusion bywa ustawiony na „Y w górę”: przy podstawie łańcucha „Oś w górę” = Y (obrót +90° wokół X).
- Komponent może mieć kilka nakładających się brył (korpus + czop) — siatka bezpieczeństwa obejmuje ich sumę.

## Tryb modułowy

Decyzja z 23.11.2026: każdy element (moduł osi, rura, wspornik, przedłużenie, adapter) wgrywany osobno, we własnym
układzie pliku; robot składany w aplikacji. Etapy:

1. **Status elementu i oś na elemencie (wdrożone).** Status: `axis` — miejsce łożyskowania osi (część obracająca się
   wokół osi przegubu, z gniazdem łożyska lub czopem; od niej zaczyna się człon tej osi), `structural` — element konstrukcyjny (sztywny; liczy się do kinematyki,
   gdy za nim w łańcuchu jest kolejny model osi), brak — nieokreślony (poza złożeniem modułowym). Oś modelu osi
   wskazywana kliknięciem w walec na podglądzie samego elementu (`ElementEditor.tsx`, `elementScene.ts`, dopasowanie
   `axisFit.ts`): punkt na osi w środku wskazanego fragmentu [mm], kierunek jednostkowy — zwrot = dodatni obrót przegubu
   wg reguły prawej dłoni („Odwróć zwrot osi”), promień, odchyłka, liczba ścian, zasięg kątowy (ostrzeżenie < 90°).
   Magazyn: `meshes/<id>.element.json` `{revision, role, axis}` — osobny plik, bo proces importu zapisuje `<id>.json`;
   `PUT /api/v1/robot/model/meshes/{id}/element {expected_revision, element: {role, axis}}` (409 = konflikt rewizji;
   oś tylko dla modelu osi i wczytanej siatki; zmiana statusu z modelu osi usuwa oś). `GET /api/v1/robot/model` zwraca
   definicję w `meshes[].element`.
2. **Łańcuch elementów ze spasowaniem (wdrożone, interfejs v2 z `temp/model_3d_kreator`).** `ModelChainBuilder.tsx`
   (podgląd `chainScene.ts`, matematyka `modelChain.ts`, testy `modelChain.test.ts`), na górze panelu „Model 3D”:
   - łańcuch kart od podstawy; ostatnie pole pyta „Co jest za …?” — [Konstrukcyjny] / [Oś Jn]; „+” między kartami
     wstawia element; przełącznik „Prowadź krok po kroku” (pamięć przeglądarki) włącza pytania i podpowiedzi;
   - spasowanie z poprzednikiem: walec na poprzedniku ↔ walec na elemencie (element przed spasowaniem stoi obok łańcucha),
     opcjonalny styk czół (płaszczyzny; przesunięcie dosuwa je wzdłuż osi), dla modelu osi bez osi — walec łożyskowania;
     potem obrót wokół osi (domyślnie najbliższy „bez obrotu”), przesunięcie, „Odwróć element” (domyślny zwrot: element
     po drugiej stronie styku niż poprzednik);
   - położenie względem poprzednika: `placement = F₁·Rz(obrót)·Tz(styk + przesunięcie)·F₂⁻¹` (F = układ na osi walca);
     zmiana elementu przesuwa wszystkie dalsze;
   - wstawienie/usunięcie/zamiana pliku poprzednika: następny element zachowuje miejsce w przestrzeni i dostaje
     `needs_parent` (⚠ „wskaż mocowanie na poprzedniku”); przy prowadzeniu po wstawieniu od razu pytanie o to mocowanie;
   - szkic z Cofnij/Ponów i historią zmian do „Zapisz model 3D”; status i oś pliku zapisują się od razu (dotyczą pliku);
   - tabela „Odległości osi z łańcucha”: kierunek osi, wektor od poprzedniej osi (J1 — od początku pliku podstawy), kąt,
     elementy pomiędzy (liczą się do kinematyki); ostrzeżenia: > 6 osi, elementy czekające na mocowanie, osie bez walca.
   Magazyn: `chain.json` `{revision, up, elements: [{id, mesh, mate, placement}]}`; `PUT /api/v1/robot/model/chain
   {expected_revision, chain: {up, elements}}` (409 = konflikt; podstawa bez spasowania i przesunięcia; ten sam plik
   wielokrotnie; siatki wczytane; ≤ 64 elementy); `GET /api/v1/robot/model` zwraca `chain`; usunięcie siatki użytej
   w łańcuchu — 409. Stary tryb (biblioteka plików, przypisanie do członów, osie na złożeniu) — zwinięta sekcja pod łańcuchem.
   Podgląd na stronie sterowania nadal korzysta ze starego trybu (do etapu 3).
3. **Kinematyka z łańcucha (wdrożone).** `chainFrames` (`modelChain.ts`), `applyChainFrames` (`frameEdit.ts`):
   - człony: element z osią Jn obraca się z Jn i zaczyna człon n; elementy przed J1 — podstawa; po J6 — człon 6;
   - zero robota: na osi J1, w najniższym punkcie siatek podstawy wzdłuż osi; Z bazy = oś J1 („w górę”), **X bazy = przód
     robota** — strona, po której oś J2 jest odsunięta od osi J1 (wspólna prostopadła; osie przecinające się → strona,
     w którą wychodzą siatki za J2), Y = Z × X (w lewo); bez J2 — najmniejszy obrót układu podglądu (24.11.2026: wcześniej
     Y = zwrot J2, co przy zwrocie z dopasowania walca dawało X do tyłu);
   - układ J1 w punkcie wskazanego walca; Jn (n > 1) w rzucie początku poprzedniego układu na oś n (osie równoległe →
     czyste przesunięcie, prostopadłe → wspólna normalna); układ konwencji Cn = najmniejszy obrót poprzedniego (bez 180°),
     lokalna oś przegubu (J1 z, J2–J3 y, J4 x, J5 y, J6 x) wzdłuż osi ze zwrotem najbliższym poprzedniemu — dodatni obrót:
     J1 przód w lewo, J2/J3/J5 do przodu / w dół; zwrot z pliku przeciwny → układ Fn = Cn·180° wokół osi poprzecznej
     (J1: X, pozostałe: Z), dalsze przeguby z modelu liczone od Cn (geometria bez zmian); kierunek < 1° od osi
     bazy/poprzedniego układu → dokładnie ta oś („przyciągnięta”);
   - opis dodatniego obrotu słownie („człon do przodu/do tyłu/w górę/w dół/w lewo/w prawo”, `motionLabel`) — ruch środka
     siatek członu napędzanego przez oś (gdy leży na osi — następnego przegubu); do porównania z ruchem robota przy JOG „›”;
   - zwrot osi: w inspektorze elementu pole „Odwrócony kierunek dodatniego obrotu Jn” (względem konwencji, z opisem
     ruchu), w edytorze pliku — względem wskazanego walca (`axis.reversed`, `direction` = kierunek wynikowy); zapis od razu
     (plik); na podglądzie strzałka na osi i łukowa strzałka dodatniego obrotu (reguła prawej dłoni). Zwrot musi zgadzać
     się z ruchem robota przy obecnej kalibracji (kierunek osi): niezgodność usuwa to pole albo kierunek w kalibracji
     (wtedy też offset i limity z przeciwnym znakiem). Przeguby wirtualne liczone są od układu Fk (jak w URDF);
   - osie wirtualne: przeguby za ostatnią osią z modelu i TCP z wartości konfiguracji (względem poprzedniego przegubu,
     `continueFrames`) — fioletowe bryły z etykietami „Jn (wirt.)” i wiersze „wirtualna” w tabeli; przełącznik
     „Brakujące osie jako bryły wirtualne”; w podglądzie konfiguracji i sterowania człony bez siatek mają bryły zastępcze;
   - tabela „Kinematyka z łańcucha”: położenie i obrót każdego układu względem poprzedniego oraz bieżący szkic
     konfiguracji (różnice wyróżnione); przycisk „Przenieś J1–Jk do szkicu konfiguracji” (zablokowany, gdy element czeka
     na mocowanie) zmienia tylko J1–Jk — dalsze przeguby i TCP zachowują wartości względem poprzedniego przegubu (jadą
     za nowym Jk); do sterowania przez „Zapisz do ROS” (zmiana kinematyki: programy w kątach bez zmian, zmieniają się
     współrzędne kartezjańskie TCP);
   - podglądy: konfiguracja — siatki łańcucha ze szkicu (także niezapisanego) w układzie bazy na tle układów ze szkicu;
     sterowanie — siatki zapisanego łańcucha dołączone do członów URDF (bez łańcucha — dawne złożenie z `assembly.json`).
   Model operatora (24.11.2026, 3 osie, X bazy = +Y złożenia): J1 Z 65,25; J2 X +59,66 / Z 78,96 mm; zwroty z plików J2/J3
   przeciwne do konwencji. Obserwacja robota (kalibracja +1): J2 „−” pochyla do przodu (= model, J2 zostaje odwrócone,
   yaw 180°), J3 „−” podnosi (model z odwróconym J3 pokazuje „+” w górę — do zmiany: J3 bez odwrócenia → yaw 180°
   względem J2, J4 przed J3).
   Odchyłki ~0,15 mm względem złożenia Fusion pochodzą z dopasowania walców przy spasowaniu „Przegub J2”.

## Architektura

- **Poza kinematycznym URDF.** Identyfikator modelu planera = SHA-256 tekstu URDF, a w C++ zmiana modelu w trakcie
  ruchu kończy się awarią — model 3D jest osobnym magazynem i łączy się z URDF w przeglądarce. Zmiana wyglądu nie
  unieważnia programów ani kalibracji.
- **Magazyn** `~/.local/state/robot/models/` (parametr mostu `robot_model_directory`), `robot_web_bridge/robot_model.py`:
  `meshes/<id>.stl` (metry, układ pliku), `<id>.safety.stl`, `<id>.json` (stan: queued → processing → ready/failed),
  `<id>.element.json` (status, oś), `chain.json` (łańcuch); `assembly.json` — dawny tryb, tylko odczyt podglądu
  sterowania, gdy łańcuch jest pusty (API `PUT /api/v1/robot/model` zostaje dla zgodności).
- **Przetwarzanie w osobnym procesie** (`robot_model_mesh.py`, nice 10, limit pamięci 2 GiB przez RLIMIT_AS, kolejka po
  jednym zadaniu): ciężkie obliczenia numpy w procesie mostu blokowałyby GIL i opóźniały KEEPALIVE JOG, a brak pamięci
  nie może dotknąć procesów sterowania. Po restarcie mostu przerwane zadania wracają do kolejki albo są oznaczane
  jako nieudane.
- **API:** `GET /api/v1/robot/model`, `PUT /api/v1/robot/model {expected_revision, assembly}`,
  `PUT /api/v1/robot/model/chain`, `PUT /meshes/{id}/element`, `POST /api/v1/robot/model/meshes?name=&units=` (surowy
  plik, ≤ 200 MiB, 202), `PATCH|DELETE /meshes/{id}` (usunięcie pliku nieużytego w łańcuchu), `GET /meshes/{id}/surface.stl|safety.stl` (niezmienne, cache 1 rok).

## Siatka bezpieczeństwa (2 mm, rzadsza)

1. Pole odległości bez znaku na siatce wokselowej (komórka z budżetu trójkątów, ≤ 1,6·odsunięcie = 3,2 mm),
   odległości z KD-drzewa próbek powierzchni.
2. Zewnętrze = obszar „daleko od powierzchni” połączony z brzegiem siatki (próg ≥ ½ komórki + krok próbek — wykluczony
   przeciek przez powierzchnię).
3. Powierzchnia izo „surface nets” na odległości 2 mm.
4. Wierzchołki rzutowane dokładnie na 2 mm od najbliższego trójkąta, po zewnętrznej stronie — strona z pseudonormalnych
   ważonych kątem (wnętrze ściany / krawędź / wierzchołek), orientacja osobno dla każdej spójnej bryły.
5. Trójkąty, których środek lub krawędź leży bliżej niż 2 mm (cięciwy przy ostrych krawędziach), dzielone spójnie
   (2 rundy, bez szczelin), resztkowy niedobór — dosunięcie wierzchołków ≤ 1 mm.
6. Gęsty eksport (próbkowanie ponad budżet — tysiące wąskich trójkątów walców): klasteryzacja wierzchołków
   (≤ 0,6 mm) i odsunięcie powiększone o błąd uproszczenia; minimum raportowane jako ograniczenie dolne względem
   oryginału.

Wyniki (testy `test_robot_model.py`): odległość min 1,99 mm, średnio 2,02 mm, max ≤ 2,5 mm (sześcian, czworościan
z ostrymi kątami dwuściennymi, walce, kształt wklęsły, nakładające się bryły z odwróconą orientacją); gęsty człon
328 tys. trójkątów → 29 tys. w 10 s / 0,4 GB, siłowo względem oryginału ≥ 2,90 mm (zachowawczo).

## Podgląd

- Budowa łańcucha (`chainScene.ts`): siatki w pozycji zerowej, osie (żółte, strzałka = dodatni obrót, etykieta Jn),
  znaczniki wskazań spasowania, nowy element obok łańcucha / przezroczysty po spasowaniu.
- Konfiguracja: siatki łańcucha w układzie bazy (odcienie członów), opcjonalnie siatki bezpieczeństwa (pomarańczowa
  siatka drutowa) i bryły zastępcze, na tle układów przegubów ze szkicu.
- Sterowanie (`robotPreviewScene.attachModel`): siatki dołączone do członów URDF z przesunięciem
  inv(człon w pozycji zerowej)·położenie w bazie — poruszają się z członami; bryły z URDF członu ukryte (człony bez siatek
  — bryły zastępcze). Model wczytywany przy otwarciu strony sterowania (po zapisie modelu — odświeżyć). Dopóki
  kinematyka z łańcucha nie jest zapisana do ROS, siatki obracają się wokół osi z aktywnej konfiguracji.
