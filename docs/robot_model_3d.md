# Model 3D robota z CAD i konfiguracja układów przegubów

Stan: 17.11.2026. Kolizje (sprawdzanie odległości między członami) — osobny etap po imporcie modelu; siatki
bezpieczeństwa są już generowane i przechowywane.

## Eksport z Fusion 360

- Każdy człon jako osobny komponent → „Zapisz jako siatkę” (STL binarny/ASCII albo OBJ).
- Robot w **pozycji zerowej** (wszystkie przeguby 0°), wszystkie pliki z **tego samego złożenia** — wspólny układ
  współrzędnych. Jednostki wybiera się przy wgrywaniu (domyślnie mm, jak w Fusion).
- Fusion bywa ustawiony na „Y w górę”: w panelu „Oś w górę w CAD” = Y (obrót +90° wokół X).
- Komponent może mieć kilka nakładających się brył (korpus + czop) — siatka bezpieczeństwa obejmuje ich sumę.

## Architektura

- **Poza kinematycznym URDF.** Identyfikator modelu planera = SHA-256 tekstu URDF, a w C++ zmiana modelu w trakcie
  ruchu kończy się awarią — model 3D jest osobnym magazynem i łączy się z URDF w przeglądarce. Zmiana wyglądu nie
  unieważnia programów ani kalibracji.
- **Magazyn** `~/.local/state/robot/models/` (parametr mostu `robot_model_directory`), `robot_web_bridge/robot_model.py`:
  `meshes/<id>.stl` (metry, układ złożenia), `<id>.safety.stl`, `<id>.json` (stan: queued → processing → ready/failed),
  `assembly.json` (przypisanie siatek do członów `base`, `link_1`…`link_6`, `tool`; przekształcenie złożenie → baza:
  oś „w górę”, x/y/z [mm], roll/pitch/yaw [°], konwencja URDF Rz·Ry·Rx), rewizja z kontrolą konfliktu (409).
- **Przetwarzanie w osobnym procesie** (`robot_model_mesh.py`, nice 10, limit pamięci 2 GiB przez RLIMIT_AS, kolejka po
  jednym zadaniu): ciężkie obliczenia numpy w procesie mostu blokowałyby GIL i opóźniały KEEPALIVE JOG, a brak pamięci
  nie może dotknąć procesów sterowania. Po restarcie mostu przerwane zadania wracają do kolejki albo są oznaczane
  jako nieudane.
- **API:** `GET /api/v1/robot/model`, `PUT /api/v1/robot/model {expected_revision, assembly}`,
  `POST /api/v1/robot/model/meshes?name=&units=` (surowy plik, ≤ 200 MiB, 202), `PATCH|DELETE /meshes/{id}`
  (usunięcie tylko nieprzypisanej), `GET /meshes/{id}/surface.stl|safety.stl` (niezmienne, cache 1 rok).

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

## Konfiguracja układów przegubów (Konfiguracja → Parametry robota → „Model 3D i układy przegubów”)

- Przycisk „Oś Jn”, potem klik na podglądzie w powierzchnię walcową przegubu (czop, otwór, gniazdo łożyska).
  `axisFit.ts`: region gładkich ścian (kąt sąsiednich normalnych < 30°), kierunek = wektor własny najmniejszej
  wartości Σ a·n·nᵀ, punkt = najbliższy prostym normalnych, odrzucenie ścian odchylonych od walca o > 15°.
  Wynik: promień, odchyłka, liczba ścian, zasięg kątowy (ostrzeżenie < 90°). Powierzchnia płaska → komunikat.
- `frameEdit.ts`: nowy układ = najmniejszy obrót obecnego (zachowane „ułożenie” wokół osi), położenie = punkt na osi
  w środku wskazanego fragmentu; kierunek < 1° od osi bazy/poprzedniego układu → przyciągnięty dokładnie; zwrot
  zgodny z dotychczasową osią („Odwróć zwrot osi”). Zmieniają się pola `origin_*` wskazanego przegubu i następnego
  (dla J6 — TCP); pozostałe układy i TCP zostają w świecie.
- „TCP: punkt” / „TCP: środek walca” — położenie TCP względem kołnierza (orientacja bez zmian).
- **Zero robota zawsze pod podstawą** (`frameEdit.rootUnderBase`): układ bazy leży na osi J1, w najniższym punkcie
  siatek członu `base` wzdłuż osi J1 (bez siatki podstawy — na osi J1, wysokość bez zmian). Zatwierdzenie wskazania
  osi J1 przenosi zero automatycznie; panel „Układ złożenia → baza robota” pokazuje stan i przycisk „Przenieś zero pod
  podstawę”. Przeniesienie = przesunięcie układu bazy o wektor do tego punktu: przesunięcie złożenie → baza (model 3D)
  i `joint.1.origin_xyz` (konfiguracja) zmieniają się o ten sam wektor; pozostałe przeguby i TCP są względne. Kąty osi
  i programy (pozycje w kątach) bez zmian; zmieniają się współrzędne kartezjańskie TCP (układ bazy). Zapis: „Zapisz model
  3D” oraz „Zapisz do ROS”.
- Kontrola szkicu: odchyłka równoległości J2 ∥ J3, odległości osi J4–J6 (0 = nadgarstek sferyczny).
- Wartości trafiają do szkicu; do sterowania przez „Zapisz do ROS” (zmiana kinematyki, jak każda zmiana geometrii).
- Zmiana przekształcenia złożenia po wskazaniu osi wymaga ponownego wskazania.

## Podgląd

- Konfiguracja: siatki w układzie złożenia → baza (odcienie członów), opcjonalnie siatki bezpieczeństwa
  (pomarańczowa siatka drutowa) i bryły zastępcze; nakładka wskazanej osi (linia i okrąg o promieniu dopasowania).
- Sterowanie (`robotPreviewScene.attachModel`): siatki dołączone do członów URDF z przesunięciem
  inv(człon w pozycji zerowej)·złożenie — poruszają się z członami; bryły z URDF członu ukryte; materiał dwustronny.
  Model wczytywany przy otwarciu strony sterowania (po zapisie modelu — odświeżyć).
