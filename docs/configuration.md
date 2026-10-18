# Konfiguracja robota i snapshoty

Otwórz **Konfiguracja → Parametry robota** (`/#configuration`). Ustawienia systemowe
pozostają puste. Kolumny J1–J6 odpowiadają CAN 1–6. Suwak przełącza geometrię
oraz przełożenia i ograniczenia. Jednostki panelu i JSON: mm, °, °/s, °/s², kg.
Puste pole oznacza brak danych; zero jest wartością.

## Szkic, snapshot i konfiguracja ROS

- **Szkic** aktualizuje się lokalnie i pozostaje w `sessionStorage` karty.
  Przetrwa nawigację i odświeżenie. Zachowano jednokrotną migrację v1 SI → v2 mm/°.
- **Zapisz snapshot** tworzy nazwany plik JSON na komputerze robota. Nie zmienia
  sterowania. Snapshoty są wspólne dla klientów LAN i dostępne po restarcie.
- **Wczytaj**, **Importuj JSON** oraz **Wczytaj aktywną** zastępują szkic po
  potwierdzeniu, jeżeli zawiera niezastosowane zmiany. Import jest walidowany.
- **Eksportuj JSON** pobiera bieżący szkic, także niekompletny.
- **Zapisz do ROS** stosuje rewizję w C++, opisie modelu i trwałym pliku.
  Nie załącza ENABLE, nie zeruje enkoderów i nie uruchamia ruchu.

Bursztynowe pola różnią się od aktywnej konfiguracji ROS. Snapshot ich nie
odznacza. Dopiero potwierdzone zastosowanie ustanawia nową podstawę porównania.
Powrót do wartości usuwa zaznaczenie; `100` i `100.0` oznaczają tę samą liczbę.
Błędne pola mają czerwone obramowanie i blokują zapis/eksport.

Rewizja bazowa chroni przed nadpisaniem zmian drugiego klienta. Konflikt zachowuje
szkic i blokuje zastosowanie. Można go wyeksportować przed wczytaniem aktywnej wersji.
Konsola pokazuje postęp i błędy. **Uzgodnij stan ROS** ponawia sprawdzenie niepewnej operacji.

## Model pod tabelą

Lokalny model składa się z walców i prostopadłościanów. Pozostaje w zerowej pozie
przegubów i nie odpytuje CAN. Mysz steruje kamerą, domek dopasowuje widok.
Niżej są zwinięte sekcje **Dane robota** oraz **Narzędzie i TCP**.

Translacje określają początki przegubów względem poprzedniego członu. Rotacje
stosują URDF RPY: `Rz(yaw) · Ry(pitch) · Rx(roll)`. Osie obrotu: Z/Y/Y/X/Y/X.
Zmiana przesuwa też wszystkie dalsze człony. Poprawny wpis aktualizuje scenę od
razu; przy błędzie pozostaje ostatnia poprawna wartość. Wybrana oś jest wyróżniona.

Geometria pokazuje lokalne XYZ i TCP. Drugi widok pokazuje łuki limitów, znaczniki
obu granic i dymki przełożeń. Zakres ponad 360° ma pełny pierścień i opis liczby
obrotów; wartości nie są redukowane modulo 360°. Braki są oznaczone jako nieokreślone.

**Wczytaj przykład** jawnie uzupełnia geometrię zatwierdzonego modelu demo i TCP.
Nie wstawia przełożeń ani limitów. Zastąpienie istniejącej geometrii wymaga
potwierdzenia. Puste składowe rysunku korzystają tymczasowo z oznaczonego przykładu;
nie zapisujemy tych zastępników jako parametrów fizycznego robota. Model ma limit
30 FPS, aktualizuje istniejące obiekty i zwalnia zasoby po opuszczeniu modułu.

## Pliki i uruchamianie

Launch panelu uruchamia węzeł `robot_configuration`, jedynego właściciela plików.
Domyślny katalog: `~/.local/state/robot/configuration/`. Inny katalog wybierz przez:

```bash
./scripts/start_web_status.sh configuration_dir:=/pełna/ścieżka/danych
```

Katalog zawiera `active.json`, `pending.json` podczas transakcji, `calibration.json`
ze stanem unieważnionej kalibracji i `snapshots/`. Każdy snapshot ma losowy
identyfikator z czasem UTC i osobne małe metadane. Lista pobiera do 20 pozycji na
stronę, a pełny dokument dopiero po wyborze. Nie ma historii po każdym klawiszu.
Usunięcie wybranego snapshotu wymaga potwierdzenia. Zapis używa `fsync + rename`,
bez bazy danych i uprawnień administratora.

JSON zawiera `schema_version: 1`, `units: "mm-deg-kg"`, `name`, `created_at` ze
strefą czasową oraz kompletny słownik `values`. Klucze odpowiadają edytorowi, np.
`joint.1.origin_x`, `joint.1.gear_ratio`, `tool.tcp_y`. Liczby lub `null` opisują
pola liczbowe, tekst lub `null` nazwy. Pełny wzorzec otrzymasz przez eksport.
Limit dokumentu to 64 KiB. Odrzucamy nieznane wersje/jednostki/klucze, NaN/infinity,
nieprawidłowe zakresy i kolizje nazw układów.

## Zastosowanie i obsługa błędów

Profil przegubu wymaga przełożenia >0, kierunku ±1, offsetu zera, limitów dolny<górny,
dodatniej prędkości i przyspieszenia. Osie są niezależne. Początkowo niekompletny
profil pozostaje niegotowy, z dotychczasowym JOG wału. Usunięcie wymaganego pola
wcześniej skonfigurowanej osi odrzuca zastosowanie całego dokumentu.

C++ sprawdza brak ruchu, aktywnej automatyki, resetu i awarii. Zmieniane podłączone
osie wymagają świeżego ENABLE=0 i potwierdzonego postoju z kolejnych pozycji.
Odłączone osie mogą otrzymać profil, ale przygotowanie wymaga późniejszej weryfikacji.
Napędy nie są wyłączane automatycznie.

Transakcja obejmuje blokadę C++, potwierdzenie nowego opisu ROS, commit C++, zapis
pliku i zwolnienie blokady. Operacje plikowe nie trafiają do pętli CAN. Błąd
wycofuje poprzedni model i parametry. Niepotwierdzone wycofanie zachowuje blokadę
i dziennik do uzgodnienia. Timeout nie jest przedstawiany jako pewne niepowodzenie.
Przy starcie odtwarzamy aktywną rewizję lub uzgadniamy przerwaną operację przed
udostępnieniem sterowania. Żądania mają identyfikatory i ograniczony czas ważności.
Blokada pliku `owner.lock` uniemożliwia uruchomienie dwóch właścicieli tego samego katalogu.

`motion_control.launch.py` również uruchamia konfigurację i publisher podglądu.
Panel do już uruchomionego sterowania sprzętowego uruchamiaj przez
`./scripts/start_web_status.sh standalone_can:=false configuration_enabled:=false preview_enabled:=false`.
Jeżeli właściciele konfiguracji i podglądu już działają osobno, można je wyłączyć
w launch sterowania przez `configuration_enabled:=false`.

Zmiany unieważniają przygotowanie zmienionych napędów i odbiór automatyki.
Podgląd w `/robot/preview` nadal jest oddzielony od modelu pełnego sterowania;
zapis geometrii nie zastępuje odbioru sprzętu, bazowania i konfiguracji trajektorii.

## JOG skonfigurowanego przegubu

Panel manualny pokazuje pozycję i limity przegubu oraz prędkość w °/s. Kąt wału
jest dostępny w opisie pozycji. Oś bez profilu ma jawne jednostki RPM wału.

`q = direction × motor_rad / gear_ratio + zero_offset_rad`.
Prędkość silnika: `|prędkość_przegubu_deg_s| × gear_ratio / 6` RPM, z kierunkiem
uwzględniającym znak przekładni. C++ pilnuje ograniczenia 30 RPM wału i limitu
przegubu oraz rozdzielczości mikrokroku. Zadana prędkość nie przekracza żądanej.

Pozostaje rampa firmware `acc=1`. Jeżeli jej przyspieszenie po przekładni przekracza
zadany limit, przygotowanie kończy się przy ENABLE=0 z opisem przyczyny, bez
załączenia napędu i bez awarii pozostałych osi. Hamowanie uwzględnia limity
przegubu; nadal działa ochrona wału ±3600°, watchdog i jedna oś naraz.
Starszy interfejs JOG wału nie uruchomi skonfigurowanego przegubu.

`acc` jest parametrem firmware, a nie wartością w °/s². Według rozdziału 9.1
[instrukcji MKS 1.0.9](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf)
odstęp zmiany prędkości o jedną jednostkę to `(256 − acc) × 50 µs`.
Przy mikrokroku 16/32/64 i przełożeniu 1:1 najmniejsza rampa `acc=1` daje
`6 / 0,01275 ≈ 470,6 °/s²` przegubu. Limit 100 °/s² jest zatem przekroczony;
przy rzeczywistym przełożeniu 5:1 byłoby to około 94,1 °/s². Większe `acc`
przyspiesza rampę, a `acc=0` nie zapewnia łagodniejszego ruszania ani hamowania.
Konsola pokazuje minimum i ustawiony limit w jednostkach panelu, °/s².
System nie zwiększa limitu ani nie zmienia przełożenia samoczynnie. Programowe
schodkowanie komend prędkości nie gwarantuje tego samego chwilowego ograniczenia
przyspieszenia: firmware nadal wykonuje przejścia między dyskretnymi prędkościami.

**Zeruj wał** unieważnia odniesienie osi; przed kolejnym ruchem trzeba ponownie
zatwierdzić offset przy wyłączonym ENABLE. Stan trafia do `calibration.json`;
reset CAN go nie kasuje. Zapis nie wykonuje fizycznego dojazdu do zera.

## Interfejsy

HTTP zachowuje dotychczasową politykę LAN, Host i Origin:

- `GET /api/v1/configuration`: rewizja, dokument i postęp/wynik operacji.
- `GET/POST /api/v1/configuration/snapshots`: lista z kursorem / utworzenie.
- `GET/DELETE /api/v1/configuration/snapshots/{id}`: wczytanie / usunięcie.
- `POST /api/v1/configuration/validate`: sprawdzenie importu bez zastosowania.
- `POST /api/v1/configuration/apply`: dokument i `expected_revision`; przyjęcie 202.
- `POST /api/v1/configuration/reconcile`: ponowne uzgodnienie niepewnego wyniku.

ROS: `/robot/configuration/command` dla plików/API, `/robot/configuration/state`
z ostatnim stanem, typowane `/robot/configuration/control` dla C++.
`JointConfiguration` używa SI. `/robot/manual/jog_joint` przyjmuje rad/s i rewizję;
`/robot/manual/jog` zachowuje RPM. Manualny WebSocket używa `space: joint`,
`velocity_degrees_s` i `revision`; pozycje nadal pochodzą z jednego strumienia.

## Weryfikacja

Testy obejmują schema/storage/transakcje, prawdziwy ROS/RSP/TF na atrapie sterowania,
emulator C++ i Chromium z WebGL. Bieżące wyniki i ograniczenia odbioru: `AGENTS.md`.
Testy sprzętowe raportujemy osobno. Przełożenia w testach nie są danymi fizycznego robota.
