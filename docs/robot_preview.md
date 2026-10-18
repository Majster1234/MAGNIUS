# Podgląd robota 3D

W ekranie **Sterowanie ręczne** nad tabelą JOG znajduje się panel o wysokości
połowy okna przeglądarki (50dvh). Scena 3D wykorzystuje całą szerokość panelu;
pozycje liczbowe pozostają wyłącznie w tabeli sterowania poniżej. Nagłówek
podglądu pokazuje oznaczenie modelu przykładowego i kompletność pomiarów X/6.
Mysz obraca, przesuwa i przybliża widok. Ikona domku w prawym górnym rogu
sceny dopasowuje kamerę do bieżącej pozy modelu. Nie ma przycisków widoków
Izometria / Przód / Bok / Góra. Model nie wysyła poleceń do napędów.

Przy każdym przegubie poświata wskazuje odpowiednią granicę „− LIMIT” lub
„+ LIMIT”. Kolor wynika z najnowszego pomiaru, bez czekania na interpolację modelu:

- powyżej 15° do granicy — brak poświaty;
- powyżej 5° do 15° włącznie — żółta;
- od 0° do 5° włącznie — czerwona, mocniejsza w ostatnim 1°;
- po przekroczeniu — pulsująca czerwona, jeden łagodny cykl na sekundę.

Znaczniki leżą przy granicach obrotu w lokalnym układzie przegubu, względem
poprzedniego członu, zgodnie z origin/RPY i osią URDF. Skonfigurowane osie używają
limitów i kąta przegubu; pozostałe ochrony wału ±3600°. Odległości wieloobrotowych
nie sprowadzamy modulo 360°. Brak pomiaru nie tworzy alarmu; utrata świeżości
zachowuje ostatnie ostrzeżenie jako przygaszone i nieruchome, z etykietą
„ostatni pomiar”. Nieważna kalibracja nie tworzy alarmu pozycji przegubu.
Poświaty korzystają z obecnego WebSocketu i nie zmieniają blokad JOG ani limitów.
Zasoby tworzymy raz dla sześciu osi i zwalniamy po opuszczeniu modułu;
pulsowanie korzysta z istniejącego limitu 30 FPS i zatrzymania przy ukrytej stronie.

Bezpośrednio pod nazwą strony znajduje się przewijana konsola zdarzeń:
informacje bez tła, ostrzeżenia na przygaszonym żółtym tle, awarie i błędy
na czerwonym. Obejmuje połączenie, świeżość danych, dostępność napędów,
komunikaty sterownika, wynik przyjęcia operacji i błędy modelu. Czas HH:mm:ss.SSS
jest czasem zaobserwowania zdarzenia przez przeglądarkę; dokładny zapis komunikacji
CAN nadal zapewnia osobny moduł nagrywania. Konsola zachowuje najwyżej 100 wpisów
po 800 znaków, wyłącznie w pamięci otwartego modułu. Powtarzane statusy 20 Hz nie
tworzą nowych wpisów. Automatycznie śledzi koniec, dopóki użytkownik nie przewinie
historii w górę; przewinięcie na dół przywraca śledzenie.

## Model demonstracyjny i pomiary

`robot_description/urdf/preview_6dof.example.urdf` zawiera własny model z brył,
sześć osi continuous i stały TCP, bez siatek, tekstur i ros2_control.
Osie: Z/Y/Y/X/Y/X. Przesunięcia w mm: (0,0,100), (0,0,100), (0,0,250),
(220,0,0), (80,0,0), (60,0,0); TCP (80,0,0). To przykładowa geometria,
zatwierdzona do podglądu, nie pomiary fizycznego robota.

CAN 1–6 odpowiada joint_1–joint_6. Osie bez profilu mają demonstracyjne odwzorowanie
wału **1:1**. Po zastosowaniu kompletnego profilu używamy przełożenia, kierunku
i offsetu: `q = direction × motor_rad / gear_ratio + zero_offset_rad`.
Niepotwierdzona kalibracja pomija pomiar przegubu. Geometria szkicu trafia do tego
podglądu dopiero przez **Zapisz do ROS**; model pod tabelą konfiguracji jest lokalny.
Panel pokazuje stopnie; URDF i wiadomości ROS używają metrów i radianów.

Węzeł diagnostyki wykorzystuje ten sam snapshot, który zasila panel manualny.
Na `/robot/preview/joint_states` wysyła po jednej osi w wiadomości JointState,
tylko dla nowych, dostępnych, skończonych i świeżych pomiarów (poniżej 250 ms).
Każda wiadomość ma czas konkretnego pomiaru, przeliczony z zegara monotonicznego
na zegar ROS. Nie nadajemy nowego czasu staremu odczytowi; nie publikujemy
fikcyjnych zer brakujących osi, prędkości ani momentów. Brak dodatkowych odczytów CAN.
Publisher ma QoS BEST_EFFORT, depth 6, zgodny z odbiornikiem robot_state_publisher;
nie czeka na potwierdzenia wolnego odbiorcy DDS w wątku usług sterowania.

`robot_state_publisher` działa w `/robot/preview`, z prefiksem ramek `preview/`.
TF i TF static są przekierowane do `/robot/preview/tf` i `/robot/preview/tf_static`,
oddzielnie od rzeczywistego modelu sterowania. Backend odbiera latched
`/robot/preview/robot_description` przez TRANSIENT_LOCAL i udostępnia dokładny
tekst ROS pod `GET /api/v1/robot/description` (XML, ETag). Bez modelu odpowiada
503; ekran ponawia pobranie co sekundę do opuszczenia modułu.
Dotychczasowe ograniczenia LAN, Host i Origin obejmują również ten endpoint.

## Odświeżanie i brak danych

Three.js i urdf-loader są budowane lokalnie i pobierane dopiero przy otwarciu
modułu. Wszystkie zasoby pochodzą z jednego originu, bez CDN i nowego portu.
Pozycje pochodzą z istniejącego WebSocketu `/api/v1/manual/ws` przy 20 Hz.
Renderowanie jest ograniczone do 30 klatek/s, a nieruchoma scena bez pulsującego
alarmu nie jest rysowana ponownie. Interpolacja używa najwyżej dwóch próbek na oś, 50 ms
opóźnienia, bez ekstrapolacji i zawijania wieloobrotowych kątów.
Ukrycie karty wstrzymuje rysowanie; opuszczenie ekranu zwalnia renderer,
materiały, geometrie, sterowanie kamerą, obserwatory i timery.

Brak odczytu od początku oznacza umowne zero wyłącznie w geometrii widoku
i „brak danych” przy wartości liczbowej. Po utracie pomiaru geometria zamiera
w ostatnio pokazanej pozycji; liczba jest oznaczona jako nieaktualna.
Brakujące osie są przygaszone. Nagłówek podaje X/6 aktualnych osi; poza przy
niekompletnych danych jest częściowa. Początkowe dopasowanie kamery uwzględnia
pierwszą otrzymaną pozę. Dalsze dopasowanie następuje na żądanie operatora.
Błąd URDF/WebGL trafia do konsoli i pozostawia tabelę pozycji oraz sterowanie dostępne.

## Uruchamianie i dalszy model

Standardowo `./scripts/start_web_status.sh` uruchamia też publisher modelu.
Argumenty launch: `preview_enabled:=false` wyłącza publisher;
`preview_urdf:=/pełna/ścieżka/model.urdf` wybiera inny opis dla podglądu.
Pierwszy etap obsługuje bryły URDF i nazwy joint_1–joint_6. Obsługa siatek
oraz zastosowanie rzeczywistych przełożeń, geometrii i bazowania są kolejnym etapem.
Przykładowy URDF nie spełnia warunków uruchomienia sterowania automatycznego.

## Sprawdzenia — 2026-10-19

Poświaty limitów:

- TypeScript, build produkcyjny i 31 testów Vitest OK; progi obu stron,
  granice 15°/5°/1°/0°, wielokrotne obroty, brak danych i nieaktualne pomiary.
- Trzy scenariusze Chromium: rzeczywiste pulsowanie WebGL przy stałej pozycji,
  kolory/strona limitu, zamrożenie nieaktualnego alarmu, brak komend sterujących
  z wizualizacji, dotychczasowy JOG i konfiguracja.
- Oddzielny pasywny Chromium w LAN: rzeczywiste ostrzeżenia/przekroczenia J2/J4 widoczne,
  bez błędów JS i komend sterujących. Nie wykonywano prób ruchu, restartu ROS
  ani zmiany parametrów. Dopasowanie kamery pomija geometrię poświat.

## Sprawdzenia — 2026-10-18

Zmiana układu i konsoli:

- TypeScript, build produkcyjny i 16 testów Vitest OK.
- Chromium na atrapach: scena na połowę wysokości okna i całą szerokość panelu,
  brak bocznych pozycji, obrót myszą i domek dopasowania bez komend do silników.
  Konsola: kolory poziomów, czas z milisekundami, brak duplikatów, limit 100,
  śledzenie końca i przewijanie historii. Dotychczasowe przypadki JOG i konfiguracji OK.
- Rozdzielczości 1280×800, 1366×768, 1600×1000 i 1920×1080: cała sekcja mieści
  się w wysokości okna; podgląd odpowiednio 400/384/500/540 px, wiersze 29 px.
- Oddzielny pasywny Chromium w działającym LAN: aktualne dane, brak fault i błędów JS,
  podgląd 384 px przy 1366×768. Brak komend JOG i żądań zmiany ENABLE lub zera.
  Nie restartowano procesów ROS/backendu ani nie wykonywano testów ruchu.

Poprzedni etap integracji modelu (nie powtarzano teraz testów ROS/backendu):

- Build pięciu właściwych pakietów ROS, TypeScript i build produkcyjny OK.
- 47 testów backendu, 14 Vitest, 2 GTest podglądu, 3 testy pluginu i 7 bramek ROS OK.
- Osobna integracja ROS: rzeczywisty robot_state_publisher w odrębnej domenie,
  identyczny opis URDF, quaternion dla −90°, częściowy TF i brak globalnego TF.
- Chromium na atrapach: rzeczywisty WebGL i zmiana obrazu po zmianie enkodera,
  kamery, retry 503, częściowa poza i zamrożenie przy braku danych, brak nowego
  strumienia pozycji, brak WebGL nie blokuje JOG. Dotychczasowe scenariusze
  przytrzymania, STOP, utraty danych i wielu klientów przeszły.
- Oddzielnie wdrożono aplikację przy postoju i ENABLE=0. Pasywny odbiór ROS
  potwierdził jedynie joint_1/2/4, około 20 nowych pomiarów/s na oś i zgodny TF.
  Działający panel LAN odczytał identyczny URDF i kąty enkoderów; bez błędów JS.
  Nie wykonywano nowych prób ruchu ani testu na drugim fizycznym kliencie LAN.
