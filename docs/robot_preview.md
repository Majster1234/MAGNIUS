# Podgląd robota 3D

W ekranie **Sterowanie ręczne** nad tabelą JOG znajduje się panel o wysokości
połowy okna przeglądarki (50dvh). Scena 3D wykorzystuje całą szerokość panelu;
pozycje liczbowe pozostają wyłącznie w tabeli sterowania poniżej. Nagłówek
podglądu pokazuje oznaczenie modelu przykładowego i kompletność pomiarów X/6.
Mysz obraca, przesuwa i przybliża widok. Ikona domku w prawym górnym rogu
sceny dopasowuje kamerę do bieżącej pozy modelu. Nie ma przycisków widoków
Izometria / Przód / Bok / Góra. Model nie wysyła poleceń do napędów.

## Osie TCP, ślad ruchu i osobliwości

Niebieski bliźniak celu oraz pomarańczowy bliźniak ruchu mają w TCP układ osi
**X — czerwona, Y — zielona, Z — niebieska**. Osie i etykiety obracają się wraz
z narzędziem według aktualnego URDF. Dla niepełnej pozycji celu układ jest ukryty.

Pomarańczowa linia pokazuje tor TCP pomarańczowego bliźniaka PTP/LIN. Narasta
podczas pierwszego przejazdu i pozostaje widoczna w kolejnych cyklach oraz po
zatrzymaniu podglądu. Skok z końca na start nie dorysowuje odcinka powrotnego.
Nowy poprawny plan zastępuje poprzedni ślad; zakończenie planowania, zmiana
kontekstu lub opuszczenie ekranu usuwa go. Samo edytowanie celu zachowuje stary
ślad do otrzymania nowego planu. FK linii korzysta z tej samej interpolacji JTC
co animacja, próbkowanej co najwyżej co 1/60 s (maks. 12001 punktów dla 200 s).
Powtarzanie pętli nie powiększa bufora. Ślad nie jest nagraniem fizycznego TCP.

Blisko konfiguracji osobliwej odpowiedni model otrzymuje żółte podświetlenie,
przy większej bliskości czerwone, siatkową kulę przy TCP i etykietę **Robot**,
**Cel** albo **Ruch** z nazwą obszaru. Nadgarstek podświetla J4–J6, a ostrzeżenie
całego łańcucha wszystkie człony. Rzeczywisty robot wymaga sześciu aktualnych
pomiarów; brakujące lub nieaktualne pozycje nie tworzą ostrzeżenia. Cel i ruch
są oceniane niezależnie na podstawie pełnych pozycji symulowanych.

Kierunki i początki osi pochodzą z aktualnych transformacji URDF. Wskaźnik
nadgarstka to stosunek najmniejszej do największej wartości singularnej części
kątowej Jacobianu J4–J6, zgodny z plannerem C++: żółty do `tan(15°)`, czerwony
do `tan(2,5°)`. Pełny Jacobian uwzględnia także utratę rzędu ramienia: część
liniową dzielimy przez największą odległość TCP od początku przegubu; stosunek
wartości singularnych do 0,02 daje żółty, do 0,005 czerwony. Są to progi
wizualizacji modelu, bez zmiany solvera, limitów prędkości lub sterowania sprzętem.
Ocena nie ogranicza się do kąta J5 i uwzględnia niestandardowe osie oraz RPY.

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

## Planowanie lokalne

Przycisk **Planowanie** jest dostępny przy wczytanym modelu, aktualnych danych,
braku aktywnej operacji fizycznego JOG oraz co najmniej jednej dostępnej osi.
Tworzy on drugi, półprzezroczysty niebieski model z pozycji enkoderów. Robot
rzeczywisty nadal śledzi enkodery, a bliźniak zachowuje pozycje planu wyłącznie
w pamięci otwartej karty. Legenda odróżnia oba modele.

W tym trybie strzałki JOG w tabeli poruszają tylko bliźniakiem; żadne polecenie
nie jest wysyłane przez WebSocket, HTTP, ROS ani CAN. **Załącz**, **Wyłącz** i
zerowanie są zablokowane. Dostępna oś musi mieć świeże połączenie i pozycję
enkodera; skonfigurowany przegub wymaga również poprawnej pozycji przegubu.
ENABLE, przygotowanie oraz `can_jog` nie są wymagane. Domyślnie brak sterownika lub pomiaru
blokuje daną oś i zatrzymuje jej lokalny ruch. Powrót danych nie wznawia go.
Wiek pozycji obejmuje wiek podany przez backend i czas od odebrania wiadomości:
po 250 ms oś jest blokowana; po 500 ms nieaktualny jest cały stan sterowania.
Brak kalibracji lub nieprawidłowe limity skonfigurowanego przegubu również blokują oś.

Plan używa tych samych jednostek, prędkości i limitów co widok ręczny: °/s dla
skonfigurowanego przegubu, dla wału RPM przeliczone na °/s w odwzorowaniu 1:1,
a bez profilu zakres −3600°…+3600°. Ruch zatrzymuje się na limicie; nie symuluje
dynamiki ani kolizji. Panel [PTP/LIN](cartesian_planning.md) udostępnia automatyczne
obliczenie i zapętlenie ruchu na trzecim, pomarańczowym modelu; nie zmienia niebieskiego celu.
Wspólny suwak w ustawieniach systemowych odblokowuje JOG wszystkich osi bez napędów,
z aktywnymi limitami profilu. Start bez pomiaru jest jawnie oznaczony jako symulowany.
Poza zakresem możliwy jest tylko
stopniowy ruch do jego wnętrza. Prędkość jest ograniczana maksimum profilu lub
`max_rpm`; opóźnienie timera ponad 100 ms zatrzymuje lokalny JOG bez nadrabiania.
**Od pozycji robota** ponownie
kopiuje dostępne enkodery, a **Zakończ planowanie** usuwa bliźniaka. Zmiana
rewizji konfiguracji unieważnia plan do jawnej synchronizacji. Odświeżenie strony
lub opuszczenie ekranu usuwa plan.

## Uruchamianie i dalszy model

Standardowo `./scripts/start_web_status.sh` uruchamia też publisher modelu.
Argumenty launch: `preview_enabled:=false` wyłącza publisher;
`preview_urdf:=/pełna/ścieżka/model.urdf` wybiera inny opis dla podglądu.
Pierwszy etap obsługuje bryły URDF i nazwy joint_1–joint_6. Obsługa siatek
oraz zastosowanie rzeczywistych przełożeń, geometrii i bazowania są kolejnym etapem.
Przykładowy URDF nie spełnia warunków uruchomienia sterowania automatycznego.

## Sprawdzenia — 2026-10-19

Planowanie lokalne:

- TypeScript/Vite i 39 testów Vitest OK, w tym 8 testów kontrolera planu.
- Cztery scenariusze Chromium na atrapach HTTP/WS OK. Osobny test planowania
  sprawdza dwa modele WebGL, brak ENABLE, brak komend sprzętowych, utratę osi,
  wiek pomiarów, rozłączenie, zmianę rewizji i synchronizację zgodnego URDF.
  Wiersze pozostają na poziomie 29 px, podgląd 50dvh w trzech rozdzielczościach desktop.
- Przebudowano `robot_web_bridge` przez `colcon build --symlink-install --packages-select robot_web_bridge`.
- Chromium w działającym panelu LAN potwierdził zgodność serwowanego buildu,
  wejście/wyjście z planowania i lokalny JOG J1. Dodatkowa blokada zapisów HTTP
  i komend sterujących WS nie zarejestrowała żadnej próby ich wysłania.
  Bez błędów JS, zmian ENABLE/zera, restartu ROS i fizycznego testu ruchu.
- Node.js 22.14.0 użyty tymczasowo po weryfikacji SHA-256 oficjalnego archiwum;
  nie instalowano go systemowo. W buildzie pozostaje ostrzeżenie o wielkości modułu Three.js.

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
