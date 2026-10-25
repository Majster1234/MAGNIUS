# Podgląd robota 3D

W ekranie **Sterowanie ręczne**, przy dostępnej szerokości strony od 1180 px, panel 3D zajmuje lewą
kolumnę kokpitu, obok tabeli JOG i TCP. Ramka ma preferowaną proporcję 4:3;
przy niższym oknie może się skrócić. Pod nią znajduje się Centrum komunikatów.
Przy węższym obszarze, także po rozwinięciu menu, podgląd o wysokości 48dvh
przechodzi nad sterowanie. Menu pozostaje dostępne jako pasek 68 px lub pełne 224 px.
Scena 3D wykorzystuje całą szerokość swojego panelu;
pozycje liczbowe pozostają wyłącznie w tabeli sterowania. Nagłówek
podglądu pokazuje oznaczenie modelu przykładowego i kompletność pomiarów X/6.
Mysz obraca, przesuwa i przybliża widok. Ikona domku w prawym górnym rogu
sceny dopasowuje kamerę do bieżącej pozy modelu. Nie ma przycisków widoków
Izometria / Przód / Bok / Góra. Model nie wysyła poleceń do napędów.

## Osie TCP, ślad ruchu i osobliwości

Niebieski bliźniak celu oraz pomarańczowy bliźniak ruchu mają w TCP układ osi
**X — czerwona, Y — zielona, Z — niebieska**. Osie i etykiety obracają się wraz
z narzędziem według aktualnego URDF. Dla niepełnej pozycji celu układ jest ukryty.

Niebieska linia (`#4DA3FF`) pokazuje planowany tor TCP bliźniaka PTP/LIN. Narasta
podczas pierwszego przejazdu i pozostaje widoczna w kolejnych cyklach oraz po
zatrzymaniu podglądu. Skok z końca na start nie dorysowuje odcinka powrotnego.
Nowy poprawny plan zastępuje poprzedni ślad; zakończenie planowania, zmiana
kontekstu lub opuszczenie ekranu usuwa go. Samo edytowanie celu zachowuje stary
ślad do otrzymania nowego planu. FK linii korzysta z tej samej interpolacji JTC
co animacja, próbkowanej co najwyżej co 1/60 s (maks. 12001 punktów dla 200 s).
Powtarzanie pętli nie powiększa bufora. Ślad nie jest nagraniem fizycznego TCP.

### Złota historia fizycznego PTP

Złota linia (`#FFC857`) pokazuje tor TCP wyliczany przez FK bieżącego URDF z
surowych pomiarów przegubów. Przekładnie, kierunki i offsety są uwzględnione
w odczytach przegubów. Klon używany tylko do FK śladu nie obcina pomiaru
do limitu URDF, więc przekroczenie nie zostaje ukryte; ograniczenia sterowania
pozostają niezmienione. Źródłem nie jest wygładzona pozycja widocznego modelu
ani pozycja zadana przez planner. Niebieski plan i złota historia mogą być
widoczne jednocześnie; legenda jest stale dostępna nad sceną.

Zapis obejmuje wyłącznie trwające wykonania PTP rozróżnione przez `program_id`
i `execution_key`. Przygotowanie, animacja podglądu i fizyczny JOG nie rozpoczynają
historii. Kolejne wykonania dodają osobne odcinki, bez linii łączących koniec
jednego z początkiem następnego. Sukces, błąd i STOP zachowują zgromadzony tor;
wyjście z trybu planowania także go nie usuwa.

Przy brakujących osiach nieuczestniczących w programie FK przyjmuje ich kąty
startowe z tego programu. Złoty ślad ma wtedy jawne oznaczenie **„Tor modelu —
niepełne pomiary”**. Nie jest potwierdzeniem fizycznego TCP. Utrata pomiaru
uczestniczącej osi, utrata statusu PTP lub przerwa w odbiorze rozdziela tor:
po odzyskaniu danych powstaje nowy odcinek, bez interpolowania luki. Zmiana
zestawu dostępnych pomiarów także rozdziela odcinki.

Istniejący strumień WebSocket dostarcza pomiary; zapis ma limit 20 Hz i pomija
powtórzenia. Pomiar przegubu musi być młodszy niż 250 ms, a ostatni status PTP
niż 750 ms. Nie powstają dodatkowe zapytania HTTP, usługi ROS ani ramki CAN.
Renderowanie i pobieranie statusu zachowują dotychczasowe częstotliwości.
Historia obejmuje tylko próbki obserwowane przez otwarty widok, nie odtwarza
ruchu sprzed jego otwarcia ani z czasu ukrycia karty.

**„Wyczyść ślad”** usuwa wyłącznie złotą historię. Podczas PTP zbieranie wznawia
się od kolejnej poprawnej próbki. Bufor mieści maksymalnie 20 000 punktów;
najstarsze fragmenty są usuwane bez łączenia rozdzielonych odcinków.
Odświeżenie, opuszczenie ekranu albo zmiana modelu/rewizji konfiguracji usuwa
historię. Nie zapisujemy jej na dysku ani w backendzie. Materiały i geometrie
śladu są zwalniane razem ze sceną.

### Ostrzeżenia o osobliwościach

Blisko konfiguracji osobliwej odpowiedni model otrzymuje żółtą, pulsującą łunę;
przy większej bliskości czerwoną, pulsującą szybciej. Łuna znajduje się w początku
J5 dla ostrzeżenia nadgarstka J4–J6 albo J3 dla ostrzeżenia ramienia. Jej pozycja
wynika z bieżących transformacji danego modelu URDF; nie jest przenoszona do TCP.
Przestrzenny znacznik **!** i linia wskazują punkt łuny. Opis rodzaju ostrzeżenia
oraz rozwiązanie pokazuje Centrum komunikatów, osobno dla **Robot**, **Cel**
i **Symulacja**. Znacznik ma również etykietę dostępności i tooltip. Przy pokrywających się modelach różne wysokości dymków
zachowują wspólny punkt kotwiczenia. Zmiana kamery i pozy przemieszcza je razem.
J3 jest umownym miejscem prezentacji ostrzeżenia całego łańcucha; pełny Jacobian
nie wyznacza pojedynczego fizycznego punktu osobliwości.

Łuna pulsuje także przy nieruchomej pozie (okres 1,6 s dla żółtej, 0,9 s dla
czerwonej), w dotychczasowym limicie 30 klatek/s. Preferencja przeglądarki
„ogranicz animacje” pozostawia stałą łunę i dymek. Wyjście ze strefy ostrzeżenia
usuwa oba znaczniki; podświetlenia nie wpływają na dopasowanie kadru modelu.
Nadgarstek dodatkowo podświetla człony J4–J6, a ostrzeżenie całego łańcucha
wszystkie człony. Rzeczywisty robot wymaga sześciu aktualnych pomiarów; brakujące
lub nieaktualne pozycje nie tworzą ostrzeżenia. Cel i ruch są oceniane niezależnie
na podstawie pełnych pozycji symulowanych.

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

Pod podglądem znajduje się [Centrum komunikatów](manual_control.md): aktualne
problemy z opisem i rozwiązaniem oraz rozwijana historia do 100 zdarzeń.
Historia pokazuje najnowsze wpisy na początku; wewnętrzne przewijanie nie przesuwa
przycisku STOP. Czas HH:mm:ss.SSS jest czasem zaobserwowania zdarzenia przez
przeglądarkę. Powtarzane snapshoty i procenty postępu nie tworzą wpisów. Aktywne
warunki są niezależne od limitu historii. Oryginalny komunikat można rozwinąć
w szczegółach technicznych; precyzyjny zapis CAN zapewnia osobny moduł nagrywania.

W trybie planowania ramka podglądu jest niebieska. Panel „Ruch do celu TCP”
znajduje się obok przycisków TCP po prawej stronie, poza sceną. Legenda
bliźniaków pozostaje w pigułce w lewym dolnym rogu sceny.
Pomarańczowy model ruchu ma krycie 22%. Checkbox **Pokaż klatki pośrednie**
w panelu symulacji ukrywa ten model wraz z jego osiami i wskaźnikiem osobliwości;
krzywa trajektorii TCP i niebieski cel pozostają widoczne. Przełącznik nie
anuluje ani nie przelicza planu i nie zmienia pozycji fizycznego robota.

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
Błąd URDF/WebGL trafia do Centrum komunikatów i pozostawia tabelę pozycji oraz sterowanie dostępne.

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

## Sprawdzenia wizualizacji osobliwości — 2026-10-23

Build TypeScript/Vite oraz `node web/test/singularity.browser.cjs` przeszły.
Osobny scenariusz uruchamia tylko scenę podglądu z demonstracyjnym URDF w WebGL:
sprawdza J5/J3 zamiast TCP, zgodność kotwiczenia dymka z osią po zmianie kamery,
pulsowanie obrazu przy stałej pozie, żółty/czerwony stan, ukrycie poza strefą,
niezależność celu/ruchu od brakujących pomiarów robota, ograniczenie animacji
i usuwanie zasobów. Nie uruchamia pełnych testów JOG, planera, trace ani backendu;
nie łączy się z ROS/CAN i nie wydaje poleceń sprzętowych.

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


## Wdrożenie złotej historii — 25.10.2026

Zmieniono wyłącznie frontend i dokumentację. TypeScript i build produkcyjny
przeszły. Wykonano 12 testów jednostkowych sceny/śladów, w tym 9 scenariuszy
złotej historii: FK surowych przegubów także poza limitem URDF, przygotowanie/JOG, sukces/STOP,
oddzielne klucze wykonania, brakujące osie, utrata świeżości/statusu/fault,
deduplikacja i limit 20 Hz, czyszczenie oraz przepełnienie bufora. Test bufora
sprawdza także rzeczywiste wierzchołki, żeby wykluczyć pozorne mosty po usunięciu
najstarszego punktu. Zakończone testy Chromium na atrapach:

- `trails.browser.cjs`: oba kolory jednocześnie, legenda, wiele PTP, przerwy,
  niepełne pomiary, czyszczenie w trakcie ruchu, zachowanie po wyjściu z planowania,
  reset przy zmianie modelu i odświeżeniu; zero poleceń sterowania.
- `motion.browser.cjs`: statusy i zatwierdzanie PTP, również po zmianie parametrów.
- `cartesian.browser.cjs`: regresja narastania planu, pętli, anulowania i zatrzymania.
  Pierwsze równoległe uruchomienie trafiło na opóźnioną końcową klatkę po blur;
  sprawdzenie zsynchronizowano z komunikatem zatrzymania i klatkami RAF zamiast
  stałego oczekiwania 100 ms. Końcowe uruchomienie przeszło.

Wdrożono sprawdzone pliki statyczne, zachowując wcześniejsze zasoby i kopię
poprzedniego panelu. HTTP potwierdziło nowe JS/CSS. Nie restartowano ROS:
`robot-web-status.service` pozostała active, MainPID=88862, NRestarts=0.
Nie zmieniano backendu, cykli CAN ani konfiguracji napędów i nie wykonywano
próby ruchowej. Dawne wyniki sprzętowe i benchmarki nie są pomiarami tej sesji.
Lokalne dowody/kopia: `log/ptp-trails/20261024T221348Z/` (znacznik katalogu UTC).

Pasywna kontrola Chromium działającego panelu LAN po odświeżeniu bez cache
potwierdziła WebGL, legendę i oba kolory. Blokowano polecenia HTTP/WS:
zero prób sterowania i zero błędów JavaScript (`live.json` w katalogu dowodów).
