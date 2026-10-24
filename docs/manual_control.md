# Sterowanie ręczne silnikami

Ekran **Sterowanie ręczne** jest przeznaczony dla stanowiska
desktop. Rozwój widoku mobilnego został zakończony. J1–J6 odpowiadają kolejno
adresom CAN 1–6; brak napędu nie oznacza zmiany numeracji pozostałych osi.
Stałe menu boczne zwija się automatycznie do 68 px, pozostawiając dostęp do
wszystkich stron; przycisk w stopce rozwija menu. Wspólny pasek górny 48 px
pokazuje ścieżkę, połączenie 20 Hz, nagrywanie, planowanie i sieć lokalną.
Rozwijanie menu wywołuje przeliczenie rozmiaru sceny, bez zmiany połączenia JOG.
Przy co najmniej 1180 px dostępnej szerokości obszaru strony panel używa kokpitu:
[podgląd URDF 3D](robot_preview.md) i konsola po lewej, tabela osi po prawej,
a pod tabelą TCP oraz panel „Ruch do celu TCP”. Ten ostatni nie zasłania sceny.
Informacja o planowaniu znajduje się w krótkim pasku pod kontrolkami.
Ramka podglądu preferuje 4:3; węższy obszar (np. po rozwinięciu menu) układa
podgląd nad sterowaniem. W niskim oknie prawa kolumna przewija się niezależnie.
Grafitowe panele i typografia są wspólne dla pięciu stron; zielone akcenty
wyróżniają LIVE, niebieskie — planowanie. Kolor nie zastępuje blokad przycisków.

Tabela ma sześć kolumn: JOG, oś/CAN wraz z połączeniem, pozycja, napęd/ENABLE,
zero wału i prędkość. Zachowuje wszystkie dotychczasowe statusy i operacje.
Przyciski JOG mają co najmniej 36×36 px. W LIVE pomiędzy nimi jest stan ruchu;
w planowaniu strzałki leżą bliżej siebie, bez powtarzania napisu „plan”.
Wartość pozycji i mniejszy cel planowany znajdują się na lewo od skali 8 px.
Biały punkt z zielonym obrysem oznacza pomiar, niebieski trójkąt — cel.
Miejsce na cel pozostaje zarezerwowane także poza planowaniem, więc przełączenie
trybu nie przesuwa kolumn ani skal. Wartości używają cyfr o stałej szerokości.
Przyciski Załącz/Wyłącz tworzą wspólną grupę; jednostka prędkości jest wewnątrz
obrysu pola. Krzyżaki TCP X/Y oraz Rx/Ry mają klawisze 44×44 px;
Z+/Z− i Rz+/Rz− tworzą oddzielne pionowe kolumny po prawej stronie krzyżaków,
także w trybie impulsowym.

Konsola ma ciemne tło i do 84 px wysokości. Kolorowa lewa krawędź i plakietka
wyróżniają informacje, ostrzeżenia i awarie, bez barwienia całych wierszy.
Historia jest ograniczona do 100 wpisów i nie powiela tego samego statusu.
Kolejne zmiany świeżości danych w odstępach poniżej 500 ms są grupowane
w jednym wpisie z ostatnim stanem i licznikiem ×N. Awarie są osobnymi wpisami;
logowanie nie opóźnia blokad JOG ani zatrzymań.
Do analizy zaniku danych służy [nagrywanie CAN/ROS/API/panelu](can_recording.md).
Po zastosowaniu kompletnego profilu w [konfiguracji](configuration.md) wiersz osi
przechodzi na pozycję przegubu i JOG w °/s z przełożeniem, kierunkiem, offsetem
oraz limitami przegubu. Poniższy opis RPM i ±3600° dotyczy trybu wału; ochrona
wału pozostaje dodatkowym ograniczeniem także przy JOG przegubu.
Jeśli zerowanie unieważni kalibrację skonfigurowanego przegubu, tabela nadal
pokazuje pomiar enkodera jako **wał · brak kalibracji**, ze skalą wału ±3600°.
Nie jest to pozycja przegubu: JOG przegubu pozostaje zablokowany do ponownego
zatwierdzenia offsetu w konfiguracji. Cel bliźniaka pozostaje osobną wartością
„Plan”; nie jest nanoszony na skalę wału. Brak odczytu CAN nadal oznacza „—”.
Wiersz każdej osi z nieważną kalibracją ma czerwoną poświatę na krawędziach.
Po najechaniu myszą tooltip opisuje naprawę: **Wyłącz** napęd, w **Konfiguracji**
sprawdź i ustaw właściwy **Offset zera przegubu** tej osi, kliknij **Zapisz do ROS**,
a po zatwierdzeniu ponownie przygotuj napęd przyciskiem **Załącz**.
Kompaktowe wiersze zawierają wskaźnik pozycji wału między −3600° i +3600°,
z zaznaczonym zerem. Wskaźnik służy wyłącznie do odczytu; nie zadaje ruchu.
Przy braku pomiaru nie pokazuje punktu pozycji. Poza limitem punkt pozostaje
na końcu skali, a wartość liczbowa nadal pokazuje rzeczywistą pozycję.
W planowaniu niebieski trójkąt nad tą samą skalą wskazuje pozycję bliźniaka celu.
Rzeczywisty pomiar pozostaje zaznaczony okrągłym punktem. Znaczniki są niezależne;
cel może być widoczny także bez pomiaru napędu, jeśli symulacja zna jego pozycję.
Przykładowa konfiguracja przegubów zawiera już te adresy, ale pozostałe nieznane
parametry robota pozostają `null`.

Przycisk **Planowanie** na wizualizacji uruchamia lokalny, przezroczysty bliźniak
z bieżącej pozycji enkoderów. W tym trybie JOG działa wyłącznie na bliźniaku,
bez komend CAN, ROS, WebSocketu sterowania ani HTTP. Nie wymaga ENABLE, lecz
domyślnie nie pozwala używać osi bez świeżego połączenia i pozycji; dla skonfigurowanego
przegubu wymaga także poprawnej pozycji przegubu. Przyciski ENABLE i zero są
zablokowane. Opis synchronizacji, limitów oraz zakończenia planu znajduje się w
[podglądzie robota](robot_preview.md#planowanie-lokalne).

Wybór **PTP/LIN** oblicza pojedynczy ruch od zapisanego startu do celu ustawionego
JOG i odtwarza go w pętli na trzecim modelu. Kolejny JOG zatrzymuje podgląd,
a puszczenie strzałki przelicza ruch. W ustawieniach systemowych można odblokować
lokalny JOG wszystkich osi bez napędów. Fizyczne sterowanie pozostaje bez zmian.
Szczegóły: [symulacja PTP/LIN](cartesian_planning.md).

## TCP w planowaniu

Panel **TCP** ma dwa krzyżaki: X/Y dla przesunięcia oraz Rx/Ry dla obrotu,
z dodatkowymi przyciskami Z± i Rz±. Przytrzymanie przesuwa cel, puszczenie zatrzymuje
edycję. Jednocześnie działa jeden kierunek TCP albo jedna oś JOG. Przełącznik
**Baza / Narzędzie** wybiera osie odniesienia; domyślnie jest to baza. Obrót odbywa
się wokół początku TCP, bez przesunięcia jego pozycji. Prędkości domyślne symulacji
wynoszą 10 mm/s oraz 5°/s. Nie są parametrami potwierdzonymi dla sprzętu.
Posuw nie jest już obcinany do 1 mm na publikację 20 Hz: zadane 100 mm/s oznacza
przyrost 5 mm na 50 ms, z kontrolą IK w odcinkach do 1 mm. Limity przegubów nadal
mogą uniemożliwić dany krok; brak rozwiązania zatrzymuje edycję zamiast wykonywać skok.

TCP i JOG zmieniają ten sam niebieski cel i jego znaczniki w tabeli. Gdy wybrano
PTP/LIN, rozpoczęcie edycji anuluje poprzednie obliczenia; TCP czeka na potwierdzenie
anulowania. Po puszczeniu kierunku podgląd przelicza się ze stałego startu.
Brak IK lub naruszenie limitów zatrzymuje TCP i zachowuje ostatni poprawny cel;
przyczyna pojawia się w konsoli. Utrata fokusu, ukrycie karty, utrata połączenia lub
zmiana kontekstu zatrzymują edycję; spóźnione wyniki nie zmieniają celu.

Bez ignorowania brakujących napędów TCP wymaga aktualnych pozycji sześciu osi,
w tym poprawnej kalibracji skonfigurowanych przegubów. Istniejąca opcja ignorowania
braków w planowaniu pozwala używać pozycji symulacyjnych, tak jak dla JOG.
Poza planowaniem **fizyczne TCP jest zawsze zablokowane**, także przy 6/6 napędów.
Panel pokazuje brakujące osie i informację o niedostępnym wykonawcy wieloosiowym.
Fizyczny JOG pojedynczych dostępnych osi zachowuje dotychczasowe zasady.

### Tryb impulsowy TCP

Przełącz **Tryb TCP** z **Ciągły** na **Impulsowy**. Wspólny pięciopozycyjny suwak
wybiera **0.1, 1, 5, 10 lub 20**: dla X/Y/Z są to mm, dla Rx/Ry/Rz stopnie.
Jedno naciśnięcie kierunku uruchamia jeden impuls; puszczenie go nie przerywa,
a przytrzymywanie nie powtarza ruchu. Domyślnie wybrane jest 1 mm / 1°.
Suwak zachowuje wybór przy zmianie kierunku i rodzaju ruchu.

Impuls przesuwa wyłącznie niebieski cel planowania. Najpierw oblicza pełną drogę LIN,
następnie odtwarza ją raz z interpolacją JTC w 60 Hz, z rozpędzaniem i hamowaniem.
Prędkości wybiera się w trybie ciągłym; impulsy korzystają z zapamiętanych wartości
oraz limitów profili. W trakcie obliczeń i animacji inne kierunki, JOG, tryb i suwak
są zablokowane; nie ma kolejki kliknięć. **Zatrzymaj impuls** przerywa obliczenia
lub animację, zachowując ostatnią pokazaną pozycję celu. Utrata fokusu, ukrycie
karty, rozłączenie lub zmiana kontekstu również przerywa impuls bez wznowienia.
Błąd planowania pozostawia cel bez zmian. PTP/LIN czeka na zakończenie lub
potwierdzone anulowanie impulsu. Fizyczne TCP pozostaje zablokowane.

## Uruchomienie i obsługa

Zbuduj frontend, następnie workspace według [instrukcji panelu](web_status.md).
Uruchom `./scripts/start_web_status.sh` i otwórz `#manual` z menu. Start aplikacji
wykonuje diagnostykę i SCAN; nie załącza napędów ani nie zmienia ich parametrów.
Początkowe maksimum JOG wynosi 30 obr./min. Można je zmienić parametrem launch
`manual_max_rpm`, w granicach protokołu; zwiększenie wymaga pomiarów na sprzęcie.

1. **Załącz** przygotowuje tylko wybrany silnik: potwierdza stabilną pozycję, włącza
   odpowiedzi `8C`, wyłącza synchronizację `4A`, ustawia `SR_vFOC` przez `82`,
   potwierdza wyłączenie `F3/3A`, zatrzymuje poprzedni ruch przez `F7` i sprawdza postój.
   Nie jest to potwierdzenie usunięcia synchronicznego celu F5; osobny odbiór
   sekwencji F7 → F6(0) opisuje dokumentacja wykonania PTP.
   Odczytuje mikrokrok `00 84`, ustawia ochronę `98` na 300 ms i sprawdza odczyt,
   następnie załącza `F3` i potwierdza ENABLE przez `3A`.
2. **Ustaw zero** wymaga świeżych odczytów, stanu postoju i stabilnej pozycji.
   Wysyła `92`, sprawdza ACK i współrzędną `31` w granicach ±3 impulsów od 0
   (około ±0,066°). Brak
   potwierdzenia nie jest przedstawiany jako sukces. Nie wykonuje bazowania/ruchu.
3. Przytrzymaj **← / →**, aby wysłać `F6` z prędkością z pola. Prawa strzałka
   odpowiada CW i dodatniemu przyrostowi `31`. Odczyt `32` ma odwrotną konwencję
   znaku, ale na badanym firmware potrafi zachować starą wartość po zatrzymaniu.
   Prędkość `speed_rpm` w stanie ręcznym jest liczona z kolejnych pozycji `31`
   z filtrem 100 ms; surowy odczyt `32` pozostaje w diagnostyce CAN jako
   `reported_speed_rpm` z własnym wiekiem. Jednostki dotyczą wału
   silnika, bez przekładni i geometrii robota.
4. Puszczenie zadaje prędkość 0. **Wyłącz** w czasie JOG najpierw hamuje,
   potwierdza postój z kolejnych pomiarów pozycji i dopiero wyłącza ENABLE.

Firmware realizuje rampę `acc=1`: zmiana jednej jednostki prędkości co 12,75 ms.
Przy mikrokroku 16/32/64 daje to około 0,3825 s dla 30 obr./min.
Inne udokumentowane mikrokroki są skalowane; nieprzedstawialna prędkość,
nieobsługiwany mikrokrok lub niepotwierdzone ustawienia blokują JOG.
Nie zmieniamy mikrokroku, prądu, adresu ani funkcji samoczynnego startu.
Nie zakładamy, że zero jest trwałym bazowaniem po odłączeniu zasilania.

Odczyty nadal mają timeout 20 ms, a zapis konfiguracji i zera 100 ms: rzeczywisty
ACK `92` wystąpił po około 27 ms. Dla JOG postój wymaga `F1=1` i pozycji utrzymującej się
przez minimum 60 ms w paśmie ±1° wokół stałego punktu odniesienia. Drobne drgania
enkodera w tym paśmie nie odbierają zezwolenia JOG ani nie zmieniają wskaźnika
„zatrzymany”; odczyt pozycji pozostaje rzeczywisty. Pasmo dotyczy odchylenia
od punktu odniesienia, a nie różnicy dwóch kolejnych próbek. Przekroczenie 1°
wymaga ponownego potwierdzenia stabilności. Nowy ruch kasuje poprzednie potwierdzenie.
Po STOP wymagane są świeże odczyty `32` i `F1` po potwierdzeniu komendy oraz
pozycja zmierzona co najmniej 60 ms po nim. Nie czekamy dodatkowo na dwa
kolejne cykle `32`; rampa musi się zakończyć i wał rzeczywiście ustabilizować.
Zerowanie i sekwencje przygotowania/wyłączenia nadal wymagają 100 ms stabilności
w dokładniejszym paśmie ±3 impulsów (około 0,066°). Tolerancja JOG nie zastępuje
świeżości danych, ENABLE, braku stall ani blokady równoczesnego ruchu drugiej osi.
`32` musi być świeży, lecz jego wartość nie rozstrzyga o postoju. Po rampie
przewidziano do 2 s na stabilizację; brak potwierdzenia zatrzaskuje błąd.

## Właściciel, harmonogram i zatrzymanie

`ManualMotorController` w C++ realizuje sekwencje i stan ruchu. Korzysta z tego
samego `CanBusManager`/`BusRuntime` co diagnostyka i adapter `ros2_control`;
manager przekazuje ramki, odpowiedzi i timeouty, posiadając jeden transport,
cztery miejsca oczekujących zapytań i planowany budżet do 40% CAN (limit 50%). Sekwencja ma najwyżej
24 kroki i stałą pamięć. Stop ma pierwszeństwo przed zwykłymi zapytaniami.

Tylko jedna oś może być w ruchu, także między kartami/przeglądarkami.
Brakujące pozostałe osie nie blokują sterowania. Niepotwierdzony postój aktywnej
osi blokuje dalszy ruch i zatrzaskuje błąd. Reset CAN jest wtedy świadomą
operacją operatora; nie wznawia ruchu ani ENABLE. Zwykły SCAN/reset są blokowane
podczas operacji ręcznej. Rozpoczęcie operacji ręcznej przerywa SCAN.

Pozycje są odczytywane przy 25 Hz na oś, ENABLE/stall przy 10 Hz. Przygotowane
osie mają odczyt prędkości i stanu przy 20 Hz również w postoju, aby następny
JOG mógł zweryfikować gotowość. Publikacja stanu manualnego działa przy 20 Hz,
diagnostyka CAN nadal przy 5 Hz, a przegląd komputera przy 1 Hz.

Limity wału to −3600°…+3600°; droga hamowania uwzględnia rampę, prędkość,
wiek pozycji, 60 ms zapasu komunikacji i margines 1°. Kierunek powrotny z pozycji
poza zakresem może być dozwolony. Są to limity programowe wymagające odbioru
na sprzęcie, nie zamiennik fizycznego obwodu zatrzymania.

Przeglądarka wysyła podtrzymanie co 50 ms, bez kolejki niepotwierdzonych
podtrzymań. C++ sprawdza sesję, sekwencję i monotoniczny czas; po 200 ms bez
ważnego podtrzymania zadaje 0. Puszczenie, utrata fokusu, ukrycie/opuszczenie
strony, zanik danych i rozłączenie wywołują STOP. Powrót połączenia nie wznawia
ruchu. Niepowodzenie STOP powoduje ostatnią próbę zadania 0, zatrzask błędu
i ciszę całej magistrali, aby zapytania nie podtrzymywały ochrony `98`.
Ochrona firmware i rzeczywista droga hamowania wymagają testów sprzętowych.

API ogranicza średnio do 20 komend/s z tolerancją krótkiej serii czterech
wiadomości po opóźnieniu sieci. STOP omija limit. Nie wymagamy minimalnego
odstępu między odbiorami; taki warunek powodował fałszywe zamykanie WebSocket.
Przy nieaktualnym połączeniu panel pokazuje „Dane nieaktualne”, ostatnią pozycję
z etykietą i blokuje sterowanie. Komunikat „Niedostępny” pochodzi z aktualnego
stanu osi w backendzie.

Tryby ręczny i automatyczny wzajemnie się wykluczają. Przygotowanie ręczne
unieważnia potwierdzenie synchronizacji automatycznej; zerowanie unieważnia
potwierdzenie uruchomienia pozycyjnego. Powrót do trajektorii wymaga ponownego
przygotowania parametrów/pozycji/synchronizacji i jawnej aktywacji.

## Interfejsy

Pakiet `robot_control_interfaces` definiuje `ManualState`, `ManualAxisState`,
`SetMotorEnable`, `ZeroMotor`, `JogMotor`. Usługi `/robot/manual/enable`,
`/robot/manual/zero`, `/robot/manual/jog` przyjmują komendy z czasem monotonicznym
tego samego komputera; żądania starsze niż 100 ms są odrzucane. Temat
`/robot/manual/state` ma QoS depth 1 i osobny wiek każdego pomiaru.

| HTTP / WebSocket | Znaczenie |
| --- | --- |
| `GET /api/v1/manual/status` | Ostatni stan sześciu osi |
| `POST /api/v1/manual/axes/{1..6}/enable` | JSON `{"enabled":true}` lub `false` |
| `POST /api/v1/manual/axes/{1..6}/zero` | Zerowanie bez ruchu |
| `/api/v1/manual/ws` | Stan 20 Hz oraz komendy `start`, `keepalive`, `stop` |

Komenda WS ma `action`, `axis`, `direction` (−1/+1 dla start), `rpm`, rosnące
`sequence`. Backend nadaje identyfikator sesji i czas ROS, zwraca `ack` z numerem
sekwencji. Stan przychodzi jako `{"type":"state","state":...}`. HTTP zwraca
202 po przyjęciu, 409 po odrzuceniu; wynik sekwencji znajduje się w statusie.
Wyświetlany stan ROS i odbiór statusów w przeglądarce mają timeout 500 ms.
Pozycja/prędkość na ekranie mają własny limit 250 ms, ENABLE 500 ms; publikacja
snapshotu nie odmładza pomiarów. Utrata WebSocket i nieaktualność danych mają
osobne komunikaty. Eliminuje to miganie wskutek przesunięć faz cykli CAN/ROS/WS.
Zgoda `can_jog/can_zero` wygasa po 150 ms wieku stanu ROS, a C++ ponownie
sprawdza warunki w chwili przyjęcia komendy. Lokalny termin pozycji 60 ms,
prędkości 100 ms, stanu pracy 150 ms, ENABLE/stall 300 ms oraz watchdog
podtrzymania 200 ms pozostają bez zmian. Brak ACK w 100 ms nadal wywołuje STOP.
Pozostają ograniczenia LAN, Host i Origin.

## Sprawdzenie

Weryfikacja wspólnego szablonu i kokpitu 2026-10-20: TypeScript/build,
75 Vitest i pełne siedem zestawów Chromium na atrapach HTTP/WS przeszły.
Sprawdzono pięć tras bez przeładowania, mini-rail bez restartu WS/3D,
mapowanie i wygaszanie obecności CAN, krzyżaki TCP, panel symulacji poza sceną,
checkbox modelu ruchu oraz dotychczasowe zatrzymania i operacje konfiguracji.
Układ sprawdzono przy 1280×800, 1366×768, 1600×1000, 1920×1080 i 2560×1440;
wiersze mają 46 px. Interakcje impulsowe używają bazowego okna 1366×768
w programowym WebGL; końcowa pozycja jest odczytywana po publikacji w scenie.
Test STOP używa dłuższej trajektorii atrapy, aby nie kończyła się przed kliknięciem.
Potwierdzono zachowanie kodu JOG, efektu WS/timerów/zatrzymań i istniejących
handlerów/blokad/ref. Nie wykonywano prób sprzętowych ani testów ROS/backendu.


`colcon test` obejmuje protokół, emulator adresów 1/2/4, pojedynczego właściciela,
hamowanie, limity, zanik lease, błędy, USB i BUS-OFF. `npm test` sprawdza logikę
przycisków i renderowanie. `npx playwright install --with-deps chromium` oraz
`npm run test:browser` uruchamiają Chromium na atrapach HTTP/WS pod `robot.test`,
bez połączenia ze sprzętem. Test zapisuje podgląd do `/tmp/robot-manual-desktop.png`.

Przed sprzętowym testem ruchu agent podaje parametry i uzyskuje zgodę użytkownika.
W bieżącej sesji 2026-10-18 użytkownik jawnie rozszerzył zgodę na testy sesji,
w tym kolejno J1 i J2; nie jest wtedy potrzebne ponowne pytanie. Poza tą sesją
obowiązuje pytanie przed każdym testem. Wyniki sprzętowe opisujemy oddzielnie od emulatorów.

Źródło protokołu: [instrukcja Makerbase 1.0.9](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).

Weryfikacja sprzętowa 2026-10-18, po zgodzie użytkownika: oba silniki sprawdzono
kolejno przez rzeczywiste API/ROS/CAN, z ENABLE, zerowaniem, CW 5 obr./min,
zadaniem 0 po puszczeniu, potwierdzeniem postoju i wyłączeniem ENABLE.

Po skróceniu potwierdzania postoju powtórzono próby w tej samej sesji:

| Oś | Czas zadawania ruchu | Pozycja przed wyłączeniem ENABLE | Postój od STOP: przed / po poprawce |
| --- | --- | --- | --- |
| J1 / CAN 1 | 821 ms | 25,181° | 651 / 521 ms |
| J2 / CAN 2 | 834 ms | 25,291° | 518 / 313 ms |

Potwierdzenie obejmuje stabilizację wału, nie jest pomiarem samego RTT ani
czasu zakończenia rampy firmware. To pojedyncze próby, a nie gwarantowane czasy.
Próby zerowania odrzucone przy chwilowym braku stabilności nie wykonały ruchu;
skrypt ponownie czekał na potwierdzony postój. Po testach J1/J2/J4 mają ENABLE=0.
Końcowy odczyt Netlink: 500000 bit/s, ERROR-ACTIVE, błędy RX/TX i BUS-OFF 0.
J1/J2 pozostają przygotowane w SR_vFOC, mikrokrok 16, ochrona 300 ms; J4 nie
otrzymał poleceń konfiguracji ani ruchu.

Próby wykazały potrzebę F7 przed ENABLE, dłuższego ACK konfiguracji, poprawki
porównania czasu odbioru i sprawdzania postoju z enkodera zamiast samego 32H.
Dodano odpowiednie regresje. Prędkość z 32H bywała opóźniona lub utrzymywała
3 obr./min przy stanie F1=1; estymata prędkości z 31H jest jawnie odrębnym pomiarem.
Nie wykonano sprzętowych prób limitów ±3600°, utraty USB, BUS-OFF ani wszystkich
sześciu osi; wyniki emulatora nie zastępują tych prób.

Sprawdzenie poprawki świeżości i układu 2026-10-18: zbudowano trzy zmienione
pakiety (`mks_servo_can`, `robot_can_diagnostics`, `robot_web_bridge`). Raporty
wybranych pakietów: CAN 43 testy (w tym wpisy CTest i 21 testów modułu ręcznego),
backend 38 testów; zero błędów/niepowodzeń. Nie powtarzano pełnego workspace.
Frontend: TypeScript, build produkcyjny, 12 Vitest i Chromium OK. Przeglądarka
sprawdza m.in. wskaźniki pozycji, wysokość wiersza do 62 px, przerwę 250 ms bez
zrywania sterowania oraz STOP po 500 ms bez statusu, bez automatycznego wznowienia.
To testy na atrapach, oddzielne od powyższych prób sprzętowych.

Rzeczywisty WebSocket przed zmianą: 189 statusów w 10 s, bez utraty socketu,
ale osie 1/2/4 oznaczone jako niedostępne odpowiednio 7/10/10 razy; wiek pozycji
dochodził do 72 ms. Po zmianie: 191 statusów w 10 s, wszystkie trzy osie dostępne
w każdym statusie. Nie sprawdzano w tej turze drugiego fizycznego urządzenia LAN.

Weryfikacja tolerancji postoju JOG ±1° (2026-10-18): zbudowano `mks_servo_can`,
`robot_can_diagnostics` i `robot_hardware`. Wszystkie 50 przypadków GTest przeszło
(raporty colcon zawierają dodatkowo 5 wpisów CTest). Trzy nowe regresje sprawdzają
5 s oscylacji ±0,989° po STOP, narastające przemieszczenie względem stałego
odniesienia oraz blokady przy aktywnym ruchu, nieaktualnym F1 i stall.
Zerowanie nadal odrzuca oscylującą pozycję poza dokładniejszą tolerancją.
Nie zmieniano frontendu ani nie powtarzano jego testów.

Oddzielne próby sprzętowe po wdrożeniu wykonano przez rzeczywisty panel Chromium,
ROS i CAN: kolejno J1 i J2, CW 5 obr./min przez 800 i 400 ms, limit próby 35°,
STOP po puszczeniu. Przemieszczenia: J1 24,35° / 12,61°, J2 25,16° / 12,85°.
Zezwolenie po STOP wróciło po 132–163 ms; każda próba obejmowała 3 s obserwacji
postoju (55 nowych statusów), bez migania `can_jog`, stanu „zatrzymany”, ENABLE
ani dostępności osi 1/2/4. Rozrzut enkodera w tych oknach: 0,088–0,110° na J1
i 0,549–0,945° na J2. Czasy dotyczą tych prób, nie są gwarancją terminów.
Na koniec wyłączono ENABLE J1/J2; nagrywanie diagnostyki pozostaje aktywne.
Wyniki lokalne: `log/manual-can/rest-tolerance-hardware-test.json` (plik ignorowany przez Git).

Zmiana wyglądu (2026-10-18): TypeScript, build produkcyjny i 12 testów Vitest OK.
Chromium na atrapach potwierdził panel 279,5 px, wiersze 29 px, tytuł/statusy u góry,
ciemne tło i brak poziomego przepełnienia przy 1280×800, 1366×768, 1600×1000
i 1920×1080; testy przytrzymania, puszczenia i utraty danych nadal przechodzą.
Osobno otwarto działający panel LAN: nowy układ sterowania, statusy 20 Hz,
ciemny przegląd i ekran CAN, bez błędów JavaScript. Zmiana plików statycznych
nie wymagała restartu backendu. Nie wykonywano nowych prób ruchu ani testów ROS/C++.

## Fizyczny PTP i niepełny zestaw napędów

Fizyczny PTP ma odrębne przygotowanie, lokalny wykonawca C++ i jawne zatwierdzenie.
Ustawienie `allow_missing_motion` dopuszcza pominięcie nieobecnych napędów;
nie potwierdza osiągnięcia TCP i nie zmienia ograniczeń osi uczestniczących.
Obowiązujące zasady wykonania i odbioru: [motion_execution.md](motion_execution.md).
Wcześniejsze opisy podglądu PTP/LIN nadal dotyczą symulacji; fizyczny LIN pozostaje zablokowany.


### Ustawianie prędkości i przyspieszenia PTP

W **Sterowanie ręczne → Planowanie** ustaw cel i w sekcji **Parametry ruchu PTP**
wpisz **Prędkość PTP °/s** oraz **Przyspieszenie PTP °/s²**. Są to wartości
przegubów. Domyślne ustawienie panelu: 6°/s i 50°/s². Prędkość jest ograniczona
aktywnym profilem osi; użytkownik zatwierdził przywrócenie limitu 80°/s.
Każda oś nadal zachowuje swoje ograniczenia prędkości i przyspieszenia.
Obecny profil przegubów ogranicza przyspieszenie do 500°/s²; większa wartość
w polu jest ograniczana przez planner. Kod rampy wewnętrznej MKS acc=1 pozostaje
osobną nastawą; pole panelu zmienia przyspieszenie trajektorii referencyjnej.

Po edycji wybierz ponownie **PTP**, następnie **Przygotuj PTP robota**,
sprawdź podgląd i ponownie zaznacz zgodę przed **Wykonaj PTP**.
Zmiana któregokolwiek parametru unieważnia poprzedni plan i zatwierdzenie,
również po obejrzeniu podglądu przygotowanego programu. Edycja nie uruchamia
ruchu ani ENABLE. Ustawienia JOG w tabeli są oddzielne od parametrów PTP.

Przywrócenie wcześniejszego limitu 80°/s z profilu prób 1 RPM:
`sudo ./scripts/set_ptp_80.sh --apply`. Procedura wymaga postoju, wyłącza napędy,
przyjmuje konfigurację przez API i restartuje usługę. Nie zeruje enkoderów i
nie wykonuje ruchu. Raport wdrożenia: [CAN 1M](can_1m_results.md).
