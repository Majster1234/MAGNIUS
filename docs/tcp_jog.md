# Fizyczny JOG TCP

W LIVE panel TCP steruje rzeczywistymi przegubami. W planowaniu zachowuje
dotychczasową symulację. Kierunki X/Y/Z zachowują orientację TCP, a Rx/Ry/Rz
obracają wokół początku TCP bez zmiany jego pozycji. Układ Baza odnosi kierunek
do bazy, Narzędzie — do orientacji TCP z chwili naciśnięcia. Jednocześnie działa
jeden kierunek. Zmiana kierunku/układu wymaga zakończenia poprzedniego ruchu.

Ruch ciągły i impulsowy w LIVE wymagają przytrzymania. Impuls wyznacza maksymalną
drogę; puszczenie rozpoczyna wcześniejsze hamowanie. Po ukończeniu impulsu
przytrzymany przycisk nie rozpoczyna kolejnego. Prędkości LIVE trzeba wpisać
osobno; wartości symulacji nie przechodzą do fizycznego ruchu.

## Warunki dostępności

Funkcja jest domyślnie wyłączona (`tcp_jog_enabled: false`). W lokalnym pliku
produkcyjnym `config/local/execution.production.yaml` ustawiono zezwolenie `true`.
Zacznie obowiązywać po restarcie usługi z nową nakładką. Funkcja wymaga wszystkich
sześciu skalibrowanych osi, świeżych pomiarów, ENABLE, potwierdzonego postoju,
wolnego sterowania i CAN 1 Mbit/s. Właściciel CAN dodatkowo sprawdza profil
napędów (wyłączone raporty/SYNC, mikrostep i tryb obsługujący ruch).

Geometria przegubów oraz wszystkie sześć składników przesunięcia/orientacji TCP
muszą być jawnie podane. Generator URDF publikuje `tcp_configured="true"` dopiero
po ich uzupełnieniu; jawne zero jest poprawną wartością, brak danych nią nie jest.
Model demonstracyjny nie uprawnia do fizycznego JOG.

Parametry ROS (węzeł `robot_can_diagnostics`, jednostki SI):

| Parametr | Znaczenie |
| --- | --- |
| `tcp_jog_enabled` | Zezwolenie na fizyczny JOG; domyślnie `false` |
| `tcp_jog_max_linear_velocity` | Maksymalna żądana prędkość w m/s |
| `tcp_jog_max_angular_velocity` | Maksymalna żądana prędkość w rad/s |
| `tcp_jog_linear_acceleration` | Przyspieszenie/hamowanie w m/s² |
| `tcp_jog_angular_acceleration` | Przyspieszenie/hamowanie w rad/s² |
| `tcp_jog_ik_position_tolerance` | Dokładność zbieżności IK w m; domyślnie `1e-9`, maks. `1e-4` |
| `tcp_jog_ik_orientation_tolerance` | Dokładność zbieżności IK w rad; domyślnie `1e-8`, maks. `1e-3` |

Cztery parametry liczbowe muszą być skończone i dodatnie. Domyślne zero jest
wewnętrznym oznaczeniem braku konfiguracji i blokuje funkcję. Nie stanowi
parametru fizycznego. W przykładzie `robot_bringup/config/tcp_jog.example.yaml`
nieznane wartości pozostają zakomentowanymi `null`. Czas hamowania wynikający
z prędkości maksymalnej i przyspieszenia nie może przekraczać 10 s — jest to
ograniczenie rozmiaru horyzontu generatora.

Limity uzupełnia się w **Konfiguracja → Parametry robota → Fizyczny JOG TCP —
wymagane limity**. Panel przyjmuje mm/s, °/s, mm/s² i °/s². Przycisk **Zapisz
do ROS** zapisuje aktywną konfigurację; węzeł konfiguracji przelicza jednostki
na SI i publikuje limity, a adapter aktualizuje powyższe parametry ROS. Aktywna
konfiguracja jest źródłem tych czterech wartości. Puste pola i starsze snapshoty
bez tych pól oznaczają brak limitów i blokują LIVE. Wartości symulacji nie są
kopiowane. Zmiana limitów lub tolerancji IK podczas ruchu jest odrzucana.
Tolerancje IK służą diagnostyce zbieżności solvera i nie zmieniają limitów
mechanicznych, prędkości, watchdogów ani pomiarowego potwierdzania postoju.
Po próbach sprzętowych z 26.10 lokalny `execution.production.yaml` zapisuje
sprawdzone tolerancje IK `1e-6 m / 1e-5 rad`; te same wartości są aktywne w ROS.
Raport rozróżnia postęp referencji od ruchu wyznaczonego z enkoderów przez FK.

## Zbiorcze ENABLE

W **Sterowaniu ręcznym** przyciski **Załącz wszystkie** i **Wyłącz wszystkie**
obsługują wykryte silniki CAN 1–6. Brakujące silniki są pomijane; wykryty silnik
bez świeżej pozycji blokuje operację. Wymagany jest postój i wolny właściciel CAN.
Przygotowanie napędów odbywa się kolejno, po czym końcowe komendy F3 są wysyłane
w kolejności J1→J6 z odstępem **co najmniej 200 ms**. Kolejny silnik wymaga także
świeżego potwierdzenia ENABLE (3A) poprzedniego; opóźnione potwierdzenie wydłuża
odstęp. Ta sama sekwencja obowiązuje przy zbiorczym wyłączaniu. Zakończenie
operacji oznacza otrzymanie potwierdzeń wszystkich objętych nią napędów.

Przyciski są blokowane podczas planowania, innej operacji i fizycznego TCP.
Błąd lub STOP przerywa operację i nie pozwala kontynuować załączania pozostałych
napędów. Przycisk zbiorczego wyłączenia korzysta z istniejącej procedury
wyłączania na postoju; do zatrzymania trwającego ruchu służy STOP robota.

## Wykonanie i zatrzymanie

Biblioteka `robot_kinematics` wyznacza kolejne pozy TCP, rozwiązuje IK od
poprzedniego rozwiązania i sprawdza q/v/a oraz całą dyskretną drogę hamowania.
Jeśli kolejny krok nie jest wykonalny, używa poprzednio zatwierdzonej drogi
hamowania. Nie przycina niezależnie osi. Punkt lookahead pochodzi z tej samej
sprawdzonej drogi, dzięki czemu ekstrapolacja rozruchu nie wykracza poza impuls.
Obliczenia mają budżet 7 ms. Przekroczenie go przed pierwszą referencją utrzymuje
postój i ponawia obliczenie w następnym cyklu; po ruszeniu wybiera sprawdzone
hamowanie. Podtrzymanie operatora i watchdog IO obowiązują również podczas
oczekiwania na pierwszy krok.

Adapter C++ pracuje w osobnym wątku co 10 ms. Właściciel CAN odbiera najnowszą
referencję i zachowuje istniejący profil streaming 100 Hz. Kinematyka nie działa
w wątku IO. Nowe źródło `tcp_jog` ma własną sesję i generację; stare referencje
oraz próby przejęcia przez JTC/JOG/PTP są odrzucane.
Kontrola źródła i generacji odbywa się również przed zapisem do skrzynki.
Nieblokująca bramka serializuje producentów, a adapter JTC wstrzymuje własne
referencje podczas TCP. Chwilowa kolizja pomija próbkę; dłuższą przerwę wykrywa IO.

Usługa `/robot/manual/jog_tcp` (`JogTcp`) przyjmuje START/KEEPALIVE/STOP. START
przekazuje kierunek, układ, prędkość, tryb/krok, model i rewizję. Most WebSocket
nadaje sesję i czas monotoniczny. Pole `emergency` przy STOP wybiera natychmiastową
ścieżkę STOP właściciela CAN. W zwykłym STOP kontroler wykonuje sprawdzone hamowanie.
Status `/robot/manual/tcp_status` (`TcpJogStatus`) trafia do `tcp_jog` w istniejących
paczkach WebSocket stanu manualnego; po 250 ms przestaje być świeży.

Most konwertuje zweryfikowane liczby JSON `velocity` i `step` na `float` przed
utworzeniem żądania ROS, również dla `step: 0` w trybie ciągłym. Test
`npm run test:tcp-panel-ros` obejmuje kliknięcia rzeczywistego panelu, produkcyjny
WebSocket i wygenerowany typ ROS na atrapach bez CAN; wymaga załadowanego
środowiska ROS z `robot_control_interfaces`. Opis awarii w panelu zawiera
konkretny komunikat właściciela. Dalsze próby fizyczne wykonuje się wyłącznie
z panelu operatorskiego, z osobnym opisem i zgodą na każdy test.

Panel wysyła podtrzymanie co 50 ms, oczekuje potwierdzenia do 100 ms. Wątek IO
zatrzymuje ruch po 200 ms bez podtrzymania, nawet jeżeli generator nadal dostarcza
referencje. Podczas zleconego hamowania nadzór świeżości referencji nadal działa
(dotychczasowy limit 60 ms). Utrata fokusu, widoczności, połączenia, modelu lub
pomiarów kończy sterowanie. Ponowne połączenie nie wznawia ruchu.

Po dojściu referencji do v=0 stan pozostaje zajęty do dwóch nowych zestawów
pomiarów oddzielonych co najmniej 100 ms: prędkość zerowa i pozycja w istniejącej
tolerancji przegubu. Przekroczenie czasu hamowania powiększonego o 3 s kończy się
awarią i wymaga resetu. Wysłanie STOP nie jest potwierdzeniem postoju.

## Build i uruchomienie

Poprawka z 26.10 jest budowana w **Release**, ponieważ generator ma budżet
obliczeń 7 ms. Nakładka `install/tcp-jog-fix-ros` nadbudowuje wcześniej aktywną
`install/tcp-settings-ros` i zawiera tylko trzy zmienione pakiety:

```bash
source /opt/ros/humble/setup.bash
source install/tcp-settings-ros/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths ros2_ws/src --build-base build/tcp-jog-fix-ros --install-base install/tcp-jog-fix-ros --symlink-install --packages-select mks_servo_can robot_kinematics robot_can_diagnostics --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Do aktywacji po potwierdzeniu postoju użyj:

```bash
sudo install -m 0644 system/application/tcp-jog-fix-ros.conf.example /etc/systemd/system/robot-web-status.service.d/tcp-settings.conf
sudo systemctl daemon-reload
sudo systemctl restart robot-web-status.service
```

Przebieg diagnostyki i stan wdrożenia poprawki opisuje
[raport z 26.10](tcp_jog_report_2026-10-26.md).

### Poprzednie wdrożenie (25.10)

Nakładka z konfiguracją TCP i zbiorczym ENABLE: `install/tcp-settings-ros`.
Zbudowano ją oddzielnie od wcześniej aktywnej `install/tcp-jog-ros`.
Zmiana struktury
referencji wymaga wspólnego przebudowania `mks_servo_can`, `robot_kinematics`,
`robot_control_interfaces`, `robot_can_diagnostics` i zależnych binariów
`robot_hardware` oraz `robot_motion_planner`. Nakładka zawiera również most,
konfigurację i bringup; frontend buduje `npm run build` w `web/`.

Aktywacja nakładki przez usługę wymaga ustawienia jej zmiennej środowiskowej
`ROBOT_OVERLAY_SETUP=/home/mk/Desktop/Projekty/install/tcp-settings-ros/setup.bash`
w pliku drop-in systemd. Gotowy plik znajduje się w
`system/application/tcp-settings-ros.conf.example`. Operator, po potwierdzeniu postoju,
może zainstalować go i wykonać restart:

```bash
sudo install -m 0644 system/application/tcp-settings-ros.conf.example /etc/systemd/system/robot-web-status.service.d/tcp-settings.conf
sudo systemctl daemon-reload
sudo systemctl restart robot-web-status.service
```

Po restarcie należy odświeżyć panel i uzupełnić zatwierdzone limity w konfiguracji.
Flaga `tcp_jog_enabled: true` nie zastępuje kompletnych limitów i geometrii.
Zakładka Ustawienia systemowe nie jest częścią tego wdrożenia.

Stan na 25.10: nowa nakładka była gotowa, lecz próba instalacji drop-in zakończyła
się komunikatem `sudo: a password is required`. Wtedy usługa nadal korzystała
z `install/tcp-jog-ros` z wyłączoną flagą TCP. Operator później aktywował
`install/tcp-settings-ros`; odczyt 26.10 potwierdził załączoną flagę TCP i limity.
Przed próbą instalacji odczyty wykazały brak aktywnego ruchu, 0 RPM wszystkich
sześciu osi i stabilne pozycje w istniejącej tolerancji trzech impulsów enkodera.

Testy jednostkowe generatora i właściciela oraz test integracyjny ROS korzystają
wyłącznie z atrap transportu. `npm run test:tcp-live` uruchamia Chromium z atrapami
HTTP/WebSocket. Odbiór sprzętowy pozostaje oddzielnym etapem: każdy przejazd wymaga
opisu osi/kierunku, prędkości, zakresu i zatrzymania oraz wyraźnej zgody operatora.
`npm run test:tcp-settings` sprawdza nowe pola i zbiorcze ENABLE na atrapach.
Ta funkcja nie dodaje wykrywania kolizji.

## Weryfikacja — 25.10.2026

- Zbudowano dziewięć wskazanych wyżej pakietów do `install/tcp-settings-ros` oraz frontend.
- Przeszły 163 przypadki GTest: CAN 133, kinematyka 7, diagnostyka/adapter 15,
  interfejs sprzętu 8. Raport `colcon test-result`: 183 elementy, zero błędów,
  porażek i pominięć (raport uwzględnia także uruchomienia CTest).
- Generator sprawdzono dla 48 kombinacji kierunku, układu i trybu, również
  z obróconym narzędziem. Zweryfikowano hamowanie, limity, anulowanie i lookahead
  niewykraczający poza impuls.
- Test integracyjny w domenie ROS 194 wykonał START, podtrzymania, ruch i STOP
  z pomiarowym potwierdzeniem postoju na emulowanym transporcie. Pięć kolejnych
  powtórzeń przeszło; obca generacja jest odrzucana przed zapisem do skrzynki.
- Przeszły 72 testy Python mostu/konfiguracji i usług ROS na atrapach oraz
  141 testów jednostkowych frontendu.
- Przeszły `tcp-live.browser.cjs` i regresja `tcp.browser.cjs`: LIVE i symulacja,
  układy odniesienia, przeliczenia SI, impulsy, podtrzymanie, blokady oraz zdarzenia
  zatrzymania. Przeglądarka korzystała wyłącznie z atrap HTTP/WebSocket.
- Nowe testy sprawdzają wspólne wysłanie końcowego F3, potwierdzenia ENABLE,
  pominięcie brakujących osi oraz brak dalszego załączania po błędzie/STOP.
  Test ROS potwierdza aktualizację czterech limitów z aktywnej konfiguracji.
- Chromium: `tcp-settings.browser.cjs`, `configuration.browser.cjs` oraz
  `configuration-snapshots.browser.cjs` przeszły na nowym frontendzie.
  Nie wysyłano rzeczywistych komend ENABLE ani ruchu. Odbiór sprzętowy pozostaje otwarty.

## Szczegółowe powody zatrzymania w Centrum komunikatów

Komunikaty napędów zawierają numer przegubu i CAN ID, np. `J3 (CAN ID 3)`.
Przy utracie świeżości podają rodzaj pomiaru, jego wiek i wymagany limit;
ENABLE=0, stall, błędy F5 oraz błędy uzbrajania są rozróżniane.
Przekroczenia uchybu i limitów zawierają wartości odczytane oraz próg.
Błędy wspólne, takie jak utrata KEEPALIVE lub przekroczenie budżetu generatora,
są opisane jako dotyczące całego ruchu, bez przypisywania ich dowolnej osi.

Pierwotny powód właściciela CAN jest przekazywany do stanu ręcznego oraz TCP.
Po awarii nie zastępuje go wcześniejszy tekst o potwierdzeniu postoju.
Centrum komunikatów pokazuje powód bezpośrednio przy błędzie i w jego historii;
przy awarii tytuł wymienia osie wskazane w otrzymanym komunikacie.
Zmiana opisu nie zmienia progów zatrzymania ani nie potwierdza postoju robota.

Wydanie z 26.10.2026: frontend wdrożony do aktywnej nakładki; wymagane Ctrl+F5.
Nowe binaria `mks_servo_can` i `robot_can_diagnostics` w
`install/tcp-jog-fix-ros` wymagają restartu usługi przy potwierdzonym postoju:

```bash
sudo systemctl restart robot-web-status.service
```

Na żądanie operatora nie wykonywano testów ani operacji na sprzęcie.

## ENABLE=0 po awarii krytycznej

W **Konfiguracja → Ustawienia systemowe → Wykonanie i zabezpieczenia** dostępny
jest przełącznik „Wyłącz ENABLE wszystkich silników przy awarii krytycznej”.
Domyślnie jest włączony, również dla starszych snapshotów bez tego pola.
Opcja jest zapisywana jako boolean `safety.disable_enable_on_critical_fault`;
obowiązuje po „Zapisz ustawienia do ROS”, z użyciem transakcji profilu robota.
Ten przycisk zapisuje tylko opcje systemowe i zachowuje niezastosowany szkic
geometrii. Rollback przywraca poprzednią regułę. Zmiana samej reguły nie
zmienia kalibracji, przełożeń ani geometrii osi.

Awaria właściciela CAN lub generatora ruchu przerywa bieżące sterowanie,
anuluje oczekujące operacje i wysyła kolejno do CAN ID 1–6 natywne `F3 ENABLE=0`.
Przy aktywnym streamingu najpierw czyści cele F7/F6(0), a ENABLE=0 jest ostatnim
poleceniem dla każdej osi.
Nie czeka na gotowość, pozycję, kalibrację, ACK ani odstęp 200 ms zwykłego
zbiorczego ENABLE. Błąd wysłania do jednej osi nie blokuje prób wysyłki do
pozostałych. Rozsyłanie odbywa się raz przy zatrzaśnięciu awarii przez istniejącego
właściciela SocketCAN. Dotyczy także awarii w trakcie załączania napędów,
JOG osi, TCP i programu PTP/LIN; nie wymaga aktywnego ruchu.

Zwykłe puszczenie przycisku, żądanie STOP operatora, anulowanie programu
oraz planowa dezaktywacja kontrolera nie są same w sobie awarią krytyczną.
Pozostają obsługiwane przez dotychczasowe zatrzymywanie i blokady. Awaryjne
przerwanie TCP przez panel/most, np. po utracie nadzoru, uruchamia regułę. Krytyczne
błędy pomiarów, watchdogów, napędu, transportu lub generatora uruchamiają regułę.
Wyłączenie przełącznika usuwa tylko automatyczne `ENABLE=0`; kontrola błędów,
STOP i zatrzaśnięcie awarii nadal obowiązują.

Centrum komunikatów zachowuje pierwotną przyczynę oraz podaje listę osi, do których
wysłano wyłączenie, i konkretne błędy wysyłki. **Wysłanie nie jest potwierdzeniem
wyłączenia napędu ani postoju.** Uszkodzona lub niedostępna magistrala może
uniemożliwić doręczenie komend. Przyczyna w tytule błędu wskazuje osie pierwotnej
awarii, a nie wszystkie osie objęte rozsyłaniem wyłączenia.

Weryfikacja tej zmiany: 12 testów C++ na atrapach (awaria podczas streamingu
lub ENABLE, brak pomiarów, błąd jednej wysyłki, brak nowych komend po awarii,
wyłączenie reguły i rollback), 2 testy konfiguracji Python, 7 testów modułu
frontendu oraz lokalna serializacja true/false przez nowy typ ROS. Bez prób
fizycznych, resetowania CAN i przełączania rzeczywistego ENABLE.

Build Release ukończony dla `mks_servo_can`, `robot_control_interfaces`,
`robot_can_diagnostics`, `robot_configuration` oraz wymagających zgodności
`robot_hardware` i `robot_motion_planner`, w `install/tcp-jog-fix-ros`.
Build frontendu ukończony; pliki wdrożone do aktywnej nakładki.

Zmiana interfejsu ROS wymaga restartu całej usługi przy potwierdzonym postoju,
a następnie odświeżenia panelu Ctrl+F5:

```bash
sudo systemctl restart robot-web-status.service
```

## Automatyczne wyłączenie ENABLE po 60 minutach bezczynności

Właściciel CAN monitoruje pozycje **wałów silników J1–J6** z enkoderów.
Jeśli przez pełne 60 minut zakres położeń żadnego wału nie przekroczy 2°,
uruchamia istniejącą sekwencję „Wyłącz wszystkie”. Sprawdza aktualny stabilny
postój, wyłącza napędy kolejno z odstępem co najmniej 200 ms i wymaga potwierdzeń
ENABLE=0. Jeżeli chwilowo brak warunków bezpiecznego rozpoczęcia sekwencji,
wyłączenie jest odroczone; samo upłynięcie czasu nie potwierdza wyłączenia.

Próg dotyczy kąta silnika, bez przeliczania przez przełożenie przegubu.
Monitor porównuje minimum i maksimum pomiarów każdego wału w bieżącym okresie;
dzięki temu wykrywa również przemieszczenie tam i z powrotem. Ruch ponad 2°
dowolnego silnika rozpoczyna nowy godzinny okres dla całego robota. Drobne
zmiany enkoderów nie są sumowane jako przebyta droga.

Odliczanie działa przy nieaktywnym sterowaniu i komplecie świeżych pozycji
sześciu napędów oraz potwierdzonych stanach ENABLE. Aktywny JOG/program,
operacja napędów, skan lub zapis konfiguracji zerują odliczanie. Brak odczytów,
przerwa obserwacji ponad 250 ms, cofnięcie zegara albo zmiana zestawu załączonych
silników również rozpoczynają nowy okres. Nie zlicza się czasu bez nadzoru CAN
ani czasu sprzed restartu. Monitor nie dodaje zapytań do magistrali i nie zmienia
watchdogów ruchu.

Powód „Automatyczne wyłączenie ENABLE: 60 min bez ruchu wałów > 2°” trafia do
stanu ręcznego i historii Centrum komunikatów, razem z przebiegiem/potwierdzeniem
sekwencji. Zakończenie bezczynności nie włącza napędów automatycznie — wymagane
jest świadome „Załącz” lub „Załącz wszystkie”. Przygotowany program zostaje
unieważniony i wymaga ponownego przygotowania. Reguła oszczędzania energii jest
niezależna od przełącznika wyłączania ENABLE po awarii krytycznej.

Weryfikacja: sześć nowych testów monitora (w tym pełna godzina na zegarze
wirtualnym i potwierdzone wyłączenie wszystkich osi na atrapach), pięć regresji
zbiorczego ENABLE i dwie regresje obsługi awarii — **13/13 OK**. Bez komend do
sprzętu i bez kolejnej próby fizycznej. Zbudowano tylko zmieniony pakiet
`mks_servo_can` w Release, do `install/tcp-jog-fix-ros`; nie zmieniano interfejsów
ROS ani frontendu.

Nowy build wymaga restartu usługi przy potwierdzonym postoju:

```bash
sudo systemctl restart robot-web-status.service
```
