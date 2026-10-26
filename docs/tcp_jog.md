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
