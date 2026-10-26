# Raport diagnostyki JOG TCP i zbiorczego ENABLE — 26.10.2026

## Zakres i warunki

Operator zgłosił rozsprzęgnięte wały i załączone silniki. Przed pierwszą próbą
API potwierdziło sześć dostępnych, skalibrowanych osi, ENABLE, świeże pomiary,
0 RPM i brak aktywnego JOG/programu. Aktywna nakładka: `install/tcp-settings-ros`;
`tcp_jog_enabled=true`, stan TCP `ready`. Aktywna rewizja konfiguracji:
`2f1186d77ba44f838f65e261aeabd866`.

Zapisane limity TCP: 100 mm/s, 100 °/s, 100 mm/s², 100 °/s².
Próbę 1 wykonano przy 2 mm/s, kroku 10 mm w układzie Baza, z zachowaniem orientacji,
limitem czasu 10 s, podtrzymaniem co 50 ms i aktywnymi watchdogami właściciela CAN.
Operator osobno zatwierdził ten konkretny ruch +X.

Przesunięcia TCP w tym raporcie są obliczone przez FK z odczytów enkoderów wałów
oraz zatwierdzonej geometrii/przełożeń. Przy rozsprzęgniętych wałach nie stanowią
pomiaru przestrzennego rzeczywistego narzędzia ani odbioru dokładności robota.
Nie zmieniano geometrii, kalibracji, przełożeń ani limitów mechanicznych.

## Próby fizyczne (maksymalnie sześć)

| Nr | Kierunek / krok | Prędkość | Tolerancje IK m / rad | Postęp referencji | FK z pomiarów: ΔX / ΔY / ΔZ | Wynik |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Baza +X / 10 mm | 2 mm/s | 1e-9 / 1e-8 | 3,820 mm | +3,682 / −0,047 / +0,071 mm | Impuls przerwany przez anulowanie obliczeń; postój potwierdzony, CAN bez awarii |
| 2 | Baza +Y / 40 mm | 2 mm/s | 1e-8 / 1e-7 | 40,000 mm | −0,040 / +39,741 / −0,070 mm | Pełny impuls, postój potwierdzony, CAN bez awarii |
| 3 | Baza +Z / 10 mm | 2 mm/s | 1e-7 / 1e-6 | 10,000 mm | −0,036 / −0,227 / +9,897 mm | Pełny impuls, postój potwierdzony, CAN bez awarii |
| 4 | Baza +X / 10 mm | 2 mm/s | 1e-6 / 1e-5 | 10,000 mm | +9,972 / −0,289 / −0,375 mm | Pełny impuls, postój potwierdzony, CAN bez awarii |

Zmiana orientacji wyznaczona z FK w próbie 1: 0,0829°.
Zmiany odczytów przegubów J1–J6: −0,01099°, +0,85034°, −0,87231°,
+0,01099°, +0,03516°, −0,09668°.
Po zakończeniu generatora spóźnione KEEPALIVE otrzymywały odpowiedź
„Brak pasującego wykonania JOG TCP”; klient nie ponawiał START. Te odpowiedzi
po zakończeniu wykonania nie oznaczają przerwania impulsów 2–4.

Próbę 2 operator zatwierdził oddzielnie po zmianie zakresu z 10 na 40 mm,
łącznie z wydłużeniem maksymalnego czasu z 10 do 25 s. Czas od START do
statusu potwierdzającego postój: 20,314 s. Niedojazd wyznaczony z FK w Y:
0,259 mm; zmiana orientacji 0,1164°. Wysłanie pełnej referencji 40 mm nie oznacza
idealnego wykonania tej drogi przez napędy. Przy zakończeniu obowiązuje dotychczasowa
tolerancja przegubów i potwierdzenie 0 RPM; nie zwiększano jej w tej próbie.
Zmiany zmierzonych przegubów J1–J6: +7,56958°, +0,60645°, −0,60864°,
−9,99316°, +0,70532°, +12,41455°.

![Przebieg próby +Y: referencja i FK z enkoderów](tcp_jog_y_2026-10-26.svg)

Próby 3 i 4 operator zatwierdził osobno przed ich wykonaniem. Obie odbyły się
przy 2 mm/s, 100 mm/s² i limicie 10 s, z potwierdzeniem postoju pomiędzy ruchami.

| Próba po poprawce | Czas do potwierdzenia postoju | Zmiana orientacji FK | Niedojazd w kierunku ruchu | Końcowe odchylenie poprzeczne (norma) | Maks. odchylenie poprzeczne z próbek FK |
| --- | --- | --- | --- | --- | --- |
| 2: +Y 40 mm | 20,314 s | 0,1164° | 0,259 mm | 0,080 mm | 0,460 mm |
| 3: +Z 10 mm | 5,237 s | 0,1180° | 0,103 mm | 0,230 mm | 0,329 mm |
| 4: +X 10 mm | 5,214 s | 0,1409° | 0,028 mm | 0,474 mm | 1,081 mm |

![Porównanie trzech udanych impulsów: referencje i FK](tcp_jog_xyz_2026-10-26.svg)

Ukończenie impulsów i potwierdzenie postoju nie są potwierdzeniem idealnej
dokładności kartezjańskiej: w ostatniej próbie końcowa norma odchylenia
poprzecznego wyniosła 0,474 mm, a maksymalna norma z próbek w trakcie ruchu
1,081 mm. Zwiększano wyłącznie dopuszczalny błąd
**zbieżności IK**, a nie tolerancję śledzenia napędów. Końcowa tolerancja
przegubów pozostała dotychczasowa (±0,5°); dopuszcza błędy widoczne w FK.
Próbkowanie enkoderów odbywa się asynchronicznie. Ocena przestrzennej dokładności
TCP wymaga odrębnego pomiaru narzędzia przy sprzęgniętym mechanizmie.

Pełne rejestracje prób 1–4, łącznie z komendami WebSocket, stanami, modelem URDF
oraz pomiarami przed/po, znajdują się w `/tmp/tcp-jog-2026-10-26/iteration-N.json`.
Trwały zapis pomiarów przed/po, przejść stanów i wysłanych komend:
[dane prób JSON](tcp_jog_test_data_2026-10-26.json).

## Ustalenia i poprawki

- Poprzedni generator pracował z tolerancją IK 1 nm / 1e-8 rad i budżetem cyklu
  7 ms. W przebadanym buildzie `CMAKE_BUILD_TYPE` było puste (brak optymalizacji
  Release). Komunikat próby 1 „Obliczenia anulowane lub model zmieniony” pochodzi
  ze sprawdzenia przerwania solvera. Rewizja/model pozostawały stałe.
- Dodano jawne parametry `tcp_jog_ik_position_tolerance` (m) i
  `tcp_jog_ik_orientation_tolerance` (rad), zmieniane tylko na postoju.
  Zakresy są ograniczone do odpowiednio (0, 0,1 mm] oraz (0, 0,001 rad].
  Domyślne wartości zachowują dotychczasową dokładność. Komunikat przekroczenia
  budżetu cyklu wskazuje teraz wprost przyczynę.
- Gotowy build Release obejmuje zmienione pakiety `mks_servo_can`,
  `robot_kinematics` i `robot_can_diagnostics` w `install/tcp-jog-fix-ros`.
- Zbiorcze ENABLE zachowuje jednego właściciela SocketCAN i przygotowanie napędów.
  Końcowe F3 są wysyłane kolejno J1→J6, co najmniej 200 ms od siebie. Kolejny napęd
  wymaga świeżego potwierdzenia 3A poprzedniego. Błąd, brak potwierdzenia, utrata
  pozycji lub STOP przerywa sekwencję. Zbiorcze wyłączanie używa tej samej kolejności.
  Nie wykonywano fizycznego przełączania ENABLE w tej sesji.

## Weryfikacja bez sprzętu

- Trzy zmienione pakiety zbudowano poprawnie. `colcon test-result`:
  **176 wyników, 0 błędów, 0 porażek, 0 pominięć** (łącznie z uruchomieniami CTest).
- Nowe testy ENABLE potwierdzają kolejność, odstępy ≥200 ms, pomijanie brakującej
  osi oraz brak kolejnego załączenia po STOP albo utracie potwierdzenia.
- Testy generatora sprawdzają 10 mm w trzech kierunkach dla różnych tolerancji,
  zachowanie pozy/orientacji i limitów dynamiki przegubów. Test ROS sprawdza
  zmianę tolerancji na postoju oraz odrzucenie wartości poza zakresem.
- Osobne odtworzenie offline użyło rzeczywistego modelu i pozycji startowej
  z próby 1. Dla +X/+Y/+Z i tolerancji pozycji 1e-9, 1e-8, 1e-7 oraz 1e-6 m
  (orientacja = 10× wartość liczbowa w rad) ukończono wszystkie **12 impulsów
  po 10 mm** w 502 próbkach. Budżet każdej próbki pozostawał 7 ms.
  Maksymalny błąd IK pozycji w tych odtworzeniach wyniósł 1,290e-9 m,
  orientacji 4,802e-9 rad. Nie jest to pomiar dokładności napędów/TCP.

## Stan końcowy

Ponowne odczyty potwierdziły postój sześciu osi przed próbą aktywacji. Pierwsza
instalacja drop-in zakończyła się `sudo: a password is required`. Następnie
operator wykonał restart; odczyt systemd potwierdził aktywną nakładkę
`install/tcp-jog-fix-ros`, PID usługi 9419. API potwierdziło gotowość TCP,
ENABLE sześciu osi i 0 RPM, bez aktywnego programu ani JOG.

Operator zmienił zakres próby 2 (+Y/Baza) z 10 na **40 mm**. Przy zachowanej
prędkości 2 mm/s wymaga to około 20 s, więc poproszono o potwierdzenie limitu
25 s zamiast wcześniejszych 10 s. Na postoju ustawiono i odczytano parametry IK:
1e-8 m / 1e-7 rad (poprzednio 1e-9 m / 1e-8 rad). Nie zmieniono innych limitów.
Odtworzenie offline z aktualnej pozycji ukończyło pełne +Y 40 mm w 2002 próbkach;
maksymalne błędy IK wyniosły 1,339e-9 m i 1,234e-8 rad.
Operator potwierdził limit 25 s i próbę 2 wykonano zgodnie z tym opisem.
Następnie po osobnych zgodach wykonano próby 3 (+Z) i 4 (+X), zwiększając
tolerancje IK zgodnie z opisami. Wszystkie trzy kierunki po poprawce ukończyły
pełne referencje i otrzymały potwierdzenie postoju, bez awarii CAN.
Łącznie wykonano **cztery próby sprzętowe z limitu sześciu**; prób 5–6 nie
uruchamiano. Nie zmieniano watchdogów, kalibracji, geometrii ani limitów
mechanicznych i nie wykonywano fizycznego przełączania ENABLE.

W ROS pozostawiono ostatnio sprawdzone tolerancje IK **1e-6 m / 1e-5 rad**.
Zapisano je również w `config/local/execution.production.yaml`, z którego
korzysta usługa, aby przetrwały kolejny restart. Ustawienia te są już aktywne;
nie wykonywano dodatkowego restartu. Domyślne wartości biblioteki/węzła nadal
wynoszą 1e-9 m / 1e-8 rad. Poniższe komendy służą odtworzeniu wdrożenia, nie są
wymagane do aktywowania już wykonanych zmian:

```bash
sudo install -m 0644 /home/mk/Desktop/Projekty/system/application/tcp-jog-fix-ros.conf.example /etc/systemd/system/robot-web-status.service.d/tcp-settings.conf
sudo systemctl daemon-reload
sudo systemctl restart robot-web-status.service
```

## Korekta obsługi panelu operatorskiego — 26.10.2026

Po czterech opisanych wyżej próbach operator zgłosił natychmiastową awarię przy
użyciu panelu. Poprzednie próby sprzętowe używały bezpośredniego klienta WS
w trybie impulsowym; nie weryfikowały całego toru kliknięcie panelu → backend →
typowane żądanie ROS. Dalsze próby fizyczne mają korzystać wyłącznie z panelu
operatorskiego, zgodnie z nową instrukcją operatora.

W logach mostu znaleziono `AssertionError: The 'step' field must be of type 'float'`.
Przeglądarka serializuje zerowy krok trybu ciągłego jako JSON `0`; Python odczytuje
go jako `int`, a wygenerowane pole `JogTcp.Request.step` wymaga `float`.
Analogiczny problem dotyczy wartości całkowitych prędkości i kroku impulsu.
Wyjątek przerywał WebSocket; zamknięcie połączenia podczas aktywnej sesji zleca
awaryjny STOP, co tłumaczy pojawienie się awarii właściciela sterowania.

Po zachowanej walidacji typu, skończoności i zakresu danych most konwertuje
`velocity` i `step` jawnie na `float`. Nie zmieniono nadzoru ruchu ani limitów.
Komunikat „Awaria sterowania” pokazuje teraz przyczynę od właściciela bezpośrednio
w opisie widocznym dla operatora, oprócz dotychczasowych szczegółów technicznych.

Weryfikacja:

- Trzy nowe przypadki odtworzyły błąd przed poprawką: krok `0`, całkowita
  prędkość i całkowity krok impulsu. Po poprawce przechodzą START/KEEPALIVE/STOP.
- Integracja WS → `ManualRosClient` → rzeczywisty typ `JogTcp` przeszła na
  atrapach usług w domenie ROS 129. Nie korzystała z CAN.
- Zbudowano wyłącznie zmieniony `robot_web_bridge` w `install/tcp-panel-ros`;
  `colcon test-result`: 177 wyników, 0 błędów, porażek i pominięć.
- Build frontendu i 142 testy jednostkowe przeszły.
- `tcp-panel-ros.browser.cjs` otwiera rzeczywisty panel w Chromium i łączy go
  z produkcyjnym backendem WebSocket oraz rzeczywistym typem ROS `JogTcp`.
  Wykonawca jest atrapą na localhost, bez węzła ROS i bez SocketCAN. Przeszły
  kliknięcia, przytrzymanie, podtrzymanie, puszczenie, impuls, klawiatura,
  zatrzymanie po utracie fokusu, widoczna przyczyna awarii i brak wznowienia.
- Przeszła również dotychczasowa regresja `tcp-live.browser.cjs`.

Uruchomienie testu z typami ROS (wymagany Node zgodny z `web/package.json`):

```bash
source /opt/ros/humble/setup.bash
source install/tcp-panel-ros/setup.bash
cd web
npm run build -- --outDir /tmp/tcp-panel-web-dist
ROBOT_WEB_TEST_DIST=/tmp/tcp-panel-web-dist npm run test:tcp-panel-ros
```

Wdrożenie: po potwierdzeniu postoju sześciu osi zapisano nowy frontend w aktywnym
`install/tcp-settings-ros/robot_web_bridge/share/robot_web_bridge/web`.
Kod mostu w tej nakładce wskazuje przez symlink na poprawione źródło.
Odświeżono wyłącznie proces mostu. Łagodne zamknięcie zatrzymało się na otwartych
połączeniach WebSocket; zakończono zawieszony most PID 9460, a istniejący ROS launch
z `respawn` odtworzył go jako PID 17337. Właściciel CAN przez cały czas pozostał
PID 9458, a główna usługa PID 9419; nie resetowano CAN ani ENABLE.

API i odczyt aktywnego panelu w Chromium potwierdziły nowy frontend, gotowość TCP,
świeże pomiary, postój sześciu osi i brak awarii. Odczyt panelu nie wysłał żadnej
komendy ruchu ani żądania HTTP zapisującego dane. W tej korekcie nie wykonano
nowych prób fizycznych — licznik pozostaje 4/6. Poprawka jest aktywna; operator
powinien odświeżyć otwartą kartę (Ctrl+F5). Kolejny restart usługi nie jest potrzebny.
