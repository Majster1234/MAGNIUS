# Symulacja pojedynczego ruchu PTP/LIN i API kinematyki

Panel oblicza pojedynczy ruch i odtwarza go na dodatkowym bliźniaku. Każdy wynik
podglądu ma `execution_ready=false`, także przy poprawnych ramkach MKS. Fizyczny
PTP ma odrębny interfejs opisany w [motion_execution.md](motion_execution.md). Planner nie otwiera SocketCAN ani nie wysyła ENABLE, zero lub JOG.
Model demonstracyjny pozwala ocenić obliczenia, nie potwierdza geometrii robota.

## Obsługa panelu

1. W **Sterowanie ręczne** włącz **Planowanie**. Niebieski bliźniak (30%) wskazuje
   cel. JOG J1–J6 i krzyżaki **TCP** zmieniają wyłącznie jego pozycję. TCP korzysta
   z FK/IK i wspólnego celu przez `PlanningJog.preview()`; obsługuje układ bazy
   oraz narzędzia. Szczegóły: [sterowanie ręczne](manual_control.md#tcp-w-planowaniu).
2. Wybierz **PTP** lub **LIN**. Kliknięcie zapisuje aktualny start, oblicza ruch
   i uruchamia pomarańczowego bliźniaka (55%). Nie ma listy celów ani przycisku obliczania.
3. **PTP** synchronizuje ruch przegubów; TCP może zataczać łuk. **LIN** prowadzi TCP
   po prostej z interpolacją orientacji. Oba dochodzą do wskazanego ustawienia robota.
4. Na celu animacja czeka 500 ms, wraca skokowo na start i ponownie odtwarza tę samą
   trajektorię. Każda pętla nie uruchamia nowych obliczeń. Narastający ślad TCP
   pozostaje widoczny po cyklu i zatrzymaniu; powrót na start nie tworzy linii.
5. Kolejny JOG lub TCP zatrzymuje podgląd. Po puszczeniu kierunku system przelicza ruch do
   nowego celu, zachowując typ i zapisany start. Nowe zadanie czeka na anulowanie starego.
6. **Zatrzymaj podgląd**, utrata fokusu, ukrycie karty lub utrata backendu zatrzymują
   obliczenia/animację. Kliknięcie PTP/LIN ponownie pobiera start i uruchamia podgląd.

Rzeczywisty model nadal śledzi enkodery; nowe pomiary nie zmieniają zapisanego startu
ani celu. Zmiana modelu, rewizji lub wspólnego ustawienia symulacji unieważnia podgląd.
Osie TCP i sygnalizację osobliwości modeli opisuje [podgląd 3D](robot_preview.md#osie-tcp-ślad-ruchu-i-osobliwości).
Zmiana dostępności napędów nie blokuje animacji sześciu osi. Wyjście z ekranu usuwa
bliźniaki. Backend trzyma w pamięci 4 plany do 8 MiB każdy oraz 8 zadań, bez zapisu
programów na dysk. Nie ma omijania przeszkód, symulacji dynamiki ani odbioru fizycznego.

## Brak sterowników i pozycja bazowa

**Konfiguracja → Ustawienia systemowe → Ignoruj brak sterowników — tylko planowanie**
odblokowuje JOG sześciu osi i wejście w planowanie bez napędów. Domyślnie jest wyłączony.
Po wyłączeniu ręczny JOG celu znów wymaga świeżych pomiarów. Bliźniak animacji zawsze
symuluje wszystkie osie. Stan rzeczywistych napędów oraz fizyczne sterowanie nie zmieniają się.
TCP podlega tej samej opcji; przy jej wyłączeniu wymaga sześciu aktualnych pozycji
do obliczenia pełnej pozy TCP. Nie zamraża brakujących osi ani nie dopisuje im zera.

Suwak jest wspólny dla klientów (odczyt co sekundę), zapisywany atomowo przez
`robot_configuration` w `system_settings.json`, w katalogu `configuration_dir`.
Nie jest częścią snapshotu robota ani transakcji zastosowania parametrów do sprzętu.
Interfejs potwierdza zmianę dopiero po zapisie; przy braku odczytu blokuje suwak.

Start korzysta z aktualnego poprawnego pomiaru, potem ostatniego poprawnego pomiaru,
a następnie 0° wyłącznie do symulacji. Cache sześciu pozycji jest w pamięci backendu,
osobno dla modelu i rewizji, i kasowany przy ich zmianie. Edytor **Start symulacji**
pokazuje pochodzenie wartości i umożliwia korektę. Gdy 0° jest poza limitem, trzeba je
poprawić przed wyborem ruchu. Wartość symulowana nie jest odczytem enkodera.
W **Prędkość podglądu** ustawienia zależą od wybranego ruchu:

- **PTP:** prędkość złączy w °/s i przyspieszenie w °/s²; domyślnie 10 i 50.
- **LIN:** prędkość TCP w mm/s, przyspieszenie TCP w mm/s² i limit obrotu TCP
  w °/s; domyślnie 50, 100 i 10. Limit obrotu obowiązuje także przy zmianie samej
  orientacji bez przesuwu. Zmiana pola automatycznie przelicza podgląd ze stałego startu.

W LIN prędkości złączy wynikają z posuwu TCP oraz profili osi; panel nie narzuca
im wspólnego limitu prędkości PTP. Prędkość TCP jest górnym ograniczeniem:
rozpędzanie, hamowanie, profile osi i osobliwości mogą wymagać wolniejszego ruchu.
W pobliżu osobliwości nadgarstka planner dodatkowo ogranicza **J4 i J6 do maks. 5°/s**.
Niższa prędkość zadana lub profil osi nadal obowiązują. To limit przegubów, a nie
prędkości obrotowej ani liniowej TCP. Pozostałe osie zachowują swoje limity;
synchronizacja może wymagać ich zwolnienia razem z nadgarstkiem.
JOG zachowuje jednostki °/s przegubu lub RPM wału i demonstracyjne 1:1 / ±3600° bez profilu.

## Obliczenia C++

`robot_kinematics` używa KDL/Eigen do FK, Jacobianu i ograniczonego numerycznego IK
z tłumioną pseudoodwrotnością. Łańcuch od korzenia URDF do linku za stałym `tool_fixed`
musi zawierać kolejno obrotowe `joint_1`–`joint_6`, bez `mimic`. Solver sprawdza
translację i pełną orientację kwaternionową. Niedostępne osie starego API można zamrozić.

`robot_motion_planner` łączy kąty w PTP bez zawijania obrotów. W LIN wyznacza cel
przez FK docelowego bliźniaka, interpoluje XYZ liniowo i orientację przez SLERP,
rozwiązując kolejne IK od poprzedniego rozwiązania. Rozdzielczość wejścia to najwyżej
1 mm / 0,01 rad; skok przegubu ponad 0,25 rad jest odrzucany.

Wspólna parametryzacja TOTG z `moveit_core` nie uruchamia `move_group`. Pojedyncze
PTP/LIN nie korzystają z zaokrąglania narożników. LIN odrzucamy, gdy końcowe rozwiązanie
różni się od docelowego o ponad 0,1° na oś, przekracza 0,1 mm / 0,1° TCP albo nie ma
ciągłego rozwiązania. Nie ma automatycznej zamiany LIN na PTP. Dotyczy to także
niejednoznaczności konfiguracji przy osobliwości.

Wynik zawiera sześć q/v/a w SI i czas od startu. Walidujemy interpolację piątego
stopnia JTC: szczyty pochodnych co 1 ms, wydłużenie czasu z zapasem 5%, zaokrąglenie
do 20 ms, następnie q/v/a i FK co 5 ms. Jest to kontrola numeryczna podglądu, nie
dowód ciągły ani odbiór sterowania. Limity: 2000 punktów IK, 200 s / 10000 punktów wyniku.

### Zwalnianie nadgarstka w symulacji

Po TOTG i kontroli pochodnych wyznaczamy osobliwość z części kątowej Jacobianu
J4–J6 aktualnego URDF. Stosunek najmniejszej do największej wartości osobliwej
jest bezwymiarowy: próg bliski wynosi `tan(2,5°)`, daleki `tan(15°)`. Dla
prostopadłego nadgarstka odpowiada to odchyleniu J5 od osobliwości o 5° i 30°;
są to progi algorytmu podglądu, nie potwierdzone parametry fizycznego robota.
W strefie bliskiej limit J4/J6 wynosi 5°/s, w przejściowej rośnie wielomianem
o ciągłej pierwszej i drugiej pochodnej, a w dalekiej wraca do wartości zadanej.

Lokalna zmiana zegara trajektorii uwzględnia prędkości i przyspieszenia wszystkich
osi: hamowanie zaczyna się przed wejściem do wolnej strefy, a ponowne rozpędzanie
nie przekracza limitów. Nie zwiększamy prędkości wolniejszych poleceń i nie
spowalniamy ruchu innych osi, gdy nadgarstek pozostaje nieruchomy. Po zmianie czasu
ponownie sprawdzamy interpolację JTC, lokalne limity J4/J6, FK i tolerancje TCP.
Podgląd MKS jest wyliczany dopiero z końcowej trajektorii. Nie zmienia się
sterowanie fizycznym JOG, ENABLE ani wykonanie sprzętowe (`execution_ready=false`).

Zwolnienie nie naprawia nieciągłej lub nieosiągalnej drogi LIN. Adaptacyjne
zagęszczanie próbek, wybór konfiguracji J4/J6 i dodatkowe wygładzanie TOTG pozostają
osobnymi usprawnieniami; opisany profil czasu nie znosi istniejących kontroli IK.

### Posuw TCP dla LIN

Nowy klient LIN zadaje normę prędkości przesuwu TCP oraz jej przyspieszenia,
a także normę prędkości kątowej narzędzia. Planner oblicza je z Jacobianu:
`v = Jv·qdot`, `omega = Jw·qdot`, `a = Jv·qddot + Jdot_v·qdot`.
Pochodną Jacobianu wyznacza numerycznie. Wspólny profil czasu uwzględnia te
ograniczenia i zwalnianie nadgarstka, z hamowaniem przed wolniejszym fragmentem.
Po zmianie czasu kontrolujemy ponownie prędkości i przyspieszenie TCP,
interpolację JTC, limity osi oraz dokładność LIN. Zachowujemy sześć pozycji
docelowych, pełną orientację i `execution_ready=false`.

## ROS i HTTP

`POST /api/v1/planning/tcp-step` przyjmuje kontekst i seed jak `tcp-jog`,
`axis`, `direction`, `frame`, `velocity` oraz `step` z listy `[0.1,1,5,10,20]`.
Wartość `step` oznacza mm dla XYZ lub stopnie dla Rx/Ry/Rz. FK i dokładne IK
wyznaczają cel; istniejący planer LIN sprawdza drogę, ciągłość, limity q/v/a
i prędkość TCP. Odpowiedź 202 zawiera zadanie obsługiwane przez istniejące
`jobs`, `plans` i anulowanie. Ten interfejs podglądu nie wykonuje sprzętu; fizyczny PTP ma odrębny wykonawca.
`GET context` sygnalizuje dostępność przez `tcp_step_available`.
Klient pobiera jedną trajektorię, odtwarza ją raz w 60 Hz bez kolejnych żądań IK
w czasie animacji i nie przyjmuje następnego impulsu do zakończenia bieżącego.

Przy czystym obrocie kontrola drogi wyznacza postęp z orientacji, gdy resztkowa
translacja FK/IK mieści się w tolerancji pozycji. Unika to fałszywego odrzucenia
obrotu na podstawie projekcji na mikrometrowy wektor; tolerancje TCP nie są zwiększane.

`POST /api/v1/planning/tcp-jog` oblicza pojedynczy przyrost celu; nie wykonuje ruchu.
Żądanie zawiera `model_id`, `configuration_revision`, `settings_revision`,
`seed_degrees[6]`, `simulated_start[6]`, `frame` (`base`/`tool`), `axis`
(`x/y/z/rx/ry/rz`), `direction` (−1/+1), `velocity` (mm/s lub °/s zgodnie z osią)
i `duration_s` (0,01–0,1 s). Przyrost odpowiada prędkości i czasowi kroku;
prędkość symulacji mieści się w 0,01–100 mm/s albo 0,01–30°/s.
Klient wysyła najwyżej jeden krok naraz, zwykle dla 50 ms. Po wolniejszej odpowiedzi
uwzględnia czas od poprzedniego wysłania, najwyżej 100 ms, bez kolejki zaległych ruchów.
Zlecenie 100 mm/s daje więc 5 mm na 50 ms zamiast poprzedniego ograniczenia do 1 mm,
które przy 20 Hz ograniczało posuw do 20 mm/s. FK wykonywane jest raz na żądanie;
IK i kontrola limitów nadal dzielą przyrost na odcinki najwyżej 1 mm lub 1°.
Cały przyrost oraz każdy odcinek muszą spełniać limity pozycji, prędkości i ciągłości.
Odczyty ustawień i konfiguracji są równoległe, przed i po obliczeniach; nie ma cache
osłabiającego kontrolę rewizji. `GET context` zwraca `tcp_jog_segmented=true`.
Przed przeładowaniem starszego backendu klient zachowuje dotychczasowy mały krok.

Backend używa istniejącego `SolveIK` do FK oraz IK z poprzednich kątów.
Przesunięcie narzędzia jest obracane do bazy, a obroty składane kwaternionowo
(z lewej dla bazy, z prawej dla narzędzia). Kontroluje rewizje, zasady brakujących
napędów, limity q i prędkości oraz ciągłość do 0,25 rad na krok. Błąd nie zmienia celu.
Odpowiedź zawiera `positions_degrees[6]`, `tcp` w mm/°, trzy identyfikatory kontekstu,
`simulation_only=true` i `execution_ready=false`. Ten endpoint nadal służy tylko podglądowi.

| Nazwa ROS | Typ / zastosowanie |
| --- | --- |
| `/robot/planning/solve_ik` | `SolveIK`; także FK przez `forward_only` |
| `/robot/planning/compute_preview_motion` | `ComputePreviewMotion`; kontekst, PTP/LIN, sześć docelowych kątów, postęp/anulowanie |
| `/robot/planning/compute_cartesian_path` | Zachowane `ComputeCartesianPath`; do 32 celów TCP, tolerancje zaokrągleń i skale czasu |
| `/robot/preview/robot_description` | URDF, transient local |
| `/robot/configuration/active` | `RobotConfiguration`, transient local, co 500 ms |
| `/robot/manual/state` | Pozycje i dostępność sprzętu dla dotychczasowego API |

`PlanningContext` zawiera model SHA-256/URDF, rewizję, seed, maski known/locked,
limity, profile oraz `simulation_only` i `simulated_start`. Podgląd PTP/LIN odblokowuje
sześć osi. Wymaga zgodnego modelu i aktywnej konfiguracji odebranej w ostatnich 1,5 s,
nie wymaga pomiarów CAN. Dotychczasowy tryb zachowuje progi 500 ms stanu i 250 ms pozycji.
Zmiana modelu/rewizji przerywa obliczenia; powrót danych nie wznawia ich automatycznie.

| HTTP `/api/v1/planning/…` | Zastosowanie |
| --- | --- |
| `GET context` | Model/rewizje, profile, pozycje bazowe z pochodzeniem, dostępność planera i ustawienie symulacji |
| `POST moves` | Podgląd PTP/LIN, sześć kątów celu/startu, oznaczenia symulacji i ustawienia prędkości opisane poniżej |
| `GET jobs/{id}`, `DELETE jobs/{id}` | Postęp i anulowanie; DELETE czeka na zakończenie zadania |
| `GET plans/{id}` | Wynik po kontroli modelu, konfiguracji i rewizji ustawień |
| `POST ik`, `POST jobs` | Zachowane FK/IK i wielopunktowe API `targets[][6]` |

POST wymaga `model_id`, `configuration_revision`, `settings_revision`,
`motion_type`, `target_degrees[6]`, `seed_degrees[6]` i `simulated_start[6]`.
PTP przyjmuje `preview_velocity_degrees_s` i `preview_acceleration_degrees_s2`.
Nowy LIN przyjmuje `tcp_velocity_mm_s`, opcjonalnie `tcp_acceleration_mm_s2`
(domyślnie 100) i `tcp_angular_velocity_degrees_s` (domyślnie 10). Skonfigurowane
osie używają limitów aktywnego profilu; osie bez profilu używają wyłącznie do
symulacji 180°/s i 100°/s². Dla starych klientów LIN bez `tcp_velocity_mm_s`
zachowano dotychczasowe parametry prędkości złączy. Akcja `ComputePreviewMotion`
przekazuje nowe limity jako `tcp_velocity`, `tcp_acceleration`,
`tcp_angular_velocity` w SI; trzy zera oznaczają dotychczasowy profil.
JSON mieści się w 64 KiB. Pozycje osi w HTTP
podaje się w stopniach; cele TCP starego API to XYZ w mm i RPY w ° (`Rz Ry Rx`).
W ROS obowiązują SI i kwaterniony. Zmiana wspólnego ustawienia:
`GET/PUT /api/v1/configuration/system`, pole `ignore_missing_drivers`, przez operacje
`system_get/system_set` istniejącej usługi `ConfigurationCommand`.

## Podgląd MKS i przyszłe wykonanie

`mks_servo_can::compile_motor_sample` przelicza q przez przełożenie, kierunek i offset
na 16384 jednostki enkodera/obrót wału, sprawdza zakres F5 ±8388607 i błąd kwantyzacji.
RPM wynika z pochodnej i przyrostu pozycji w cyklu, w zakresie profilu oraz 1–3000.
Ruch poniżej 1 RPM lub zmiana kierunku wewnątrz próbki może odrzucić podgląd MKS,
zachowując trajektorię symulacji. Utrzymanie celu używa RPM=1, nie `speed=0` oznaczającego stop.

Sześć kodów `mks_acceleration_codes` wymaga potwierdzenia 1–255; przykład ma `-1`.
Kod producenta nie jest rad/s². Bez pełnego profilu lub kodów `mks_compatible=false`.
Podgląd zawiera co 20 ms pozycje, RPM, acc, operacje i 48 bajtów F5 z CRC.
F5 nie zadaje czasu dojazdu, ACK nie oznacza osiągnięcia celu. Przygotowanie sześciu
komend i komplet ACK muszą poprzedzać SYNC, także pierwszy w chwili 0.
Nie zakładamy kolejki przyszłych odcinków w napędzie.

Istniejący manager odrzuca przeterminowane zestawy i wymaga kompletu ACK.
`CanSystem`/przykład JTC obsługują position+velocity, starszy interfejs position nadal
szacuje prędkość z różnic pozycji. Bramki `FollowJointTrajectory`, aktywacja, lease i stop
pozostają obowiązujące. Przyszły executor wymaga osobnego wdrożenia: zatwierdzenia
planu po id, zgodności startu/modelu/rewizji, sześciu odebranych osi, nadzoru terminów
i błędu śledzenia oraz zatrzymania bez automatycznego wznowienia.

## Uruchamianie i testy

Zależności deklarują manifesty: KDL, Eigen, OpenSSL, `moveit_core` i SRDF.
Po `rosdep` zbuduj frontend i workspace według [instrukcji panelu](web_status.md).
Launch domyślnie uruchamia planner (`planning_enabled`, `planning_parameters`).
Starsza działająca sesja wymaga restartu dla nowych tras, akcji i publikacji konfiguracji.

```bash
cd web
npm run build
npm test
npm run test:browser
cd ../ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Testy bez sprzętu obejmują FK/IK, synchronizację PTP, prostoliniowość LIN, końcową
konfigurację, limity, anulowanie i zgodność HTTP→C++. Chromium sprawdza trzy modele,
stały start/cel, automatyczne przeliczanie, pętlę, opóźnione wyniki oraz globalny suwak
dwóch klientów. Osobne testy CAN używają emulatora sześciu napędów.

## Fizyczny PTP i niepełny zestaw napędów

Fizyczny PTP ma odrębne przygotowanie, lokalny wykonawca C++ i jawne zatwierdzenie.
Ustawienie `allow_missing_motion` dopuszcza pominięcie nieobecnych napędów;
nie potwierdza osiągnięcia TCP i nie zmienia ograniczeń osi uczestniczących.
Obowiązujące zasady wykonania i odbioru: [motion_execution.md](motion_execution.md).
Wcześniejsze opisy podglądu PTP/LIN nadal dotyczą symulacji; fizyczny LIN pozostaje zablokowany.
