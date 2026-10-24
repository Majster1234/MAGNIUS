# Fizyczne wykonanie PTP — streaming 100 Hz

Kontrakt źródeł z 24.10.2026 zastępuje wcześniejsze F5/ACK/SYNC i oczekiwanie
na pozycję w punktach pośrednich. Program jest lokalnym strumieniem F5 co 10 ms.
Regulację silnika realizuje MKS. Na rozsprzęglonych 1/2/4 osiach sprawdzono ruch i powrót przy zadanym limicie 1 RPM.
Osiągnięto cele końcowe ±0,5° bez błędów CAN po poprawkach. Śledzenie całej
trajektorii, płynność, acc 240 i sześć osi nadal nie mają pełnego odbioru:
[raport sprzętowy](motion_hardware_results.md).

Później 24.10.2026 użytkownik przestawił napędy na 1 Mbit/s, a host został
przełączony i zaliczył test samych odczytów J1/J2/J3/J5: 4000 pomiarów pozycji,
bez błędów CAN i timeoutów uczestników. [Raport migracji](can_1m_results.md)
opisuje również procedurę ponownego zerowania i odblokowania profilu prób
1 RPM/acc=1. Nie oznacza to nowego testu ruchu ani odbioru sześciu osi.

## Zegar i matematyka

Planner dostarcza pozycję q, prędkość v i przyspieszenie w SI. Kompilator przygotowuje
całą trajektorię przed ruchem, sprawdzając limity q/v/a w całych przedziałach
splajnu (otoczki Bernsteina), nie tylko w węzłach. Maksimum to 10000 punktów
wejściowych i 200 s. Ramki są przygotowane dla t=0, 0,01, … oraz końcowego celu
na najbliższej kolejnej granicy zegara. Końce PTP muszą mieć v=0.

Dla każdej uczestniczącej osi:

```text
q_la = clamp(q + v * lookahead_time_sec, lower, upper)
pulses = clamp(llround(direction * (q_la - zero_offset_rad) * gear_ratio * 16384 / (2π)), -8388608, 8388607)
rpm = clamp(lround(abs(v) * gear_ratio * 60 / (2π) * velocity_ff_scale), min_holding_rpm, max_rpm)
```

Domyślnie lookahead=25 ms, FF=1,15, holding=3 RPM, max=2500 RPM, acc=240.
Dodatkowo obowiązuje rzeczywisty limit prędkości osi; za mały limit dla holding
powoduje odrzucenie profilu. Przy mikrokrokach innych niż 16 kod RPM uwzględnia
skalę MKS odczytaną z `84`; podane RPM oznaczają prędkość wału. Offset jest
odejmowany przed przeliczeniem, aby kierunek i kalibracja zachowały znaczenie.
Domyślne acc nie jest wynikiem identyfikacji dynamiki ani wartością w rad/s².

Zegar opiera się na całkowitych odstępach 10 ms. Przy opóźnieniu procesu wysyłamy
jedną aktualną klatkę, pomijając nieaktualne; nie odtwarzamy zaległości serią ramek.
Brak budżetu zestawu daje `waiting_budget`, bez częściowego nadania. Postęp
czasowy odpowiada fazie trajektorii, dochodzi do 99% w dolocie i 100% po sukcesie.
Nie ma już wyboru `native_ptp`; API zachowuje nazwę trybu `streamed`.

## Start, koniec i zatrzymanie

Przygotowanie nadal wymaga jawnego `execution_enabled`, zgodnej rewizji profilu,
modelu i ustawień, kalibracji, ENABLE, świeżego stanu i prawidłowych limitów.
Uzbrajanie potwierdza F1=postój, bieżącą pozycję startową, SR_vFOC=05,
mikrokroki i watchdog=300 ms. Przed pierwszym F5 zapisuje/odczytuje `8C 00 00`
(CanRSP i raport ukończenia wyłączone) oraz `4A=0` (bez oczekiwania na SYNC).
Odczyty i zapisy konfiguracji nadal mają odpowiedzi i timeouty; dotyczy to
uzbrajania przed zegarem ruchu, nie każdego punktu.

F5/F6/F7/FE nie tworzą oczekujących transakcji ACK. Pozycje `31` zbieramy nominalnie
100 Hz/oś asynchronicznie. Odchyłka i brak pomiaru nie zatrzymują punktów
pośrednich. **Tylko po ostatnim F5** `waiting_position` oznacza dolot poza
±0,5° przegubu, a `waiting_feedback` brak świeżych danych. Nie ponawiamy celu.
Sukces wymaga dwóch kompletnych kontroli pozycji po ostatnim celu, oddzielonych
co najmniej 100 ms. Bez dokładnego warunku `32=0` i bez limitu dolotu.

Jawne ENABLE=0, stall/błąd napędu, BUS-OFF, utrata transportu i STOP operatora
nadal przerywają wykonanie. STOP nadaje F7 do wszystkich skonfigurowanych osi,
następnie F6(0), bez czekania na odpowiedzi i bez SYNC. Potem manager kończy
nadawanie. Jest to próba zatrzymania, nie potwierdzenie fizycznego postoju.
JOG zachowuje własny watchdog i weryfikację postoju. Zakończenie programu nie
uruchamia nadzoru starych nastaw JTC; przejęcie wymaga jawnej aktywacji.

`allow_missing_motion` dopuszcza pominięcie nieobecnych osi; uczestnicy są ustaleni
przy przygotowaniu. Osi obecnej bez kalibracji/ENABLE nie wolno pominąć. Niepełny
zestaw nie potwierdza rzeczywistego TCP (`tcp_verified=false`).

## CanSystem i jeden właściciel CAN

`CanSystem` eksportuje position i velocity zarówno dla komend, jak i stanu.
Controller manager ma update_rate=100. `write()` publikuje q/v do atomowej
skrzynki najnowszej próbki. Jeden wątek IO wylicza wspólne F5, wysyła je, następnie
planuje odczyty `31`; nie ma drugiego nadawcy ani socketu kontrolera.
`read()` odczytuje atomowy cache i przelicza radiany. Prędkość liczy z różnicy
pozycji i **rzeczywistych znaczników pomiarów**, aby powtórzony odczyt cache nie
udawał zerowej prędkości. Aktywacja inicjuje zadane pozycje pomiarami, v=0.

Osobny wątek SocketCAN RX korzysta z nieblokującego fd i poll (domyślnie 1 ms),
sprawdza ramki `31` i aktualizuje cache impulsów/czasu. Inne odpowiedzi przekazuje
kolejką SPSC do managera. read/write nie czekają na mutex, recv ani ACK.
To ogranicza blokowanie; Linux/USB/ROS bez pomiarów nie zapewniają twardego RT.
Przy wyłączonym SYNC cele osi trafiają na przewód kolejno — wspólny czas
referencji nie oznacza sprzętowo jednoczesnego startu.

## Parametry i budżet

Plik ROS `execution.example.yaml` pozostaje **wyłączony**. Parametry PTP:

| Parametr | Wartość domyślna |
| --- | --- |
| execution_enabled / execution_configuration_revision | false / pusta |
| execution_use_ack | false; true jest odrzucane |
| execution_lookahead_time_sec / execution_velocity_ff_scale | 0,025 / 1,15 |
| execution_min_holding_rpm / execution_max_rpm | 3 / 2500 |
| execution_acceleration_codes | sześć razy 240 |
| execution_position_period_ms | 10; inny okres jest odrzucany |
| execution_position_tolerance_rad | sześć razy 0,00872664626; tylko koniec |

Stare flagi odbioru i `execution_ack_timeout_ms` pozostają zgodnością API;
nie sterują strumieniem ani nie są sztucznie ustawiane na true. YAML osi i Xacro
mają osobno encoder_cpr=16384, default_acc, lookahead_time_sec, velocity_ff_scale,
min_holding_rpm, max_rpm. Nieznane przełożenia/limity pozostają `null`. Podgląd planera ma zgodny
domyślny `mks_acceleration_codes`=240; nieobsługiwane dawne pola
`tracking_error_rad`/`mks_acceleration_code` usunięto z szablonu osi.

Budżet używa konserwatywnego kosztu `55+10*DLC` bitów i obejmuje F5+31 100 Hz,
ENABLE/stall 10 Hz, prędkość/F1 20 Hz. Bez ACK, dodatkowych raportów i SYNC:

| Osie | Bitrate | Robot | +20 punktów rezerwy | Ocena |
| --- | --- | ---: | ---: | --- |
| 4 | 500 kbit/s | 35,44% | 55,44% | mieści się w 40%/60% |
| 6 | 500 kbit/s | 53,16% | 73,16% | odrzucone |
| 4 | 1 Mbit/s | 17,72% | 37,72% | zapas |
| 6 | 1 Mbit/s | 26,58% | 46,58% | zapas 13,42 punktu w puli robota |

Limit robot=40%, przyszły moduł=20 punktów, łącznie 60%. Dotychczasowy maksymalny
limit konfiguracyjny managera 50% nie powiększa puli robota. To model transmisji
bez retransmisji, nie pomiar przewodu. Cztery miejsca zapytań są potokowane;
F5 nie czeka na wolne miejsce ani odpowiedź enkodera. Przy dużych opóźnieniach
napędów faktyczna częstotliwość feedbacku może spaść.

## Build i wdrożenie

Nowy pełny workspace: `install/streaming100-ros`, Release `-O1 -DNDEBUG`.
Produkcyjna usługa używa już nowego overlay. `can0` ma 500 kbit/s i TX=1000.
Wykonano zatwierdzone próby rozsprzęglonych 1/2/4 osi; aktualne wyniki i granice
odbioru opisuje [raport sprzętowy](motion_hardware_results.md). Nowy profil sześciu osi wymaga osobnego przełączenia **wszystkich**
urządzeń na 1 Mbit/s, poprawnej rewizji, kalibracji i odbioru sprzętowego.

`start_web_status.sh` obsługuje jawny `ROBOT_OVERLAY_SETUP`. Bez overlay, gdy
underlay nie zawiera nowej biblioteki, wybiera wyłączony profil
`execution.legacy.example.yaml`; chroni to stary binarny proces przed nowymi
parametrami. Jawny `execution_parameters:=...` ma pierwszeństwo.
Nie uruchamiać drugiego właściciela CAN. Restart usługi wymaga osobnej zgody.

Ścieżki HTTP i akcji ROS pozostają bez zmian: prepare/status/prepared/execute/stop
w `/api/v1/motion`, `/robot/planning/compute_physical_ptp`,
`/robot/motion/execute_program`. Zamknięcie strony nie zatrzymuje lokalnego programu.
Fizyczny LIN i blending pozostają przyszłe.

Wyniki: [benchmarki](benchmark_results.md), [progi](ptp_timing_limits.md).
Źródło: [instrukcja MKS CAN V1.0.9, §5.2.12 i rozdział 11](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).
Dla CAN wymagany jest **SR_vFOC**, nie wymieniony w zleceniu CR_vFOC (wejście
impulsowe). Nie zmieniamy trybu pracy automatycznie podczas strumieniowania.

## Sprawdzenia oprogramowania — 24.10.2026

Cały workspace 11 pakietów skompilował się bez ostrzeżeń kompilatora.
Pełny zestaw ROS: 327 testów bez błędów; panel: 91 testów i 9 scenariuszy Chromium
na atrapach. Sprawdzono też rosdep, bash/AST/XML/YAML i szablon konfiguracji CAN.
Testy obejmują bajty F5/int48, 100 Hz bez odpowiedzi F5, końcową tolerancję,
brak pośrednich pomiarów, STOP/USB/BUS-OFF, budżet, następny PTP oraz skrzynki RT.

Narzędzie `scripts/ptp_group_hardware.cpp` jest zbudowane osobno; nie uruchomiono
jego ruchu. Ma jawne bramki wykonania i utrzymuje wcześniejsze ograniczenie 1 RPM
wału/acc 1 (odstępstwo od domyślnego holding 3 i acc 240). Pythonowy wrapper wskazuje
nową bibliotekę/binarium, nie stary ACK build. Analizator śladów rozróżnia stary
SYNC 20 ms i nowy F5 10 ms; dla streamingu porównuje nominalną trajektorię oraz
rzeczywiste cele F5 z przekładnią, kierunkiem i offsetem, także po pominięciu klatek.
Test syntetyczny i ponowna analiza kopii starej próby 3 dały oczekiwane wyniki.
Późniejsze próby sprzętowe: dla czterech osi 86,38% zaakceptowanych próbek w ±0,5°
względem trajektorii nominalnej, 84,58% względem nadanych celów F5. Są to wyniki
profilu 1 RPM/acc 1, bez odbioru płynności i rzeczywistej prędkości chwilowej.

Powtarzalne komendy pełnego builda/testów z katalogu głównego:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --build-base build/streaming100-ros \
  --install-base install/streaming100-ros --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
source install/streaming100-ros/local_setup.bash
colcon test --base-paths ros2_ws/src --build-base build/streaming100-ros \
  --install-base install/streaming100-ros
colcon test-result --test-result-base build/streaming100-ros --verbose
```

Historyczny odczyt przed wdrożeniem 24.10.2026: `can0` 500000 bit/s, `txqueuelen` 10; PID usługi
22808 i licznik restartów 2 bez zmiany. Szablon 1000000/1000 nie jest
ówczesną konfiguracją. Później wdrożono nowy overlay i TX=1000, zachowując 500k.

## Przygotowanie produkcji i nastawy MKS — 24.10.2026

Skrypt `scripts/deploy_streaming100.sh --dry-run` kontroluje aktualną rewizję,
brak aktywnego programu/JOG, profil i build. Wariant `sudo ... --apply` instaluje
odwracalny drop-in `streaming100.conf`, nową jednostkę CAN oraz kolejkę TX=1000,
przełącza aplikację na `install/streaming100-ros` i sprawdza HTTP. Zachowuje aktualny
bitrate linku. Profil `config/local/execution.production.yaml` ma
`execution_enabled=false`; wdrożenie nie nadaje ruchu ani ENABLE/zera.
Kopie i wyniki są w `log/streaming100-deployment/`. Przy niepowodzeniu skrypt
odtwarza pliki, kolejkę i uruchamia poprzednią usługę.

24.10: użytkownik uruchomił instalację administratora. Potwierdzono drop-in,
nowe procesy/bibliotekę, cykl 10 ms, motion_use_ack=false i TX=1000 przy 500k.
Początkowe komunikaty ConnectionRefused pochodziły z oczekiwania na start HTTP;
instalacja zakończyła się sukcesem. Skrypt obsługuje teraz ten stan bez tracebacku.
Profil wykonania pozostaje wyłączony do ukończenia odbioru/kalibracji.

Nie wszystkie nastawy wymagają ręcznej obsługi wyświetlacza:

| Nastawa | Kto ją ustawia/sprawdza |
| --- | --- |
| CAN ID 1–6 | istniejące mapowanie; aplikacja nie zmienia adresów |
| CanRate | obecnie 500K; migracja do 1M jest osobnym krokiem, wspólnym dla wszystkich urządzeń i hosta |
| SR_vFOC 05 | przygotowanie ENABLE ustawia i odczytuje; uzbrojenie PTP sprawdza |
| CanRSP/raport końca | przygotowanie ENABLE i uzbrojenie PTP ustawiają `8C 00/00` i odczytują |
| Synchronizacja 4A | ustawiana i odczytywana jako 0, bez SYNC 4B |
| Watchdog 300 ms | przygotowanie ENABLE ustawia i sprawdza; PTP sprawdza |
| Mikrokroki | odczyt 84 i dostosowanie skali; aplikacja nie przestawia ich automatycznie |
| Prąd, Protect, kalibracja silnika | pozostają istniejące; nie zmieniamy ich przy wdrożeniu |

CanRate można zmienić z LCD albo komendą `8A 03` (instrukcja V1.0.9, §5.2.10).
Bieżąca aplikacja nie wykonuje migracji bitrate. Nie należy przestawiać samego
hosta na 1M, pozostawiając napędy przy 500K. Migracja wymaga postoju, braku
nadawania przez aplikację, zmiany wszystkich podłączonych napędów i hosta,
a następnie kontroli odczytów/błędów magistrali. W tej sesji nie zmieniono CanRate.
Cztery osie 100 Hz przy 500K modelowo mieszczą 35,44%; sześć osi nie mieści się
w puli 40% i wymaga 1M (26,58%).

Użytkownik ponownie potwierdził rozsprzęglenie J1/J2/J3/J5 i zatwierdził próby:
J1 → J1+J2 → J1+J2+J3+J5, 0°→+20°→0° przegubu, maksymalnie 1 RPM wału,
STOP przy błędzie. Próby zakończono i opisano w [raporcie sprzętowym](motion_hardware_results.md). Nie należy testować profilu
holding 3 RPM w ramach limitu 1 RPM; potrzebny jest ograniczony profil prób.

Przy przygotowaniu sprzętu ujawniono brak ACK F7 po `8C 00/00`. Poprawiono także
ręczną sekwencję ENABLE: F7 nie czeka na ACK, ale następujące 32/F1 i pozycje
nadal potwierdzają postój. Test emulatora odtwarza ten brak odpowiedzi oraz
zgubioną komendę F7; 107 testów pakietu CAN po tej poprawce przeszło.
Zerowanie 1/2/3/5 przez API zakończyło się poprawnie, nieważność kalibracji została
utrwalona w calibration.json. Nie przywracano sztucznie flag ważnej kalibracji.

Pierwsza próba dwóch osi osiągnęła cele, lecz wystąpiły błędy BIT1. Ślad ujawnił
zapytania 31 około 1 ms przed pierwszym F5. Poprawka fazy wstrzymuje diagnostykę
przed pierwszym F5 i inicjuje odczyty po całym zestawie F5; następne odczyty mają
ten sam okres 10 ms. Nie dodaje oczekiwania na odpowiedź ani tolerancję pośrednią.
Test kolejności początkowo wykazał 4 przedwczesne odczyty, po poprawce przechodzi;
108 testów pakietu CAN bez błędów. Narzędzie prób teraz przerywa również przy
ramkach błędu CAN, poza jednoznacznym komunikatem powrotu do ERROR-ACTIVE.

## Końcowe okno odczytów i stan produkcji

Druga próba dwóch osi wykazała kolizję F1 z kolejnym F5. W PTP nowe zapytania
pozycji i diagnostyki nie rozpoczynają się w ostatnich 4 ms przed następnym F5.
F5 nadal nadaje się według własnego zegara, bez czekania na ACK/pomiar. Odczyty
mogą się odroczyć; w czterech osiach ich średni okres wyniósł 11,12–11,32 ms,
maksymalny 19,97 ms podczas strumienia. To nie stała gwarancja 100 Hz pomiarów.
Zakaz dotyczy PTP; faza JTC i sześć osi wymagają osobnej próby sprzętowej.

Po tej poprawce J1+J2 oraz J1+J2+J3+J5 zakończyły oba kierunki bez błędów CAN
i timeoutów odczytów uczestników. Przy potwierdzeniu końca cztery osie mieściły
się w ±0,5°; po ENABLE=0 nastąpiło przesunięcie wałów, opisane oddzielnie w raporcie.
Napędy są wyłączone, kalibracje 1/2/3/5 nieważne po zerowaniu i execution_enabled=false.
Pełna praca z panelu wymaga ponownego zatwierdzenia offsetów, zgodnej rewizji
wykonawcy i jawnego włączenia. Nie zaliczono TCP, acc 240, holding 3 RPM ani 6 osi.
Końcowy proces panelu jest active/fresh, bez busy/fault, z nową biblioteką.

Aktualne sprawdzenia poprawek: 109 testów CAN oraz testy konsumentów ROS/hardware
w oddzielnej domenie 87, bez dotykania produkcyjnego ROS 0. Suma zapisanych wyników
331 bez błędów obejmuje także wcześniejsze testy pozostałych pakietów. Frontendu
nie testowano ponownie, bo nie zmieniano go w tej sesji. Benchmarki przed/po i CPU:
[wyniki](benchmark_results.md).


## Parametry PTP z panelu — 24.10.2026

Sekcja **Parametry ruchu PTP** w sterowaniu ręcznym udostępnia prędkość i
przyspieszenie trajektorii w °/s oraz °/s². Te same żądane limity są zapisywane
z podglądem i przekazywane do przygotowania fizycznego PTP; profil każdej osi
nadal ogranicza wynik. Domyślnie panel wybiera 6°/s oraz 50°/s². Edycja usuwa
lokalne wskazanie gotowego planu i zatwierdzenie, również po podglądzie fizycznym.
Konieczne jest ponowne obliczenie, przygotowanie i zatwierdzenie ruchu.

Zerowanie odłączonych J1/J2/J3/J5 zakończono o 23:35:47 CEST: ACK i 0 impulsów
potwierdzone dla wszystkich. Profil produkcyjny włączono na nowej, zgodnej rewizji
przez normalne API konfiguracji; potwierdzono parametry działającego ROS.
Użytkownik zlecił następnie przywrócenie limitu 80°/s. Służy do tego
`scripts/set_ptp_80.sh --apply`: postój i wyłączenie napędów, konfiguracja przez
API, aktualizacja rewizji i restart, bez zerowania i zadawania ruchu.
Holding pozostaje 1 RPM, kod acc=1; limit RPM wykonawcy wyliczany jest z 80°/s
i rzeczywistych przełożeń (14 RPM globalnie dla obecnego 1:1, dodatkowo stosowany
limit każdej osi i kwantyzacja MKS). Procedurę wykonano o 23:49:47 CEST; HTTP i parametry ROS potwierdziły
limit 80°/s, holding1/max14 RPM, acc1 i zgodną nową rewizję. Napędy wyłączone,
PTP idle; bez nowej próby ruchu. Szczegóły i aktualny stan:
[raport migracji](can_1m_results.md).
