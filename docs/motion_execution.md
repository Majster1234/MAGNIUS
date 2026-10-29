# Fizyczne wykonanie PTP/LIN — streaming 100 Hz

Od 28.10.2026 dostępny jest opcjonalny [streaming synchroniczny](synchronized_streaming.md).
Poniższy kontrakt bez ACK/SYNC nadal opisuje domyślny tryb `streamed`;
nowy tryb wymaga oddzielnego odbioru sprzętowego i nie obejmuje JOG ani JTC.

## LIN na wspólnym wykonawcy — 25.10.2026

PTP i LIN korzystają z tego samego wykonawcy C++ oraz strumienia CAN 100 Hz.
Różni je obliczona trajektoria q/v/a: PTP synchronizuje przeguby, a LIN wyznacza
prostą TCP z interpolacją orientacji SLERP i ciągłym IK wzdłuż drogi. Planner
zachowuje kontrolę gałęzi IK, końcowych kątów, osobliwości, limitów osi i posuwu.
Tolerancje obliczonej referencji LIN wynoszą 0,1 mm i 0,1°. Nie oznaczają
zmierzonej dokładności fizycznego toru TCP.

Przebieg panelu: wybór LIN i podgląd → **Przygotuj LIN robota** → opcjonalny
**Podgląd przygotowanego LIN** → osobne zatwierdzenie → **Wykonaj LIN**.
Przygotowanie przelicza drogę od aktualnych pomiarów, z celem i zachowanymi
limitami posuwu z wybranego podglądu. Zmiana celu, typu lub parametrów unieważnia
zgodę. Stary podgląd LIN bez zapisanych trzech limitów TCP wymaga przeliczenia.
Zapis ustawień ani przygotowanie nie rozpoczyna ruchu.

ROS: nowa akcja `/robot/planning/compute_physical_motion` typu
`ComputePhysicalMotion` przyjmuje typ PTP=0/LIN=1, dotychczasowy kontekst i cel
przegubów oraz `tcp_velocity`, `tcp_acceleration`, `tcp_angular_velocity` w SI.
LIN wymaga trzech dodatnich limitów, PTP trzech zer. Dotychczasowa akcja
`/robot/planning/compute_physical_ptp` pozostaje zgodną nakładką dla PTP.
Usługi przygotowania i wykonania są wspólne; adapter nie przyjmuje symulacji
ani innych typów ruchu. Format statusu wykonawcy ROS pozostaje bez zmian.

HTTP `/api/v1/motion/{prepare,status,prepared,execute,stop}` zachowuje adresy.
Przygotowanie i status podają `motion_type`. Typ w statusie pochodzi z fizycznego
planu zgodnego z `program_id`; bez zgodnego planu jest `null`, także po restarcie
mostu. Pole `execution_limits` zapisuje znormalizowane limity LIN w jednostkach
panelu: mm/s, mm/s² i °/s, łącznie z domyślnym przyspieszeniem 100 i obrotem 10.

Wybrano nadzór taki jak PTP: wspólne lookahead, korekcja osi, dithering, STOP
i potwierdzanie pozycji końcowych ±0,5° przegubów. Nie dodano zatrzymania od
odchyłki TCP w trakcie ruchu. Osobna korekcja osi, dynamika napędów i lookahead
mogą zmienić rzeczywisty tor względem prostej referencji; odbiór sprzętowy musi
ocenić ten efekt. Nie zmieniono profilu produkcyjnego ani parametrów CAN.

Istniejące `allow_missing_motion` obejmuje również próby LIN na niepełnym
zestawie. Nieobecne osie nie otrzymują komend, a ich podgląd pozostaje na starcie.
Panel oznacza **Próba LIN — tor TCP nie jest potwierdzony**; `tcp_verified=false`.
Nadal nie wolno pomijać obecnej osi z nieważną kalibracją lub bez ENABLE.
Blending, fizyczny JOG TCP i wykrywanie kolizji nie należą do tej zmiany.

Kontrakt źródeł z 25.10.2026 (85 ms, korekcja pozycji i dithering RPM) zastępuje wcześniejsze F5/ACK/SYNC i oczekiwanie
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
wejściowych i 200 s. Siatka nadawania to t=0, 0,01, … oraz końcowy cel
na najbliższej kolejnej granicy zegara. Podczas wykonania q/v/a są próbkowane
przez kwintyczną interpolację Hermite’a dla rzeczywistego czasu wysyłania,
także przy spóźnieniu pomiędzy węzłami. Końce PTP muszą mieć v=0; nie zerujemy
pochodnej w pośrednich punktach, gdy trajektoria przewiduje dalszy ruch.

Poprawka dokładności i zachowania kątów po zmianie przełożeń znajduje się
w `install/gear-ratio-continuity-ros`. Nowy panel korzysta z tego overlay jako underlay; szczegóły punktu startowego
i restartu znajdują się w sekcji wdrożenia panelu na końcu dokumentu. Poprzedni `install/lin-ros`
zachowuje liniowy lookahead i korekcję bez wieku RX.
Wyniki symulacji, analizy dawnych nagrań i ograniczenia pomiaru opisuje
[raport dokładności](streaming_accuracy.md).
Zasady zmiany przełożeń i potwierdzania odniesienia:
[zachowanie kątów](gear_ratio_continuity.md).

Dla każdej uczestniczącej osi w nowym kodzie:

```text
t_la = min(lookahead_time_sec, remaining_time_sec)
t_kin = min(t_la, abs(v)/max(abs(a_ref), 1e-9)) if v*a_ref < 0 else t_la
q_la = clamp(q + v*t_kin + 0.5*a_ref*t_kin², lower, upper)
pulses = clamp(llround(direction * (q_la - zero_offset_rad) * gear_ratio * 16384 / (2π)), -8388608, 8388607)
age = clamp(actual_age_sec, 0, 0.05)
error = q - v*age + 0.5*a_ref*age² - q_actual
motion_sign = sign(v) if abs(v) > 1e-6 else (sign(error) if abs(error) > 0.0005 else 0)
v_corr = position_kp * error * motion_sign  # tylko ważny pomiar
v_eff = clamp(abs(v) * velocity_ff_scale + v_corr, 0, axis_max_velocity)
rpm_float = min(v_eff * gear_ratio * 60 / (2π) / speed_scale, wire_max_rpm)
rpm_int = floor(rpm_float + residual)
residual = rpm_float + residual - rpm_int
```

Wartość pomiaru i znacznik RX są pobierane razem. Wiek oznacza czas od RX,
bez założenia o nieznanym opóźnieniu wewnętrznym napędu. Bez pomiaru korekcja
wynosi zero. PTP/LIN przekazują przyspieszenie i pozostały czas programu;
JTC przekazuje a_ref=0 i pełny lookahead, ponieważ nie zna tutaj końca trajektorii.

Domyślnie lookahead=85 ms, FF=1,00, Kp=2,5 1/s, holding=1 RPM,
max=2500 RPM i acc=135. Każda oś ma osobny akumulator ułamków RPM,
zerowany przy starcie, postoju i zmianie kierunku. Prędkość 1,73 RPM przy
mikrokroku16 daje 73 ramki z 2 RPM i 27 z 1 RPM na 100 cykli. W innych
mikrokrokach dithering dotyczy odpowiednio przeskalowanych jednostek MKS.
Poniżej minimum używamy holding zamiast zera; dla prędkości poniżej 1 RPM
nie gwarantuje to ciągłości fizycznego ruchu. Korekcja zwiększa prędkość przy
pozostawaniu za referencją i zmniejsza ją przy wyprzedzeniu, w obu kierunkach.
Może przekroczyć nominalny posuw referencji, do limitu osi i max_rpm.
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
Most HTTP dopuszcza zarówno `idle` po uzgodnieniu startowym, jak i `done`
po zapisie konfiguracji, wyłącznie z `success=true` i aktywną rewizją zgodną
z planem oraz stanem osi. Trwająca, nieudana lub niepotwierdzona transakcja
blokuje fizyczne przygotowanie i pierwszy start. Powtórzenie tego samego klucza
wykonania pozostaje idempotentne. Podgląd zachowuje dotychczasową politykę.

Rewizję fizycznego PTP wyznacza aktywna konfiguracja zatwierdzona przez transakcję
`PREPARE → model ROS → COMMIT → trwały zapis → FINISH`. Zapis poprawnych przełożeń,
kierunków, offsetów i limitów w panelu nie wymaga przepisywania UUID do YAML ani
restartu backendu. Nadal wymaga postoju i ENABLE=0 dla zmienionych obecnych osi.
`execution_enabled` pozostaje osobną bramką włączenia PTP.
Ponowny zapis identycznych wartości z ważną kalibracją zachowuje rewizję
konfiguracji, także gdy zmieniła się nazwa lub data snapshotu. Rzeczywista zmiana
nadaje nową rewizję; plan i stan osi muszą się z nią zgadzać. Przyjęty PREPARE
unieważnia wcześniej przygotowany program, także po późniejszym rollbacku.
Nieuzgodniony start, trwająca transakcja, nieważna kalibracja lub stary plan
nadal blokują przygotowanie. Po zmianie konfiguracji trzeba przeliczyć podgląd,
przygotować fizyczny PTP i osobno zatwierdzić wykonanie. Zapis nie rozpoczyna ruchu.

Od poprawki mechanizmu rewizji z 25.10 `execution_configuration_revision`
i pole C++ `program_revision` są przestarzałe i ignorowane. ROS przyjmuje dawny
parametr dla zgodności YAML oraz ostrzega przy niepustej wartości. Stare pliki
działają bez ręcznego uzgadniania UUID. Uruchomiony wcześniej backend wymaga
jednorazowego wdrożenia i restartu z nowym kodem; dalsze zapisy konfiguracji
nie wymagają restartu. Samo odświeżenie przeglądarki nie ładuje poprawki C++.
Na późniejsze polecenie operatora lokalny profil produkcyjny ma
`execution_max_rpm=500` zamiast 14. Przełożenia 3:1 oraz limity przegubów
80°/s pozostają zachowane; przy tym przełożeniu limit przegubu odpowiada
40 RPM wału. Wartość 500 RPM jest górnym ograniczeniem wykonawcy,
a nie zadaną prędkością ani zwiększeniem limitów osi. Zmiana YAML wymaga
restartu backendu w postoju, bez automatycznego uruchamiania ruchu.

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
| execution_enabled | false; niezależne włączenie fizycznego PTP |
| execution_configuration_revision | przestarzały, ignorowany; rewizję wyznacza zatwierdzona konfiguracja ROS |
| execution_use_ack | false; true jest odrzucane |
| execution_lookahead_time_sec / execution_velocity_ff_scale | 0,085 / 1,00 |
| execution_position_kp | 2,5 1/s; skończone, nieujemne |
| execution_min_holding_rpm / execution_max_rpm | 1 / 2500 |
| execution_acceleration_codes | sześć razy 135 |
| execution_position_period_ms | 10; inny okres jest odrzucany |
| execution_position_tolerance_rad | sześć razy 0,00872664626; tylko koniec |

Stare flagi odbioru i `execution_ack_timeout_ms` pozostają zgodnością API;
nie sterują strumieniem ani nie są sztucznie ustawiane na true. YAML osi i Xacro
mają osobno encoder_cpr=16384, default_acc, lookahead_time_sec, velocity_ff_scale, position_kp,
min_holding_rpm, max_rpm. Nieznane przełożenia/limity pozostają `null`. Podgląd planera ma zgodny
domyślny `mks_acceleration_codes`=135; nieobsługiwane dawne pola
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
Fizyczny LIN dodano 25.10.2026 (opis na początku dokumentu); blending pozostaje poza zakresem.

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


## Zmiana strumieniowania — 25.10.2026

Rzeczywisty sampler kwintyczny q/v/a był już obecny. Poprawiono jego użycie:
spóźniony tick np.17 ms korzysta teraz z referencji17 ms, zamiast starej klatki10 ms.
Przygotowanie nadal weryfikuje cały profil i siatkę; wykonawca oblicza wszystkie
payloady dla jednego wspólnego czasu przed pierwszym F5. Potem wysyła zwartą serię
aktywnych osi i dopiero asynchroniczne odczyty. W stanie waiting_budget akumulatory
nie zmieniają się. Po opóźnieniu nie odtwarzamy historii ani nie ponawiamy
niepewnego, częściowo wysłanego zestawu.

Korekcja używa cache RX SocketCAN lub świeżej pozycji managera (wiek<250 ms),
po przeliczeniu na przegub. Przy braku pomiaru działa sam feed-forward/dithering;
trajektoria nie czeka, a potwierdzenie końca nadal wymaga pozycji. JTC korzysta
z tego samego generatora RPM; jego własne warunki aktywacji i świeżości pozostają.

`shaft_trace` już wcześniej był pasywnym konsumentem kolejki SPSC. Nie ma osobnego
shaft_trace.cpp wysyłającego31; pobieranie wykresów nie zajmuje mutexu CAN.
Test sprawdza brak nadawania przy opróżnianiu kolejki. Przy brakujących4/6 test
potwierdza, że podczas PTP nie wysyła się do nich ani F5, ani zapytań. Uczestnicy
są ustalani przed startem; utrata pomiaru uczestnika nie usuwa go z programu.
Nie wprowadzono niepełnego JTC ani sztucznej kalibracji. Bufory/telemetria nie
zwiększają liczby ramek, więc budżet robot40% + rezerwa20 punktów pozostaje.

Nowy build: `install/streaming-smooth-ros`, osobno od produkcyjnego
`install/streaming100-ros`. Zmiana struktury parametrów wymaga użycia całego
nowego overlay, bez mieszania bibliotek/nagłówków. Lokalny profil produkcji
ma przygotowane85 ms/FF1/Kp2,5/min1/acc135 oraz zachowany limit80°/s i max14 RPM.
Wcześniejsza działająca usługa miała już min1/acc1, więc część diagnozy o jej
min3/acc240 dotyczyła dawnych domyślnych nastaw. Nowy acc135 jest zmianą także
względem tamtego profilu prób; nie oznacza odebranego zachowania sprzętu.
Nie wykonywano nowego ruchu, restartu usługi ani przełączenia produkcji.
Skrypty wcześniejszych prób jawnie wybierająceacc1 pozostają profilami historycznymi.
Wykonalność i fizyczną płynność nowego algorytmu trzeba potwierdzić oddzielną
próbą i przebiegiem shaft_trace. Benchmarki: [wyniki](benchmark_results.md).


Do wdrożenia nowego kodu trzeba przełączyć `ROBOT_OVERLAY_SETUP` w drop-in
`/etc/systemd/system/robot-web-status.service.d/streaming100.conf` na
`/home/mk/Desktop/Projekty/install/streaming-smooth-ros/local_setup.bash`,
zachowując argument profilu produkcyjnego i bitrate1M. Dopiero wtedy wykonać
osobno uzgodniony restart w postoju. Sam restart dotychczasowego overlay
nie uruchamia nowego algorytmu. Zmieniony YAML produkcyjny jest przygotowany
dla nowego builda, a uruchomiony proces nie przeładowuje go w locie.

## Build i wdrożenie poprawki rewizji — 25.10.2026

Zbudowano wszystkie 11 pakietów wymaganych przez `robot_bringup` w osobnym
overlay `install/ptp-revision-ros`, z katalogiem `build/ptp-revision-ros`.
Konfiguracja produkcyjna: Release, `-O1 -DNDEBUG`, `BUILD_TESTING=OFF`.
Build zakończył się poprawnie. Na jego bibliotekach uruchomiono wcześniej
zbudowane binaria regresji: 7 testów rewizji i 6 testów parametrów ROS NoIo
w domenie 188; wszystkie przeszły. Poprzednie 71 testów dotyczyło wcześniejszej
tury implementacji. Sprawdzono wybór wszystkich 11 pakietów w środowisku usługi,
zależności pięciu artefaktów bez starych bibliotek oraz `launch --show-args`.
Nie uruchamiano sprzętowego CAN ani ruchu.

Systemd nadal wskazuje `install/streaming-smooth-ros/local_setup.bash`.
Ten plik jest teraz krótkim przekierowaniem do nowego
`install/ptp-revision-ros/local_setup.bash`. Poprzednią wersję zachowano jako
`log/ptp-revision-build-20261025/deployment/local_setup.bash.before`.
Nie nadpisano bibliotek używanych przez działający proces. Systemowego drop-in
nie zmieniono, ponieważ zapis przez sudo wymagał hasła; przekierowanie znajduje
się w zapisywalnym katalogu projektu. Build kolejnej wersji powinien używać
nowego prefixu i aktualizować punkt startowy usługi; budowanie do starego
`streaming-smooth-ros` może zastąpić ten wygenerowany plik.

Wdrożenie jest gotowe na następny start. Usługa pozostała aktywna,
MainPID=1388 i NRestarts=0 przed/po; nie wykonano restartu. Profil fizyczny
i parametry CAN pozostały bez zmian. Operator wykonuje w postoju:

```bash
sudo systemctl restart robot-web-status.service
```

Po restarcie pozostaje kontrola przygotowania PTP bez wykonania programu.
Wyniki buildu, hashe artefaktów i potwierdzenie wdrożenia:
`log/ptp-revision-build-20261025`.

## Wdrożenie wspólnego PTP/LIN — 25.10.2026

Zbudowano 11 pakietów do `install/lin-ros` (Release, `-O1 -DNDEBUG`,
`BUILD_TESTING=OFF` dla buildu produkcyjnego) i frontend z Node.js 22.23.4.
Testy zbudowano osobno. Zaliczone: 109 scenariuszy C++/Python/ROS, 2 testy
interpolacji TypeScript i 2 scenariusze Chromium. Wśród nich rzeczywisty planner
ROS w domenie 132, adapter NoIo w domenie 188 oraz pełny/niepełny LIN na
emulatorze z przełożeniami, offsetami, ujemnymi celami, kontrolą rewizji i STOP.
Ruchu sprzętowego ani fizycznego CAN w tej turze nie uruchamiano.

Frontend wdrożono do `web/dist`, zachowując poprzednie zasoby dla otwartych kart
i wymieniając `index.html` po skopiowaniu nowych plików. Następnie odświeżono
zasoby pakietu `robot_web_bridge`. Sprawdzono 11 prefixów pakietów, komplet
zależności pięciu artefaktów, typ nowej akcji, zasoby panelu i argumenty launch.

Dotychczasowy punkt startowy `install/streaming-smooth-ros/local_setup.bash`
przekierowuje teraz do `install/lin-ros/local_setup.bash`. Kopia poprzedniego
przekierowania i frontendu, hashe oraz wyniki: `log/lin-20261025/deployment`.
Systemowego drop-in nie zmieniano. Przy przełączeniu MainPID=12021,
ActiveState=active, NRestarts=0 pozostały bez zmian; nowy backend oczekuje na
restart operatora. Parametry wykonania i konfiguracja CAN pozostają dotychczasowe.

```bash
sudo systemctl restart robot-web-status.service
```

Restart wykonać w postoju. Po nim odświeżyć panel i sprawdzić przygotowanie LIN
bez wykonania programu. Osobna próba sprzętowa wymaga uzgodnienia osi, kierunku,
prędkości, zakresu i zatrzymania; testy emulatora nie potwierdzają rzeczywistego
toru TCP. Pomiary programowe: [benchmark_results.md](benchmark_results.md),
raporty testów: `log/lin-20261025`.

## Panel sterowania i Centrum komunikatów — 25.10.2026

PTP i LIN mają wspólną kartę z etapami **Podgląd**, **Przygotowanie robota** oraz
**Zatwierdzenie i wykonanie**. Podsumowanie pokazuje typ, czas, postęp, uczestników
i rozwijane pozycje start → cel. Szczegóły obsługi oraz znaczenie komunikatów:
[sterowanie ręczne](manual_control.md#fizyczny-ptplin-i-niepełny-zestaw-napędów).
STOP znajduje się w nagłówku poza przewijaną prawą kolumną; zatrzymanie animacji
pozostaje oddzielną operacją.

`ControlMessages` oddziela aktywne warunki od wyników operacji i historii
`EventLog` (100 wpisów). Rozwiązanie warunku wymaga nowej oceny stanu;
wyparcie wpisu z historii nie usuwa blokady. Katalog opisów pomaga operatorowi,
a gotowość przycisków nadal wynika ze stanu i dotychczasowych bramek sterowania.
Nieznany błąd pokazuje rzeczywisty tekst bez przypisywania przyczyny. Otwieranie
szczegółów i linków do konfiguracji nie załącza napędów ani nie uruchamia ruchu.

Status wykonania ma granicę świeżości 750 ms. Odpowiedzi sprzed rozpoczęcia
nowszej operacji są odrzucane; gotowość wymaga zgodnego identyfikatora przygotowania,
programu, podglądu i typu. Utrata świeżości lub zmiana planu kasuje lokalną zgodę.
Błąd sieci, 5xx lub odpowiedź HTTP 408/499 podczas startu oznaczają wynik nieznany.
Panel blokuje kolejny start do uzgodnienia stanu; zachowuje `execution_key`.
Przyjęcie STOP przez HTTP i późniejsze potwierdzenie nieaktywnego wykonawcy są
osobnymi zdarzeniami. Żadne z nich nie zastępuje pomiaru zatrzymania wałów.
API HTTP/ROS oraz wykonawca CAN pozostają bez zmian.

Regresja TCP wykryła wcześniejszy problem pierwszej klatki RAF: jej znacznik mógł
poprzedzać chwilę zakończenia obliczeń, co przerywało poprawny impuls symulacji.
Początek i kolejne próbki `PlanningTcpStep` korzystają teraz z bieżącego
`performance.now()`; zatrzymanie przy przerwie ponad 150 ms lub utracie warunków
pozostaje aktywne. Test jednostkowy obejmuje starszy znacznik RAF i późniejszą
przerwę. Zmiana nie dotyczy zegara ani trajektorii fizycznego wykonawcy.

Weryfikacja tej zmiany używa atrap HTTP/WebSocket bez ROS i CAN. Katalog,
deduplikacja, trwałość warunków po 100 wpisach, świeżość, zgodność przygotowania
oraz dotychczasowe JOG/TCP/ślady mają 86 zaliczonych testów w ośmiu plikach TS.
Pięć scenariuszy Chromium zaliczono; kod znajduje się w `web/test/{control-center,motion,manual,cartesian,tcp}.browser.cjs`.
Obejmują PTP/LIN, ponowienie po błędzie, starsze odpowiedzi statusu i przygotowania,
zanik HTTP/WS, brakujące osie, niepotwierdzony start oraz opóźnione potwierdzenie STOP.
Układ sprawdzany jest przy 1280×800, 1366×768, 1920×1080, rozwiniętym menu
oraz powiększeniu strony 125%. Sprawdzenia klawiatury i ARIA obejmują zgodę,
wykonanie, fokus STOP, etykiety pól oraz brak ogłoszeń każdego procentu postępu;
nie wykonano odrębnej sesji z zewnętrznym czytnikiem ekranu.

Build frontendu: Node 22 / TypeScript / Vite. Pakiet `robot_web_bridge` zbudowano
w `install/motion-control-ui-ros` na `install/gear-ratio-continuity-ros`.
Pozostałe biblioteki, także ciągłość przełożeń i poprawka F5, pochodzą z tego
underlay; sprawdzono prefiksy, pola `ConfigureRobot` i `launch --show-args`.
Frontend nie modyfikuje cykli ROS ani CAN, więc nie dodawano benchmarków CPU tych
procesów. Pozostaje znane ostrzeżenie Vite o większym pakiecie Three.js.

Artefakty sprawdzeń i kopie wdrożenia: `log/motion-control-ui-20261025/`.
Nowy pakiet zawiera rzeczywisty plik `index.html` i zachowane wcześniejsze
hashowane zasoby dla otwartych kart. Punkt startowy
`install/streaming-smooth-ros/local_setup.bash` ładuje underlay przełożeń,
a następnie panel. Kopia poprzedniego punktu i zasobów:
`log/motion-control-ui-20261025/deployment/`.
Przywrócenie poprzedniej wersji polega na odtworzeniu `local_setup.bash.before`
w punkcie startowym; poprzedni katalog panelu nie jest nadpisywany.

Nowy panel i pakiet backendu wchodzą razem przy restarcie. Restart pozostaje
operatorowi w postoju:

```bash
sudo systemctl restart robot-web-status.service
```

Po restarcie odśwież panel i sprawdź stan oraz przygotowanie bez wykonania.
Próba sprzętowa nadal wymaga osobnego uzgodnienia osi, kierunku, prędkości,
zakresu oraz zatrzymania.

Wdrożenie zweryfikowano również uruchamiając scenariusz centrum na zasobach
z nowego prefiksu, nadal z atrapami API. PID usługi 66323, licznik restartów 0
oraz konfiguracja CAN 1 Mbit/s / TX1000 były identyczne przed i po operacji.
Nie restartowano usługi, nie otwierano fizycznego CAN ani nie wykonywano ruchu.
