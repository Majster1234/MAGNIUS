# Sterowanie i diagnostyka przez wspólny manager CAN

Ręczne ENABLE, zero i prędkościowy JOG pojedynczego wału opisuje
[moduł ręczny](manual_control.md). Działa bez modelu robota, przy tym samym
właścicielu CAN, i wyklucza się ze sterowaniem sześciu osi opisanym poniżej.

## Przepływ i właściciel magistrali

```mermaid
flowchart TD
    OP[Przeglądarka / operator]
    MOVE[MoveIt: trajektorie lub Servo]
    GATE[robot_motion_gate: jedno źródło i lease]
    CTRL[JointTrajectoryController / ros2_control]
    HW[robot_hardware/CanSystem]
    BUS[CanBusManager / C++ / wątek IO]
    CAN[SocketCAN / U2C / MKS]
    DIAG[Diagnostyka ROS / backend / panel]
    OP --> MOVE --> GATE --> CTRL
    CTRL <--> HW
    HW <--> BUS
    BUS <--> CAN
    BUS --> DIAG
```

`CanBusManager` jest jedynym modułem planującym ramki. W trybie diagnostycznym
działa w `robot_can_diagnostics`. Podczas sterowania ten sam kod i publikator
diagnostyki działają w pluginie `robot_hardware/CanSystem`; osobny właściciel
magistrali nie jest uruchamiany. Panel należy wtedy uruchamiać przez
`web_status.launch.py standalone_can:=false configuration_enabled:=false preview_enabled:=false`
(konfiguracja i podgląd są wtedy uruchomione przez launch sterowania).

Blokada procesu dla pary użytkownik/interfejs w `/tmp` zapobiega uruchomieniu
dwóch właścicieli z projektu przez to samo konto. Nie blokuje obcych programów
SocketCAN ani innych kont. Magistrala musi mieć jednego nadawcę poleceń ruchu.

## Harmonogram i budżet

| Operacja | Profil startowy | Zasada |
| --- | --- | --- |
| Stop i błędy | Przy wykryciu | Przerwanie pracy, zatrzask błędu |
| Zestaw pozycji sześciu osi | 50 Hz | Najnowsza próbka, bez historii poleceń |
| Pozycje rzeczywiste `0x31` | 50 Hz | Zarezerwowany budżet, osobna świeżość |
| ENABLE i stall | 10 Hz | Osobne znaczniki czasu |
| Szczegółowy stan pracy | 2 Hz | Ustępuje komendom i pozycjom |
| Tryb pracy | Przy wykryciu / co 5 s w postoju | W ruchu bez zmian konfiguracji |
| Publikacja do panelu | 5 Hz | Korzysta z cache |
| SCAN, reset | Bez aktywnego sterowania | Brak automatycznego wznowienia |

Oczekują maksymalnie cztery zapytania i jedno na adres. Koszt jest rezerwowany
dla zapytania i odpowiedzi: konserwatywnie `55 + 10 * DLC` bitów na standardową
ramkę CAN wraz ze stuffingiem i odstępem. Limit to 50% przepustowości 500000 bit/s;
stop omija limit. „Planowane obciążenie” nie obejmuje retransmisji i nie jest
fizycznym pomiarem wykorzystania magistrali.

Sześć F5, sześć ACK, SYNC i sześć odczytów pozycji kosztują w tym oszacowaniu
2655 bitów. Przy 50 Hz pozostaje miejsce na diagnostykę. Przy 100 Hz przekraczają
budżet 2500 bitów/cykl, więc obecny profil odrzuca taką konfigurację. Samo
`cycle_ms=10` nie zapewni szybszego sterowania; potrzebna jest zweryfikowana zmiana
budżetu, protokołu lub transportu. Program nie zmienia bitrate.

Odpowiedzi dopasowuje się do adresu, komendy, DLC, sumy i terminu. Błędna ramka
nie usuwa oczekującego zapytania. Po timeoutcie obowiązuje dodatkowa kwarantanna
adresu. MKS nie ma numeru transakcji: nie da się niezawodnie odróżnić dowolnie
spóźnionej identycznej odpowiedzi od nowej. Dopuszczalną latencję trzeba zmierzyć;
wykryta utrata terminów powoduje błąd sterowania.

## Wykonanie i zatrzymanie

JTC lokalnie interpoluje trajektorię. `read()` i `write()` adaptera używają prób
blokady (`try_lock`), tablic sześciu osi i ostatnich wartości; nie wywołują CAN
i nie czekają na odpowiedzi. Wątek IO jest niezależny od executorów ROS.
Nie stanowi to potwierdzenia twardych gwarancji czasu rzeczywistego.

Każdy zestaw ma numer, źródło, czas utworzenia i termin ważności. Po sprawdzeniu
limitów pozycji, prędkości i przyspieszenia manager wysyła sześć F5. Dopiero komplet
odpowiedzi `F5 05 CRC` pozwala wysłać broadcast `4B 4B`. Brak ACK lub przeterminowana
próbka blokują SYNC i wywołują stop. SYNC nie ma potwierdzenia odbioru; zgodność
ruchu trzeba nadzorować pozycjami.

Próbka z `CanSystem` może zawierać pozycje i prędkości JTC. Wspólny kompilator
F5 wyznacza RPM z profilu próbki zamiast zawsze używać maksimum osi; kod `acc`
pozostaje osobnym parametrem napędu. Podgląd przyszłych zestawów 50 Hz oblicza
[planner kartezjański](cartesian_planning.md), który nie wysyła niczego do CAN.
Przygotowanie przed planowanym SYNC i odbiór śledzenia pozostają wymaganiami
późniejszego executora; zgodność ramek nie gwarantuje wspólnego czasu dojazdu.

Startowe progi oprogramowania: 60 ms dla pozycji i nowych zapisów kontrolera,
300 ms dla ENABLE/stall, 200 ms dla lease źródła. Nie wyznaczają one dopuszczalnej
drogi zatrzymania robota i wymagają odbioru na sprzęcie.

Przy fault manager jednorazowo próbuje wysłać `F7` do skonfigurowanych osi, bez
wyłączania momentu. Potem przestaje wysyłać wszystkie zapytania i komendy, żeby
diagnostyka nie podtrzymywała firmware watchdog. Ta polityka wymaga wcześniejszej
weryfikacji z obciążeniem i hamulcami (`stop_verified`). Sprzętowy obwód awaryjny
pozostaje niezależny. Reset lub rekonfiguracja kasują zatrzask, lecz nie aktywują
ruchu. Program nie ustawia automatycznie trybu, ENABLE, synchronizacji ani watchdog.

## Interfejs ruchu i konfiguracja

`motion_gate.py` udostępnia:

- `/robot/motion/follow_joint_trajectory`: proxy akcji `FollowJointTrajectory`,
  z feedbackiem, wynikiem i anulowaniem; jeden aktywny cel.
- `/robot/motion/jog_trajectory`: kompletne `JointTrajectory` z Servo,
  ze świeżym nagłówkiem i horyzontem najwyżej 200 ms.
- `/robot/motion/select_trajectory`, `/robot/motion/select_jog`,
  `/robot/motion/select_none`: `Trigger`. Wybór źródła przed aktywacją hardware.
- `/robot/motion/lease`: wewnętrzny `UInt64MultiArray` co 20 ms:
  `[źródło, monotonic_ns]`, gdzie 0 = brak, 1 = trajektoria, 2 = jog. Procesy
  działają na tym samym Linuxie. Opóźnione wiadomości nie odnawiają lease.
  Zanik gate zatrzymuje ruch niezależnie od zapisów JTC.
- `/robot/motion/stop`: `Trigger` publikatora diagnostyki, zatrzymanie z zatrzaskiem.

MoveIt wysyła akcje do proxy, Servo kieruje wyjściowy topic do jog. Nie wysyła się
równolegle komend bezpośrednio do JTC. ROS localhost ogranicza zasięg sieciowy,
ale nie autoryzuje lokalnych procesów. Jog po 200 ms bez wiadomości wycofuje lease.
Aktywna trajektoria bez feedbacku kontrolera przez 500 ms też wycofuje lease.
Poprawnie zakończona trajektoria utrzymuje ostatnią pozycję.

`robot_bringup/config/ros2_control.example.xacro` jest fragmentem rzeczywistego
URDF, bez geometrii. Wymaga pełnego YAML sześciu osi: adresów, przełożeń, kierunków,
offsetów, limitów, dopuszczalnego błędu śledzenia i kodu przyspieszenia MKS.
Kod MKS nie jest przyspieszeniem w rad/s². `null`, duplikaty i wartości nieskończone
są odrzucane. Adresy 1, 2 i 4 nie są automatycznie przypisywane do przegubów.

Po uzupełnieniu modelu i odbiorze sprzętu:

```bash
ros2 launch robot_bringup motion_control.launch.py \
  robot_description_file:=/ścieżka/do/pełnego.urdf \
  controllers_file:=/ścieżka/do/uzupełnionych_kontrolerów.yaml
./scripts/start_web_status.sh standalone_can:=false configuration_enabled:=false preview_enabled:=false
```

Launch pozostawia hardware i kontroler trajektorii nieaktywne. Po skanie,
wyborze źródła i potwierdzeniu gotowości wymagana jest jawna aktywacja. Cztery
flagi `motion_enabled`, `commissioning_verified`, `synchronization_verified`,
`stop_verified` muszą być świadomie ustawione po odbiorze SR_CLOSE/SR_vFOC,
bazowania, ENABLE, synchronizacji, ACK, hamulców i watchdog. Bieżące CR_vFOC
nie spełnia warunków. Model i konfiguracja MoveIt/Servo nie są wymyślane.

## Testy i odbiór

Emulator sześciu napędów i wstrzykiwany zegar sprawdzają pełny skan, echo, błędne
i spóźnione odpowiedzi, ACK/SYNC, utratę pozycji, komend, lease, BUS-OFF oraz USB.
Minuta symulowanego ruchu 50 Hz sprawdza budżet i wzrost RSS po rozgrzaniu.
To nie jest test płynności fizycznego robota ani minuta czasu ściennego.

Przed ruchem trzeba zmierzyć sześć osi: terminy ACK, rozrzut aktualizacji,
błąd śledzenia i fizyczne obciążenie/retransmisje. RTT jest mierzone zegarem
monotonicznym przy odczycie socketu w userspace; obejmuje USB i pobudkę IO
(docelowo 1 ms), nie jest czasem samego przewodu. Odbiór musi potwierdzić F7,
synchronizację i watchdog. Jeśli terminy nie są dotrzymane, cykl wykonawczy
należy przenieść do odpowiedniego lokalnego kontrolera.
