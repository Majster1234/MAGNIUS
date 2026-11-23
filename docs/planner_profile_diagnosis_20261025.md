# Konfiguracja planera i nieprawidłowy PTP — 25.10.2026

## Wynik sprawdzenia

Nie wykryto innego przełożenia niż **1** w aktywnej konfiguracji, kontekście
składanym dla planera ani profilu właściciela CAN. Wszystkie sześć osi ma kierunek
**+1** i offset **0°**. Rewizja wynosi `f35e80a97d4a4bcdafbd231c6eabc92a`.
Nie zmieniono konfiguracji, nie przygotowano ani nie uruchomiono nowego programu,
nie wysłano komend ruchu/zerowania/ENABLE i nie restartowano usług.

## Dowody i zakres

| Miejsce | Sprawdzenie | Wynik |
| --- | --- | --- |
| HTTP `GET /api/v1/configuration` | Aktywny dokument, zakończona operacja, kalibracja | Przełożenia 1; kierunki +1; offsety 0; flagi nieważności false |
| ROS `/robot/configuration/active` | Odczyt wiadomości `RobotConfiguration` | Te same wartości i rewizja |
| Właściciel CAN `/robot/configuration/control` | Wyłącznie faza `STATUS`, bez transakcji zapisu | Te same wartości; `finished`, `locked=false` |
| ROS `/robot/manual/state`, HTTP manual | Pozycja wału i przegubu | Równe dla dostępnych J1/J2/J3/J5, zgodnie z profilem 1:1 / +1 / 0 |
| Kontekst planowania HTTP | Rewizja i model | Zgodne z konfiguracją i URDF |
| Builder `PlanningClient.context` | Odtworzenie offline i serializacja `ComputePhysicalMotion.Goal` | Sześć przełożeń 1; kierunki +1; offsety 0 |
| Zachowany podgląd PTP | Końcowe wartości `mks_samples.position_ticks` | Przeliczenie dla 1:1, błąd kwantyzacji <0,5 impulsu |
| Działające biblioteki | `/proc/.../maps` diagnostyki i planera | `install/gear-ratio-continuity-ros/.../libmks_servo_can.so` |

Subskrybentem aktywnej konfiguracji jest działający `robot_motion_planner`.
Sam planner kontroluje model i rewizję; profile osi otrzymuje w kontekście akcji.
Builder kopiuje `gear_ratio` bez zmiany, a kąty/offsety/limity przelicza z ° na rad.
Fizyczne przygotowanie korzysta z tego samego buildera. Odtworzenie offline nie
wysłało nowego celu do działającego planera — użyło zarchiwizowanych danych i
wygenerowanego typu ROS, bez uruchamiania klientów akcji.

Wykonawca nie odtwarza bezpośrednio podglądowych ramek MKS. `ProgramApi` przekazuje
q/v/a, `CanBusManager::prepare_program` kompiluje je z bieżącym `config_.axes`,
a `schedule_program` ponownie korzysta z tego profilu przy wysyłce. COMMIT
konfiguracji aktualizuje te osie. Nie znaleziono osobnej kopii przełożenia 3 lub 10
używanej w tej ścieżce PTP. Stary identyfikator rewizji w YAML jest ignorowany;
plik produkcyjny nie zawiera zastępujących przełożeń.

## Zachowany plan i historia wałów

Podgląd `5683c82f2dfd0771a8f15dcfd3bd39c84c969950da4affeb3725c3d387aa1cc3`
ma tę samą rewizję oraz parametry PTP 6°/s i 50°/s². Czas podglądu to około
11,88 s. Metadane przygotowania fizycznego odnoszą się do tego podglądu;
znacznik w identyfikatorze programu odpowiada **18:49:16 CET**.

| Oś | Start podglądu [°] | Cel [°] | Końcowe impulsy F5 podglądu |
| --- | ---: | ---: | ---: |
| J1 | 84,221 | 127,046 | 5782 |
| J2 | 3,054 | −20,728 | −943 |
| J3 | −183,208 | −125,749 | −5723 |
| J4 | 0 | 0 | 0 |
| J5 | 9,998 | 9,998 | 455 |
| J6 | 0 | 0 | 0 |

Wszystkie końcowe wartości odpowiadają `round(q_rad × 16384 / (2π))`.
Ujemny cel J2 jest zapisany jako int24 ze znakiem. Instrukcja producenta
potwierdza bezwzględne współrzędne F5 w układzie wieloobrotowego enkodera 31,
16384 jednostki/obrót; wymienia też wymagane ustalenie zera przed użyciem F5:
[instrukcja MKS CAN V1.0.9, §11.4](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).
Nie ustalono w tej turze stanu tego odniesienia wewnątrz napędów.

Zapis wałów z tej sesji zawiera ruch około **18:49:25–18:49:35**: J2 rośnie
z około 3° do 411°, J3 maleje do około −324°, a J5 rośnie z około 10° do ponad
500°. J5 w zachowanym planie ma stały cel, więc zwykła zmiana mnożnika przełożenia
nie wyjaśnia takiej rozbieżności. Wykres i dane są w
`log/planner-profile-diagnosis-20261025/ptp-plan-vs-shafts.png` oraz
`ptp-trace-history.json`.

**Ograniczenie:** nie zachował się zapis TX F5 ani potwierdzony czas START tej
próby. Historia wałów nie identyfikuje źródła każdej komendy. Korelacja czasowa
z przygotowaniem nie zastępuje rejestracji wykonania. Końcowe próbki podglądu
nie są dowodem zawartości ramek rzeczywiście wysłanych na magistralę.

Aktualny wykonawca zgłasza `idle`, pusty `program_id`, `motion_type=null`.
Most nadal przechowuje starsze metadane przygotowania `ready`, ale GET
`/api/v1/motion/prepared` odrzuca je jako nieaktualne, a panel nie ma warunków
wykonania takiego programu. Nie jest to dowód starego aktywnego przełożenia.

## Stan bieżący i dalsza diagnoza

W odczycie około 18:52 CET J1=250,03°, J2=406,56°, J3=−288,94°, J5=512,16°.
Są poza odpowiednimi limitami: J1 [−171,171], J2 [−200,30], J3 [−196,57],
J5 [−100,100]. Odtworzony builder odrzuca taki start z komunikatem
`J1: pozycja lub profil poza limitami`. Nie ustalono, czy położenia są skutkiem
zgłoszonego incydentu, czy innych operacji operatora; nie użyto ich do zmiany
odniesienia lub wyliczania nowego offsetu.

Wniosek dotyczy konfiguracji: **nie potwierdzono błędu dostarczania przełożeń**.
Przyczyna nieprawidłowego ruchu wymaga porównania rzeczywistych TX F5 i RX 31,
odniesienia współrzędnych napędów oraz momentu rozpoczęcia programu. Ewentualna
próba ruchu wymaga osobnego ustalenia osi, kierunku, prędkości, zakresu i STOP.
Nie odtwarzać zapisanego programu automatycznie w ramach diagnostyki.

Artefakty odczytów i odtworzenia offline: `log/planner-profile-diagnosis-20261025/`.
Brak zmian kodu, nowego buildu i testu sprzętowego ruchu.
