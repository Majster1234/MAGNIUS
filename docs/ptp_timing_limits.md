# Progi czasowe nadzoru PTP

Stan sprawdzony w źródłach 22.10.2026. To wartości bieżącego kodu i domyślnej
konfiguracji, bez odczytu parametrów działającego procesu i napędów. Przegląd
nie obejmował uruchomienia ruchu, zmiany flag wykonania ani nastaw.

Późniejszy odbiór sprzętowy tego dnia obejmował pięć prób i izolowane warianty
limitów SYNC 2,5 ms / końca diagnostyki 19 ms. Domyślne wartości opisane poniżej
pozostały bez zmian; [wyniki i ograniczenia](motion_hardware_results.md#seria-limitów--2026-10-22).

## Cykl i odpowiedzi CAN

Czasy faz liczone są od początku każdego cyklu zegara monotonicznego.
Deadline odpowiedzi jest wyłączny: odpowiedź ze znacznikiem równym terminowi
jest już spóźniona. Wymagany jest również znacznik nie wcześniejszy od zapytania.

| Sprawdzenie | Próg / okres | Znaczenie podczas testu |
| --- | --- | --- |
| Cykl `streamed` | 20 ms (50 Hz) | Kolejne SYNC według bezwzględnych terminów; brak nadrabiania próbek. `native_ptp` ma jeden zestaw F5 i jeden SYNC. |
| F5 i komplet ACK | Okno [0, 8) ms | Deadline każdej odpowiedzi: minimum z wysłanie + `response_timeout` oraz końca okna 8 ms. Przy domyślnym timeout 20 ms decyduje koniec okna. |
| Spóźnienie SYNC | Do +1 ms włącznie | `now > due + 1 ms` przerywa program; kontrola także bezpośrednio przed zapisem SYNC. Nie ma nadania przed `due`. |
| Zapytania enkoderów | Co 40 ms na oś; nadawanie w [8, 12) ms | Dwie naprzemienne grupy uczestników. Odpowiedź musi przyjść przed 14 ms danego cyklu, więc ma faktycznie około 2–6 ms. |
| Zapytania diagnostyczne | Nadawanie w [14, 16) ms | Odpowiedź przed 18 ms, faktycznie około 2–4 ms. Maksymalnie 8 transakcji na cykl. |
| Rezerwa cyklu | [18, 20) ms | Bez nowych zapytań; następny SYNC na granicy cyklu. |
| Zwykły timeout odpowiedzi | Domyślnie 20 ms | Podczas faz jest skracany jak wyżej. W uzbrajaniu PTP obowiązuje osobne 100 ms. |
| Reakcja na timeout | Pierwszy timeout transakcji podczas PTP | Natychmiastowy błąd i STOP w obsługującej go iteracji IO. Próg 3 kolejnych timeoutów służy oznaczaniu niedostępności urządzenia, nie opóźnia przerwania PTP. |
| Budżet CAN | Maks. 40% na każde 20 ms | Przy 500 kbit/s: 4000 bitów modelowego kosztu na cykl, 8 ms równoważnego czasu magistrali. Obowiązuje minimum z 40% i skonfigurowanego limitu (domyślnie 50%). Brak budżetu na F5/SYNC przerywa ruch; diagnostyka może zostać odłożona. |

Okna zapytań i odpowiedzi obowiązują po wejściu w fazowy harmonogram,
także podczas końcowego potwierdzania pozycji. Uzbrajanie i odświeżenie przed
startem korzystają z odrębnych warunków poniżej.

Źródła: [stałe harmonogramu](../ros2_ws/src/mks_servo_can/include/mks_servo_can/motion_timing.hpp),
[wykonawca PTP](../ros2_ws/src/mks_servo_can/src/program_bus.cpp),
[transakcje i budżet](../ros2_ws/src/mks_servo_can/src/bus_manager.cpp).

## Świeżość i potwierdzenie ruchu

| Sprawdzenie | Próg | Znaczenie |
| --- | --- | --- |
| Pozycja enkodera | Wiek **<60 ms** domyślnie | Osobno od odebrania odpowiedzi i od wysłania jej zapytania. Równe 60 ms lub czas z przyszłości są niepoprawne; utrata świeżości przerywa PTP. Odczyt co 40 ms daje nominalnie 20 ms zapasu od czasu zapytania. |
| ENABLE | Wiek **<300 ms**, odczyt nominalnie co 100 ms | Wymagane ENABLE=1. Świeży odczyt ENABLE=0 przerywa program bez dodatkowej zwłoki. |
| Stall | Wiek **<300 ms**, odczyt nominalnie co 100 ms | Wymagany brak stall. Świeży stall przerywa program bez dodatkowej zwłoki. |
| Prędkość wału | Wiek **<150 ms** | Wymagana prędkość 0 przy przejęciu z JTC i potwierdzaniu końca. Odczyt podczas faz nominalnie co 50 ms. Nie jest to osobny próg świeżości stale przerywający cały ruch. |
| Stan pracy silnika | Odczyt nominalnie co 50 ms | Brak osobnego runtime progu wieku dla tej próbki. Timeout jej zapytania przerywa PTP jak każdy inny timeout. Przed startem wymagany stan postoju. |
| Błąd śledzenia | Każdy nowy pomiar; **bez czasowego debounce** | Pomiar porównywany z otoczką referencji między wysłaniem zapytania a odebraniem odpowiedzi. Tolerancja przestrzenna z profilu osi; jej przekroczenie od razu przerywa ruch. |
| Potwierdzanie końca | Co najmniej **100 ms** | Cel w 1/4 tolerancji śledzenia i świeża prędkość 0. Pozycja musi pochodzić z zapytania wysłanego nie wcześniej niż nominalny koniec programu. Sukces sprawdzany przy nowych pomiarach; rzeczywisty czas jest skwantowany ich napływem. Utrata warunku zeruje licznik. |
| Niepotwierdzony koniec | **>1 s** po nominalnym końcu | Błąd, jeżeli nadal brak sukcesu. Te 100 ms zawiera się w dodatkowej sekundzie. |

Świeżość sprawdzana jest w każdej iteracji wykonawcy, a śledzenie po nowym
pomiarze. Niezależne progi działają jednocześnie: krótki timeout zapytania może
przerwać ruch wcześniej niż limit wieku ostatniego poprawnego pomiaru.

Źródła: [świeżość próbek i wartości domyślne](../ros2_ws/src/mks_servo_can/include/mks_servo_can/bus_manager.hpp),
[nadzór, śledzenie i zakończenie](../ros2_ws/src/mks_servo_can/src/program_bus.cpp).

## Przed startem

| Sprawdzenie | Warunek czasowy |
| --- | --- |
| Aktualność stanu dla planera fizycznego PTP | Wiek wiadomości stanu **<500 ms**; wiek pozycji dostępnej, nieblokowanej osi powiększony o wiek wiadomości **<250 ms**. To bramki planowania, nie progi wykonawcy CAN. |
| Aktualność ustawień wykonania | **<1500 ms** przy przygotowaniu i uruchomieniu programu. Nie jest stale sprawdzana jako timer podczas wykonania. |
| Wiek żądania `prepare_program` | **≤3 s**; brak znacznika lub znacznik z przyszłości odrzucony. |
| Uzbrajanie i odświeżenie pozycji | Wspólny deadline **3 s** od przyjęcia wykonania; przy `now >= deadline` błąd. Przejście do odświeżania nie daje kolejnych 3 s. |
| Pojedyncza transakcja uzbrajania | Odpowiedź **przed 100 ms** od zapytania; inne zabezpieczenia, w tym świeżość pozycji, nadal działają. |
| Watchdog odczytany z napędu (`0x98`) | Akceptowane **60–1000 ms włącznie**. Uzbrajanie odczytuje i weryfikuje wartość. Ręczne przygotowanie ENABLE zapisuje 300 ms, lecz aktualnej wartości sprzętu nie odczytano w tym przeglądzie. |
| Czas trajektorii | **0 < T ≤200 s**. Dla `streamed` koniec na siatce 20 ms z tolerancją numeryczną 1e-8 s (10 ns). |
| Zgodność modelu `native_ptp` | Błąd modelowego czasu końca względem planu oraz różnice czasów ramp i końca między osiami **≤20 ms**. To warunek wyboru trybu; nie dopuszczalne spóźnienie runtime SYNC. |

Źródła: [bramki planera](../ros2_ws/src/robot_motion_planner/src/node.cpp),
[wiek żądania ROS](../ros2_ws/src/robot_can_diagnostics/src/program_api.cpp),
[przygotowanie wykonawcy](../ros2_ws/src/mks_servo_can/src/program_bus.cpp),
[walidacja czasu i dobór trybu](../ros2_ws/src/mks_servo_can/src/motion_executor.cpp),
[ENABLE i watchdog](../ros2_ws/src/mks_servo_can/src/manual_motor_controller.cpp).

## STOP i interpretacja pomiarów

Wszystkie F7 do CAN 1–6 poprzedzają wszystkie F6 z RPM=0. Na odpowiedzi F7
przeznaczono `response_timeout` (domyślnie 20 ms). F6 rusza wcześniej, jeśli
odpowiedzą wszyscy wymagani obecni uczestnicy. Odpowiedzi F6 mają najwyżej kolejne
20 ms, a wspólny deadline to **40 ms od początku sekwencji**. Odpowiedź dokładnie
na deadline jest spóźniona. STOP omija zwykły budżet, blokuje dalsze SYNC i działa
również przy zatrzaśniętym błędzie. Po zakończeniu procedury dalsze nadawanie ustaje.

To budżet logiki komunikacyjnej. Przejścia wykonuje najbliższa iteracja IO;
opóźnienie procesu lub wywołania transportu może przesunąć ich rzeczywistą obsługę.
**40 ms nie określa czasu zatrzymania mechanicznego ani skasowania bufora.** ACK
F7/F6 potwierdzają odpowiedzi na rozkazy; skuteczność sekwencji wymaga odbioru
sprzętowego. BUS-OFF, błąd transportu i raport F5 błędu/krańcówki nie mają
dodatkowego timera zwłoki; reakcja następuje przy ich obsłużeniu. Awaria transportu
może uniemożliwić dostarczenie STOP.

IO ma nominalne pobudki co **1 ms**; to nie gwarancja maksymalnego czasu reakcji.
Przerwa IO **>5 ms** i wykonanie iteracji **>2 ms** tworzą zdarzenia śladu,
same nie uruchamiają STOP. Limit komend **60 ms** i lease źródła **200 ms**
dotyczą ścieżki JTC/`ros2_control`, nie lokalnego wykonania przygotowanego PTP.
Timer statusu akcji ROS **50 ms** jest okresem raportowania, nie nadzoru CAN.

Źródła: [STOP i błędy magistrali](../ros2_ws/src/mks_servo_can/src/bus_manager.cpp),
[pętla IO](../ros2_ws/src/mks_servo_can/src/bus_runtime.cpp),
[raportowanie akcji](../ros2_ws/src/robot_can_diagnostics/src/program_api.cpp).

Do testów granicznych stosować wartości tuż poniżej, równe i tuż powyżej progów.
Sprawdzać `ack_margin_ms`, `sync_lateness_ms`, timeouty, zdarzenia IO i wynik STOP.
Margines ACK jest liczony do SYNC, a deadline ACK przypada na koniec okna 8 ms;
to różne punkty odniesienia. Czas zapisu SocketCAN i odbioru w procesie nie
wyznacza dokładnej chwili transmisji ani próbkowania w napędzie.
