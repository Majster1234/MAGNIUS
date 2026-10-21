# Fizyczne wykonanie PTP

Pierwszy etap wykonuje pojedynczy PTP. LIN i łączenie odcinków pozostają kolejnym
etapem. Wykonawca C++ posiada cały program i kolejne ramki w pamięci; nie oczekuje
na taktowanie z przeglądarki, HTTP ani kolejny cel akcji ROS.

## Obsługa z panelu

1. Skonfiguruj i przygotuj przez ENABLE obecne osie, z potwierdzonym odniesieniem
   przegubu. Samo wykrycie napędu nie wystarcza do wykonania.
2. W planowaniu ręcznym ustaw cel i oblicz PTP. LIN pozostaje symulacją.
3. Wybierz „Przygotuj PTP robota”. Planner obliczy odrębny fizyczny plan od
   bieżących pomiarów, z prędkością i przyspieszeniem wybranego podglądu.
4. Sprawdź listę osi, ich start/cel, czas i tryb. „Podgląd przygotowanego PTP”
   pokazuje tę trajektorię; pominięte osie pozostają w założonej pozycji startowej.
5. Zaznacz zgodę na fizyczny ruch i wybierz „Wykonaj PTP”. Powtórzenie żądania
   z tym samym kluczem nie uruchamia drugiego przejazdu.

Status i „STOP robota” są dostępne w widoku ręcznym także poza planowaniem,
po przeładowaniu strony i ponownym połączeniu. STOP działa niezależnie od przycisku
zatrzymania symulacji. Zatwierdzony program trwa po blur, zamknięciu strony i utracie
LAN. Awaria procesu właściciela CAN lub magistrali przerywa ruch, bez wznowienia.
Launch panelu ponawia sam bridge po awarii; nie zamyka z tego powodu właściciela CAN.
Zamknięcie całego launch/usługi nadal zatrzymuje jego procesy.

## Ruch z niepełnym zestawem

„Ustawienia systemowe → Zezwalaj na ruch bez wszystkich silników” zapisuje
`allow_missing_motion`, domyślnie `false`. Jest niezależne od
`ignore_missing_drivers`, które dotyczy wyłącznie planowania/symulacji.
Stare pliki ustawień otrzymują brakującą flagę jako `false`; pozostałe ustawienia
są zachowywane przy zapisie jednego przełącznika.

Przy włączonej opcji właściciel CAN ustala maskę rzeczywiście obecnych napędów.
Nie wysyła F5 do pominiętych osi i zbiera ACK tylko od uczestników; wymaga co
najmniej jednego uczestnika. Obecnej osi bez ENABLE, kalibracji lub świeżego stanu
nie można pominąć tą opcją. Maska nie zmienia się podczas programu. Utrata osi
uczestniczącej wywołuje STOP; ponowne podłączenie nie dołącza jej do programu.

Dla brakujących osi planner używa jawnie oznaczonych założeń startu zapisanych
w podglądzie. Nie są one pomiarami. Program może zakończyć się po osiągnięciu
celów uczestniczących przegubów, mimo że TCP nie osiągnie obliczonej pozycji.
Panel pokazuje ten stan stale. Pole `tcp_verified` oznacza komplet uczestniczących
osi odniesionych do modelu, nie zewnętrzny pomiar rzeczywistego TCP.
Zmiana ustawień unieważnia przygotowany program; trwający zatwierdzony program
zachowuje swoją politykę do końca.

## Wykonawca i protokół

`MotionExecutor` jest częścią istniejącego `CanBusManager/BusRuntime`; nie otwiera
nowego socketu. Program ma niezmienne q/v/a i czas, model, rewizję konfiguracji,
rewizję ustawień, maskę osi oraz skompilowane polecenia. SI obowiązuje w ROS/C++,
stopnie w panelu. Limit: 10000 punktów i 200 s; jeden aktywny program, bez kolejki
kolejnych zleceń od operatora i bez pomijania próbek.

- `native_ptp`: jedna F5 na uczestnika, wspólny SYNC. Dobór dyskretnych RPM/acc
  uwzględnia mikrokroki, przełożenie, czas i fazy trapezów. Wymaga zgodności modelu
  profilu z referencją w 1/4 skonfigurowanej tolerancji śledzenia oraz rozbieżności
  faz/czasu nie większej niż cykl. To model do odbioru sprzętowego, nie dowód dynamiki
  obciążonego robota.
- `streamed`: pełny bufor próbek 20 ms, z osobną barierą F5/ACK/SYNC dla każdego
  zestawu. Kwantyzacja RPM jest ograniczona tolerancją; zerowa prędkość referencji
  nie staje się przypadkową komendą STOP. Walidacja obejmuje otoczki Bernsteina
  pozycji/prędkości/przyspieszenia interpolacji quintic, również między węzłami.
- Wybór trybu zachodzi przed zatwierdzeniem. Nie ma przełączenia po częściowym starcie.
- Przed pierwszym SYNC sprawdzane są postój, tryb CAN, watchdog i zgodność mikrokroków;
  ustawiane i odczytywane jest `4A=1`. Każdy uczestnik musi zwrócić `F5 05`.
  Spóźnione `F5 02` poprzedniego celu nie zużywa oczekującego ACK kolejnej próbki.
  Zachowujemy raporty krańcówek; budżet uwzględnia opcjonalną drugą odpowiedź F5.
  Sześć osi programu przy 50 Hz to konserwatywnie 3165 bitów/cykl przed diagnostyką.
- Terminy wynikają z zegara monotonicznego. IO używa bezwzględnych pobudek.
  Przekroczenie terminu SYNC o ponad 1 ms, brak ACK lub budżetu przerywa program;
  nie ma nadrabiania opóźnionych poleceń ani powtarzania SYNC w ciemno.
- Nadzór porównuje enkodery z bieżącą referencją, nie z końcowym celem od chwili startu.
  Zakończenie wymaga pozycji w 1/4 tolerancji śledzenia i prędkości wału 0 przez
  co najmniej 100 ms; po dodatkowej sekundzie bez potwierdzenia następuje błąd.
- Awaria wysyła F7 do skonfigurowanych osi i zatrzaskuje błąd. Kasowanie błędu usuwa
  potwierdzenie opróżnienia poleceń w napędach. Kolejny start wymaga ponownego odbioru
  tego warunku; samo programowe „reset” nie jest potwierdzeniem opróżnienia sprzętu.

Bramka CAN odrzuca konkurencyjne źródła podczas programu. W trybie ros2_control
bridge dezaktywuje JTC w postoju; CanSystem nie nadpisuje programu starymi nastawami.
Sprzęt nie jest przez to dezaktywowany. Po przejęciu pozostaje tryb programu:
ponowna aktywacja JTC wymaga jawnego cyklu resetu/aktywacji sprzętu z aktualnymi
pozycjami. Powrót do ręcznego JOG wymaga ponownego przygotowania ENABLE, które
przywraca `4A=0`. Kolejny PTP nie wymaga ponownego przełączania wszystkich ENABLE.

## Odbiór i parametry uruchomienia

`robot_bringup/config/execution.example.yaml` jest wyłączonym szablonem ROS.
Argument `execution_parameters:=/pełna/ścieżka/do/pliku.yaml` obsługują
`web_status.launch.py` i `motion_control.launch.py`. Nie zmieniamy bitrate 500000.

Przed fizycznym wykonaniem trzeba podać i potwierdzić:

- `execution_enabled`, `execution_configuration_revision` zgodną z aktywnym profilem;
- `execution_stop_verified`, `execution_sync_verified`,
  `execution_staged_reset_verified` po odbiorze tych zachowań na napędach;
- sześć `execution_tracking_error_rad` oraz `execution_acceleration_codes` (1–255).
  W standalone nieuzupełnione wartości blokują wykonanie. CanSystem może zachować
  wcześniej skonfigurowane poprawne tolerancje/kody z URDF.

`-1` w szablonie oznacza brak potwierdzenia, nie parametr fizyczny. Nie wolno
ustawiać flag odbioru w celu obejścia brakujących pomiarów. Przełącznik niepełnego
zestawu nie wyłącza limitów i zatrzymywania uczestniczących osi. Aktualizacja
profilu wymaga zgodnego nowego odbioru rewizji.

## API i przyszłe odcinki

HTTP: `POST /api/v1/motion/prepare` z `plan_id` zwraca zadanie przygotowania;
`GET /motion/status` zwraca jego stan i stan lokalnego programu;
`GET /motion/prepared` pobiera odrębny podgląd; `POST /motion/execute` przyjmuje
`program_id` i `execution_key`; `POST /motion/stop` zatrzymuje. Wszystkie ścieżki
poza pierwszą również mają prefiks `/api/v1`. Zajętość daje 409, niezgodny program
422, brak usług 503. Żądania zachowują istniejące ograniczenia LAN/origin.

ROS: akcje `/robot/planning/compute_physical_ptp`, `/robot/motion/execute_program`;
usługi `/robot/motion/prepare_program`, `/robot/motion/program_status` i istniejący
`/robot/motion/stop`; polityka pochodzi z `/robot/motion/settings` właściciela
konfiguracji. Właściciel CAN ponownie sprawdza rewizje, start i maskę przed ruchem.

Przyszły LIN i sekwencje skorzystają z tego samego bufora czasu, próbek i barier.
Planner musi wcześniej policzyć całą sekwencję i jej połączenia. Promień 0 oznacza
punkt dokładny z zatrzymaniem, dodatni jawnie dopuszcza zaokrąglenie. Blending i
fizyczny LIN nie są jeszcze udostępnione; osobne kolejne akcje PTP nie tworzą
płynnego połączenia.

## Sprawdzenia

2026-10-21: emulator protokołu, testy kompilatora i planera, akcja ROS na syntetycznych
pomiarach w osobnej domenie oraz przeglądarka z atrapami API. Szczegóły pomiarów
oprogramowania: [benchmark_results.md](benchmark_results.md).

2026-10-21, późniejsza próba sprzętowa na rozsprzęglonym CAN 1: F5 czekało na
SYNC, ale po F7 z ACK=1 późniejszy SYNC uruchomił wcześniej przygotowany cel.
Test przerwano przy przesunięciu około 0,154°, wyłączono wszystkie napędy
i przywrócono usługę. Etapów 2 i 4 silników nie wykonano. Wykonawca pozostaje
wyłączony: ACK F7 nie potwierdza opróżnienia bufora. Nie wysyłać kolejnego SYNC
do czasu potwierdzenia skutecznej sekwencji kasowania/zastępowania rozkazu.
Szczegóły i ślady: [motion_hardware_results.md](motion_hardware_results.md).
Płynność fizycznego robota pozostaje niepotwierdzona.

Źródło protokołu i zachowania raportów: [instrukcja MKS CAN V1.0.9, §5.2.12 i §11–12](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).
