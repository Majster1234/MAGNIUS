# Synchroniczny streaming PTP/LIN — 28.10.2026

Tryb jest opcjonalnym rozszerzeniem obecnego wykonawcy PTP/LIN panelu.
Trajektoria q/v/a pochodzi z planowania MoveIt/TOTG i zachowuje swoją siatkę czasu.
JTC/FollowJointTrajectory, JOG TCP i JOG pojedynczej osi nie korzystają z tego trybu.
**Odbiór płynności fizycznej nie został wykonany. Dawne 6/6 zgód na ruch nie obowiązuje
w tym etapie.** Zbudowanie i testy na atrapach nie potwierdzają dynamiki firmware.

29.10 wykonano osobne próby odzyskiwania z rozdzieleniem skoku ENABLE od
postoju. Pięć osi zaliczyło nadpisanie znanego bufora; J4 nie zaliczyła kontroli
dodatniej. Odczyty mają zapas 20% (24 ms). Na późniejsze jawne polecenie operatora
automatyczne zdjęcie blokady SYNC włączono mimo J4 i wdrożono bez nowych testów;
nie zmienia to wyniku odbioru. [Wyniki, czasy i wdrożenie](sync_recovery_timing_2026-10-29.md).

## Konfiguracja i API

W **Konfiguracja → Ustawienia systemowe → Wykonanie i zabezpieczenia**
wybór „Sposób wysyłania PTP/LIN” udostępnia „Streaming” i „Synchroniczny”.
Zmianę zatwierdza przycisk „Zapisz ustawienia do ROS”. Snapshot v1 zawiera `motion.ptp_lin_dispatch_mode`:
`streamed` (domyślnie) albo `synchronized`. Brak pola w starym snapshocie oznacza
`streamed`; null, wartości logiczne i nieznane napisy są odrzucane.

Pole `ptp_lin_dispatch_mode` przechodzi przez RobotConfiguration oraz transakcję
PREPARE/COMMIT/ROLLBACK/FINISH. Właściciel CAN potwierdza faktycznie przyjętą wartość.
Zmiana wymaga nieaktywnego sterowania i potwierdzonego postoju obecnych osi:
stabilnej pozycji z ENABLE=0 albo stabilnej pozycji ze świeżymi 0 RPM i F1=1.
Warunki są sprawdzane ponownie przed COMMIT. Sama zmiana trybu nie zmienia
kalibracji, przełożenia ani offsetu, ale unieważnia przygotowany program także
po rollbacku. Zapis konfiguracji nie wysyła 4A ani 8C do silników.

Istniejące adresy HTTP i ROS pozostają bez zmian. `mode=streamed` zachowuje znaczenie
wykonawcy strumieniowego. Status otrzymuje:

- `dispatch_mode`: tryb związany z konkretnym przygotowanym programem;
- `sync_cycle`: indeks ostatniego zatwierdzonego zestawu, liczony od zera;
- `sync_state`: preparing, waiting_ack, waiting_tx_confirmation, activated,
  restoring, restored lub fault;
- `sync_ack_mask`: bity J1–J6 aktualnego zestawu;
- `sync_due_monotonic_ns`, `sync_sent_monotonic_ns`, `sync_confirmed_monotonic_ns`;
- `sync_lateness_ms`: opóźnienie obserwowanego potwierdzenia nadania względem terminu.

Monotoniczne znaczniki są lokalne dla hosta, nie są czasem ROS/UTC ani znacznikami
zegarów napędów. Przed pierwszym potwierdzeniem indeks i opóźnienie mają zero;
o aktywacji rozstrzyga stan i niezerowy znacznik potwierdzenia.

## Cykl wykonania

Obsługiwany profil to SERVO42D CAN firmware 1.0.8/1.0.9. Wersja patch jest
odczytywana z odpowiedzi 40. Uzbrojenie sprawdza dotychczasowe warunki oraz
ustawia i odczytuje `4A=1`, `8C=01/00`. Odpowiedzi komend są włączone, spontaniczne
raporty zakończenia wyłączone. Samo przygotowanie programu nie wysyła F5 ani SYNC.

Pierwszy termin T0 jest wyznaczany 20 ms po uzbrojeniu. Pierwszy zestaw musi być
przygotowany co najmniej 10 ms przed nim. Kolejne terminy Tk=T0+k·10 ms pozostają
stałe. Po potwierdzeniu nadania SYNC k wykonawca przygotowuje zestaw k+1.

Wszystkie osie korzystają z referencji interpolowanej dla wspólnej fazy k·10 ms.
Cel F5 pochodzi z rzeczywistej interpolacji dla wspólnego horyzontu, domyślnie
85 ms dalej; horyzont jest skracany do końca trajektorii, najbliższego nawrotu
lub granicy zaplanowanego postoju. Pierwiastki prędkości kwintycznych przedziałów
są wyznaczane przed ruchem, a nie szukane w cyklu IO. Uchyb regulacji odnosi
pomiar do referencji z czasu RX tej osi. FF, Kp, dithering, mikrokrok, offset,
przełożenie i limity pozostają obowiązujące.

Wykonawca rezerwuje cały zestaw F5, odpowiedzi F5 oraz pojedynczy broadcast 4B.
Wymaga poprawnego `F5 05` każdego uczestnika ze znacznikiem wcześniejszym od Tk.
ACK przyjęcia nie potwierdza ruchu ani postoju. W Tk można wysłać jeden SYNC,
ze spóźnieniem najwyżej 2,5 ms. Brak kompletu ACK nie przesuwa terminu. Nie ma
ponawiania SYNC, nadrabiania zestawów ani przejścia awaryjnego na zwykły streaming.

SocketCAN rozróżnia własne potwierdzenie nadania MSG_CONFIRM od odpowiedzi napędu.
Do managera trafia tylko własne echo SYNC; pozostałe własne echa nie zmieniają
pomiarów ani oczekujących odpowiedzi. Dopiero potwierdzenie SYNC zatwierdza
referencję, dithering i numer zestawu. Jest to lokalne potwierdzenie transportu,
nie dowód odbioru przez wszystkie napędy ani pomiar jednoczesności fizycznego ruchu.
Moment odczytu przez wątek RX może zawierać dodatkowe opóźnienie; sprawdzanie
2,5 ms jest przez to konserwatywne. Zapis do socketu sam nie wystarcza.

Nowe zapytania diagnostyczne są wstrzymywane co najmniej 4 ms przed najwcześniejszą
kolejną wysyłką F5. RX działa niezależnie. Koszt konserwatywny dla sześciu osi
przy 1 Mbit/s: dotychczasowe 26,58% + ACK 5,10% + SYNC 0,75% = **32,43%**.
To rezerwacja modelowa, nie pomiar przewodu. Nadal obowiązuje 40% dla robota,
20 punktów rezerwy i 60% łącznie; sześć osi przy 500 kbit/s jest odrzucane.

## STOP, zakończenie i ograniczenia

Niekompletny/niewłaściwy/spóźniony ACK, błąd częściowej wysyłki, nieoczekiwany
SYNC innego nadawcy, utrata potwierdzenia nadania i spóźnienie >2,5 ms zatrzaskują
awarię. STOP blokuje nowe F5/SYNC, unieważnia zestaw i uruchamia F7,
udokumentowane F5 STOP z zerowymi polami oraz odpowiednią politykę ENABLE.
Już przekazanego do
transportu SYNC nie można uznać za cofnięty; zatrzymanie wymaga pomiarów.

Globalny STOP i wykrycie pozostawionego `4A=1` uruchamiają maszynę odzyskiwania
opisaną w [procedurze bufora SYNC](sync_buffer_recovery_proposal.md). Nie wysyła
ona `4B`: po potwierdzonym `ENABLE=0` i postoju ustawia `8C=01/00`, przy `4A=1`
wysyła neutralne F5 STOP z wymaganym `05`. Po pozytywnym odbiorze sprzętowym
odtwarza i odczytuje `4A=0` oraz `8C=00/00`. Zwykły reset CAN nie wznawia
starego programu.

Automatyczne zdjęcie blokady po stwierdzonym `4A=1` wymaga pozytywnego odbioru
sprzętowego i `execution_staged_reset_verified=true`. Do tego czasu sekwencja
kończy się `recovery_failed` przed zmianą `4A=0`, mimo poprawnego wyniku emulatora.
Pozostawienie `SYNC=1` uniemożliwia ukrycie blokady przez restart procesu. Odczyty
identyfikacji, pozycji, RPM, ENABLE, stall, F1, SYNC i watchdogu pozostają
aktywne również po niepowodzeniu; stan online nie oznacza uprawnienia do ruchu.

Odbiór z 29.10.2026 pozostaje **niezaliczony**: ACK `05` przy ENABLE=0
potwierdzono na sześciu osiach, ale pierwszy powrót ENABLE J1 przekroczył
tolerancję postoju (zmiana odczytu 3,823° >0,5°), zanim wysłano jakikolwiek 4B.
Szczegóły i domknięcie: [raport sprzętowy odzyskiwania](sync_recovery_hardware_2026-10-29.md).

Przy normalnym zakończeniu ostatni zestaw również przechodzi ACK i SYNC.
Przywracanie trybu wymaga dwóch kontroli pozycji odległych o co najmniej 100 ms,
świeżych 0 RPM i F1=1 po ostatnim SYNC oraz kolejnych pomiarów postoju oddalonych
co najmniej o 100 ms. Następnie każda oś otrzymuje 4A=0 i 8C=00/00, oba ustawienia
są odczytywane. Dopiero wtedy program zgłasza sukces i zwalnia sterowanie.
Błąd przywracania zatrzaskuje awarię.

Protokół F5/05 nie zawiera numeru cyklu. Dopasowanie wykorzystuje jedyny oczekujący
zestaw, adres, opcode, DLC, checksum, okno czasu i wyłączenie raportów spontanicznych.
Nie wolno mieć drugiego nadawcy. Nie można dowieść pochodzenia dowolnie opóźnionej,
poprawnej kopii ACK z samej ramki. Potwierdzenie broadcast nie zastępuje nadzoru osi.
Lookahead nie gwarantuje fizycznej ciągłości przy kwantyzacji RPM, ograniczeniu
momentu, nieznanej dynamice lub działaniu bufora innym niż założone.

Trace rozróżnia sync_prepared, sync_axis_accepted, sync_submitted, sync_activated
oraz sync_reference_committed. Zachowuje numer zestawu, termin, fazę, horyzont,
q/v/a, payload F5/RPM i dane pomiaru z czasem RX w sync_feedback. Istniejący
zapis CAN zawiera surowe odpowiedzi i ich znaczniki. Nagrywanie wymaga istniejącego
trybu `--record` skryptu startowego albo ustawienia `ROBOT_TRACE_DIR` przed startem
procesu. Log C++ rotuje w czterech plikach po 16 MiB; przy długich programach
trzeba odebrać/archiwizować dane przed nadpisaniem. Brak pełnego zapisu nie
potwierdza terminów ani płynności całej próby.

## Weryfikacja i wdrożenie

Testy deterministyczne nie otwierają fizycznego CAN. Emulator przechowuje osobny
cel aktywny i oczekujący; nie jest modelem dynamiki rzeczywistego napędu.
Raport wyników offline i porównanie czasu wykonania: [benchmark_results.md](benchmark_results.md).

Etap sprzętowy pozostaje osobny: najpierw jedna oś i sprawdzenie kontynuacji
poprzedniego ruchu podczas buforowania oraz usuwania zadania przez STOP; dopiero
po pozytywnym wyniku dwie i sześć osi. Przed każdym ruchem trzeba przygotować
kartę na podstawie bieżącej pozycji, kalibracji i limitów: osie, kierunki,
prędkości, czas/zakres, warunki STOP i osobna zgoda operatora. Ruch uruchamia się
wyłącznie przez panel. Brak tych danych nie jest zgodą na test ani powód do
przyjęcia fikcyjnych wartości. Porównanie obu trybów wymaga tej samej pozycji
początkowej, trajektorii, obciążenia i metody pomiaru. Wymagane: ACK przed każdym
SYNC, spóźnienia ≤2,5 ms, brak nieplanowanych postojów, dotychczasowe tolerancje
końca. Jeśli pomiary nie rozstrzygają płynności, odbiór pozostaje niezaliczony.

Build jest odseparowany w `install/synchronized-ros` i jawnie Release.
Przygotowane pliki nie oznaczają aktywacji w usłudze. Po potwierdzeniu postoju
operator może zastosować drop-in i zrestartować całą usługę:

```bash
sudo install -m 0644 system/application/synchronized-ros.conf.example /etc/systemd/system/robot-web-status.service.d/tcp-settings.conf
sudo systemctl daemon-reload
sudo systemctl restart robot-web-status.service
```

Następnie Ctrl+F5. Domyślny tryb starych konfiguracji pozostaje streamed.
W tym etapie nie wykonywano restartu, resetu CAN, zmiany ENABLE ani ruchu sprzętu.
