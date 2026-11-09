# CAN 1 Mbit/s — test komunikacji 24.10.2026

Użytkownik przestawił podłączone MKS na 1 Mbit/s. Na hoście wdrożono
`BITRATE=1000000`, `TX_QUEUE_LEN=1000` w `/etc/robot-can/can0.conf` oraz
`can_bitrate:=1000000` w drop-in usługi `robot-web-status.service`.
Rzeczywisty link i metadane aplikacji potwierdzają 1 Mbit/s.

## Wynik testu samych odczytów

Test rozpoczęto 24.10.2026 o 23:23:24 CEST. Usługa aplikacji była zatrzymana;
skrypt `scripts/test_can_communication.py` był jedynym właścicielem poleceń CAN.
Wysyłał wyłącznie odczyty `40/31/32/3A/3E/F1` i `00+parametr`.
Nie nadawał ENABLE, zerowania, F5/F6/F7/FE ani zapisów konfiguracji.

| Pomiar | Wynik |
| --- | --- |
| Czas cyklu testowego | 10,00016 s; 1000 cykli, bez pominięć |
| Osie odpowiadające | J1/J2/J3/J5 |
| Pozycje | 1000 odpowiedzi/oś, łącznie 4000; 99,998 Hz/oś |
| Wszystkie poprawne odpowiedzi | 4052 |
| Timeouty uczestników / błędne ramki / błędy CAN | 0 / 0 / 0 |
| Nieobecne osie | J4/J6; dwa oczekiwane timeouty wykrywania |
| ENABLE przed/po | 0 dla wszystkich czterech osi, bez zmian |
| Stan kontrolera po teście | ERROR-ACTIVE |

| Oś | Średni RTT [ms] | p95 [ms] | Maksymalny RTT [ms] |
| --- | ---: | ---: | ---: |
| J1 | 1,056 | 1,575 | 3,202 |
| J2 | 1,057 | 1,872 | 4,007 |
| J3 | 1,077 | 1,860 | 3,663 |
| J5 | 1,107 | 1,890 | 3,780 |

RTT obejmuje także odczyty przygotowawcze. Wszystkie cztery napędy zwróciły
SR_vFOC=`05`, mikrokroki=`10`, CanRate=`03`, CanRSP/raporty=`0000`,
watchdog=`0000012c` i synchronizację `4A=00`.
Zarejestrowano jeden komunikat powrotu kontrolera do ERROR-ACTIVE; nie jest
to nowy błąd magistrali. Historyczne liczniki warning/passive nie wzrosły.

Surowe wyniki lokalne:
`log/can-1m-20261024/20261024T212323Z-84037/probe/result.json`
oraz `probe/frames.jsonl` w tym samym katalogu próby.

## Zakres odbioru

Test potwierdza komunikację czterech dostępnych osi przy 1 Mbit/s.
Nie uruchamiał ruchu i nie mierzy zgodności ruchu z trajektorią.
Wcześniejsze próby ruchowe przy 500 kbit/s pozostają osobnymi pomiarami:
[raport sprzętowy PTP](motion_hardware_results.md).

Model sześciu osi w ruchu 100 Hz daje 26,58% magistrali przy 1 Mbit/s.
Z rezerwą 20 punktów procentowych dla przyszłego modułu jest to 46,58%
z limitu 60%; sam robot mieści się w swojej puli 40%.
To model, a nie pomiar sześciu rzeczywistych osi ani nowego modułu.

Procedury wdrożenia: `scripts/set_host_can_1m.sh` i
`scripts/complete_can1m_ptp.sh`. Zmiana hosta i odczyty przeszły; pierwsze
odblokowanie PTP zatrzymała kontrola bazy: J1 miał 13,10° zamiast pozycji
z poprzedniej próby. Profil pozostał wyłączony. Użytkownik zlecił następnie
rzeczywiste zerowanie enkoderów J1/J2/J3/J5 na rozsprzęglonych wałach.
Służy do tego `scripts/unlock_ptp_after_zero.sh --zero-decoupled`:
sprawdza ENABLE=0 i postój wszystkich uczestników przed pierwszym zerowaniem,
potwierdza ACK i odczyty zera, przyjmuje bazę przez zwykłą transakcję
konfiguracji i przygotowuje profil 1 RPM/acc=1, bez automatycznego ruchu.

Pierwsze wywołanie procedury zerowania (23:31 CEST) przerwało się na warunku
`J1: wymagane ENABLE=0`, przed pierwszym `92`. W międzyczasie J1 został
załączony z panelu i miał około 50,3°. Wyłączono go standardowym API i
potwierdzono ENABLE=0 wszystkich czterech osi. Ponowienie zakończyło się
poprawnie o 23:35:47 CEST: wszystkie cztery enkodery zwróciły 0 impulsów,
ACK zerowania przyjęte, ENABLE pozostało 0. Normalna transakcja konfiguracji
przyjęła rewizję `a0958be8245b475ba90853c9b9bbe185`. Odczyt ROS potwierdził
`execution_enabled=true`, zgodną rewizję, holding/max 1 RPM i sześć kodów acc=1.
Nie zmieniano ręcznie flag kalibracji. Dowód: katalog lokalny
`log/can-1m-20261024/zero-unlock-20261024T213546Z-86059/`.

Nowe skrypty sprawdzono przez `bash -n` (3 pliki), analizę składni Python
(3 pliki) oraz 5 scenariuszy atrapy CAN dla zerowania: sukces, ENABLE=1,
odmowa ACK, błędna suma i długość. Wszystkie przeszły. Atrapa nie zastępuje
sprawdzenia sprzętowego. Nie zmieniano cykli wykonawcy/ROS i nie wykonywano
nowych benchmarków ani pełnych testów wcześniejszej implementacji.

## Parametry PTP w panelu

24.10.2026 dodano widoczne pola prędkości przegubów i przyspieszenia trajektorii
w sterowaniu ręcznym, w trybie planowania. Zmiana unieważnia wcześniejszy plan
oraz zgodę na wykonanie; wymaga ponownego wyboru PTP, przygotowania i zatwierdzenia.
Panel domyślnie wybiera 6°/s i 50°/s². Użytkownik zatwierdził przywrócenie
wcześniejszego limitu przegubów 80°/s. Procedura `scripts/set_ptp_80.sh --apply`
zmienia limity przez normalne API konfiguracji, aktualizuje rewizję wykonawcy
i usuwa ograniczenie profilu do 1 RPM. Kod MKS acc=1 pozostaje bez zmian.
Wyłącza napędy w postoju; nie zeruje i nie uruchamia ruchu. Użytkownik wykonał
procedurę 24.10.2026 o 23:49:47 CEST. Odczyt HTTP i parametrów ROS potwierdził
nową rewizję `4061beb8c2c344a089e9a1d936d3ae32`, sześć limitów 80°/s,
`execution_enabled=true`, holding=1 RPM, max=14 RPM i sześć kodów acc=1.
Globalny limit RPM wynika z przełożeń; dodatkowo obowiązuje prędkość każdej osi
i kwantyzacja MKS. Dostępne osie mają ważną kalibrację oraz ENABLE=0, PTP jest
idle. Usługa active, can0 ERROR-ACTIVE/1M/TX1000; od restartu zero błędów CAN.
Wyniki lokalne: `log/can-1m-20261024/ptp80-20261024T214947Z-88821/`.

Nowy panel wdrożono do `web/dist` i potwierdzono HTTP 200 jego JS/CSS.
TypeScript/build i dwa testy Chromium na atrapach API przeszły (PTP/wykonanie
oraz regresja PTP/LIN). Sprawdzono 80°/s, pustą/przekroczoną prędkość,
zerowe przyspieszenie i unieważnienie wcześniej zatwierdzonego programu po
zmianie parametrów, także po podglądzie przygotowanego PTP. Dwa scenariusze
atrapy API sprawdziły profil domyślny 1 RPM i limit 80°/s z holding1/acc1.
Nie przeprowadzano nowego sprzętowego testu zwiększonej prędkości.

Zmiana prędkości nie zmienia zegara F5 100 Hz ani ustalonej rezerwy magistrali.
