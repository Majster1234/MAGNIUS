# Powtórne próby bufora SYNC i dobór czasów — 29.10.2026

**Późniejsze wdrożenie na polecenie operatora:** automatyczne odblokowanie SYNC
jest teraz włączone (`execution_staged_reset_verified=true`) mimo niezaliczonej
kontroli J4. Nie uruchamiano nowych testów i nie zmieniono wyniku odbioru.
Gotowe pakiety z `install/stop-recovery-timing-ros` wdrożono do aktywnej
`install/stop-recovery-ros`, zachowując poprzednią wersję w
`install/deployment-backups/sync-auto-1793301016056132003`.
Usługa została odtworzona (MainPID 47062). Odczyt po wdrożeniu:
`idle_disabled`, `fault=false`, maska oczekujących osi 0, ENABLE=0 na J1–J6.
Wymogi świeżego postoju, ENABLE=0, ACK i odczytu ustawień pozostają aktywne;
poprzedni ruch nie jest wznawiany. [Zapis wdrożenia](sync_auto_unlock_deployment_2026-10-29.json).
Poniższe wyniki i informacja o wyłączonej bramce dotyczą wcześniejszego etapu prób.

**Odczyty CAN przechodzą; pełny odbiór odzyskiwania nadal niezaliczony.**
J1–J3, J5 i J6 zaliczyły po dwa nadpisania znanego oczekującego rozkazu,
późniejsze SYNC oraz ponowne ENABLE. J4 nie zaliczyła kontroli dodatniej ruchu;
nie wykonano jej dwóch prób nadpisania znanego bufora. Nie ustawiono
`execution_staged_reset_verified=true`.

To kolejny etap po [pierwszym raporcie](sync_recovery_hardware_2026-10-29.md).
Nie zmieniono historycznych wyników ani pierwotnych zapisów ramek.

## Zgoda i kryterium ENABLE

Obowiązywała zgoda operatora na wszystkie rozsprzęgnięte osie, samodzielny
dobór parametrów i bezpośrednie próby CAN w tej sesji. Karta: J1–J6 kolejno,
±5° wału, 1 RPM, ograniczenie obserwacji ±6° od aktualnej pozycji odniesienia,
STOP przy błędzie ramki/ACK, niezgodnym ENABLE, przekroczeniu zakresu lub
nieplanowanym przemieszczeniu. Dla każdej osi przewidziano dwa nadpisania
oczekujących celów, kolejno +5° i −5°.

Na polecenie operatora skok przy przełączaniu ENABLE jest rejestrowany jako
osobne przejście. Po przejściu wymaga się świeżego właściwego ENABLE, F1=1,
0 RPM i dwóch stabilnych zestawów pozycji oddalonych o ≥100 ms. Dopiero wtedy
powstaje zmierzona pozycja odniesienia testu. Nie zapisuje to offsetu ani
kalibracji robota. Osie bez przejścia ENABLE nadal podlegają kontroli ±0,5°.
Zmiany w przejściu pozostają w danych; nie są kasowane ani przedstawiane jako
potwierdzony postój. Dodatkowo po nadpisaniu porównuje się pozycje po ustaleniu
ENABLE przed/po próbie, z granicą 44 ticków wynikającą z dwóch pasm 22 ticków.

## Dobór czasów

| Parametr | Baza / obserwacja | Końcowa wartość | Zakres |
| --- | --- | --- | --- |
| Odpowiedź odczytu CAN | 20 ms bez timeoutów | **24 ms**, +20% | launch i produkcyjny YAML |
| Odstęp między seriami STOP | 5 ms bez błędów w poprzedniej próbie | **6 ms**, +20% | narzędzie oraz nowy właściciel CAN |
| ACK kroku odzyskiwania | 100 ms | **120 ms**, +20% | nowe C++; narzędzie używało 120 ms |
| Minimalny okres po ENABLE | zmierzone 1340,809 ms | **1610 ms**, +20%, zaokrąglone w górę | narzędzie odbioru |
| Maksymalny czas ustalania ENABLE | największe zmierzone 3738,952 ms | **4490 ms**, +20%, w górę do 10 ms | narzędzie; limit wyliczony po próbach |
| Okno stabilności kontroli dodatniej | próba dłuższego okna 500 ms | **600 ms**, +20% | końcowe próby J4–J6 |
| Limit kontroli dodatniej | próba obserwacji 5 s | **6 s**, +20% | końcowe próby J4–J6; J4 nadal niezaliczona |

Próby ruchowe miały limit ustalania ENABLE 3720 ms; jeden pełny zestaw
pomiarowy zakończył się po 3738,952 ms. Wartość 4490 ms jest końcowym zapasem
wyliczonym z tej obserwacji, bez kolejnego przejazdu. Nie oznacza ponownego
odbioru ani dowodu, że dowolne przyszłe warunki zmieszczą się w tym terminie.

W ośmiu zapisach wykonano **337059 poprawnie zakończonych zapytań, 0 timeoutów
i 0 ramek błędów CAN**. Maksymalny czas odpowiedzi odczytu wyniósł
**19,134613 ms**. Każda seria kalibracyjna zawierała 30 obiegów przy 20 ms oraz
10 obiegów przy 24 ms; odpytywano wszystkie sześć osi. Nie obniżano kryteriów
ważności odpowiedzi, nie zastępowano błędnych odczytów zerem. Pojedyncze 32H=−1
przy stabilnym enkoderze wstrzymuje potwierdzenie postoju; nie jest samodzielnym
dowodem przemieszczenia wału.

Nie zmieniono świeżości sprzężenia zwrotnego podczas ruchu, watchdogu 300 ms,
cyklu 10 ms, tolerancji spóźnienia SYNC 2,5 ms, ograniczeń mechanicznych ani
tolerancji pozycji. Nie strojono regulatorów MKS.

## Wyniki osi

| Oś | Kontrola dodatnia +5° / powrót | Znane bufory nadpisane | Późniejsze SYNC / ENABLE |
| --- | --- | --- | --- |
| J1 | zaliczona | 2/2 | zaliczone |
| J2 | zaliczona | 2/2 | zaliczone |
| J3 | zaliczona | 2/2 | zaliczone |
| J4 | niezaliczona | niewykonane po kontroli | same próby po neutralizacji nieznanego bufora przeszły; brak pełnego odbioru |
| J5 | zaliczona, okno 600 ms | 2/2 | zaliczone |
| J6 | zaliczona, okno 600 ms | 2/2 | zaliczone |

J1–J3 mają wyniki przy wcześniejszym oknie kontroli dodatniej 100 ms. Nie
przedstawia się ich jako ponowienia przy 600 ms. Wszystkie próby neutralnego
nadpisania i wyzwoleń nadal kontrolowały stabilność ±0,5° oraz rzeczywiste 0 RPM.

Pierwszy niezaliczony etap J4: po kontroli dodatniej nastąpiła dalsza zmiana
465→415 ticków, poza przejściem ENABLE. W kolejnej próbie 6 s cel wynosił
644 ticki, a w ostatniej sekundzie pozycja wynosiła 600–601 ticków
(niedojazd 0,945–0,967°). W dodatkowej próbie sprawdzającej samą wykrywalność
ruchu J4 również nie spełniła warunku. Błąd dojazdu nie został zamaskowany
większą tolerancją pozycji ani uznany za skok ENABLE. To nie był timeout
odpowiedzi CAN. Nie wyznaczono na tej podstawie przyczyny w regulatorze,
mechanice ani samym buforowaniu firmware.

## Zmiany wykonawcy i wdrożenie

Nowy właściciel wysyła F7 natychmiast, po ≥6 ms F5 STOP, po następnych ≥6 ms
ENABLE=0 zgodnie z polityką awarii. Po kolejnych ≥6 ms wznawia zapytania
diagnostyczne. Są to terminy minimalne; rzeczywista wysyłka zależy od
harmonogramu IO. Pętla RX przez cały czas odbiera ramki. Kolejny STOP w trakcie
tych serii nie przesuwa odcięcia ENABLE; może wymusić wyłączenie zgodnie z
żądaniem operatora. Reset interfejsu jest blokowany do zakończenia wysyłek.
Nie ma blokującego sleep ani produkcyjnego 4B w odzyskiwaniu.

Rozróżniono już zatrzaśniętą awarię od nowej awarii podczas odbioru potwierdzeń
nadania. Wcześniej każde echo przy aktywnej awarii kończyło obieg RX po jednej
ramce; teraz odczyty mogą opróżniać normalny limit kolejki.

**Aktywny jest timeout odczytów 24 ms** w dotychczasowej nakładce
`install/stop-recovery-ros`. Nowe C++ przygotowano osobno w
`install/stop-recovery-timing-ros`; ta nakładka wymaga osobnego wdrożenia.
Odstępy 6 ms potwierdzono w narzędziu sprzętowym; nie przedstawia się tego jako
sprzętowego odbioru nowej binarki właściciela. Polecenia wdrożenia znajdują się
przy [przykładowym drop-in](../system/application/stop-recovery-timing-ros.conf.example).

Przy potwierdzonym postoju, z katalogu projektu:

```bash
sudo install -m 0644 system/application/stop-recovery-timing-ros.conf.example /etc/systemd/system/robot-web-status.service.d/zz-stop-recovery-timing.conf
sudo systemctl daemon-reload
sudo systemctl restart robot-web-status.service
```

Build Release pięciu pakietów/konsumentów (`mks_servo_can`,
`robot_can_diagnostics`, `robot_hardware`, `robot_motion_planner`,
`robot_bringup`) — OK. Po ostatnich zmianach ponownie zbudowano zmienione
pakiety. Testy `mks_servo_can` i `robot_can_diagnostics`: **222 wyniki,
0 błędów, 0 porażek, 0 pominięć** (w tym 20 opakowań CTest).
Obejmują nowe odstępy STOP, brak przesuwania odcięcia ENABLE przy kolejnych
STOP, blokadę resetu w trakcie wysyłek, regresję obu streamingów, JOG,
konfigurację i odzyskiwanie. Dziewięć testów Python procedury oraz py_compile
narzędzia i launch — OK. Frontendu nie zmieniano ani nie budowano.

Logi build/test: `/tmp/stop-recovery-timing-build.log`,
`/tmp/stop-recovery-timing-build-final.log`,
`/tmp/stop-recovery-timing-test-results.log`,
`/tmp/sync-recovery-procedure-tests.log`.

Po końcu prób potwierdzono ENABLE=0, 0 RPM, F1=1 i stabilność sześciu osi.
Przywrócono SYNC=1 jako widoczny znacznik blokady, bez dodatkowego wyzwolenia.
Odtworzona usługa utrzymuje odczyty; `execution_staged_reset_verified=false`.
Automatyczne usuwanie blokady pozostaje niedopuszczone do pełnego odbioru.

## Dane

[Podsumowanie JSON](sync_recovery_timing_2026-10-29.json) zawiera liczniki,
poszczególne etapy, próbki końca J4, nazwy pełnych zapisów w `log/hardware/`
oraz ich SHA-256. Rejestrowano wszystkie TX/RX, własne potwierdzenia nadania,
pozycje, RPM, ENABLE i F1. Odczyty enkoderów nie zastępują niezależnego pomiaru
wału ani odbioru przestrzennego TCP. Poprzednich raportów dokładności TCP nie
zmieniono.
