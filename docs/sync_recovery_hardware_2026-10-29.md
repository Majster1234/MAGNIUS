# Odbiór sprzętowy odzyskiwania SYNC — 29.10.2026

Ten raport zachowuje pierwszy etap. Późniejsze próby z osobnym okresem
ustalania ENABLE opisuje [kolejny raport](sync_recovery_timing_2026-10-29.md).

**Odbiór niezaliczony. Automatyczne odblokowanie pozostaje wyłączone.**
Neutralne F5 STOP uzyskało ACK `05` przy ENABLE=0 na wszystkich sześciu osiach,
ale pierwszy test ponownego ENABLE J1 przekroczył tolerancję postoju. Nie
wykonano kontroli dodatniej +5° ani późniejszych wyzwoleń SYNC; nie potwierdzono
usunięcia znanego rozkazu ruchowego z bufora.

## Zgoda i karta

Operator jawnie zezwolił na wszystkie osie fizyczne, samodzielny dobór parametrów
i odstąpienie w tej sesji od ograniczeń AGENTS.md dotyczących prób ruchowych.
W kontekście sesji potwierdził rozsprzęgnięcie wałów i firmware 1.0.9. Był to
osobny etap; nie wykorzystywano dawnych zgód ani licznika 6/6 prób TCP.

Karta: J1–J6 kolejno, 1 RPM wału, +5° i powrót −5°, limit obserwowanej pozycji
±6° od startu, timeout dojścia 3 s, zmiana w oknie postoju ≤0,5° (22 ticki).
Warunki STOP: niezgodny/brakujący ACK, ramka błędu CAN, przekroczenie zakresu,
niezgodny ENABLE/stan napędu lub przekroczenie tolerancji postoju. Dwa
powtórzenia nadpisania znanego bufora zaplanowano osobno dla każdej osi.

Sprawdzono rzeczywiste dane: CAN ID 1–6, MKS SERVO42D, identyfikacja
`1201000920`, firmware 1.0.9, mikrokrok 16, SR_vFOC `05`, watchdog 300 ms,
CAN 1 Mbit/s. Nie zmieniano kalibracji, geometrii, prądu ani przełożeń.

Test prowadził jeden nadajnik z tą samą blokadą SocketCAN co aplikacja. Właściciel
CAN aplikacji został zakończony przez SIGINT. Po próbach kontrolowane zamknięcie
launch pozwoliło systemd odtworzyć usługę z istniejącym `Restart=always`.
Nie resetowano interfejsu CAN ani zasilania napędów.

## Wykonanie i wynik

| Etap | Wynik |
| --- | --- |
| Początkowe odczyty J1–J6 | ENABLE=0, 0 RPM, F1=1, SYNC=1; wszystkie osie odpowiadają |
| Próba 1: serie F7 → F5 STOP → F3 OFF bez przerw | Przerwana ramką `CAN_ERR_PROT` / `BIT1`; brak ENABLE=1 i brak 4B |
| Próba 2: odstępy 5 ms między seriami STOP | Bez ramek błędów CAN |
| Neutralne F5 przy ENABLE=0 i SYNC=1 | Dedykowany ACK `05` każdej osi J1–J6 |
| Przywrócenie i odczyt 4A=0, 8C=00/00 | Potwierdzone na J1–J6; przed ENABLE rozrzut pozycji 0/0/0/0/1/0 ticków |
| Pierwsze ponowne ENABLE: J1 | Odczyt −3 → +171 ticków, zmiana **174 ticki = 3,823242° >0,5°**; STOP |
| Kontrola dodatnia +5°/powrót i nadpisanie znanego bufora | Niewykonane wskutek warunku STOP |
| Późniejsze SYNC i ponowne ENABLE J2–J6 | Niewykonane |

W próbie 1 zapisano dwie ramki błędu o CAN ID `0x20000008` i danych
`0000100000000800`, `0000100000001000`. Wskazują `CAN_ERR_PROT_BIT1`; nie
ustalono jednoznacznie źródła. Kolizja żądania i odpowiedzi używających tego
samego CAN ID jest hipotezą zgodną z charakterem wysyłki. Wariant z odstępami
nie wykazał takich ramek, ale zmieniał także tempo przetwarzania rejestratora,
więc nie jest izolowanym dowodem przyczyny ani odbiorem produkcyjnego STOP.
Cleanup próby 1 został przerwany kolejną ramką błędu; późniejsza diagnostyka
usługi ponownie potwierdziła ENABLE=0 i 0 RPM wszystkich osi.

W próbie 2 pierwszy odczyt pozycji J1 po wysłaniu ENABLE otrzymano po ok.
4,061 ms. W tym samym zestawie 32H raportowało 0 RPM, a F1=1. Te punktowe
odczyty nie wykluczyły zmiany pozycji. Nie wyliczano z tego rzeczywistej
prędkości i nie przypisano przyczyny do bufora, regulatora ani enkodera.
Bez niezależnego pomiaru nie rozstrzygnięto, czy nastąpił fizyczny doskok wału,
czy zmiana odniesienia raportowanej pozycji.

**W całym etapie nie wysłano ani jednego broadcast 4B ani F5 z niezerowym RPM.**
Jedynym załączeniem był F3 ENABLE=1 dla J1. Zaplanowane ruchy 1 RPM nie zostały
uruchomione. Nie kontynuowano prób załączania po przekroczeniu 0,5° i nie
zwiększano tolerancji w celu uzyskania pozytywnego wyniku.

## Domknięcie

Po STOP potwierdzono świeżymi odczytami ENABLE=0, 0 RPM i F1=1 wszystkich osi
oraz stabilność pozycji w oknie dłuższym niż 100 ms. Następnie odczytano
`4A=1` i `8C=01/00` każdej osi, zachowując widoczną blokadę po niezaliczonej
próbie. Domknięcie nie zawierało ENABLE=1 ani 4B i nie wykazało błędów CAN.

Końcowe pozycje wałów J1–J6: 2/0/−1/−1/−1/0 ticków. Są to odczyty enkoderów,
nie pomiar zewnętrzny ani przestrzenny TCP. Przywrócona usługa (PID 37632)
korzysta z `install/stop-recovery-ros`; potwierdzono świeże pozycje, 0 RPM i
ENABLE=0 sześciu osi, `fault=true`, `recovery_state=recovery_failed`, maskę 63.
`execution_staged_reset_verified` pozostało `false`.

Następny etap powinien wyjaśnić skok pozycji J1 przy samym ENABLE, z niezależną
obserwacją wału. Dopiero potem można wrócić do kontroli dodatniej i badania
rzeczywistego nadpisania znanego rozkazu. Błąd grupowego STOP również wymaga
osobnego potwierdzenia i poprawki przed odbiorem tej ścieżki.

## Dane

[Trwałe podsumowanie JSON](sync_recovery_hardware_2026-10-29.json) zawiera liczniki,
warunki, wyniki i SHA-256 zapisów. Pełne lokalne pliki JSONL zachowano w
`log/hardware/` (katalog nagrań ignorowany w Git):

- [Próba 1](../log/hardware/sync-recovery-20261029-1793297412454099221.jsonl):
  530 TX, 503 potwierdzenia nadania, 499 RX. Zapis końca niepełny po awarii.
- [Próba 2](../log/hardware/sync-recovery-paced-20261029-1793297519697201706.jsonl):
  1876 TX i potwierdzeń nadania, 1866 RX, 73 zestawy pozycji/RPM/ENABLE/F1.
- [Domknięcie](../log/hardware/sync-recovery-latch-20261029-1793297677354693047.jsonl):
  930 TX i potwierdzeń nadania, 920 RX, 37 zestawów pomiarów.

Narzędzie: `scripts/test_sync_recovery_hardware.py`; pięć celowanych testów
procedury na atrapach i kompilacja składni Python — OK. Narzędzie ma teraz
cleanup przywracający SYNC=1 po niezaliczonej próbie. Nie zmieniono kodu
produkcyjnego C++ ani nie włączono automatycznego odblokowania.
