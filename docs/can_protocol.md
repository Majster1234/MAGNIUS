# CAN: U2C v2.1 i MKS SERVO42D

## Podział warstw

U2C jest mostkiem USB–CAN. Na tym komputerze Linux wykrywa go jako interfejs
SocketCAN `can0` ze sterownikiem `gs_usb`. Biblioteka `mks_servo_can` korzysta z tego
interfejsu do wymiany ramek z napędami MKS SERVO42D CAN.

Użytkownik potwierdził bitrate **500000 bit/s**. Konfigurację systemową i sposób
jej instalacji opisuje [integracja SocketCAN](socketcan.md).

Transport CAN i protokół komend napędu są odrębnymi warstwami. Projekt jest
przygotowywany pod protokół producenta MKS; nie zakłada CANopen/CiA 402.
Materiały dla wersji RS485 nie zastępują instrukcji wersji CAN.

## Dokumentacja źródłowa

- [BIGTREETECH — dokumentacja U2C](https://github.com/bigtreetech/docs/blob/master/docs/U2C.md).
- [BIGTREETECH — repozytorium U2C i firmware](https://github.com/bigtreetech/U2C).
- [Makerbase — MKS SERVO42D/57D, instrukcje i firmware](https://github.com/makerbase-motor/MKS-SERVO42D-57D).

Zapisz w `hardware/datasheets` instrukcję odpowiadającą faktycznie posiadanej
wersji CAN i firmware. Zachowaj nazwę dokumentu, wersję, adres źródła i datę
pobrania. Wersja sprzętowa U2C `2.1` nie określa wersji jego firmware.

## Dane do zebrania przed implementacją

| Dane | Miejsce zapisu |
| --- | --- |
| Wersja firmware U2C i sposób udostępnienia przez SocketCAN | `can.example.yaml`: `adapter.firmware_version`; notatki w `hardware/datasheets` |
| Nazwa interfejsu i wspólny bitrate magistrali w bit/s | `can.example.yaml`: `interface`, `bitrate` |
| CAN ID i firmware każdego napędu | `joints.example.yaml`: `can_id`, `firmware_version` |
| Rewizje płytek i wersja instrukcji protokołu | Notatki w `hardware/datasheets` |
| Schemat połączeń, zasilanie i terminacja magistrali | `hardware/wiring` |
| Format identyfikatorów i ramek, długości, kolejność bajtów i suma kontrolna | Ten dokument po weryfikacji instrukcji |
| Komendy odczytu pozycji, ruchu, zatrzymania, enable/disable i obsługa odpowiedzi | Ten dokument po weryfikacji instrukcji |
| Jednostki napędu, zakres licznika, tryb sterowania i bazowanie | Ten dokument oraz konfiguracja osi |
| Timeouty, błędy, częstotliwość odczytu i obciążenie magistrali dla sześciu osi | Ten dokument po pomiarach |

Mapowanie potwierdzone przez użytkownika: CAN 1–6 → J1–J6, odpowiednio
`joint_1`–`joint_6`; zapisano je w szablonie osi. Bitrate 500000 bit/s został potwierdzony na sprzęcie; nowy szablon 100 Hz wskazuje 1 Mbit/s.
Zmiana szablonu nie przestawia interfejsu ani napędów.

## Streaming F5 + 31 — kontrakt 24.10.2026

Classical CAN, standardowe identyfikatory 1–2047, dane big-endian. Suma kontrolna
jest sumą bajtów danych i CAN ID modulo 256 (nie wielomianowym CRC).

| Komenda | Format z sumą kontrolną | Znaczenie |
| --- | --- | --- |
| F5 | `F5 speed_hi speed_lo acc pos_hi pos_mid pos_lo checksum`, DLC8 | bezwzględny cel int24; speed_hi ma maskę 0F; RPM<=3000 |
| 31 zapytanie | `31 (ID+31)&FF`, DLC2 | pozycja enkodera wieloobrotowego |
| 31 odpowiedź | `31 p5 p4 p3 p2 p1 p0 checksum`, DLC8 | int48 ze znakiem; rozszerzenie znaku do int64 |
| 8C zapis | `8C 00 00 checksum` | wyłączenie odpowiedzi ruchu i raportu zakończenia |
| 4A zapis | `4A 00 checksum` | bez oczekiwania na SYNC |

F5 koduje pełny int24 [-8388608,8388607]; fizyczna konfiguracja dodatkowo
sprawdza reprezentowalność limitów i zakres instrukcji. Enkoder ma 16384 impulsy
na obrót wału, niezależnie od przekładni. ROS używa radianów przegubu,
z uwzględnieniem kierunku i offsetu. Parametr speed dla innych mikrokroków niż 16
wymaga skali MKS odczytanej z 84; liczba impulsów enkodera się nie zmienia.

Wymagane ustawienia dla sześciu osi 100 Hz: **CanRate=1M, CanRSP=Disable,
Work Mode=SR_vFOC (05)**. CR_vFOC jest trybem wejścia impulsowego; nie nadaje
się do tego sterowania CAN. Źródło: [instrukcja MKS CAN V1.0.9, §5.2.12,
rozdział 11](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).
Wyłączone CanRSP nie wyłącza odpowiedzi na odczyty i konfigurację. F5 może
aktualizować cel w trakcie ruchu; nie oczekujemy jego ACK ani zakończenia.

Każde 10 ms wspólny właściciel nadaje uczestnikom F5 z lookahead 25 ms i FF 1,15,
a następnie asynchronicznie odpytuje 31. Zwykłe zapytania mają cztery miejsca
oczekiwania, więc sześć osi obsługiwane jest potokowo. Osobny wątek RX waliduje
adres/typ/DLC/sumę, aktualizuje atomowe impulsy/czas, przekazuje pozostałe ramki
kolejką do managera. Dopasowanie zwykłych odpowiedzi dodatkowo sprawdza komendę
i termin. Błędna ramka nie potwierdza zapytania.

Tolerancja jest wyłącznie warunkiem końcowym; odchyłka pośrednia nie blokuje
następnego F5. Brak ACK nie jest błędem, bo nie ma transakcji ACK ruchu.
Nie wysyłamy SYNC 4B; osie dostają ramki kolejno ze wspólnego zegara odniesienia.
BUS-OFF, utrata transportu i jawne błędy napędu nadal przerywają program.
Szczegóły matematyki, limitów i budżetu: [wykonanie PTP](motion_execution.md).

Fizyczny can0 zachowuje 500 kbit/s. Wdrożenie źródeł nie jest zmianą CanRate,
CanRSP/trybu rzeczywistych napędów ani odbiorem płynności. Ręczne ENABLE/zero/JOG:
[sterowanie ręczne](manual_control.md). Nieznane parametry osi pozostają do ustalenia.
