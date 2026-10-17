# CAN: U2C v2.1 i MKS SERVO42D

## Podział warstw

U2C jest mostkiem USB–CAN. Na tym komputerze Linux wykrywa go jako interfejs
SocketCAN `can0` ze sterownikiem `gs_usb`. Biblioteka `mks_servo_can` będzie korzystać z tego
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

Każdy z sześciu napędów wymaga jednoznacznego mapowania do osi. CAN ID pozostają
nieustalone. Bitrate 500000 bit/s został potwierdzony przez użytkownika i zapisany
w szablonie konfiguracji; musi odpowiadać ustawieniom fizycznych urządzeń.

## Stan implementacji

Nie ma jeszcze kodowania ani dekodowania ramek, socketów CAN, sekwencji
inicjalizacji lub automatycznego wykrywania urządzeń. Szablony są materiałem
do zebrania parametrów. Projekt nie wysyła komend i nie aktualizuje firmware.

Przyszłe testy protokołu powinny sprawdzać ramki referencyjne z instrukcji,
niepoprawne długości i sumy kontrolne, dopasowanie odpowiedzi do napędu oraz
timeouty. Integrację z transportem można później sprawdzać na `vcan`,
oddzielnie od testów z fizycznym U2C i napędami.
