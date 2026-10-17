# mks_servo_can

Miejsce na bibliotekę C++ obsługującą napędy MKS SERVO42D CAN przez Linux
SocketCAN i mostek U2C v2.1.

- `include/mks_servo_can/` — przyszłe publiczne nagłówki;
- `src/` — transport SocketCAN oraz kodowanie i dekodowanie protokołu MKS;
- `test/` — przyszłe testy protokołu i transportu.

Pakiet nie zawiera jeszcze kodu, celów biblioteki ani eksportowanych nagłówków.
Przyszła implementacja powinna oddzielać protokół od transportu i korzystać
z instrukcji odpowiadającej firmware posiadanych napędów.

Źródło: [materiały Makerbase MKS SERVO42D/57D](https://github.com/makerbase-motor/MKS-SERVO42D-57D).
Lista danych do ustalenia znajduje się w [opisie CAN](../../../docs/can_protocol.md).
Ścieżka względna dotyczy README w drzewie źródłowym.
