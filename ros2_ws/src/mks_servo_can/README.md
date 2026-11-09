# mks_servo_can

Biblioteka C++17 obsługuje MKS SERVO42D CAN przez SocketCAN i U2C v2.1.

- `include/mks_servo_can/` — protokół, transport, `CanBusManager` i `BusRuntime`;
- `src/` — transport SocketCAN oraz kodowanie i dekodowanie protokołu MKS;
- `test/` — protokół i emulator sześciu osi z wstrzykiwanym czasem.

Silnik planuje odczyty i kompletne zestawy F5 z barierą ACK/SYNC. W diagnostyce
wysyła tylko zapytania odczytowe. Ruch wymaga konfiguracji i flag odbioru;
fault blokuje wznowienie aż do jawnego resetu. Wątek IO i cache są wspólne
dla adaptera sprzętowego i diagnostyki. Szczegóły: [strategia CAN](../../../docs/can_strategy.md).

`ManualMotorController` obsługuje oddzielnie ENABLE, zero i prędkościowy JOG
jednego wału (CAN 1–6 → J1–J6). Nie wymaga konfiguracji geometrii robota;
korzysta z harmonogramu i tego samego transportu. Opis: [sterowanie ręczne](../../../docs/manual_control.md).

Źródło: [materiały Makerbase MKS SERVO42D/57D](https://github.com/makerbase-motor/MKS-SERVO42D-57D).
Lista danych do ustalenia znajduje się w [opisie CAN](../../../docs/can_protocol.md).
Ścieżka względna dotyczy README w drzewie źródłowym.
