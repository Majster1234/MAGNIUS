# robot_bringup

Miejsce na przyszłe pliki launch i konfigurację całego robota.
Katalog `launch/` jest pusty. Nie ma jeszcze polecenia uruchamiającego robota
ani programu wczytującego szablony CAN i osi.

## Szablony konfiguracji

| Plik w `config/` | Przeznaczenie |
| --- | --- |
| `can.example.yaml` | Mostek U2C, docelowy interfejs SocketCAN i bitrate |
| `joints.example.yaml` | Sześć osi: CAN ID, firmware, przełożenia, kierunki, offsety i limity |
| `controllers.example.yaml` | Zarys parametrów `controller_manager`, broadcastera stanu i kontrolera trajektorii |

Wartości `null` są celowo nieustalone; nie oznaczają zera ani wartości domyślnej.
Nie podawaj tych szablonów bezpośrednio jako `--params-file`. Pliki CAN i osi
opisują proponowany format wejścia dla przyszłej implementacji, a nie standardowe
parametry ROS 2. Plik kontrolerów odwzorowuje układ parametrów ROS, ale zawiera
nieuzupełnioną częstotliwość. Samo zastąpienie `null` nie dostarczy brakującego
modelu, pluginu sprzętowego ani plików launch.

## Znaczenie parametrów osi

- `can_id` — identyfikator napędu ustalony zgodnie z instrukcją firmware;
  wymagane jednoznaczne mapowanie sześciu urządzeń.
- `firmware_version` — rzeczywista wersja firmware danej osi.
- `gear_ratio` — dodatnia liczba obrotów wału silnika na jeden obrót przegubu.
- `direction` — `1` lub `-1`, określa zgodność kierunku silnika z dodatnim
  kierunkiem osi URDF.
- `zero_offset_rad` — przesunięcie zera w radianach po stronie przegubu.
- `limits.lower_rad`, `upper_rad` — dolny i górny limit pozycji przegubu.
- `limits.velocity_rad_s`, `acceleration_rad_s2` — maksymalne wartości
  prędkości i przyspieszenia przegubu, dodatnie, po uwzględnieniu przekładni.

Docelowa konwencja przeliczenia, po przeliczeniu odczytu napędu na radiany
wału silnika:

```text
joint_position_rad = direction * motor_position_rad / gear_ratio + zero_offset_rad
```

Offset i metoda bazowania muszą odpowiadać tej konwencji. Limity powinny być
spójne z przyszłym modelem URDF. Szablon kontrolera przewiduje komendę pozycji
i odczyt pozycji dla wszystkich sześciu osi; obsługa zależy od przyszłego adaptera.
`update_rate` oznacza częstotliwość `controller_manager` w Hz i wymaga dobrania
do wydajności napędów, protokołu oraz obciążenia magistrali.

## Instalacja

CMake instaluje `launch/` i `config/` do `share/robot_bringup`.
Zależności uruchomieniowe od pozostałych pakietów, `launch_ros` i kontrolerów
należy dodać wraz z rzeczywistymi plikami launch.
