# robot_bringup

Pliki launch i konfiguracja całego robota.
`web_status.launch.py` uruchamia monitor, diagnostykę CAN i backend panelu,
bez kontrolerów oraz poleceń ruchu. Szczegóły: [panel diagnostyczny](../../../docs/web_status.md).
`motion_control.launch.py` ładuje pełny URDF i konfigurację kontrolerów z jawnych
ścieżek. Hardware i kontroler trajektorii pozostają nieaktywne. Szczegóły:
[strategia CAN](../../../docs/can_strategy.md).

## Szablony konfiguracji

| Plik w `config/` | Przeznaczenie |
| --- | --- |
| `can.example.yaml` | Mostek U2C, docelowy interfejs SocketCAN i bitrate |
| `joints.example.yaml` | Sześć osi: CAN ID, firmware, przełożenia, kierunki, offsety i limity |
| `controllers.example.yaml` | Zarys parametrów `controller_manager`, broadcastera stanu i kontrolera trajektorii |
| `ros2_control.example.xacro` | Fragment rzeczywistego modelu: plugin i parametry sześciu osi |

Wartości `null` są celowo nieustalone; nie oznaczają zera ani wartości domyślnej.
Nie podawaj tych szablonów bezpośrednio jako `--params-file`. Pliki CAN i osi
opisują format projektu, nie standardowe parametry ROS 2. Makro `mks_control`
wczytuje uzupełniony YAML osi do fragmentu `ros2_control` w rzeczywistym modelu.
Profil kontrolerów zaczyna od 50 Hz; wymaga odbioru na sprzęcie. Samo zastąpienie
`null` nie dostarczy brakującego modelu ani konfiguracji MoveIt.

## Znaczenie parametrów osi

- `can_id` — identyfikator napędu ustalony zgodnie z instrukcją firmware;
  wymagane jednoznaczne mapowanie sześciu urządzeń.
- `firmware_version` — rzeczywista wersja firmware danej osi.
- `gear_ratio` — dodatnia liczba obrotów wału silnika na jeden obrót przegubu.
- `direction` — `1` lub `-1`, określa zgodność kierunku silnika z dodatnim
  kierunkiem osi URDF.
- `zero_offset_rad` — przesunięcie zera w radianach po stronie przegubu.
- `limits.lower_rad`, `upper_rad` — dolny i górny limit pozycji przegubu.
- `tracking_error_rad` — dopuszczalny błąd śledzenia przegubu.
- `mks_acceleration_code` — kod MKS 1–255, odrębny od przyspieszenia SI.
- `limits.velocity_rad_s`, `acceleration_rad_s2` — maksymalne wartości
  prędkości i przyspieszenia przegubu, dodatnie, po uwzględnieniu przekładni.

Docelowa konwencja przeliczenia, po przeliczeniu odczytu napędu na radiany
wału silnika:

```text
joint_position_rad = direction * motor_position_rad / gear_ratio + zero_offset_rad
```

Offset i metoda bazowania muszą odpowiadać tej konwencji. Limity powinny być
spójne z przyszłym modelem URDF. Szablon kontrolera przewiduje komendę pozycji
i odczyt pozycji dla wszystkich sześciu osi; obsługuje go `robot_hardware/CanSystem`.
`update_rate` oznacza częstotliwość `controller_manager` w Hz i wymaga dobrania
do wydajności napędów, protokołu oraz obciążenia magistrali.

## Instalacja

CMake instaluje `launch/` i `config/` do `share/robot_bringup`.
Manifest deklaruje zależności diagnostyki, adaptera, kontrolerów i uruchamiania.
`motion_gate.py` kieruje trajektorię lub jog do jednego JTC oraz wystawia lease źródła.
