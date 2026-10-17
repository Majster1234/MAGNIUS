# Architektura projektu

## Platforma i stan początkowy

- System docelowy: Ubuntu 22.04, ROS 2 Humble, workspace budowany przez `colcon`.
- Sześć osi o stałych nazwach `joint_1`–`joint_6`.
- Mostek USB–CAN: BIGTREETECH U2C v2.1, docelowo udostępniony przez SocketCAN.
- Napędy: MKS SERVO42D CAN; rewizje płytek i firmware wymagają zapisania.
- C++ dla przyszłej biblioteki komunikacji i adaptera sprzętowego.
- Wszystkie pakiety są obecnie szkieletami `ament_cmake`, bez kodu sterowania.

## Docelowy przepływ danych

```text
Kontroler trajektorii (ros2_control)
                 ↕
robot_hardware: sześć osi, konwersja jednostek, cykl życia
                 ↕
mks_servo_can: protokół MKS i transport SocketCAN
                 ↕
Linux can0 ↔ USB / U2C v2.1 ↔ magistrala CAN ↔ napędy
```

`robot_description` dostarczy model i opis osi dla warstwy ROS.
`robot_bringup` będzie składał model, kontrolery i konfigurację sprzętu podczas
uruchamiania. Diagram opisuje docelową architekturę, nie istniejącą implementację.

## Odpowiedzialności pakietów

| Pakiet | Przeznaczenie | Zawartość początkowa |
| --- | --- | --- |
| `robot_description` | URDF/Xacro, geometria i ustawienia RViz | Katalogi `urdf`, `meshes`, `rviz` |
| `mks_servo_can` | Transport SocketCAN, kodowanie i dekodowanie protokołu producenta | Katalogi `include`, `src`, `test` |
| `robot_hardware` | Jeden komponent `hardware_interface::SystemInterface` dla sześciu osi | Katalogi `include`, `src`, `test` |
| `robot_bringup` | Pliki launch i konfiguracja całego robota | Katalog `launch`, trzy szablony YAML |

Biblioteka `mks_servo_can` powinna oddzielać transport od kodowania wiadomości,
aby przyszłe testy protokołu nie wymagały napędów. Adapter `robot_hardware`
będzie odpowiadał za mapowanie osi, przekładnie, znaki, offsety oraz konwersję
jednostek między ROS i napędami.

Obecne manifesty deklarują tylko używany system budowania `ament_cmake`.
Zależności od `hardware_interface`, `pluginlib`, `rclcpp`, biblioteki CAN,
kontrolerów i narzędzi wizualizacji należy dodać wraz z korzystającym z nich
kodem. Nie ma jeszcze eksportowanych bibliotek, pluginów, tematów ani usług.

Przyszły adapter należy oprzeć na API Humble, opisanym w
[instrukcji tworzenia komponentu sprzętowego ros2_control](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).
Eksport pluginu będzie dodany dopiero razem z rzeczywistą biblioteką.

## Konwencje i dane do ustalenia

- Oś `joint_1` oznacza pierwszą oś łańcucha od podstawy, a `joint_6` ostatnią
  przed narzędziem. Tych nazw należy używać w modelu, adapterze i kontrolerze.
- Jednostki po stronie ROS: radiany, rad/s i rad/s²; długości modelu w metrach.
- Wymiary, osie obrotu, układy współrzędnych, masy i bezwładności wymagają danych
  mechanicznych. Szkielet nie dostarcza zastępczej geometrii.
- Dla każdego przegubu należy ustalić CAN ID, firmware, przełożenie, kierunek,
  offset zera, limity pozycji, prędkości i przyspieszenia oraz metodę bazowania.
- Przed implementacją trzeba określić częstotliwość cyklu sterowania, sposób
  odczytu pozycji i obsługę timeoutów oraz błędów napędów.
- Wzór konwersji i znaczenie pól przełożenia opisuje README `robot_bringup`.

Pliki CAD należy umieszczać w `hardware/cad`, schematy w `hardware/wiring`,
a dokumentację odpowiadającą posiadanym wersjom urządzeń w `hardware/datasheets`.
Model wynikowy dla ROS będzie należał do `robot_description`.
