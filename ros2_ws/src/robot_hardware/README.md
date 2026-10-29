# robot_hardware

Plugin C++ `robot_hardware/CanSystem` (`hardware_interface::SystemInterface`) dla sześciu osi
`joint_1`–`joint_6`, zgodny z `ros2_control` w ROS 2 Humble.

- `include/robot_hardware/` — nagłówki adaptera;
- `src/` — cykl życia, odczyt i zapis oraz konwersja jednostek;
- `test/` — walidacja konfiguracji, interfejsy i ładowanie pluginu.

Adapter używa `BusRuntime` i wspólnego publikatora diagnostyki. `read()` i `write()`
korzystają z prób blokady i cache; nie czekają na IO. Wymagana konfiguracja:
adresy, przekładnie, kierunki, offsety, limity, błąd śledzenia i kod przyspieszenia MKS.

Aktywacja wymaga świeżych pozycji, ENABLE, braku stall, trybu CAN, lease źródła
oraz odbioru synchronizacji i zatrzymania. Plugin nie ustawia automatycznie trybu
ani ENABLE. Parametry i uruchomienie: [strategia CAN](../../../docs/can_strategy.md).

Punkt odniesienia:
[tworzenie komponentu sprzętowego dla Humble](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).
