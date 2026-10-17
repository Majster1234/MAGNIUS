# robot_hardware

Miejsce na komponent C++ `hardware_interface::SystemInterface` dla sześciu osi
`joint_1`–`joint_6`, zgodny z `ros2_control` w ROS 2 Humble.

- `include/robot_hardware/` — przyszłe nagłówki adaptera;
- `src/` — cykl życia, odczyt i zapis oraz konwersja jednostek;
- `test/` — przyszłe testy adaptera i ładowania pluginu.

Adapter będzie korzystał z `mks_servo_can` i mapował napędy na przeguby.
Przeliczenia uwzględnią przełożenia, kierunki, offsety i jednostki ROS.
Planowany interfejs pozycyjny pokazuje szablon kontrolerów w `robot_bringup`.

Obecnie nie ma biblioteki, pliku eksportu pluginu ani aktywacji sprzętu.
Zależności od `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`
i `mks_servo_can` należy zadeklarować podczas implementacji.

Punkt odniesienia:
[tworzenie komponentu sprzętowego dla Humble](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).
