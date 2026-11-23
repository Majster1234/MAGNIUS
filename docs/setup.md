# Przygotowanie środowiska

## Wymagania

Docelowe środowisko to Ubuntu 22.04 i ROS 2 Humble.
Poniższe polecenia są instrukcją do ręcznego wykonania. Utworzenie szkieletu
projektu nie instaluje pakietów ani nie modyfikuje systemu.

Najpierw skonfiguruj system i repozytorium pakietów ROS zgodnie z
[oficjalną instrukcją instalacji Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Jeżeli strona dokumentacji jest niedostępna, dostępne jest także
[źródło instrukcji dla Humble](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst).

Po przygotowaniu repozytorium zainstaluj bazę ROS i narzędzia deweloperskie:

```bash
sudo apt update
sudo apt install ros-humble-ros-base ros-dev-tools
```

`ros-dev-tools` dostarcza narzędzia deweloperskie, w tym `colcon` i `rosdep`.
Projekt potrzebuje `ament_cmake`, `ament_python` i zależności zadeklarowanych
w manifestach; panel korzysta z systemowych pakietów FastAPI, Uvicorn i psutil.

## Zależności workspace

W nowym terminalu Bash wczytaj środowisko:

```bash
source /opt/ros/humble/setup.bash
# Z katalogu głównego repozytorium:
cd ros2_ws
```

Jeśli `rosdep` nie był jeszcze inicjalizowany na tym komputerze, wykonaj
jednorazowo:

```bash
sudo rosdep init
```

Następnie zaktualizuj bazę i sprawdź zależności:

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

## Budowanie

Najpierw zbuduj frontend zgodnie z [instrukcją panelu](web_status.md).
Następnie z katalogu `ros2_ws`, po wczytaniu ROS 2 Humble:

```bash
colcon list
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix robot_description
ros2 pkg prefix mks_servo_can
ros2 pkg prefix robot_hardware
ros2 pkg prefix robot_bringup
ros2 pkg prefix robot_system_monitor
ros2 pkg prefix robot_web_bridge
```

`colcon list` powinien wykryć sześć pakietów tego projektu.
Po udanym budowaniu każde polecenie `ros2 pkg prefix` powinno wskazać pakiet
w lokalnym katalogu `install`. Cztery pakiety CMake nadal zawierają zasoby
i szablony. Dwa pakiety Python dostarczają węzły diagnostyczne i testy.

CMake instaluje README każdego pakietu oraz katalogi zasobów modelu
i konfiguracji. Pliki `.gitkeep` nie są instalowane. Katalogi `include`, `src`
i `test` są miejscem na przyszły kod i nie są eksportowane jako biblioteka.

## Codzienna sesja pracy

Po instalacji środowiska możesz otwierać przygotowane terminale bez ręcznego
wczytywania skryptów `setup.bash`:

```bash
# Z katalogu głównego repozytorium:
./start_ros2.sh
```

Każde okno wczyta Humble oraz dostępny lokalny workspace. Przy pierwszym
budowaniu, gdy katalog `install` jeszcze nie istnieje, wczytany zostanie tylko
Humble. Po budowaniu wykonaj `source install/local_setup.bash` w każdym oknie,
w którym potrzebujesz nowych pakietów, albo otwórz nowy zestaw terminali.

Skrypt nie buduje projektu ani nie uruchamia węzłów automatycznie. Polecenie
startowe aplikacji można przekazać po `--`; szczegóły są w
[głównym README](../README.md#terminale-do-pracy-z-ros-2).

## Następny etap implementacji

Po dodaniu modelu, adaptera i plików launch potrzebne będą odpowiednie
zależności ROS, m.in. `xacro`, `robot_state_publisher`, `ros2_control`,
`ros2_controllers` i opcjonalnie `rviz2`. Dodaj je w odpowiednich manifestach
i ponownie wykonaj `rosdep install`; obecny szkielet nie korzysta z tych pakietów.

Panel uruchamia `ros2 launch robot_bringup web_status.launch.py`.
Nie ma jeszcze launch sterowania robotem. Szablony `.example.yaml`
nie stanowią gotowych parametrów ROS 2, a wartości `null` trzeba zastąpić
rzeczywistymi danymi podczas implementacji ich obsługi.

## Przygotowanie CAN

Podłączony U2C jest wykrywany jako `can0` przez systemowy sterownik `gs_usb`.
Prędkość potwierdzona dla napędów to **500000 bit/s**. Trwałą konfigurację
systemd i instalację `can-utils` wykonuje `scripts/install_socketcan.sh`.
Instrukcja znajduje się w [integracji SocketCAN](socketcan.md).

Konfiguracja systemowa jest niezależna od terminali i pakietów ROS.
Firmware urządzeń nie jest aktualizowany przez ten skrypt. Dane protokołu
i sprzętu opisuje [dokumentacja CAN](can_protocol.md).
