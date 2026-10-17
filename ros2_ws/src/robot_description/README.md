# robot_description

Miejsce na model sześciu osi `joint_1`–`joint_6`, od podstawy do narzędzia:

- `urdf/` — przyszłe pliki URDF/Xacro i opis integracji z `ros2_control`;
- `meshes/` — geometria wizualna i kolizyjna;
- `rviz/` — konfiguracja wizualizacji.

Katalogi są obecnie puste. Model należy przygotować na podstawie rzeczywistych
wymiarów, osi obrotu, mas i limitów. Nie dodano fikcyjnej geometrii ani limitów.
Jednostki: metry i radiany.

Pakiet używa `ament_cmake` i instaluje zasoby w `share/robot_description`.
Zależności od Xacro, narzędzi publikacji modelu i wizualizacji należy dodać
wraz z korzystającymi z nich plikami.
