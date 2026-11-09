# robot_description

Miejsce na model sześciu osi `joint_1`–`joint_6`, od podstawy do narzędzia:

- `urdf/` — przyszłe pliki URDF/Xacro i opis integracji z `ros2_control`;
- `meshes/` — geometria wizualna i kolizyjna;
- `rviz/` — konfiguracja wizualizacji.

`urdf/preview_6dof.example.urdf` to zatwierdzony model demonstracyjny z prostych
brył dla panelu 3D. Sześć osi continuous, TCP, bez ros2_control i zewnętrznych
zasobów. Opis i oddzielenie od sterowania: `docs/robot_preview.md`.
Model fizycznego robota nadal wymaga rzeczywistych wymiarów, osi, mas i limitów.
Jednostki URDF: metry i radiany; konfiguracja w GUI: milimetry i stopnie.

Pakiet używa `ament_cmake` i instaluje zasoby w `share/robot_description`.
Zależności od Xacro, narzędzi publikacji modelu i wizualizacji należy dodać
wraz z korzystającymi z nich plikami.
