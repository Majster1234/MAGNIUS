# SI, joint_1..joint_6. Nieznane pozycje wymagają jawnego uzupełnienia do podglądu.
string model_id
string configuration_revision
string urdf
float64[6] seed
bool[6] known
bool[6] locked
float64[6] lower
float64[6] upper
float64[6] max_velocity
float64[6] max_acceleration
robot_control_interfaces/JointConfiguration[6] profiles
# Wyłącznie obliczenia; dostępność CAN nie jest warunkiem tej symulacji.
bool simulation_only
bool[6] simulated_start

# Jawne założenia dla nieobecnych osi fizycznego PTP; TCP niepotwierdzone.
bool allow_missing_execution
