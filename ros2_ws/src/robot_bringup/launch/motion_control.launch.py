"""Uruchomienie po uzupełnieniu i odbiorze modelu oraz parametrów sprzętowych."""
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    model_file = Path(LaunchConfiguration('robot_description_file').perform(context))
    controllers_file = Path(LaunchConfiguration('controllers_file').perform(context))
    if not model_file.is_file() or not controllers_file.is_file():
        raise RuntimeError('Podaj kompletny plik URDF i konfigurację kontrolerów; szablony .example nie są gotowe.')
    if '.example.' in model_file.name or '.example.' in controllers_file.name:
        raise RuntimeError('Uzupełnij konfigurację sprzętu zamiast uruchamiać szablon .example.')
    model = model_file.read_text()
    systems = [element.attrib['name'] for element in ET.fromstring(model).findall('ros2_control')]
    if not systems:
        raise RuntimeError('URDF nie zawiera komponentu ros2_control.')
    manager = Node(package='controller_manager', executable='ros2_control_node', output='screen',
                   parameters=[str(controllers_file), {'robot_description': model,
                                'hardware_components_initial_state.inactive': systems}])
    gate = Node(package='robot_bringup', executable='motion_gate.py', output='screen')
    # Kontroler pozycyjny jest ładowany INACTIVE; aktywacja sprzętu/kontrolera jest jawna.
    return [manager, gate,
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster'], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_trajectory_controller', '--inactive'], output='screen'),
            RegisterEventHandler(OnProcessExit(target_action=manager,
                                 on_exit=[EmitEvent(event=Shutdown(reason='Controller manager zakończył pracę'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_description_file'), DeclareLaunchArgument('controllers_file'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'), OpaqueFunction(function=start),
    ])
