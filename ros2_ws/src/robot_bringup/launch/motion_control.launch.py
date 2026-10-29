"""Uruchomienie po uzupełnieniu i odbiorze modelu oraz parametrów sprzętowych."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


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
                   parameters=[str(controllers_file), LaunchConfiguration('execution_parameters'), {'robot_description': model,
                                'hardware_components_initial_state.inactive': systems}])
    gate = Node(package='robot_bringup', executable='motion_gate.py', output='screen')
    preview = Node(package='robot_state_publisher', executable='robot_state_publisher',
        name='state_publisher', namespace='/robot/preview', output='screen',
        condition=IfCondition(LaunchConfiguration('configuration_enabled')),
        parameters=[{'robot_description': Path(LaunchConfiguration('preview_urdf').perform(context)).read_text(),
                     'frame_prefix': 'preview/'}],
        remappings=[('/tf','/robot/preview/tf'),('/tf_static','/robot/preview/tf_static')])
    configuration = Node(package='robot_configuration', executable='configuration', output='screen',
        condition=IfCondition(LaunchConfiguration('configuration_enabled')),
        parameters=[{'configuration_dir': LaunchConfiguration('configuration_dir')}])
    # Kontroler pozycyjny jest ładowany INACTIVE; aktywacja sprzętu/kontrolera jest jawna.
    return [manager, gate, preview, configuration,
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster'], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_trajectory_controller', '--inactive'], output='screen'),
            RegisterEventHandler(OnProcessExit(target_action=manager,
                                 on_exit=[EmitEvent(event=Shutdown(reason='Controller manager zakończył pracę'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_description_file'), DeclareLaunchArgument('controllers_file'),
        DeclareLaunchArgument('execution_parameters', default_value=str(Path(get_package_share_directory('robot_bringup')) / 'config/execution.example.yaml')),
        DeclareLaunchArgument('configuration_enabled', default_value='true'),
        DeclareLaunchArgument('configuration_dir', default_value=str(Path.home() / '.local/state/robot/configuration')),
        DeclareLaunchArgument('preview_urdf', default_value=str(Path(get_package_share_directory('robot_description'))/'urdf/preview_6dof.example.urdf')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'), OpaqueFunction(function=start),
    ])
