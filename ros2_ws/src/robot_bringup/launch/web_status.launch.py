"""Panel i diagnostyka, bez uruchamiania kontrolerów ani napędów."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    monitor = Node(
        package='robot_system_monitor', executable='monitor', output='screen',
        parameters=[{'can_interface': LaunchConfiguration('can_interface')}])
    bridge = Node(
        package='robot_web_bridge', executable='bridge', output='screen',
        parameters=[{
            'lan_interface': LaunchConfiguration('lan_interface'),
            'lan_subnet': LaunchConfiguration('lan_subnet'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('lan_interface', default_value='enp2s0'),
        DeclareLaunchArgument('lan_subnet', default_value='192.168.1.0/24'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        # Błąd serwera nie może pozostawić osieroconego monitora.
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Backend zakończył działanie'))])),
        monitor, bridge,
    ])
