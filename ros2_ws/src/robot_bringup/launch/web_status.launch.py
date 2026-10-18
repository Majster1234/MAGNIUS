"""Panel i diagnostyka, bez uruchamiania kontrolerów ani napędów."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    monitor = Node(
        package='robot_system_monitor', executable='monitor', output='screen',
        parameters=[{'can_interface': LaunchConfiguration('can_interface')}])
    can_diagnostics = Node(
        package='robot_can_diagnostics', executable='can_diagnostics', output='screen',
        condition=IfCondition(LaunchConfiguration('standalone_can')),
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'bitrate': ParameterValue(LaunchConfiguration('can_bitrate'), value_type=int),
            'response_timeout_ms': ParameterValue(LaunchConfiguration('can_timeout_ms'), value_type=int),
            'manual_max_rpm': ParameterValue(LaunchConfiguration('manual_max_rpm'), value_type=int),
        }])
    bridge = Node(
        package='robot_web_bridge', executable='bridge', output='screen',
        parameters=[{
            'lan_interface': LaunchConfiguration('lan_interface'),
            'lan_subnet': LaunchConfiguration('lan_subnet'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'can_interface': LaunchConfiguration('can_interface'),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('lan_interface', default_value='enp2s0'),
        DeclareLaunchArgument('lan_subnet', default_value='192.168.1.0/24'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='500000'),
        DeclareLaunchArgument('can_timeout_ms', default_value='20'),
        DeclareLaunchArgument('manual_max_rpm', default_value='30'),
        DeclareLaunchArgument('standalone_can', default_value='true',
                              description='false, gdy właścicielem CAN jest plugin robot_hardware'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        # Błąd serwera nie może pozostawić osieroconego monitora.
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Backend zakończył działanie'))])),
        monitor, can_diagnostics, bridge,
    ])
