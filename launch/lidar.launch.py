import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip_address',
            default_value='172.18.96.1',
            description='IP address to use for the nodes'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='Port to use for the nodes'
        ),
        Node(
            package = 'computer_vision_functionality',
            executable = 'read_lidar_point_cloud_node',
            name='read_lidar_point_cloud_node',
            output='screen',
            parameters=
            [
                {'host_ip':LaunchConfiguration('ip_address')},
                {'port':LaunchConfiguration('port')}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_scan',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.15',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_scan'
            ]
        )
    ])
    