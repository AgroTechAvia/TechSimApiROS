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
            default_value='41451',
            description='Port to use for the nodes'
        ),
        Node(
            package='computer_vision_functionality',
            executable='get_image_from_sim_node',
            name='get_image_from_sim_node',
            output='screen',
            parameters=[
                {'host_ip': LaunchConfiguration('ip_address')},
                {'port': LaunchConfiguration('port')}
            ]
        ),
        Node(
            package='computer_vision_functionality',
            executable='recognition_of_aruco_marker_node',
            name='recognition_of_aruco_marker_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/.rviz2/vision_launch.rviz']
        )
    ])
    

        

   