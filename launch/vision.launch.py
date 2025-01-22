import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

current_directory = os.path.dirname(os.path.abspath(__file__))
current_directory_split = current_directory.split('/')

remove_directory = "install/computer_vision_functionality/share/computer_vision_functionality"
remove_directory_split = remove_directory.split('/')

for remove_dir in remove_directory_split:
    current_directory_split.remove(remove_dir)

current_directory =  '/'.join(current_directory_split)

relative_path = "src/TechSimApiROS/launch/rviz_configs/vision_launch.rviz"
absolute_path = os.path.join(current_directory, relative_path)


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
            arguments=['-d', absolute_path]
        )
    ])
    

        

   