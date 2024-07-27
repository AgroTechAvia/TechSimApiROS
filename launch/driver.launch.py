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

    launch_description = LaunchDescription()

    declare_ip_address_cmd = DeclareLaunchArgument(
        'ip_address',
        default_value='172.18.96.1',  
        description='IP address to use for the nodes'
    )

    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value= '41451',  
        description='Port to use for the nodes'
    )

    get_image_from_sim_node = Node(
        package = 'computer_vision_functionality',
        executable = 'get_image_from_sim_node',
        name='get_image_from_sim_node',
        output='screen',
        parameters=[
            {'host_ip':LaunchConfiguration('ip_address')},
            {'port':LaunchConfiguration('port')}
        ]
    )

    recognition_of_aruco_marker_node = Node(
        package = 'computer_vision_functionality',
        executable = 'recognition_of_aruco_marker_node',
        name='recognition_of_aruco_marker_node',
        output='screen'
    )

    read_lidar_point_cloud_node = Node(
        package = 'computer_vision_functionality',
        executable = 'read_lidar_point_cloud_node',
        name='read_lidar_point_cloud_node',
        output='screen',
        parameters=[
            {'host_ip':LaunchConfiguration('ip_address')},
            {'port':LaunchConfiguration('port')}
        ]
    )

    rviz2_launcher = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'rviz_config': '/home/user/.rviz2/sim_config.rviz'}]
        )

    launch_description.add_action(declare_ip_address_cmd)
    launch_description.add_action(declare_port_cmd)
    launch_description.add_action(get_image_from_sim_node)
    launch_description.add_action(recognition_of_aruco_marker_node)
    launch_description.add_action(read_lidar_point_cloud_node)
    launch_description.add_action(rviz2_launcher)

    return launch_description
    
    '''LaunchDescription([
        Node(
            package='computer_vision_functionality',
            executable='get_image_from_sim_node',
            name='get_image_from_sim_node',
            output='screen',
            parameters=[
                {'host_ip': host_ip}
            ]
        )
    ])'''