import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

host_ip = os.environ.get('HOST_WSL_IP')
def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='computer_vision_functionality',
            executable='get_image_from_airsim_node',
            name='get_image_from_airsim_node',
            output='screen',
            parameters=[
                {'host_ip': host_ip}
            ]
        ),
        Node(
            package='computer_vision_functionality',
            executable='read_lidar_point_cloud_node',
            name='read_lidar_point_cloud_node',
            output='screen',
            parameters=[
                {'host_ip': host_ip}
            ]
        ) 
        ,
        
         Node(
            package='computer_vision_functionality',
            executable='get_imu_and_bar_data_node',
            name='get_imu_and_bar_data_node',
            output='screen',
            parameters=[
                {'host_ip': host_ip}
            ]
        ),
        Node(
            package='computer_vision_functionality',
            executable='sim_driver_node',
            name='sim_driver_node',
            output='screen',
            parameters=[
                {'host_ip': host_ip}
            ]
        ),
        #  Node(
        #     package='pointcloud_to_laserscan',
        #     executable='pointcloud_to_laserscan_node',
        #     name='pointcloud_to_laserscan_node',
        #     output='screen',
        #     remappings=[
        #         ('cloud_in', '/drone_vision/point_cloud'),
        #         ('scan', '/drone_vision/LaserScan')
        #     ],
        #      parameters=[
                
        #         {'min_height': -0.5},
        #         {'max_height': 10.0},
        #         {'target_frame': "lidar"},
        #         {'angle_increment': 0.01}
                
        #     ]

        # ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_scan',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.15',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_scan'
            ]
        ),
                 Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.15',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar'
            ]
        ),
                 Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[
                '--x', '0.05', '--y', '0', '--z', '0.1',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'camera'
            ]
        )

 
    ])