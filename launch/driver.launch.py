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
            package = 'computer_vision_functionality',
            executable = 'read_lidar_point_cloud_node',
            name='read_lidar_point_cloud_node',
            output='screen',
            parameters=[
                {'host_ip':LaunchConfiguration('ip_address')},
                {'port':LaunchConfiguration('port')}
            ]
        ),
        Node(
            package='computer_vision_functionality',
            executable='sim_driver_node',
            name='sim_driver_node',
            output='screen',
            parameters=[
                {'host_ip': LaunchConfiguration('ip_address')}
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[
                ('cloud_in', '/drone_vision/point_cloud') 
            ],
            parameters=[   
                {'min_height': -3.0},
                {'max_height': 10.0},
                {'range_max': 10.0},
                {'target_frame': "laser_scan"},
                {'angle_increment': 0.01}
                    
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
        ),
                 Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_scan',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_frame', '--child-frame-id', 'odom'
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
            name='base_link_to_lidar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_frame', '--child-frame-id', 'map'
            ]
        ),
                            Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0.7071068', '--qw', '0.7071068',
                '--frame-id', 'base_frame', '--child-frame-id', 'odom'
            ]
        ),
                 Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[
                '--x', '0.05', '--y', '0', '--z', '0.1',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'forward_camera'
            ]
        ),
                 Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[
                '--x', '0.0', '--y', '0', '--z', '0.0',
                '--qx', '0', '--qy', '0.7071068', '--qz', '0', '--qw', '0.7071068',
                '--frame-id', 'base_link', '--child-frame-id', 'down_camera'
            ]
        )

    ])
    ''',
        Node(
            package = 'computer_vision_functionality',
            executable = 'read_lidar_point_cloud_node',
            name='read_lidar_point_cloud_node',
            output='screen',
            parameters=[
                {'host_ip':LaunchConfiguration('ip_address')},
                {'port':LaunchConfiguration('port')}
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[
                ('cloud_in', '/drone_vision/point_cloud') 
            ],
            parameters=[   
                {'min_height': -3.0},
                {'max_height': 10.0},
                {'target_frame': "laser_scan"},
                {'angle_increment': 0.01}
                    
            ]
        )'''
    

    '''launch_description = LaunchDescription()

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
    cloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/drone_vision/point_cloud') 
        ],
        parameters=[   
            {'min_height': -3.0},
            {'max_height': 10.0},
            {'target_frame': "laser_scan"},
            {'angle_increment': 0.01}
                
        ]
    )
    tf_laser_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_scan',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.15',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_scan'
        ]
    )
    tf_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_scan',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_frame', '--child-frame-id', 'odom'
            ]
        )
    tf_lidar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.15',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar'
            ]
        )
    tf_map = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_frame', '--child-frame-id', 'map'
            ]
        )
    tf_odom2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0.7071068', '--qw', '0.7071068',
                '--frame-id', 'base_frame', '--child-frame-id', 'odom'
            ]
        )
    tf_camera_forward = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[
                '--x', '0.05', '--y', '0', '--z', '0.1',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'forward_camera'
            ]
        )
    tf_camera_down = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[
                '--x', '0.0', '--y', '0', '--z', '0.0',
                '--qx', '0', '--qy', '0.7071068', '--qz', '0', '--qw', '0.7071068',
                '--frame-id', 'base_link', '--child-frame-id', 'down_camera'
            ]
        )

'''

    '''rviz2_launcher = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'rviz_config': '/home/user/.rviz2/sim_config.rviz'}]
        )'''

    '''launch_description.add_action(declare_ip_address_cmd)
    launch_description.add_action(declare_port_cmd)
    launch_description.add_action(get_image_from_sim_node)
    launch_description.add_action(recognition_of_aruco_marker_node)
    launch_description.add_action(read_lidar_point_cloud_node)
    launch_description.add_action(cloud_to_laserscan)

    launch_description.add_action(tf_laser_scan)
    launch_description.add_action(tf_odom)
    launch_description.add_action(tf_lidar)
    launch_description.add_action(tf_map)
    launch_description.add_action(tf_odom2)
    launch_description.add_action(tf_camera_forward)
    launch_description.add_action(tf_camera_down)

    return launch_description'''

        

   