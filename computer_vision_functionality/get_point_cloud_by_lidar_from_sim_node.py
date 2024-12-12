#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

from agrotechsimapi import SimClient

import numpy as np
import pprint
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2, PointField, LaserScan

class LidarReaderClass(Node):

    def __init__(self):

        """
        When an instance of the Node class is initialized,
        a client is created to connect to AirSim, 
        as well as a publisher with a timer that will save the point cloud 
        from the lidar to the topic
        """
        
        super().__init__("point_cloud_from_sim")

        self.get_logger().info("Point_cloud_by_lidar_from_sim_node has been started")
        self.declare_parameter('host_ip', "172.18.96.1")
        self.declare_parameter('port', 8080)
        
        HOST = self.get_parameter('host_ip').get_parameter_value().string_value
        PORT = self.get_parameter('port').get_parameter_value().integer_value
        
        self.sim_client = SimClient(address = HOST, port = PORT)
        
        self.is_connected_to_server = self.connect_to_server()

        self.point_cloud_from_airsim_publisher_ = self.create_publisher(msg_type = LaserScan, 
                                                                topic = "/drone_sensors/laser_scan",
                                                                qos_profile =  10)
        
        
        self.publisher_timer_ = self.create_timer(timer_period_sec = 0.1, callback = self.lidar_callback)

    def connect_to_server(self) -> bool:
        """
        Connects to the airsim API and returns a status flag

        Returns:
            bool: connecting status
        """

        try:
            self.get_logger().info("Connecting to server...") 
            self.sim_client.is_connected()
            self.get_logger().info("Connection successful!") 

            return True
        
        except:
            self.get_logger().info("Connection error")

            return False
        

    def lidar_callback(self):
        """
        Saves the point cloud from the lidar to the topic 
        if there was a connection to the server
        """

        if self.is_connected_to_server is True:

            ranges_data = self.sim_client.get_laser_scan(angle_min=-np.pi, 
                                                        angle_max=np.pi, 
                                                        range_min = 0.1, 
                                                        range_max=10, 
                                                        num_ranges=360, 
                                                        is_clear=True)

            
            ranges_data = ranges_data[::-1]

            laser_scan = LaserScan()
            laser_scan.header.frame_id = "laser_scan"
            laser_scan.header.stamp = self.get_clock().now().to_msg()

            laser_scan.angle_min = -np.pi 
            laser_scan.angle_max = np.pi    
            laser_scan.angle_increment = np.deg2rad(1)  
            laser_scan.time_increment = 0.0
            laser_scan.scan_time = 0.1
            laser_scan.range_min = 0.1
            laser_scan.range_max = 10.0
            
            laser_scan.ranges = ranges_data 
            
            self.point_cloud_from_airsim_publisher_.publish(laser_scan)

def main(args = None):
    rclpy.init(args = args)

    points_cloud_from_airsim_node = LidarReaderClass()
    rclpy.spin(points_cloud_from_airsim_node)

    rclpy.shutdown()
