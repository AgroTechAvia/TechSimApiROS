#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

from .agrotechsimapi import *

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
        self.declare_parameter('port', 41451)
        
        HOST = self.get_parameter('host_ip').get_parameter_value().string_value
        PORT = self.get_parameter('port').get_parameter_value().integer_value
        
        self.sim_client = MultirotorClient(ip = HOST, port = PORT)
        self.is_connected_to_server = self.connect_to_server()


        self.point_cloud_from_airsim_publisher_ = self.create_publisher(msg_type = PointCloud2, 
                                                                topic = "/drone_vision/point_cloud",
                                                                qos_profile =  qos_profile_sensor_data)
        
        
        self.publisher_timer_ = self.create_timer(timer_period_sec = 0.1, callback = self.lidar_callback)

    def connect_to_server(self) -> bool:
        """
        Connects to the airsim API and returns a status flag

        Returns:
            bool: connecting status
        """

        try:
            self.get_logger().info("Connecting to server...") 
            self.sim_client.confirmConnection()
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

            for i in range(1,2):
                lidarData = self.sim_client.getLidarData()
                if (len(lidarData.point_cloud) < 3):
                    pass
                    self.get_logger().info("\tNo points received from Lidar data")
                else:
                    points = self.parse_lidarData(lidarData)

                    msg = PointCloud2()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'lidar'
                    msg.height = 1  # Unordered point cloud
                    msg.width = len(points)  # 100 points

                    point_cloud_data = points.tobytes()

                    msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    #PointField(name='ring', offset=12, datatype=PointField.FLOAT32, count=1),
                    ]

                    msg.is_bigendian = False
                    msg.point_step = 12  
                    msg.row_step = msg.point_step * msg.width  
                    msg.is_dense = True
                    msg.data = point_cloud_data

                    self.point_cloud_from_airsim_publisher_.publish(msg)
                    self.get_logger().info("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
        else:
            self.get_logger().info("\tNot connected")

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

def main(args = None):
    rclpy.init(args = args)

    points_cloud_from_airsim_node = LidarReaderClass()
    rclpy.spin(points_cloud_from_airsim_node)

    rclpy.shutdown()
