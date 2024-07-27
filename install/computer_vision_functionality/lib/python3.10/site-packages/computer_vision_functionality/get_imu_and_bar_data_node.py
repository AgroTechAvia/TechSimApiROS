#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from .agrotechsimapi import *
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
import numpy as np
import pprint
from sensor_msgs.msg import Imu
from eagle_eye_msgs.msg import Barometer
import matplotlib.pyplot as plt
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener


class GetImuAndBar(Node):
    def __init__(self):
        super().__init__('GetImuAndBar_node')
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_bar = self.create_publisher(Barometer, 'barometer', 10)
        self.publisher_imu = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.altitude_at_start = None
        
        
        self.declare_parameter('host_ip', "172.18.96.1")
        HOST = "172.18.96.1"#self.get_parameter('host_ip').get_parameter_value().string_value
        PORT = 41451
        self.airsim_client = MultirotorClient(ip = HOST,port = PORT)
        self.is_connected_to_server = self.connect_to_server()
        
        

    def connect_to_server(self) -> bool:
        """
        Connects to the airsim API and returns a status flag

        Returns:
            bool: connecting status
        """

        try:
            self.get_logger().info("Connecting to server...") 
            self.airsim_client.confirmConnection()
            self.get_logger().info("Connection successful!") 

            return True
        except:
            self.get_logger().info("Connection error")

            return False    

   
    def timer_callback(self):
        if self.is_connected_to_server is True:
            msg = Imu()
            try:
                bar_data = self.airsim_client.getBarometerData()
                imu_data = self.airsim_client.getImuData()
                if self.altitude_at_start is None:
                    self.altitude_at_start = bar_data.altitude

                try:

                    imu = Imu()

                    imu.linear_acceleration.x = imu_data.linear_acceleration.x_val
                    imu.linear_acceleration.y = imu_data.linear_acceleration.y_val
                    imu.linear_acceleration.z = imu_data.linear_acceleration.z_val
                
                    imu.angular_velocity.x = imu_data.angular_velocity.x_val
                    imu.angular_velocity.y = imu_data.angular_velocity.y_val
                    imu.angular_velocity.z = imu_data.angular_velocity.z_val
                    
                    imu.orientation.x = imu_data.orientation.x_val
                    imu.orientation.y = imu_data.orientation.y_val
                    imu.orientation.z = imu_data.orientation.z_val
                    imu.orientation.w = imu_data.orientation.w_val
                    
                    imu.header.frame_id = "imu"
                    imu.header.stamp = self.get_clock().now().to_msg()

                    bar = Barometer()
                    bar.altitude = bar_data.altitude - self.altitude_at_start

                    self.publisher_imu.publish(imu)
                    self.publisher_bar.publish(bar)
                    self.tf_handler(bar, imu)

                except Exception as e:
                    print(e)

            except Exception as e:
                    print(e)

    def tf_handler(self, bar, imu):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_frame'
        t.child_frame_id = 'base_link'
        t.transform.rotation = imu.orientation
        t.transform.translation.z = bar.altitude

        self.tf_broadcaster.sendTransform(t)


def main(args = None):
    rclpy.init(args = args)

    driver = GetImuAndBar()
    rclpy.spin(driver)
    
    rclpy.shutdown()


        