#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from .agrotechsimapi import *
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
import numpy as np
import pprint
import matplotlib.pyplot as plt

class EduSimDriver(Node):
    def __init__(self):
        super().__init__('EduSimDriver_node')
        
        self.subscription = self.create_subscription(
             Twist,
             'cmd_vel',
             self.cmd_callback,
             1)
        
        self.declare_parameter('host_ip', "172.18.96.1")
        HOST = self.get_parameter('host_ip').get_parameter_value().string_value
        PORT = 41451
        self.airsim_client = MultirotorClient(ip = HOST)
        self.is_connected_to_server = self.connect_to_server()

        self.airsim_client.reset()
        self.airsim_client.enableApiControl(True)
        print("API Control enabled: %s" % self.airsim_client.isApiControlEnabled())
        self.airsim_client.setMode(AirMultirotorMode.STABILIZE)
        self.airsim_client.setMode(AirMultirotorMode.COPTER)



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

    def cmd_callback(self, data):
        
        pitch = self.constrain(data.linear.x, 1)
        roll = self.constrain(data.linear.y, 1)
        yaw = self.constrain(data.angular.z, 1)
        trust = self.constrain(data.linear.z, 1)  
        
        if(data.angular.x > 0):
            self.airsim_client.armDisarm(True)
        else:
            self.airsim_client.armDisarm(False)
        if(data.angular.y > 0):
            self.airsim_client.setMode(AirMultirotorMode.PLANE)
        else:
            self.airsim_client.setMode(AirMultirotorMode.COPTER)
        #self.get_logger().info(str(roll) + str(pitch) + str(yaw) + str(trust))
        self.airsim_client.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, trust, 0.05)
        

    def constrain(self, value, _constrain):
        if value >  _constrain:
            value =  _constrain
        elif value <  - _constrain:
            value =  - _constrain
        return value

def main(args = None):
    rclpy.init(args = args)

    driver = EduSimDriver()
    rclpy.spin(driver)
    rclpy.shutdown()


        