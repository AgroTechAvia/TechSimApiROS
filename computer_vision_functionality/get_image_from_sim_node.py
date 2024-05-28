#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from .agrotechsimapi import *

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image,CameraInfo

import numpy as np

class ImageFromAirsimNode(Node):

    def __init__(self):

        """
        When an instance of the Node class is initialized,
        a client is created to connect to AirSim, 
        as well as a publisher with a timer that will save the image 
        from the camera to the topic
        """

        super().__init__("image_from_sim")
        self.get_logger().info("Image_from_sim_node has been started")
        self.declare_parameter('host_ip', "172.18.96.1")
        HOST = self.get_parameter('host_ip').get_parameter_value().string_value
        self.sim_client = MultirotorClient(ip = HOST, port = 41451)
        self.is_connected_to_server = self.connect_to_server()

        self.cv_bridge = CvBridge()

        self.image_from_sim_publisher_ = self.create_publisher(msg_type = Image, 
                                                                topic = "/drone_vision/image_from_sim",
                                                                qos_profile = 10)
        self.camera_info_publisher_ = self.create_publisher(msg_type = CameraInfo,
                                                            topic = "/drone_vision/camera_info",
                                                            qos_profile = 10)
        self.camera_info = None

        
        self.publisher_timer_ = self.create_timer(timer_period_sec = 0.05, callback = self.image_callback)

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
        
        except NameError:
            self.get_logger().info("Connection error")
            self.get_logger().info(str(NameError))
            return False

    def get_image_from_sim(self) -> np.ndarray:
        """
        Accesses the API and receives an image from the camera, 
        converts it into a format for working in openCV

        Returns:
            ndarray: openCV image
        """

        raw_image_from_airsim = self.sim_client.simGetImage(camera_name = "0", image_type =  ImageType.Scene)
        raw_cv2_image = cv2.imdecode(string_to_uint8_array(bstr = raw_image_from_airsim), flags = cv2.IMREAD_UNCHANGED)
            
        return raw_cv2_image

    def image_callback(self):
        """
        Saves the image from the camera to the topic 
        if there was a connection to the server
        """

        if self.is_connected_to_server is True:
            image_to_bridge = self.get_image_from_sim()
            self.GetCameraInfo()
            

            rgb_image_to_bridge = cv2.cvtColor(src = image_to_bridge, code = cv2.COLOR_RGBA2RGB)
            image_to_msg = self.cv_bridge.cv2_to_imgmsg(cvim = rgb_image_to_bridge, encoding = "rgb8")
            image_to_msg.header.frame_id = 'camera'
            image_to_msg.header.stamp = self.camera_info.header.stamp
            self.camera_info_publisher_.publish(msg = self.camera_info)
            self.image_from_sim_publisher_.publish(msg = image_to_msg)
        
    #publish camera parameters
    def GetCameraInfo(self):
        if self.camera_info is None:
            self.camera_info = CameraInfo()
            self.camera_info.header.frame_id = "camera"
            self.camera_info.width = 640
            self.camera_info.height = 480 
            fx = 1000.0  # x-axis focal length
            fy = 1000.0  # н-axis focal length
            cx = 640.0   # optical center x-axis
            cy = 480.0   # optical center н-axis
            self.camera_info.distortion_model = "plumb_bob"
            
            self.camera_info.p = [fx, 0.0,   cx, 0.0,
                                  0.0,  fy,  cy, 0.0, 
                                  0.0,  0.0,   1.0, 0.0]
            self.camera_info.k = [fx, 0.0,   cx,
                                  0.0,  fy,  cy, 
                                  0.0,  0.0,   1.0]
            
            k1 = 0.1  # radial distortion factor
            k2 = 0.01  # second radial distortion factor
            p1 = 0.001  # first tangential distortion factor
            p2 = 0.002  # second tangential distortion factor
            self.camera_info.d = [k1, k2, p1, p2, 0.0]
        self.camera_info.header.stamp = self.get_clock().now().to_msg()

def main(args = None):
    rclpy.init(args = args)

    image_from_airsim_node = ImageFromAirsimNode()
    rclpy.spin(image_from_airsim_node)

    rclpy.shutdown()
        
       
