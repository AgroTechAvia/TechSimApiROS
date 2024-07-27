import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from eagle_eye_msgs.srv import StartPoseHold, StopPoseHold
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
import cv2
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from eagle_eye_msgs.msg import Barometer
import copy
from math import atan2, pi
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Example(Node):
    def __init__(self):
        super().__init__('example')
        self.callback_group = ReentrantCallbackGroup()
        timer_period = 0.1  # seconds
        self.__timer = self.create_timer(timer_period, self.__timer_callback)
        self.__cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.__subscription_alt = self.create_subscription(
            Barometer,
            'barometer',
            self.__alt_callback,
            10)
        self.__subscription_imu = self.create_subscription(
            Imu,
            'imu',
            self.__imu_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.__altitude = 0
        self.__imu = [0,0,0]
        self.__base_frame = None
        self.__frame = None
        self.__bridge = CvBridge()
        self.__camera_cb = self.create_subscription(Image,'/drone_vision/image_from_airsim',self.__camera_cb,1)
        
    def __imu_callback(self,data):
        q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.__imu[0] = roll
        self.__imu[1] = pitch
        self.__imu[2] = yaw
        print("angles: ",roll, pitch, yaw)

    def __alt_callback(self, data):
        self.__altitude=data.altitude
        print("alt", self.__altitude)

    def __camera_cb(self, data):
        self.__frame = self.__bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
    def __timer_callback(self):
        ### you can code here
        a=1

        # print(self.__pose_hold_active)
        



def main(args=None):
    rclpy.init(args=args)
    example = Example()
    rclpy.spin(example)
    example.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
