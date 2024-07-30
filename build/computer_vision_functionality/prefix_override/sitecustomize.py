import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/ros2_ws/src/TechSimApiROS/install/computer_vision_functionality'
