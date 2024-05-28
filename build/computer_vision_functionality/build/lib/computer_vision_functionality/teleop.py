import rclpy
from geometry_msgs.msg import Twist
import sys, select, os
from rclpy.qos import QoSProfile
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 3.0

LIN_VEL_STEP_SIZE = 0.05

msg = """
Control Your drone!
---------------------------
Moving around:
   q    w    e 
   a    s    d
   z    x    c
   r    t
w/x : increase/decrease pitch angle 
a/d : increase/decrease roll angle
q/e : increase/decrease yaw angle
z/c : increase/decrease trust
f/g : arm/disarm
r/t : drone/plane
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
settings = termios.tcgetattr(sys.stdin)
def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_roll, target_pitch, target_yaw, target_trust, arm, mode):
    st = "currently:\troll %s\tpitch  %s\tyaw  %s\ttrust  %s\tarm  %s\tmode %s " % (target_roll,target_pitch,target_yaw,target_trust,arm,mode)
    return st 

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def checkLimit(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel

def main():
    # if os.name != 'nt':
    
    print(settings)
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    
    status = 0
    target_pitch_vel   = 0.0
    target_roll_vel  = 0.0
    target_yaw_vel = 0.0
    target_trust = 0.0
    arm = 0.0
    mode = 0.0 #0.0 for drone/1.0 for plane
    try:
        print(msg)
        while 1:
            key = getKey()
            if key == 'w' :
                target_pitch_vel = checkLimit(target_pitch_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                # print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_pitch_vel = checkLimit(target_pitch_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
                # print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_roll_vel = checkLimit(target_roll_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
                # print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_roll_vel = checkLimit(target_roll_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
                # print(vels(target_linear_vel,target_angular_vel))
            elif key == 'q':
                target_yaw_vel = checkLimit(target_yaw_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == 'e':
                target_yaw_vel = checkLimit(target_yaw_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == 'z':
                target_trust = checkLimit(target_trust - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == 'c':
                target_trust = checkLimit(target_trust + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == 'f':
                arm = 1.0
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == 'g':
                arm = 0.0
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == 'r':
                mode = 0.0
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
            elif key == 't':
                mode = 1.0
                status = status + 1
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
                
            elif key == ' ' or key == 's' :
                target_roll_vel = 0.0
                target_pitch_vel = 0.0
                target_yaw_vel = 0.0
                target_trust = 0.0
                print(vels(target_roll_vel, target_pitch_vel, target_yaw_vel, target_trust, arm, mode))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()
            twist.linear.x = target_pitch_vel
            twist.linear.y = target_roll_vel
            twist.linear.z = target_trust

            twist.angular.x = arm 
            twist.angular.y = mode
            twist.angular.z = target_yaw_vel

            pub.publish(twist)

    except Exception as es:
        print(es)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        # pub.publish(twist)

    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()