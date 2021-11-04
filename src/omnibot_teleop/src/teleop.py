#! /usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist
rospy.init_node("teleop_node")
vels_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=50)
rate = rospy.Rate(30)
import os
import select
import sys


forward_key = "w"
backward_key = "x"
left_key = "a"
right_key = "d"
stop_key = "s"
cw_key = "e"
ccw_key = "q"
info_msg = """
----------------------------------------------------------
TELEOP CONFIGURATION:\033[04m\033[01m\033[36mWASDX\033[0m
    w              ccw_rotate: q  cw_rotate: e
a   s   d
    x
w - forward
x - backward
a - left
d - right
s - stop
----------------------------------------------------------
"""
print(info_msg)

stop_teleop_key = '\x03'


if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios


if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
#//////////////////////////////////////////////////////////////////////
linear_x_max = 1.0
linear_y_max = 1.0
angular_max = 1.0

linear_y_step = 0.01
linear_x_step = 0.01
angular_step = 0.01

linear_x = 0.0
linear_y = 0.0
angular = 0.0
#//////////////////////////////////////////////////////////////////////
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
#//////////////////////////////////////////////////////////////////////

count = 0

vels = Twist()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        if count == 3:
            os.system("clear")
            print(info_msg)
            count = 0
        key = getKey()
        if key != None:
            if (key == forward_key):
                linear_x += linear_x_step
            if(key == backward_key):
                linear_x -= linear_x_step
            if (key == left_key):
                linear_y += linear_y_step
            if(key == right_key):
                linear_y -= linear_y_step
            if(key == cw_key):
                angular -= angular_step
            if(key == ccw_key):
                angular += angular_step    
            if(key == stop_key):
                linear_x = 0
                linear_y = 0
                angular = 0
                
            linear_x = round(linear_x,3)   
            linear_y = round(linear_y,3)  
            angular = round(angular,3)    
            linear_x = constrain(linear_x,-linear_x_max,linear_x_max)
            linear_y = constrain(linear_y,-linear_y_max,linear_y_max)
            angular_z = constrain(angular,-angular_max,angular_max)

            print(linear_x,linear_y,angular)


            count += 1
            if (key == stop_teleop_key):
                print("-------------------EXIT-------------------")
                break            
            
        elif key == None:
            linear_x = 0
            linear_y = 0
            angular = 0

        vels.linear.x = linear_x
        vels.linear.y = linear_y
        vels.angular.z = angular_z

        vels_pub.publish(vels)