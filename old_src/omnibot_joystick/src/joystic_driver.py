#! /usr/bin/env python2 
import rospy
from geometry_msgs.msg import Pose,Twist

rospy.init_node("joystic2cmd_vel")
rate = rospy.Rate(50)
left_joy_msg = Pose()
right_joy_msg = Pose()

def left_joy_cb(msg:Pose):
    global left_joy_msg
    left_joy_msg = msg

def right_joy_cb(msg:Pose):
    global right_joy_msg
    right_joy_msg = msg

rospy.Subscriber("/remote_cotroller/left_joy/value",Pose,left_joy_cb)
rospy.Subscriber("/remote_cotroller/right_joy/value",Pose,right_joy_cb)
cmd_vel_pub = rospy.Publisher(rospy.get_param("/driver_settings/system/velocities_input_topic_name"),Twist,queue_size=10)

while not rospy.is_shutdown():
    cmd_vel_msg = Twist()

    if abs(left_joy_msg.position.y)  < 30:
        left_joy_msg.position.y = 0
# 
    if abs(left_joy_msg.position.x)  < 30:
        left_joy_msg.position.x = 0
# 
    if abs(right_joy_msg.position.y)  < 30:
        right_joy_msg.position.y = 0
# 
    if abs(right_joy_msg.position.x)  < 30:
        right_joy_msg.position.x = 0

    # right_joy_msg.position.x = round(right_joy_msg.position.x,1)
    # right_joy_msg.position.y = round(right_joy_msg.position.y,1)

    # left_joy_msg.position.x = round(left_joy_msg.position.x,1)
    # left_joy_msg.position.y = round(left_joy_msg.position.y,1)

    cmd_vel_msg.linear.x = left_joy_msg.position.y / 3000.0
    cmd_vel_msg.angular.z = -left_joy_msg.position.x / 2000.0 
    cmd_vel_msg.linear.y = -right_joy_msg.position.y / 3000.0 

    cmd_vel_pub.publish(cmd_vel_msg)
    rate.sleep()