#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32 as wheel_type
from geometry_msgs.msg import Twist

rospy.init_node("omni_driver_node")

base_long = rospy.get_param("/driver_settings/wheel_base_long")
base_width = rospy.get_param("/driver_settings/wheel_base_width")
wheel_deameter = rospy.get_param("/driver_settings/wheel_deameter")

OUTPUT = rospy.get_param("/driver_settings/output_data")

linear_x = 0.0
linear_y = 0.0
angular_z = 0.0

MAX_WHEEL_VELOCITY = rospy.get_param("/driver_settings/max_wheel_angular_velocity")
MIN_PWM = rospy.get_param("/driver_settings/min_pwm_output")

def constrain(x, min, max):
    if x > max:
        x = max
    elif x < min:
        x = min
    return x


def goOutDeadZone(x):
    if x < 0 and x > -MIN_PWM:
        x = -MIN_PWM
    if x > 0 and x < MIN_PWM:
        x = MIN_PWM
    return x

def map(x, in_min,  in_max,  out_min,  out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


w1 = 0
w2 = 0
w3 = 0
w4 = 0

def callback(msg : Twist):
    global base_long
    global base_width
    global wheel_deameter

    global w1
    global w2
    global w3
    global w4

    Ly = base_long / 2
    Lx = base_width / 2
    r = wheel_deameter / 2
    
    vx = msg.linear.x
    vy = msg.linear.y
    wz = msg.angular.z
    
    w1 = (1 / r) * (vx - vy - (Lx + Ly) * wz) 
    w2 = (1 / r) * (vx + vy + (Lx + Ly) * wz) 
    w3 = (1 / r) * (vx + vy - (Lx + Ly) * wz) 
    w4 = (1 / r) * (vx - vy + (Lx + Ly) * wz) 

    if OUTPUT == "PWM":
        w1 = map(w1, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY,-255,255)
        w2 = map(w2, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY,-255,255)
        w3 = map(w3, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY,-255,255)
        w4 = map(w4, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY,-255,255)

        w1 = constrain(w1, -255,255)
        w2 = constrain(w2, -255,255)
        w3 = constrain(w3, -255,255)
        w4 = constrain(w4, -255,255)

        w1 = goOutDeadZone(w1)
        w2 = goOutDeadZone(w2)
        w3 = goOutDeadZone(w3)
        w4 = goOutDeadZone(w4)

        w1 = int(w1)
        w2 = int(w2)
        w3 = int(w3)
        w4 = int(w4)
    elif OUTPUT == "ANGULAR_VELOCITIES":
        pass

    # print(w1,w2,w3,w4)
    
    forward_left_pub.publish(w1)
    forward_right_pub.publish(w2)
    backward_left_pub.publish(w3)
    backward_right_pub.publish(w4)


rospy.Subscriber(rospy.get_param("/driver_settings/system/velocities_input_topic_name"), Twist, callback=callback)

# forward_left_output_topic_name
# forward_right_output_topic_name
# backward_left_output_topic_name
# backward_right_output_topic_name

forward_left_pub = rospy.Publisher(
    rospy.get_param("/driver_settings/system/forward_left_output_topic_name"), wheel_type, queue_size=10)

forward_right_pub = rospy.Publisher(
    rospy.get_param("/driver_settings/system/forward_right_output_topic_name"), wheel_type, queue_size=10)

backward_left_pub = rospy.Publisher(
    rospy.get_param("/driver_settings/system/backward_left_output_topic_name"), wheel_type, queue_size=10)

backward_right_pub = rospy.Publisher(
    rospy.get_param("/driver_settings/system/backward_right_output_topic_name"), wheel_type, queue_size=10)


rospy.spin()
