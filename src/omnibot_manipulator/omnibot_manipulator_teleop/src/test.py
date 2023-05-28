import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import cv2
import numpy as np

mat = np.zeros((120,120,3), dtype=np.uint8)



rospy.init_node("test_manip_teleop")
pub = rospy.Publisher("/joy",Joy,queue_size=10)
joy = Joy()
joy.axes = [0.0 for i in range(0,8)]
joy.buttons = [0 for i in range(0,11)]
while not  rospy.is_shutdown():

    joy.header.stamp = rospy.Time.now()

    if (cv2.waitKey(1) == ord("q")):
        break

    if (cv2.waitKey(1) == ord("w")):
        print("Pressed w")
        joy.axes[0] = 0.15

    else:
        joy.axes[0] = 0.0
    pub.publish(joy)
 

    cv2.imshow("frame",mat)