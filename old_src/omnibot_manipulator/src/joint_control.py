import rospy
from std_msgs.msg import Float64

rospy.init_node("joint_manual_control_node")

pub = rospy.Publisher("/angle_joint1_controller/command",Float64,queue_size=10)

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    data = Float64()
    data.data = 3.14
    pub.publish(data)
    rate.sleep()