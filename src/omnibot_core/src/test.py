import sys
import rospy
from moveit_commander import roscpp_initialize,RobotCommander,PlanningSceneInterface,MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Empty
from tf.transformations import quaternion_from_euler
roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()
group_name = "maipulator_group"
move_group = MoveGroupCommander(group_name)
move_group.allow_replanning(True)
move_group.set_num_planning_attempts(10)
move_group.set_planning_time(10)
display_trajectory_publisher = rospy.Publisher( "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=100)
pose_target = geometry_msgs.msg.Pose()
q = quaternion_from_euler(1.57,-1.57,0.0)
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.x = 0.0
pose_target.position.y = 0.5
pose_target.position.z = 0.0
move_group.set_pose_target(pose_target)
plan = move_group.plan()
move_group.go(wait=True)
move_group.clear_pose_targets()