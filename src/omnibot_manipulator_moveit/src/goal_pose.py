from __future__ import print_function
import sys
import rospy
from moveit_commander import roscpp_initialize,RobotCommander,PlanningSceneInterface,MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Empty

roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()

group_name = "arm"
move_group = MoveGroupCommander(group_name)
move_group.allow_replanning(True)
move_group.set_num_planning_attempts(100)
move_group.set_planning_time(10)
# move_group.set_planning_pipeline_id("chomp")
# move_group.set_planner_id("SBL")
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
rviz_goal_state_trigger = rospy.Publisher("/rviz/moveit/update_goal_state",Empty,queue_size=10)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.0
pose_target.orientation.y = 0.0
pose_target.orientation.z = 0.0
pose_target.position.x = 0.0
pose_target.position.y = 0.3
pose_target.position.z = 0.1
move_group.set_pose_target(pose_target)
plan = move_group.plan()
move_group.go(wait=True)

move_group.clear_pose_targets()
