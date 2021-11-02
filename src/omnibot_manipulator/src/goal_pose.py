from __future__ import print_function
import sys
import rospy
from moveit_commander import roscpp_initialize,RobotCommander,PlanningSceneInterface,MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg

roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()

group_name = "arm"
move_group = MoveGroupCommander(group_name)
move_group.allow_replanning(True)
move_group.set_num_planning_attempts(100)
move_group.set_planning_time(10)
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

pose_target = move_group.get_current_pose().pose
pose_target.position.x = -0.2
pose_target.position.y = -0.2
pose_target.position.z = 0.1
move_group.set_pose_target(pose_target)
print(move_group.get_current_pose().pose)

plan1 = move_group.plan()
rospy.loginfo("Planning Time: {}".format(move_group.get_planning_time()))
move_group.go(wait=True)
# move_group.clear_pose_targets()

# joint_goal = move_group.get_current_joint_values()
# joint_goal[4] = 1.57
# move_group.go(joint_goal, wait=True)
# print(move_group.get_current_pose().pose)
# # move_group.stop()
move_group.clear_pose_targets()