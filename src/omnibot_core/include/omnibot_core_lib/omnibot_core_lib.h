#ifndef __OMNIBOT_CORE_LIB_H__
#define __OMNIBOT_CORE_LIB_H__
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>


namespace rvt = rviz_visual_tools;


class OMNIBOT_CORE_LIB
{
private:
    YAML::Node _config;


    std::string _manip_arm_planning_group;
    std::string _manip_grp_planning_group;
    std::string _visual_link;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface *move_group;
    const robot_state::JointModelGroup *joint_model_group;
 

public:
    OMNIBOT_CORE_LIB(ros::NodeHandle *nh, std::string config_path);
    ~OMNIBOT_CORE_LIB();
     

    void run();
    void terminate();
};




#endif // __OMNIBOT_CORE_LIB_H__
