#ifndef __OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB_H__
#define __OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB_H__
#include <ros/ros.h>
#include "math.h"
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#define degreesToRadians(degrees) degrees * M_PI / 180


class OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB
{
private:
    ros::NodeHandle *_nh;
    YAML::Node _cfg;

    std::vector<double> _arm_home_joints_positions;
    std::vector<double> _arm_pack_joints_positions;

    std::string _arm_planning_group_name;
    std::string _gripper_planning_group_name;
    moveit::planning_interface::MoveGroupInterface *_move_group_interface_arm;
    moveit::planning_interface::MoveGroupInterface *_move_group_interface_gripper;

    moveit::planning_interface::MoveGroupInterface::Plan _arm_plan;

    moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;

    void moveByJointValues(std::vector<double> joint_values);
    void moveByPosition(geometry_msgs::Pose new_pose);
public:
    OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB(ros::NodeHandle *nh, std::string config_path);
    void init();
    void moveHome();
    void movePackPose();
    ~OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB();
};

#endif // __OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB_H__