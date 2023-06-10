#ifndef __OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB_H__
#define __OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB_H__
#include <ros/ros.h>
#include "math.h"
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <omnibot_manipulator_control/manipulator_cmd.h>
#include <omnibot_manipulator_control/gripper_cmd.h>
#include <std_msgs/Empty.h>
#include "action_server_lib/action_server_lib.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>

#define degreesToRadians(degrees) degrees *M_PI / 180

class OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB
{
private:
    ros::NodeHandle *_nh;
    YAML::Node _cfg;

    std::map<std::string, std::vector<double>> _arm_saved_poses;
    std::map<std::string, double> _grp_saved_poses;

    std::string _arm_planning_group_name;
    std::string _arm_cmd_service_name;
    std::string _gripper_planning_group_name;
    std::string _gripper_cmd_service_name;

    moveit::planning_interface::MoveGroupInterface *_move_group_interface_arm;
    moveit::planning_interface::MoveGroupInterface *_move_group_interface_gripper;

    moveit::planning_interface::MoveGroupInterface::Plan _arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan _gripper_plan;

    moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;

    ros::ServiceServer _arm_cmd_service_server;
    ros::ServiceServer _gripper_cmd_service_server;
    
    bool ManipMoveByJointValues(std::vector<double> joint_values);
    bool ManipMoveByPosition(geometry_msgs::Pose new_pose);
    bool ManipMoveBySavedPosition(std::string pose_name);

    bool GripperMoveByJointValue(double joint_value);
    bool GripperMoveByPosition(double new_pose);
    bool GripperMoveBySavedPosition(std::string pose_name);

    bool _arm_cmd_service_server_cb_f(omnibot_manipulator_control::manipulator_cmd::Request &req,
                                      omnibot_manipulator_control::manipulator_cmd::Response &res);
    bool _grp_cmd_service_server_cb_f(omnibot_manipulator_control::gripper_cmd::Request &req,
                                      omnibot_manipulator_control::gripper_cmd::Response &res);

    ros::Publisher _rviz_goal_state_updater_pub;

    void updateGoalState();

public:
    OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB(ros::NodeHandle *nh, std::string config_path);
    void init();
    bool moveHome();
    bool movePackPose();
    ~OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB();
};

#endif // __OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB_H__