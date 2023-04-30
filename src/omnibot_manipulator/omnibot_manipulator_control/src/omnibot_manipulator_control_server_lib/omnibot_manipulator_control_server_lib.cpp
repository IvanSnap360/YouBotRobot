#include "omnibot_manipulator_control_server_lib/omnibot_manipulator_control_server_lib.h"

OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB(ros::NodeHandle *nh, std::string config_path)
{
    _cfg = YAML::LoadFile(config_path);

    if (_cfg == NULL)
    {
        ROS_FATAL("CONFIG FILE LOAD FAILED!!!");
    }
    else
    {
        ROS_INFO("CONFIG FILE LOAD SUCCESS!!!");
    }

    _arm_planning_group_name = _cfg["moveit"]["manipulator"]["planning_group_name"].as<std::string>();
    _gripper_planning_group_name = _cfg["moveit"]["gripper"]["planning_group_name"].as<std::string>();

    if (!_cfg["moveit"]["manipulator"]["saved_poses"]["home"].IsSequence())
        ROS_FATAL("Saved Position is not list!!!");
    _arm_home_joints_positions = _cfg["moveit"]["manipulator"]["saved_poses"]["home"].as<std::vector<double>>();

    if (!_cfg["moveit"]["manipulator"]["saved_poses"]["pack"].IsSequence())
        ROS_FATAL("Saved Position is not list!!!");
    _arm_pack_joints_positions = _cfg["moveit"]["manipulator"]["saved_poses"]["pack"].as<std::vector<double>>();
}

void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::init()
{
    ROS_INFO("Initilazing manipulator move group interface...");
    _move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(_arm_planning_group_name);
    _move_group_interface_arm->setPlanningTime(20);
    ROS_INFO("Initilazing manipulator move group interface      DONE");
}

void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::moveHome()
{
    ROS_INFO("Homming manipulator...");
    for (auto &pose : _arm_home_joints_positions)
        pose = degreesToRadians(pose);
    moveByJointValues(_arm_home_joints_positions);
    ROS_INFO("Homming manipulator DONE");
}

void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::movePackPose()
{
    ROS_INFO("Setting manipulator to pack pose...");
    for (auto &pose : _arm_pack_joints_positions)
        pose = degreesToRadians(pose);
    moveByJointValues(_arm_pack_joints_positions);
    ROS_INFO("Setting manipulator to pack pose DONE");
}

void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::moveByJointValues(std::vector<double> joint_values)
{

    const robot_state::JointModelGroup *joint_model_group =
        _move_group_interface_arm->getCurrentState()->getJointModelGroup(_arm_planning_group_name);

    std::vector<double> joint_group_positions;
    _move_group_interface_arm->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    if (joint_values.size() != joint_group_positions.size())
    {
        ROS_ERROR("JOINTS LISTS HAVE DIFFERENT SIZE!!!");
    }
    _move_group_interface_arm->setJointValueTarget(joint_values);

    ROS_INFO("Planning for joints positions [%.2lf %.2lf %.2lf %.2lf %.2lf]",
        joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4]);

    bool success = (_move_group_interface_arm->plan(_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Planning %s", success ? "SUCCESS" : "FAILED");

    ROS_INFO("STARTING EXECUTION !!!");
    _move_group_interface_arm->move();
    ROS_INFO("EXECUTION DONE!!!");
}


void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::moveByPosition(geometry_msgs::Pose new_pose)
{
    new_pose.orientation.w = 1.0;
    _move_group_interface_arm->setPoseTarget(new_pose);
}




OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::~OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB()
{
}