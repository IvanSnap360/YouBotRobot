#include "omnibot_core_lib/omnibot_core_lib.h"

OMNIBOT_CORE_LIB::OMNIBOT_CORE_LIB(ros::NodeHandle *nh, std::string config_path)
{
    _config = YAML::LoadFile(config_path);

    if (_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    _manip_arm_planning_group = _config["omnibot_manipulator"]["moveit"]["manipulator"]["planning_group_name"].as<std::string>();
    _manip_grp_planning_group = _config["omnibot_manipulator"]["moveit"]["gripper"]["planning_group_name"].as<std::string>();
    _visual_link = _config["visual"]["link"].as<std::string>();


   
}

void OMNIBOT_CORE_LIB::run()
{
    moveit::planning_interface::MoveGroupInterface _arm_move_group(_manip_arm_planning_group);
    ROS_INFO("Start main program!!!");
    const robot_state::JointModelGroup* _arm_joint_model_group = _arm_move_group.getCurrentState()->getJointModelGroup(_manip_arm_planning_group);
     ROS_INFO("Start main program!!!");
    geometry_msgs::Pose target_pose1;
    ROS_INFO("Start main program!!!");
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    _arm_move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = ( _arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    _arm_move_group.execute(my_plan);
}

OMNIBOT_CORE_LIB::~OMNIBOT_CORE_LIB()
{
}