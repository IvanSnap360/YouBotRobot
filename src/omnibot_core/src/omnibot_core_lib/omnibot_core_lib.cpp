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
    
}

OMNIBOT_CORE_LIB::~OMNIBOT_CORE_LIB()
{
}


/*
    ros::AsyncSpinner spinner(1); spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group(_manip_arm_planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group.setPlanningTime(10);
    
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(_manip_arm_planning_group);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.40;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.10;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        move_group.asyncMove();
    }
*/