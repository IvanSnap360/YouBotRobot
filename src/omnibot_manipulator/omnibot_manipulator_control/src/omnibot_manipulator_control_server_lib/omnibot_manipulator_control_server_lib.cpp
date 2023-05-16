#include "omnibot_manipulator_control_server_lib/omnibot_manipulator_control_server_lib.h"



void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::updateGoalState()
{
    std_msgs::Empty _rviz_goal_state_updater_msg;
    _rviz_goal_state_updater_pub.publish(_rviz_goal_state_updater_msg);
}


OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB(ros::NodeHandle *nh, std::string config_path)
{
    _cfg = YAML::LoadFile(config_path);
    _nh = nh;

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

    // ##################################################################### //
    // ###################### READ MANIPULATOR CONFIG ###################### //
    // ##################################################################### //
    if (!_cfg["services"]["manipulator_cmd_service"]["saved_poses"].IsSequence())
    {
        ROS_FATAL("Saved Positions is not list!!!");
        return;
    }
    else
    {
        for (auto pose : _cfg["services"]["manipulator_cmd_service"]["saved_poses"])
        {
            std::cout << "find manipulator pose: " << pose["name"].as<std::string>() << "  vals: [ ";
            auto temp = pose["vals"].as<std::vector<double>>();
            for (auto &val : temp)
            {
                val = degreesToRadians(val);
                std::cout << val << " , ";
            }
            std::cout << "]" << std::endl;
            _arm_saved_poses[pose["name"].as<std::string>()] = temp;
        }
    }

    _arm_cmd_service_name = _cfg["services"]["manipulator_cmd_service"]["name"].as<std::string>();

    // ##################################################################### //
    // ######################## READ GRIPPER CONFIG ######################## //
    // ##################################################################### //

    if (!_cfg["services"]["gripper_cmd_service"]["saved_poses"].IsSequence())
    {
        ROS_FATAL("Saved Positions is not list!!!");
        return;
    }
    else
    {
        for (auto pose : _cfg["services"]["gripper_cmd_service"]["saved_poses"])
        {
            std::cout << "find gripper pose: " << pose["name"].as<std::string>() << "  val:  ";
            auto temp = pose["val"].as<double>();
            std::cout << temp << std::endl;
            _grp_saved_poses[pose["name"].as<std::string>()] = temp;
        }
    }
    _gripper_cmd_service_name = _cfg["services"]["gripper_cmd_service"]["name"].as<std::string>();

    _rviz_goal_state_updater_pub = _nh->advertise<std_msgs::Empty>("/rviz/moveit/update_goal_state",10);
}

void OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::init()
{
    ROS_INFO("Wait for move_group status");
    auto msg = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_group/status");
    ROS_INFO("Got move_group status");

    // ##################################################################### //
    // ############ INITILAZING MANIPULATOR MOVE GROUP INTERFACE ########### //
    // ##################################################################### //
    ROS_INFO("Initilazing manipulator move group interface...");
    _move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(_arm_planning_group_name);
    _move_group_interface_arm->setPlanningTime(20);
    _move_group_interface_arm->setPlannerId("RRT");

    _arm_cmd_service_server = _nh->advertiseService(
        _arm_cmd_service_name,
        &OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::_arm_cmd_service_server_cb_f,
        this);

    ROS_INFO("Initilazing manipulator move group interface      DONE");

    // ##################################################################### //
    // ############## INITILAZING GRIPPER MOVE GROUP INTERFACE ############# //
    // ##################################################################### //
    ROS_INFO("Initilazing gripper move group interface...");

    _move_group_interface_gripper = new moveit::planning_interface::MoveGroupInterface(_gripper_planning_group_name);
    _gripper_cmd_service_server = _nh->advertiseService(
        _gripper_cmd_service_name,
        &OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::_grp_cmd_service_server_cb_f,
        this);

    ROS_INFO("Initilazing gripper move group interface      DONE");
}


// ##################################################################### //
// ####################### MANIPULATOR FUNCTIONS ####################### //
// ##################################################################### //
bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::moveHome()
{
    ROS_INFO("Homming manipulator...");
    bool res = ManipMoveBySavedPosition("home");
    ROS_INFO("Homming manipulator DONE");
    return res;
}

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::movePackPose()
{
    ROS_INFO("Setting manipulator to pack pose...");
    bool res = ManipMoveBySavedPosition("pack");
    ROS_INFO("Setting manipulator to pack pose   DONE");
    return res;
}

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::ManipMoveBySavedPosition(std::string pose_name)
{
    ROS_INFO("Setting manipulator to pose '%s' DONE", pose_name.c_str());
    if (_arm_saved_poses.find(pose_name) != _arm_saved_poses.end())
    {
        ManipMoveByJointValues(_arm_saved_poses[pose_name]);
        ROS_INFO("Setting manipulator to pose '%s' DONE", pose_name.c_str());
        return true;
    }
    else
    {
        ROS_WARN("The pose for manipulator with name '%s' doesnt exist!!!", pose_name.c_str());
        return false;
    }
}

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::ManipMoveByJointValues(std::vector<double> joint_values)
{

    const robot_state::JointModelGroup *joint_model_group =
        _move_group_interface_arm->getCurrentState()->getJointModelGroup(_arm_planning_group_name);

    std::vector<double> joint_group_positions;
    _move_group_interface_arm->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    if (joint_values.size() != joint_group_positions.size())
    {
        ROS_ERROR("JOINTS LISTS HAVE DIFFERENT SIZE!!!");
        return false;
    }
    _move_group_interface_arm->setJointValueTarget(joint_values);
    
    ROS_INFO("Planning for joints positions [%.2lf %.2lf %.2lf %.2lf %.2lf]",
             joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4]);
    updateGoalState();
    bool success = (_move_group_interface_arm->plan(_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Planning %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        ROS_INFO("STARTING EXECUTION !!!");
        _move_group_interface_arm->move();
        ROS_INFO("EXECUTION DONE!!!");
    }
    return success;
}

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::ManipMoveByPosition(geometry_msgs::Pose new_pose)
{
    new_pose.orientation.w = 1.0;
    _move_group_interface_arm->setPoseTarget(new_pose);
    updateGoalState();
    ROS_INFO("Planning for position [x:%.2lf y:%.2lf z:%.2lf rx:%.2lf ry:%.2lf rz:%.2lf]",
             new_pose.position.x, new_pose.position.y, new_pose.position.z,
             new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z);

    bool success = (_move_group_interface_arm->plan(_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Planning %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        ROS_INFO("STARTING EXECUTION !!!");
        _move_group_interface_arm->move();
        ROS_INFO("EXECUTION DONE!!!");
    }
    return success;
}

// ##################################################################### //
// ######################### GRIPPER FUNCTIONS ######################### //
// ##################################################################### //

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::GripperMoveByJointValue(double joint_value)
{
    return false;
}

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::GripperMoveByPosition(double new_pose)
{
    const robot_state::JointModelGroup *grp_joint_model_group =
        _move_group_interface_gripper->getCurrentState()->getJointModelGroup(_gripper_planning_group_name);

    std::vector<double> joint_group_positions;
    _move_group_interface_gripper->getCurrentState()->copyJointGroupPositions(grp_joint_model_group, joint_group_positions);

    joint_group_positions[0] = new_pose;
    _move_group_interface_gripper->setJointValueTarget(joint_group_positions);

    ROS_INFO("Planning gripper for  position [%.2lf]", new_pose);

    bool success = (_move_group_interface_gripper->plan(_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Planning %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        ROS_INFO("STARTING EXECUTION !!!");
        _move_group_interface_gripper->move();
        ROS_INFO("EXECUTION DONE!!!");
    }
    return success;
}

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::GripperMoveBySavedPosition(std::string pose_name)
{
    ROS_INFO("Setting gripper to pose '%s'", pose_name.c_str());
    if (_grp_saved_poses.find(pose_name) != _grp_saved_poses.end())
    {
        GripperMoveByPosition(_grp_saved_poses[pose_name]);
        ROS_INFO("Setting gripper to pose '%s' DONE", pose_name.c_str());
        return true;
    }
    else
    {
        ROS_WARN("The pose for gripper with name '%s' doesnt exist!!!", pose_name.c_str());
        return false;
    }
}


// ##################################################################### //
// #################### SERVISES CALLBACK FUNCTIONS #################### //
// ##################################################################### // 

bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::_grp_cmd_service_server_cb_f(omnibot_manipulator_control::gripper_cmd::Request &req,
                                                                          omnibot_manipulator_control::gripper_cmd::Response &res)
{

    switch (req.command_type)
    {
    case req.JOINT_CMD_MODE:
    {
        ROS_WARN("Method JOINT_CMD_MODE for gripper not working!!! Sorry :( ");
        res.result = false;
        res.callback_msg = res.result ? "Success" : "Failed";
        break;
    }
    case req.POSITION_CMD_MODE:
    {
        double val;
        try
        {
            val = atof(req.command.c_str());
        }
        catch (...)
        {
            ROS_ERROR("Wrong value format!!!");
            res.result = false;
            res.callback_msg = res.result ? "Success" : "Failed";
        }
        res.result = GripperMoveByPosition(val);
        res.callback_msg = res.result ? "Success" : "Failed";
        break;
    }
    case req.SAVED_POSE:
    {

        res.result = GripperMoveBySavedPosition(req.command);
        res.callback_msg = res.result ? "Success" : "Failed";
        break;
    }
    }
    return true;
}



bool OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::_arm_cmd_service_server_cb_f(
    omnibot_manipulator_control::manipulator_cmd::Request &req,
    omnibot_manipulator_control::manipulator_cmd::Response &res)
{
    switch (req.command_type)
    {
    case req.SAVED_POSE:
    {

        res.result = ManipMoveBySavedPosition(req.command);

        res.callback_msg = res.result ? "Success" : "Failed";
        break;
    }
    case req.POSITION_CMD_MODE:
    {
        geometry_msgs::Pose pose;
        std::vector<double> pose_array;
        pose.orientation.w = 1;
        std::string sval[6];
        std::stringstream s(req.command);
        s >> sval[0] >> sval[1] >> sval[2] >> sval[3] >> sval[4] >> sval[5];

        for (auto sval_seg : sval)
        {
            try
            {
                pose_array.push_back(atof(sval_seg.c_str()));
            }
            catch (...)
            {
                ROS_ERROR("Wrong value format!!!");
                res.result = false;
                res.callback_msg = res.result ? "Success" : "Failed";
            }
        }
        pose.position.x = pose_array[0];
        pose.position.y = pose_array[1];
        pose.position.z = pose_array[2];
        pose.orientation.x = pose_array[3];
        pose.orientation.y = pose_array[4];
        pose.orientation.z = pose_array[5];

        res.result = ManipMoveByPosition(pose);
        res.callback_msg = res.result ? "Success" : "Failed";
    }

    break;
    case req.JOINT_CMD_MODE:
    {

        std::vector<double> joint_poses;
        std::string sval[5];
        std::stringstream s(req.command);
        s >> sval[0] >> sval[1] >> sval[2] >> sval[3] >> sval[4];

        for (auto sval_seg : sval)
        {
            try
            {
                joint_poses.push_back(atof(sval_seg.c_str()));
            }
            catch (...)
            {
                ROS_ERROR("Wrong value format!!!");
                res.result = false;
                res.callback_msg = res.result ? "Success" : "Failed";
            }
        }

        res.result = ManipMoveByJointValues(joint_poses);
        res.callback_msg = res.result ? "Success" : "Failed";

        break;
    }
    }
    return true;
}

OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB::~OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB()
{
}