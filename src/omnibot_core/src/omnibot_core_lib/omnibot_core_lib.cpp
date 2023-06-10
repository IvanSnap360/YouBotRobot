#include "omnibot_core_lib/omnibot_core_lib.h"

OMNIBOT_CORE_LIB::OMNIBOT_CORE_LIB(ros::NodeHandle *nh, std::string config_path)
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

    if (_cfg["manipulator_on"].as<bool>())
    {
        ROS_INFO("Creating manipulator_cmd service...");
        _manipulator_service_name = _cfg["manipulator_cmd_service_name"].as<std::string>();
        _manipulator_service_client = nh->serviceClient<omnibot_manipulator_control::manipulator_cmd::Request,
                                                        omnibot_manipulator_control::manipulator_cmd::Response>(_manipulator_service_name);
        ROS_INFO("Creating manipulator_cmd service...DONE");
        ROS_INFO("Waiting manipulator_cmd service server...");
        _manipulator_service_client.waitForExistence();
        ROS_INFO("Waiting manipulator_cmd service server...DONE");

        ROS_INFO("Creating gripper_cmd service...");
        _gripper_service_name = _cfg["gripper_cmd_service_name"].as<std::string>();
        _gripper_service_client = nh->serviceClient<omnibot_manipulator_control::gripper_cmd::Request,
                                                    omnibot_manipulator_control::gripper_cmd::Response>(_gripper_service_name);
        ROS_INFO("Creating gripper_cmd service...DONE");
        ROS_INFO("Waiting gripper_cmd service server...");
        _gripper_service_client.waitForExistence();
        ROS_INFO("Waiting gripper_cmd service server...DONE");

        ROS_INFO("Send manipulator to home pose...");
        omnibot_manipulator_control::manipulator_cmd::Request manip_requset;
        omnibot_manipulator_control::manipulator_cmd::Response manip_response;
        manip_requset.command_type = manip_requset.SAVED_POSE;
        manip_requset.command = "home";
        _manipulator_service_client.call(manip_requset, manip_response);
        ROS_INFO("Send manipulator to home pose...%s", manip_response.result ? "DONE" : "FAILED");

        ROS_INFO("Send gripper to home pose...");
        omnibot_manipulator_control::gripper_cmd::Request gripper_requset;
        omnibot_manipulator_control::gripper_cmd::Response gripper_response;
        gripper_requset.command_type = gripper_requset.SAVED_POSE;
        gripper_requset.command = "open";
        _gripper_service_client.call(gripper_requset, gripper_response);
        ROS_INFO("Send gripper to home pose...%s", gripper_response.result ? "DONE" : "FAILED");
    }
}

OMNIBOT_CORE_LIB::~OMNIBOT_CORE_LIB()
{
}
