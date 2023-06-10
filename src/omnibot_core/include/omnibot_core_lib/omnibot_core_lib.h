#ifndef __OMNIBOT_CORE_LIB_H__
#define __OMNIBOT_CORE_LIB_H__
#include <ros/ros.h>
#include <omnibot_manipulator_control/manipulator_cmd.h>
#include <omnibot_manipulator_control/gripper_cmd.h>
#include <yaml-cpp/yaml.h>

class OMNIBOT_CORE_LIB
{
private:
    YAML::Node _cfg;
    ros::NodeHandle *_nh;

    std::string _manipulator_service_name;
    ros::ServiceClient _manipulator_service_client;

    std::string _gripper_service_name;
    ros::ServiceClient _gripper_service_client;

public:
    OMNIBOT_CORE_LIB(ros::NodeHandle *nh, std::string config_path);
    ~OMNIBOT_CORE_LIB();
};


#endif // __OMNIBOT_CORE_LIB_H__
