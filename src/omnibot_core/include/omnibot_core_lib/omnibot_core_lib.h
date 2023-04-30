#ifndef __OMNIBOT_CORE_LIB_H__
#define __OMNIBOT_CORE_LIB_H__
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

class OMNIBOT_CORE_LIB
{
private:
    YAML::Node _config;

public:
    OMNIBOT_CORE_LIB(ros::NodeHandle *nh, std::string config_path);
    ~OMNIBOT_CORE_LIB();
    void init();

    void run();
    void terminate();
};


#endif // __OMNIBOT_CORE_LIB_H__
