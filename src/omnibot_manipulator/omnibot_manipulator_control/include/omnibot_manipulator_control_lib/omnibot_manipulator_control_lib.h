#ifndef __OMNIBOT_MANIPULATOR_CONTROL_LIB_H__
#define __OMNIBOT_MANIPULATOR_CONTROL_LIB_H__
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>


#include "joint.h"

class OMNIBOT_MANIPULATOR_CONTROL_LIB
{
private:
    ros::NodeHandle *_nh;
public:
    OMNIBOT_MANIPULATOR_CONTROL_LIB(/* args */);
    ~OMNIBOT_MANIPULATOR_CONTROL_LIB();
};



#endif // __OMNIBOT_MANIPULATOR_CONTROL_LIB_H__