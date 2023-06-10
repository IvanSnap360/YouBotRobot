#ifndef __STATE_MACHINE_LIB_H__
#define __STATE_MACHINE_LIB_H__
#include <ros/ros.h>

class STATE_MACHINE_LIB
{
private:
    ros::NodeHandle *_nh;

public:
    STATE_MACHINE_LIB(ros::NodeHandle *nh);
    ~STATE_MACHINE_LIB();
};

#endif // __STATE_MACHINE_LIB_H__