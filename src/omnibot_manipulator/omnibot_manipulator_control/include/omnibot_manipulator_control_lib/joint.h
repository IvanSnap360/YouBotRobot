#ifndef __JOINT_H__
#define __JOINT_H__
#include <iostream>
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include <ros/ros.h>
typedef struct
{
    std::string name;
    uint32_t id;
    double velocity;
    double home_position;

    double costrains_max_position;
    double costrains_min_position;
} joint_cfg_t;

class JOINT
{
private:
    DynamixelWorkbench *_dxl;
    joint_cfg_t *_cfg;
    bool _res = false;
public:
    JOINT(joint_cfg_t *config_ptr,DynamixelWorkbench *dxl);
    bool setTorque(bool state);
    bool setLED(bool state);
    bool setVelocity(double val);
    bool setPosition(double val);
    bool getPosition(double &data);
    
    ~JOINT();
};

#endif // __JOINT_H__