#ifndef __OMNIBOT_BATTERY_MANAGER_LIB_H__
#define __OMNIBOT_BATTERY_MANAGER_LIB_H__
#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "yaml-cpp/yaml.h"
#include <serial/serial.h>


class OMNIBOT_BATTERY_MANAGER_LIB
{
private:
    ros::NodeHandle *_nh;
    YAML::Node      _cfg;


    std::string _port_path;
    uint32_t    _port_baud;
    uint32_t    _port_timeout;


    float _update_topic_rate_hz = 0;

    ros::Timer _main_timer;
    serial::Serial *ser;

    void process(const ros::TimerEvent&);

public:
    OMNIBOT_BATTERY_MANAGER_LIB(ros::NodeHandle *nh, std::string config_path);
    void run();
    void stop();
    ~OMNIBOT_BATTERY_MANAGER_LIB();
};







#endif // __OMNIBOT_BATTERY_MANAGER_LIB_H__