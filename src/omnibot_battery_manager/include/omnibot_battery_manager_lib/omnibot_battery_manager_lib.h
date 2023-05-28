#ifndef __OMNIBOT_BATTERY_MANAGER_LIB_H__
#define __OMNIBOT_BATTERY_MANAGER_LIB_H__
#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "yaml-cpp/yaml.h"
#include <serial/serial.h>

enum
{
    BATTERY_1,
    BATTERY_2
} battery_enum;

enum
{
    CELL_1,
    CELL_2,
    CELL_3,
} cell_enum;

class OMNIBOT_BATTERY_MANAGER_LIB
{
private:
    ros::NodeHandle *_nh;
    YAML::Node _cfg;

    std::string _port_path;
    uint32_t _port_baud;
    uint32_t _port_timeout;

    float _update_topic_rate_hz = 0;

    std::string _battery_1_topic_name;
    std::string _battery_2_topic_name;

    ros::Timer _main_timer;
    serial::Serial *ser;

    std::string _parser__cell_det;
    std::string _parser__batt_det;



    uint32_t _batt_count;
    uint32_t _cell_count;

    std::vector<sensor_msgs::BatteryState> _batt_msg;
    ros::Publisher _batt1_pub, _batt2_pub;

    std::string _battery_1_frame_name;
    std::string _battery_2_frame_name;

    char _info_msg[400];

public:
    OMNIBOT_BATTERY_MANAGER_LIB(ros::NodeHandle *nh, std::string config_path);
    void run();
    void stop();
    void process(const ros::TimerEvent &);
    ~OMNIBOT_BATTERY_MANAGER_LIB();
};

#endif // __OMNIBOT_BATTERY_MANAGER_LIB_H__