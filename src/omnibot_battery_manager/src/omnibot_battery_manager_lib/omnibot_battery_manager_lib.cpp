#include "omnibot_battery_manager_lib/omnibot_battery_manager_lib.h"

OMNIBOT_BATTERY_MANAGER_LIB::OMNIBOT_BATTERY_MANAGER_LIB(ros::NodeHandle *nh, std::string config_path)
{

    _nh = nh;

    _cfg = YAML::LoadFile(config_path);

    if (_cfg == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    _port_path = _cfg["COM"]["port"].as<std::string>();
    _port_baud = _cfg["COM"]["baudrate"].as<uint32_t>();
    _port_timeout = _cfg["COM"]["timeout_ms"].as<uint32_t>();

    _update_topic_rate_hz = _cfg["update_topic_data_rate_hz"].as<float>();

    //

    _main_timer = _nh->createTimer(ros::Duration(ros::Rate(_update_topic_rate_hz)), &OMNIBOT_BATTERY_MANAGER_LIB::process, this, false);
}

void OMNIBOT_BATTERY_MANAGER_LIB::run()
{

    serial::Serial *ser = new serial::Serial();
    ser->setPort(_port_path);
    ser->setBaudrate(_port_baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(5000);
    ser->setTimeout(to);
    ser->open();

    if (ser->isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
        ros::Duration(5).sleep();
        _main_timer.start();
    }
    else
    {
        ROS_FATAL_STREAM("Serial Port NOT initialized!!!");
    }

   
}

void OMNIBOT_BATTERY_MANAGER_LIB::stop()
{
    _main_timer.stop();
}

OMNIBOT_BATTERY_MANAGER_LIB::~OMNIBOT_BATTERY_MANAGER_LIB()
{
}

void OMNIBOT_BATTERY_MANAGER_LIB::process(const ros::TimerEvent &event)
{
    std::cout << "Process run" << std::endl;
    if (ser->available() > 0)
    {
        std::string data = ser->read(ser->available());
        std::cout << data << std::endl;
    }
}