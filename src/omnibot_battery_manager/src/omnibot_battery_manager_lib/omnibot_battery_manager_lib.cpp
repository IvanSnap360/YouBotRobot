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

    _parser__cell_det = _cfg["parser"]["cell_determinator"].as<std::string>();
    _parser__batt_det = _cfg["parser"]["battery_determinator"].as<std::string>();

    _batt_count = _cfg["Battery"]["count"].as<uint32_t>();
    _cell_count = _cfg["Battery"]["serial_cells_count"].as<uint32_t>();

    _update_topic_rate_hz = _cfg["update_topic_data_rate_hz"].as<float>();

    _battery_1_topic_name = _cfg["battery_1_topic_name"].as<std::string>();
    _battery_2_topic_name = _cfg["battery_2_topic_name"].as<std::string>();

    //
    _batt_msg.resize(_batt_count);

    _batt_msg[BATTERY_1].cell_voltage.resize(_cell_count);
    _batt_msg[BATTERY_1].cell_temperature.resize(_cell_count);

    _batt_msg[BATTERY_2].cell_voltage.resize(_cell_count);
    _batt_msg[BATTERY_2].cell_temperature.resize(_cell_count);

    _battery_1_frame_name = _cfg["battery_1_frame_name"].as<std::string>();
    _battery_2_frame_name = _cfg["battery_2_frame_name"].as<std::string>();
   
    _main_timer = _nh->createTimer(ros::Duration(ros::Rate(_update_topic_rate_hz)), &OMNIBOT_BATTERY_MANAGER_LIB::process, this, false);

    _batt1_pub = _nh->advertise<sensor_msgs::BatteryState>(_battery_1_topic_name, 10);
    _batt2_pub = _nh->advertise<sensor_msgs::BatteryState>(_battery_2_topic_name, 10);
}

void OMNIBOT_BATTERY_MANAGER_LIB::run()
{

    ser = new serial::Serial();
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
    ser->close();
}

OMNIBOT_BATTERY_MANAGER_LIB::~OMNIBOT_BATTERY_MANAGER_LIB()
{
}

std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

void OMNIBOT_BATTERY_MANAGER_LIB::process(const ros::TimerEvent &event)
{

    if (ser->available() > 0)
    {

        std::string data = ser->read(ser->available());
        // std::cout << data << std::endl;

        std::vector<std::string> batteries_v = split (data, _parser__batt_det);
        // for (auto i : batteries_v) std::cout << i << std::endl;


        /* PARSE BATTERY 1 DATA */

        std::vector<std::string> batt_1_cells_v = split (batteries_v[BATTERY_1], _parser__cell_det);
        // for (auto i : batt_1_cells_v) std::cout << std::stof(i) / 100.0 << std::endl;


        /* PARSE BATTERY 2 DATA */

        std::vector<std::string> batt_2_cells_v = split (batteries_v[BATTERY_2], _parser__cell_det);
        // for (auto i : batt_2_cells_v) std::cout << std::stof(i) / 100.0 << std::endl;


        _batt_msg[BATTERY_1].cell_voltage[CELL_1] = std::stof(batt_1_cells_v[CELL_1]) / 100.0;
        _batt_msg[BATTERY_1].cell_voltage[CELL_2] = std::stof(batt_1_cells_v[CELL_2]) / 100.0;
        _batt_msg[BATTERY_1].cell_voltage[CELL_3] = std::stof(batt_1_cells_v[CELL_3]) / 100.0;

        _batt_msg[BATTERY_2].cell_voltage[CELL_1] = std::stof(batt_2_cells_v[CELL_1]) / 100.0;
        _batt_msg[BATTERY_2].cell_voltage[CELL_2] = std::stof(batt_2_cells_v[CELL_2]) / 100.0;
        _batt_msg[BATTERY_2].cell_voltage[CELL_3] = std::stof(batt_2_cells_v[CELL_3]) / 100.0;

        _batt_msg[BATTERY_1].header.frame_id   = _battery_1_frame_name;
        _batt_msg[BATTERY_1].header.stamp.sec  =  ros::Time::now().toSec();
        _batt_msg[BATTERY_1].header.stamp.nsec =  ros::Time::now().toNSec();


        _batt_msg[BATTERY_2].header.frame_id = _battery_2_frame_name;
        _batt_msg[BATTERY_2].header.stamp.sec  =  ros::Time::now().toSec();
        _batt_msg[BATTERY_2].header.stamp.nsec =  ros::Time::now().toNSec();
        
        _batt1_pub.publish(_batt_msg[BATTERY_1]);
        _batt2_pub.publish(_batt_msg[BATTERY_2]);

        
        sprintf(_info_msg, R"anystring(
        #########################################################
            Battery 1
                - cell_1: %1.2f
                - cell_2: %1.2f
                - cell_3: %1.2f
                
            Battery 2
                - cell_1: %1.2f
                - cell_2: %1.2f
                - cell_3: %1.2f
        #########################################################
       
        )anystring",
        _batt_msg[BATTERY_1].cell_voltage[CELL_1],
        _batt_msg[BATTERY_1].cell_voltage[CELL_2],
        _batt_msg[BATTERY_1].cell_voltage[CELL_3],
        _batt_msg[BATTERY_2].cell_voltage[CELL_1],
        _batt_msg[BATTERY_2].cell_voltage[CELL_2],
        _batt_msg[BATTERY_2].cell_voltage[CELL_3]
        );

        std::cout << _info_msg << std::endl;

        memset(_info_msg,0,sizeof(_info_msg));
    }
}