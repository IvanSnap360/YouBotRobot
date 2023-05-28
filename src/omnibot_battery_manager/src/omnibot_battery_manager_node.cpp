#include <ros/ros.h>
#include "omnibot_battery_manager_lib/omnibot_battery_manager_lib.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_battery_manager");
    ros::NodeHandle nh;
    std::string config_path = nh.param<std::string>("omnibot_battery_manager_config_path", " ");
    auto bat_manager = OMNIBOT_BATTERY_MANAGER_LIB(&nh,config_path);
    bat_manager.run();
    ros::spin();
    bat_manager.stop();
    ros::shutdown();
    return 0;
}
