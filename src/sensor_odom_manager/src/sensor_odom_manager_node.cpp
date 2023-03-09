#include <ros/ros.h>
#include "sensor_odom_manager_lib/sensor_odom_manager_lib.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sensor_odom_manager_node");
    ros::NodeHandle nh;

    std::string config_path = nh.param<std::string>("sensor_odom_manager_config_path","");


    

    auto mngr = SENSOR_ODOM_MANAGER_LIB(&nh,config_path);
    mngr.run();
    ros::spin();
    ros::shutdown();
    mngr.stop();
    return 0;
}
