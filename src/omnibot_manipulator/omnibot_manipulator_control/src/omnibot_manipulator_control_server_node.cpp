#include <ros/ros.h>
#include "omnibot_manipulator_control_server_lib/omnibot_manipulator_control_server_lib.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_manipulator_control_server_node");
    ros::NodeHandle nh;
    std::string config_path = nh.param<std::string>("omnibot_manipulator_control_server_config_path", " ");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    auto srvr = OMNIBOT_MANIPULATOR_CONTROL_SERVER_LIB(&nh,config_path);
    srvr.init();
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
