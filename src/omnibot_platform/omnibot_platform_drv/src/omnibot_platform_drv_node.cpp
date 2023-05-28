#include <ros/ros.h>

#include "omnibot_platform_drv_lib/omnibot_platform_drv_lib.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_platform_drv_node");
    ros::NodeHandle nh;
    std::string omnibot_platform_config_yaml_file_path = nh.param<std::string>("omnibot_platform_drv_config_file_path", " ");

    auto omnibot_platform_drv = OMNIBOT_PLATFORM_DRV_LIB(&nh,omnibot_platform_config_yaml_file_path);
    ros::Timer _process_timer = nh.createTimer(ros::Duration(ros::Rate(omnibot_platform_drv.getWorkRate())),
                                                         &OMNIBOT_PLATFORM_DRV_LIB::process, &omnibot_platform_drv);
    ROS_INFO("OMNIBOT_PLATFORM_DRV node started!!!");
    ros::spin();
    ros::shutdown();
    return 0;
}
