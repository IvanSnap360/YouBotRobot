#include <ros/ros.h>

#include "omnibot_platform_teleop_lib/omnibot_platform_teleop_lib.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "omnibot_platform_teleop_node");
    ros::NodeHandle nh;
    std::string keys_mode;

    std::string omnibot_platform_teleop_config_yaml_file_path = 
        nh.param<std::string>("omnibot_platform_teleop_config", " ");

    if (argc > 1)
    {
        keys_mode = argv[1];
        ROS_INFO("%s control keys variant was chosen!", keys_mode.c_str());
    }
    else
    {
        ROS_FATAL("No chosen control keys variant (wasd or numpad)");
        return 0;
    }
        
    auto teleop = OMNIBOT_PLATFORM_TELEOP_LIB(&nh, omnibot_platform_teleop_config_yaml_file_path,keys_mode);

    auto rate = ros::Rate(teleop.getWorkRate());

    while (ros::ok())
    {
        if (teleop.process() == -1) break;

        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
