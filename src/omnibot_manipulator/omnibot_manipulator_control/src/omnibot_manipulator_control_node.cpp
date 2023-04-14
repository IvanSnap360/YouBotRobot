#include <ros/ros.h>
#include "omnibot_manipulator_control_lib/omnibot_manipulator_control_lib.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_manipulator_control_node");
    ros::NodeHandle nh("~");

    bool hw_connection = false;
    nh.getParam("hw_connection",hw_connection);

    ROS_INFO("Hardware connection %s", hw_connection ? "enabled" : "disabled");

    auto node = OMNIBOT_MANIPULATOR_CONTROL_LIB();
    ros::spin();
    ros::shutdown();
    return 0;
}
