#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_platform_drv");
    
    ros::spin();
    ros::shutdown();
    return 0;
}
