#include <ros/ros.h>



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_core");
    // auto /* node_name */ = /* namespace_name::ClassName */();
    ros::spin();
    ros::shutdown();
    return 0;
}
