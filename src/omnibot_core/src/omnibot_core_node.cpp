#include <ros/ros.h>
#include "omnibot_core_lib/omnibot_core_lib.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "omnibot_core");
    ros::NodeHandle nh;
    std::string config_path = nh.param<std::string>("omnibot_core_config_path", " ");
    auto core = OMNIBOT_CORE_LIB(&nh,config_path);
    ros::spin();
    ros::shutdown();
    
    return 0;
}
