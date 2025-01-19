#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "omnibot_platform_mecanum_drv_lib/omnibot_platform_mecanum_drv_lib.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OMNIBOT_PLATFORM_MECANUM_DRV>());
    rclcpp::shutdown();
    return 0;
}
