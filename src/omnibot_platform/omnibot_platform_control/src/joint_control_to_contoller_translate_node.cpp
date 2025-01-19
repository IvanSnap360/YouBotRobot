#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joint_control_to_contoller_translate_lib/joint_control_to_contoller_translate_lib.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB>());
    rclcpp::shutdown();
    return 0;
}
