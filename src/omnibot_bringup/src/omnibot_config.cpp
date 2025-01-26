#include <memory>

#include "rclcpp/rclcpp.hpp"



class omnibot_config: public rclcpp::Node
{
public:
    omnibot_config() : Node("omnibot_config",rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {
        rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        
        std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_results = 
            parameters_client->load_parameters(this->get_parameter("config_file_path").as_string());
        
    }

private:

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<omnibot_config>());
    rclcpp::shutdown();
    return 0;
}
