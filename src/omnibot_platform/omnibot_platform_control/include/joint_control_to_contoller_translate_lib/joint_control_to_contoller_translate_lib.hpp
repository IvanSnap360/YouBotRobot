#ifndef JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB_HPP_
#define JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include  "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"


class JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB : public rclcpp::Node
{
public:
    JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB() : Node("joint_control_to_contoller_translate_node")
    {
        this->declare_parameter("config_file_path","");

        std::string config_file_path = this->get_parameter("config_file_path").as_string();

        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config.IsNull())
            RCLCPP_FATAL(this->get_logger(), "Failed to load config file: %s", config_file_path.c_str());
        
        

        _from_topic_name = config["from"].as<std::string>();
        _to_topic_list = config["to"].as<std::vector<std::string>>();

        _from_topic_sub = this->create_subscription<sensor_msgs::msg::JointState>(_from_topic_name, 
                                                                                    10, 
                                                                                    std::bind(
                                                                                        &JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB::_from_topic_cb_f, 
                                                                                        this, 
                                                                                        std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\033[32mSuccessfully loaded config file:\033[0m %s", config_file_path.c_str());
        std::cout << "\033[93m#########################################\033[0m" << std::endl;
        std::cout << "\033[36mFrom topic:\033[0m " << _from_topic_name << std::endl;
        std::cout << "\033[36mTo topics:\033[0m " << std::endl;
        for (std::size_t i = 0; i < _to_topic_list.size(); i++)
        {
            std::cout << '\t' << _to_topic_list[i] << std::endl;
        }
        std::cout << "\033[93m#########################################\033[0m" << std::endl;

    }

private:
    std::string _from_topic_name;
    sensor_msgs::msg::JointState _from_topic_msg;
    bool _init_done = false;

    std::vector<std::string> _to_topic_list;


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _from_topic_sub;
    

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> _controllers_publishers;

    void _from_topic_cb_f(const sensor_msgs::msg::JointState::SharedPtr msg);
};

void JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB::_from_topic_cb_f(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!_init_done)
    {
        _controllers_publishers.resize(msg->name.size());

        for (std::size_t i = 0; i < msg->name.size(); i++)
        {
            _controllers_publishers[i] = this->create_publisher<std_msgs::msg::Float64>(_to_topic_list[i], 10);
        }    
        
        _init_done = true;
    }
    
    for (std::size_t i = 0; i < msg->name.size(); i++)
    {
        static std_msgs::msg::Float64 msg_to_publish;
        msg_to_publish.data = msg->velocity[i];
        _controllers_publishers[i]->publish(msg_to_publish);
        
    }
    
     
}

#endif  // JOINT_CONTROL_TO_CONTROLLERS_TRANSLATE_LIB_HPP_
