#ifndef __OMNIBOT_PLATFORM_MECANUM_DRV_LIB_HPP_
#define __OMNIBOT_PLATFORM_MECANUM_DRV_LIB_HPP_
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"

class OMNIBOT_PLATFORM_MECANUM_DRV : public rclcpp::Node
{
public:
    OMNIBOT_PLATFORM_MECANUM_DRV() : Node("omnibot_platform_mecanum_drv")
    {
        this->declare_parameter<std::string>("config_file_path", ""); // создаем параметр

        std::string config_file_path = this->get_parameter("config_file_path").as_string(); // получаем значение
        
        YAML::Node config = YAML::LoadFile(config_file_path);                                             // загружаем конфиг
        if (config.IsNull())                                                                              // если конфиг не загрузился т.е.  он НУЛЬ
            RCLCPP_FATAL(this->get_logger(), "Failed to load config file: %s", config_file_path.c_str()); // выводим ФАТАЛЬНУЮ ошибку

        _wheel_base_width = config["platform_parameters"]["wheel_base_width"].as<float>(); //записываем ширину колесной базы 
        _wheel_base_length = config["platform_parameters"]["wheel_base_lenth"].as<float>(); //записываем длину колесной базы
        _wheel_radius = config["platform_parameters"]["wheel_radius"].as<float>(); //записываем радиус колеса
        _wheel_count = config["platform_parameters"]["wheel_count"].as<int>(); //записываем количество колес

        _input_topic_name = config["input_topic_name"].as<std::string>(); //записываем имя входного топика
        _output_topic_name = config["output_topic_name"].as<std::string>(); //записываем имя выходного топика

        _loop_frequency = config["calculate_frequency_hz"].as<float>(); //записываем частоту расчетов

        _target_linear_x_velocity_reverse = config["input_topic_inverse"]["linear_x"].as<bool>(); //записываем инверсию линейной скорости    
        _target_linear_y_velocity_reverse = config["input_topic_inverse"]["linear_y"].as<bool>(); //записываем инверсию линейной скорости        
        _target_angular_z_velocity_reverse = config["input_topic_inverse"]["angular_z"].as<bool>(); //записываем инверсию угловой скорости
        
        _left_front_wheel_inverse_velocity = config["wheels_joints_names"]["left_front"]["reverse"].as<bool>(); //записываем инверсию левого переднего колеса
        _right_front_wheel_inverse_velocity = config["wheels_joints_names"]["right_front"]["reverse"].as<bool>(); //записываем инверсию правого переднего колеса
        _left_back_wheel_inverse_velocity = config["wheels_joints_names"]["left_back"]["reverse"].as<bool>(); //записываем инверсию левого заднего колеса
        _right_back_wheel_inverse_velocity = config["wheels_joints_names"]["right_back"]["reverse"].as<bool>(); //записываем инверсию правого заднего колеса

        _left_front_wheel_joint_name = config["wheels_joints_names"]["left_front"]["name"].as<std::string>(); //записываем имя левого переднего колеса
        _right_front_wheel_joint_name = config["wheels_joints_names"]["right_front"]["name"].as<std::string>(); //записываем имя правого переднего колеса
        _left_back_wheel_joint_name = config["wheels_joints_names"]["left_back"]["name"].as<std::string>(); //записываем имя левого заднего колеса
        _right_back_wheel_joint_name = config["wheels_joints_names"]["right_back"]["name"].as<std::string>(); //записываем имя правого заднего колеса
        

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(_input_topic_name, /*создаем подписку на топик */
                                                                            10,
                                                                            std::bind(
                                                                                &OMNIBOT_PLATFORM_MECANUM_DRV::_cmd_vel_sub_cb_f,
                                                                                this,
                                                                                std::placeholders::_1));

        _wheels_joints_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(_output_topic_name, 10); //создаем публикатор 
        

        RCLCPP_INFO(this->get_logger(), "OMNIBOT_PLATFOR_MECANUM_DRV config loaded successfully!!!");
        std::cout << "\033[93m#########################################\033[0m" << std::endl;
        std::cout << "\033[36mPlatform parameters config:\033[0m " << std::endl;
        std::cout << "--wheel_base_width: " << _wheel_base_width << std::endl;
        std::cout << "--wheel_base_length: " << _wheel_base_length << std::endl;
        std::cout << "--wheel_radius: " << _wheel_radius << std::endl << std::endl;
        std::cout << "\033[36mTopics config:\033[0m " << std::endl;
        std::cout << "--input_topic_name: " << _input_topic_name << std::endl;
        std::cout << "--output_topic_name: " << _output_topic_name << std::endl;
        std::cout << "--calculate_frequency_hz: " << _loop_frequency << std::endl << std::endl;
        std::cout << "\033[36mInput topic inverse config:\033[0m " << std::endl;
        std::cout << "--linear_x: " << (_target_linear_x_velocity_reverse ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl;
        std::cout << "--linear_y: " << (_target_linear_y_velocity_reverse ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl;
        std::cout << "--angular_z: " << (_target_angular_z_velocity_reverse ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl << std::endl;
        std::cout << "\033[36mWheels joints config:\033[0m " << std::endl;
        std::cout << "--left_front_wheel:"<<std::endl;
        std::cout << "\t---name: " << _left_front_wheel_joint_name << std::endl;
        std::cout << "\t---reverse: " << (_left_front_wheel_inverse_velocity ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl;
        std::cout << "--right_front_wheel:"<<std::endl;
        std::cout << "\t---name: " << _right_front_wheel_joint_name << std::endl;
        std::cout << "\t---reverse: " << (_right_front_wheel_inverse_velocity ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl;
        std::cout << "--left_back_wheel:"<<std::endl;
        std::cout << "\t---name: " << _left_back_wheel_joint_name << std::endl;
        std::cout << "\t---reverse: " << (_left_back_wheel_inverse_velocity ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl;
        std::cout << "--right_back_wheel:"<<std::endl;
        std::cout << "\t---name: " << _right_back_wheel_joint_name << std::endl;
        std::cout << "\t---reverse: " << (_right_back_wheel_inverse_velocity ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m") << std::endl << std::endl;
        std::cout << "\033[93m#########################################\033[0m" << std::endl;

        _wheels_joints_cmd_msg.name.resize(_wheel_count);
        _wheels_joints_cmd_msg.position.resize(_wheel_count);
        _wheels_joints_cmd_msg.velocity.resize(_wheel_count);
        _wheels_joints_cmd_msg.effort.resize(_wheel_count);

        _wheels_joints_cmd_msg.name = {
            _left_front_wheel_joint_name, 
            _right_front_wheel_joint_name, 
            _left_back_wheel_joint_name, 
            _right_back_wheel_joint_name
        };

        _wheels_joints_cmd_msg.effort   = {0.0, 0.0, 0.0, 0.0};
        _wheels_joints_cmd_msg.position = {0.0, 0.0, 0.0, 0.0};
        _wheels_joints_cmd_msg.velocity = {0.0, 0.0, 0.0, 0.0};

        auto loop = [this]() -> void {
            
            _wheels_joints_cmd_msg.velocity[0] = (1 / _wheel_radius) * (_target_linear_x_velocity - _target_linear_y_velocity - (_wheel_base_length / 2 + _wheel_base_width / 2) * _target_angular_z_velocity);
            _wheels_joints_cmd_msg.velocity[1] = (1 / _wheel_radius) * (_target_linear_x_velocity + _target_linear_y_velocity + (_wheel_base_length / 2 + _wheel_base_width / 2) * _target_angular_z_velocity);
            _wheels_joints_cmd_msg.velocity[2] = (1 / _wheel_radius) * (_target_linear_x_velocity + _target_linear_y_velocity - (_wheel_base_length / 2 + _wheel_base_width / 2) * _target_angular_z_velocity);
            _wheels_joints_cmd_msg.velocity[3] = (1 / _wheel_radius) * (_target_linear_x_velocity - _target_linear_y_velocity + (_wheel_base_length / 2 + _wheel_base_width / 2) * _target_angular_z_velocity);
            
            _wheels_joints_cmd_msg.velocity[0] = _left_front_wheel_inverse_velocity ? -_wheels_joints_cmd_msg.velocity[0] : _wheels_joints_cmd_msg.velocity[0];
            _wheels_joints_cmd_msg.velocity[1] = _right_front_wheel_inverse_velocity ? -_wheels_joints_cmd_msg.velocity[1] : _wheels_joints_cmd_msg.velocity[1];
            _wheels_joints_cmd_msg.velocity[2] = _left_back_wheel_inverse_velocity ? -_wheels_joints_cmd_msg.velocity[2] : _wheels_joints_cmd_msg.velocity[2];
            _wheels_joints_cmd_msg.velocity[3] = _right_back_wheel_inverse_velocity ? -_wheels_joints_cmd_msg.velocity[3] : _wheels_joints_cmd_msg.velocity[3];
            
            _wheels_joints_cmd_msg.header.stamp = this->now();
            _wheels_joints_cmd_pub->publish(_wheels_joints_cmd_msg);
        };

        timer_ = this->create_wall_timer(
            rclcpp::Rate(_loop_frequency).period(), loop
        );
    };

    ~OMNIBOT_PLATFORM_MECANUM_DRV()
    {

    }


private:
    std::string _input_topic_name = ""; 
    std::string _output_topic_name = "";
    float _loop_frequency = 1.0;

    sensor_msgs::msg::JointState _wheels_joints_cmd_msg;

    float _wheel_base_width = 0.0;
    float _wheel_base_length = 0.0;
    float _wheel_radius = 0.0;
    uint8_t _wheel_count = 0;


    std::string _left_front_wheel_joint_name = "";
    std::string _right_front_wheel_joint_name = "";
    std::string _left_back_wheel_joint_name = "";
    std::string _right_back_wheel_joint_name = "";

    bool _left_front_wheel_inverse_velocity = false;
    bool _right_front_wheel_inverse_velocity = false;
    bool _left_back_wheel_inverse_velocity = false;
    bool _right_back_wheel_inverse_velocity = false;
    

    float _target_linear_x_velocity = 0.0;
    bool _target_linear_x_velocity_reverse = false;

    float _target_linear_y_velocity = 0.0;
    bool _target_linear_y_velocity_reverse = false;

    float _target_angular_z_velocity = 0.0;
    bool _target_angular_z_velocity_reverse = false;

    float _current_linear_x_velocity = 0.0;
    float _current_linear_y_velocity = 0.0;
    float _current_angular_z_velocity = 0.0;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _wheels_joints_cmd_pub;

    void _cmd_vel_sub_cb_f(const geometry_msgs::msg::Twist msg);
    
};

void OMNIBOT_PLATFORM_MECANUM_DRV::_cmd_vel_sub_cb_f(const geometry_msgs::msg::Twist msg)
{
    _target_linear_x_velocity = _target_linear_x_velocity_reverse ? -msg .linear.x : msg.linear.x;
    _target_linear_y_velocity = _target_linear_y_velocity_reverse ? -msg .linear.y : msg.linear.y;
    _target_angular_z_velocity = _target_angular_z_velocity_reverse ? -msg .angular.z : msg.angular.z;
}

#endif // __/* FILE_NAME */_HPP_
