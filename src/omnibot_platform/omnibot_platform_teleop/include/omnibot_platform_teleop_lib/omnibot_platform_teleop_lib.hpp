#ifndef OMNIBOT_PLATFORM_TELEOP_HPP_
#define OMNIBOT_PLATFORM_TELEOP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "yaml-cpp/yaml.h"
#include <unistd.h>
#include <termios.h>
#include <cstdlib>
#include <signal.h>
#include <chrono>
#include <iomanip>

using namespace std::chrono_literals;

double constrain(double x, double min, double max)
{
    if (x > max)
        return max;
    else if (x < min)
        return min;
    else
        return x;
}

int getch(void)
{
     int ch = -1;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

class OMNIBOT_PLATFORM_TELEOP : public rclcpp::Node
{
public:
    OMNIBOT_PLATFORM_TELEOP() : Node("omnibot_platform_teleop")
    {
        rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "omnibot_config");
        while (!parameters_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // ##############################################################################
        this->declare_parameter("config_file_path", "");
        this->declare_parameter("keys_mode", "numpad_keys");

        std::string config_file_path = this->get_parameter("config_file_path").as_string();
        YAML::Node config = YAML::LoadFile(config_file_path);
        if (config.IsNull())
            RCLCPP_FATAL(this->get_logger(), "Failed to load config file: %s", config_file_path.c_str());

        // ##############################################################################
        _output_topic_path = config["output_topic"].as<std::string>();
        _keys_mode = this->get_parameter("keys_mode").as_string();
        _work_rate = config["workRate"].as<double>();

        _info_msg_skip_keys_counter = config["info_msg_counter_max"].as<int>();

        _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(_output_topic_path, 10);
        _cmd_vel_msg.linear.x = 0.0;
        _cmd_vel_msg.linear.y = 0.0;
        _cmd_vel_msg.angular.z = 0.0;

        // ##############################################################################
        velocities_limits__linear_x_max__path = config["velocities_params_paths"]["max_linear_x_velocity"].as<std::string>();
        velocities_limits__linear_x_min__path = config["velocities_params_paths"]["min_linear_x_velocity"].as<std::string>();
        // ##############################################################################
        velocities_limits__linear_y_max__path = config["velocities_params_paths"]["max_linear_y_velocity"].as<std::string>();
        velocities_limits__linear_y_min__path = config["velocities_params_paths"]["min_linear_y_velocity"].as<std::string>();
        // ##############################################################################
        velocities_limits__angular_z_max__path = config["velocities_params_paths"]["max_angular_z_velocity"].as<std::string>();
        velocities_limits__angular_z_min__path = config["velocities_params_paths"]["min_angular_z_velocity"].as<std::string>();
        // ##############################################################################

        velocities_limits__linear_x_max = parameters_client->get_parameter(velocities_limits__linear_x_max__path, 1.0);
        velocities_limits__linear_x_min = parameters_client->get_parameter(velocities_limits__linear_x_min__path, -1.0);

        // ###########################

        velocities_limits__linear_y_max = parameters_client->get_parameter(velocities_limits__linear_y_max__path, 1.0);
        velocities_limits__linear_y_min = parameters_client->get_parameter(velocities_limits__linear_y_min__path, -1.0);

        // ###########################

        velocities_limits__angular_z_max = parameters_client->get_parameter(velocities_limits__angular_z_max__path, 1.0);
        velocities_limits__angular_z_min = parameters_client->get_parameter(velocities_limits__angular_z_min__path, -1.0);

        // ##############################################################################
        _linear_x_inverse = config["velocities_inverse"]["linear_x_velocity"].as<bool>();
        _linear_y_inverse = config["velocities_inverse"]["linear_y_velocity"].as<bool>();
        _angular_z_inverse = config["velocities_inverse"]["angular_z_velocity"].as<bool>();
        // ##############################################################################
        _moving_forward_key_code = config[_keys_mode]["moving"]["forward"].as<char>();
        _moving_backward_key_code = config[_keys_mode]["moving"]["backward"].as<char>();
        _moving_left_key_code = config[_keys_mode]["moving"]["left"].as<char>();
        _moving_right_key_code = config[_keys_mode]["moving"]["right"].as<char>();
        _moving_forward_left_key_code = config[_keys_mode]["moving"]["forward_left"].as<char>();
        _moving_forward_right_key_code = config[_keys_mode]["moving"]["forward_right"].as<char>();
        _moving_backward_left_key_code = config[_keys_mode]["moving"]["backward_left"].as<char>();
        _moving_backward_right_key_code = config[_keys_mode]["moving"]["backward_right"].as<char>();
        _stop_key_code = config[_keys_mode]["moving"]["stop"].as<char>();

        _rotating_clockwise_key_code = config[_keys_mode]["rotating"]["clockwise"].as<char>();
        _rotating_counterclockwise_key_code = config[_keys_mode]["rotating"]["counterclockwise"].as<char>();
        _terminate_program_key = config["exit_key"].as<char>();

        sprintf(_info_msg, R"anystring(
            Reading from the keyboard and Publishing to Twist!
                ⬉     ⬆      ⬈
                 %c   %c   %c
                ⬅%c   %c   %c ➡  
                 %c   %c   %c 
                ⬋     ⬇      ⬊

                ↻ ClockWise Rotating        - %c
                ↺ CounterClockWise Rotating - %c
                
                Terminate program - %c
            )anystring",
                _moving_forward_left_key_code, _moving_forward_key_code, _moving_forward_right_key_code,
                _moving_left_key_code, _stop_key_code, _moving_right_key_code,
                _moving_backward_left_key_code, _moving_backward_key_code, _moving_backward_right_key_code,
                _rotating_clockwise_key_code, _rotating_counterclockwise_key_code,
                _terminate_program_key);

        // ##############################################################################

        RCLCPP_INFO(this->get_logger(), "\033[32mSuccessfully loaded config file:\033[0m %s", config_file_path.c_str());
        std::cout << "\033[93m#########################################\033[0m" << std::endl;
        std::cout << "\033[0mOutput topic:\033[0m " << _output_topic_path << std::endl;
        std::cout << "\033[0mKeys mode:\033[0m " << _keys_mode << std::endl;
        std::cout << "\033[0mWork rate:\033[0m " << _work_rate << " [Hz]" << std::endl;
        std::cout << "\033[0mVelocity control_step:\033[0m " << _velocity_control_step << " [m/s]" << std::endl;
        std::cout << "\033[36m Velocities limits:\033[0m " << std::endl;
        std::cout << "\033[0m   max_linear_x_velocity:\033[0m " << std::fixed << std::setprecision(1) << velocities_limits__linear_x_max << " m/s" << std::endl;
        std::cout << "\033[0m   min_linear_x_velocity:\033[0m " << std::fixed << std::setprecision(1) << velocities_limits__linear_x_min << " m/s" << std::endl;
        std::cout << "\033[0m   max_linear_y_velocity:\033[0m " << std::fixed << std::setprecision(1) << velocities_limits__linear_y_max << " m/s" << std::endl;
        std::cout << "\033[0m   min_linear_y_velocity:\033[0m " << std::fixed << std::setprecision(1) << velocities_limits__linear_y_min << " m/s" << std::endl;
        std::cout << "\033[0m   max_angular_z_velocity:\033[0m " << std::fixed << std::setprecision(1) << velocities_limits__angular_z_max << " rad/s" << std::endl;
        std::cout << "\033[0m   min_angular_z_velocity:\033[0m " << std::fixed << std::setprecision(1) << velocities_limits__angular_z_min << " rad/s" << std::endl;
        std::cout << "\033[36mVelocities_inverse:\033[0m " << std::endl;
        std::cout << "\033[0m   linear_x_velocity:\033[0m " << (_linear_x_inverse ? "true" : "false") << std::endl;
        std::cout << "\033[0m   linear_y_velocity:\033[0m " << (_linear_y_inverse ? "true" : "false") << std::endl;
        std::cout << "\033[0m   angular_z_velocity:\033[0m " << (_angular_z_inverse ? "true" : "false") << std::endl;
        std::cout << "\033[93m#########################################\033[0m" << std::endl;

        _info_msg_skip_keys_counter = _info_msg_skip_keys_counter_max+10;
        _cmd_vel_pub->publish(_cmd_vel_msg);
        
        rclcpp::Rate work_rate(_work_rate);
        while (rclcpp::ok())
        {

            if (process() == -1) break;
            _cmd_vel_pub->publish(_cmd_vel_msg);
            rclcpp::spin_some(this->get_node_base_interface());   
            work_rate.sleep();
        }
        rclcpp::shutdown();
    }

private:
    std::string _keys_mode;
    std::string _output_topic_path = "/cmd_vel";
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
    geometry_msgs::msg::Twist _cmd_vel_msg;
    int info_msg_counter_max = 30;

    float _velocity_control_step = 0.05;
    std::string workMode = "terminal";

    uint16_t _info_msg_counter_max = 30;

    std::string velocities_limits__linear_x_max__path;
    std::string velocities_limits__linear_x_min__path;
    float velocities_limits__linear_x_max = 1.0;
    float velocities_limits__linear_x_min = -1.0;

    std::string velocities_limits__linear_y_max__path;
    std::string velocities_limits__linear_y_min__path;
    float velocities_limits__linear_y_max = 1.0;
    float velocities_limits__linear_y_min = -1.0;

    std::string velocities_limits__angular_z_max__path;
    std::string velocities_limits__angular_z_min__path;
    float velocities_limits__angular_z_max = 1.0;
    float velocities_limits__angular_z_min = -1.0;

    bool _linear_x_inverse = false;
    bool _linear_y_inverse = false;
    bool _angular_z_inverse = false;

    float _current_linear_x_velocity = 0.0;
    float _current_linear_y_velocity = 0.0;
    float _current_angular_z_velocity = 0.0;

    char
        _moving_forward_key_code,
        _moving_backward_key_code,
        _moving_left_key_code,
        _moving_right_key_code,
        _moving_forward_left_key_code,
        _moving_forward_right_key_code,
        _moving_backward_left_key_code,
        _moving_backward_right_key_code,
        _stop_key_code,
        _rotating_clockwise_key_code,
        _rotating_counterclockwise_key_code,
        _terminate_program_key;

    char _info_msg[400];
    int _info_msg_skip_keys_counter = 0;
    int _info_msg_skip_keys_counter_max = 0;

    float _work_rate = 10.0;

    int process()
    {
        char ch = getch();
        if (ch != -1)
        {
            if (_info_msg_skip_keys_counter > _info_msg_skip_keys_counter_max)
            {
                std::cout << _info_msg << std::endl;
                _info_msg_skip_keys_counter = 0;
            }
            else
                _info_msg_skip_keys_counter++;
            
            if (ch == _terminate_program_key)
            {
                return 0;
            }
            else if (ch == _moving_forward_key_code)
            {
                _current_linear_x_velocity += (_linear_x_inverse ? -_velocity_control_step : _velocity_control_step);
            }
            else if (ch == _moving_backward_key_code)
            {
                _current_linear_x_velocity += (_linear_x_inverse ? _velocity_control_step : -_velocity_control_step);
            }
            else if (ch == _moving_left_key_code)
            {
                _current_linear_y_velocity += (_linear_y_inverse ? -_velocity_control_step : _velocity_control_step);
            }
            else if (ch == _moving_right_key_code)
            {
                _current_linear_y_velocity += (_linear_y_inverse ? _velocity_control_step : -_velocity_control_step);
            }
            else if (ch == _moving_forward_left_key_code)
            {
                _current_linear_x_velocity += (_linear_x_inverse ? -_velocity_control_step : _velocity_control_step);
                _current_linear_y_velocity += (_linear_y_inverse ? -_velocity_control_step : _velocity_control_step);
            }
            else if (ch == _moving_forward_right_key_code)
            {
                _current_linear_x_velocity += (_linear_x_inverse ? -_velocity_control_step : _velocity_control_step);
                _current_linear_y_velocity += (_linear_y_inverse ? _velocity_control_step : -_velocity_control_step);
            }
            else if (ch == _moving_backward_left_key_code)
            {
                _current_linear_x_velocity += (_linear_x_inverse ? _velocity_control_step : -_velocity_control_step);
                _current_linear_y_velocity += (_linear_y_inverse ? -_velocity_control_step : _velocity_control_step);
            }
            else if (ch == _moving_backward_right_key_code)
            {
                _current_linear_x_velocity += (_linear_x_inverse ? _velocity_control_step : -_velocity_control_step);
                _current_linear_y_velocity += (_linear_y_inverse ? _velocity_control_step : -_velocity_control_step);
            }
            else if (ch == _rotating_clockwise_key_code)
            {
                _current_angular_z_velocity += (_angular_z_inverse ? -_velocity_control_step : _velocity_control_step);
            }
            else if (ch == _rotating_counterclockwise_key_code)
            {
                _current_angular_z_velocity += (_angular_z_inverse ? _velocity_control_step : -_velocity_control_step);
            }
            else if (ch == _stop_key_code)
            {
                _current_linear_x_velocity = 0.0;
                _current_linear_y_velocity = 0.0;
                _current_angular_z_velocity = 0.0;
            }
            else
            {
                return -1;
            }

            _current_linear_x_velocity  = constrain(_current_linear_x_velocity, velocities_limits__linear_x_min, velocities_limits__linear_x_max);
            _current_linear_y_velocity  = constrain(_current_linear_y_velocity, velocities_limits__linear_y_min, velocities_limits__linear_y_max);
            _current_angular_z_velocity = constrain(_current_angular_z_velocity, velocities_limits__angular_z_min, velocities_limits__angular_z_max);

            RCLCPP_INFO(this->get_logger(), "Publishing velocities to [%s] topic : linear_x: %f linear_y: %f angular_z: %f",
                        _output_topic_path.c_str(), _current_linear_x_velocity, _current_linear_y_velocity, _current_angular_z_velocity);

            _cmd_vel_msg.linear.x = _current_linear_x_velocity;
            _cmd_vel_msg.linear.y = _current_linear_y_velocity;
            _cmd_vel_msg.angular.z = _current_angular_z_velocity;
            

        };

        
        _cmd_vel_pub->publish(_cmd_vel_msg);
        return 0;
    };
};

#endif // OMNIBOT_PLATFORM_TELEOP_HPP_
