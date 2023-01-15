#ifndef __OMNIBOT_PLATFORM_TELEOP_LIB_H__
#define __OMNIBOT_PLATFORM_TELEOP_LIB_H__
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>

class OMNIBOT_PLATFORM_TELEOP_LIB
{
private:
    ros::Publisher _cmd_vel_pub;
    YAML::Node _config;
    std::string _keys_mode;
    std::string _output_topic;

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
    int  _info_msg_skip_keys_counter = 0;
    int  _info_msg_skip_keys_counter_max = 0;

    double  _max_linear_x_velocity,
            _min_linear_x_velocity,
            _max_linear_y_velocity,
            _min_linear_y_velocity,
            _max_angular_z_velocity,
            _min_angular_z_velocity,

            _velocity_step;

    double  _current_linear_x_velocity,
            _current_linear_y_velocity,
            _current_angular_z_velocity;



    bool _angular_z_velocity_inverse,
         _linear_x_velocity_inverse,
         _linear_y_velocity_inverse;
    


    geometry_msgs::Twist _cmd_vel_msg;

public:
    OMNIBOT_PLATFORM_TELEOP_LIB(ros::NodeHandle *nh, const std::string teleop_param_config_path, std::string keys_mode);
    int process();
    double getWorkRate();
    void print_instructions();
    ~OMNIBOT_PLATFORM_TELEOP_LIB();
};

#endif // __OMNIBOT_PLATFORM_TELEOP_LIB_H__