#include "omnibot_platform_teleop_lib/omnibot_platform_teleop_lib.h"
double constrain(double x, double min, double max)
{
    if (x > max)
        return max;
    else if (x < min)
        return min;
    else
        return x;
}

OMNIBOT_PLATFORM_TELEOP_LIB::OMNIBOT_PLATFORM_TELEOP_LIB(ros::NodeHandle *nh, const std::string teleop_param_config_path, std::string keys_mode)
{
    _config = YAML::LoadFile(teleop_param_config_path);

    if (_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    _output_topic = _config["output_topic"].as<std::string>();
    _cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(_output_topic, 10);
    
    _moving_forward_key_code              = _config[keys_mode + "_keys"]["moving"]["forward"].as<char>();
    _moving_backward_key_code             = _config[keys_mode + "_keys"]["moving"]["backward"].as<char>();
    _moving_left_key_code                 = _config[keys_mode + "_keys"]["moving"]["left"].as<char>();
    _moving_right_key_code                = _config[keys_mode + "_keys"]["moving"]["right"].as<char>();
    _moving_forward_left_key_code         = _config[keys_mode + "_keys"]["moving"]["forward_left"].as<char>();
    _moving_forward_right_key_code        = _config[keys_mode + "_keys"]["moving"]["forward_right"].as<char>();
    _moving_backward_left_key_code        = _config[keys_mode + "_keys"]["moving"]["backward_left"].as<char>();
    _moving_backward_right_key_code       = _config[keys_mode + "_keys"]["moving"]["backward_right"].as<char>();
    _stop_key_code                        = _config[keys_mode + "_keys"]["moving"]["stop"].as<char>();
    _rotating_clockwise_key_code          = _config[keys_mode + "_keys"]["rotating"]["clockwise"].as<char>();
    _rotating_counterclockwise_key_code   = _config[keys_mode + "_keys"]["rotating"]["counterclockwise"].as<char>();
    _terminate_program_key                = _config["exit_key"].as<int>(); 
    _info_msg_skip_keys_counter_max       = _config["info_msg_counter_max"].as<int>();  
    _velocity_step                        = _config["velocity_control_step"].as<double>();

    _angular_z_velocity_inverse           = _config["velocities_inverse"]["angular_z_velocity"].as<bool>();         
    _linear_x_velocity_inverse            = _config["velocities_inverse"]["linear_x_velocity"].as<bool>();  
    _linear_y_velocity_inverse            = _config["velocities_inverse"]["linear_y_velocity"].as<bool>();  
     
    if(!nh->getParam(_config["velocities_params_paths"]["max_linear_x_velocity"].as<std::string>(),_max_linear_x_velocity))
    {
        ROS_WARN("Can`t get value for param max_linear_x_velocity,  set default value to  1.0");
        _max_linear_x_velocity = 1.0;
    }

    if(!nh->getParam(_config["velocities_params_paths"]["min_linear_x_velocity"].as<std::string>(),_min_linear_x_velocity))
    {
        ROS_WARN("Can`t get value for param min_linear_x_velocity,  set default value to -1.0 \n");
        _min_linear_x_velocity = -1.0;
    }

    if(!nh->getParam(_config["velocities_params_paths"]["max_linear_y_velocity"].as<std::string>(),_max_linear_y_velocity))
    {
        ROS_WARN("Can`t get value for param max_linear_y_velocity,  set default value to  1.0");
        _max_linear_y_velocity = 1.0;
    }

    if(!nh->getParam(_config["velocities_params_paths"]["min_linear_y_velocity"].as<std::string>(),_min_linear_y_velocity))
    {
        ROS_WARN("Can`t get value for param min_linear_y_velocity,  set default value to -1.0 \n");
        _min_linear_y_velocity = -1.0;
    }

    if(!nh->getParam(_config["velocities_params_paths"]["max_angular_z_velocity"].as<std::string>(),_max_angular_z_velocity))
    {
        ROS_WARN("Can`t get value for param max_angular_z_velocity, set default value to  1.0");
        _max_angular_z_velocity = 1.0;
    }

    if(!nh->getParam(_config["velocities_params_paths"]["min_angular_z_velocity"].as<std::string>(),_min_angular_z_velocity))
    {
        ROS_WARN("Can`t get value for param min_angular_z_velocity, set default value to -1.0 \n");
        _min_angular_z_velocity = -1.0;
    }

    
    sprintf(_info_msg, R"anystring(
       Reading from the keyboard and Publishing to Twist!
           ⬉     ⬆    ⬈
             %c   %c   %c
          ⬅  %c   %c   %c ➡  
             %c   %c   %c 
           ⬋     ⬇    ⬊

           ↻ ClockWise Rotating        - %c
           ↺ CounterClockWise Rotating - %c
                
    )anystring",
            _moving_forward_left_key_code, _moving_forward_key_code, _moving_forward_right_key_code, 
            _moving_left_key_code, _stop_key_code, _moving_right_key_code, 
            _moving_backward_left_key_code, _moving_backward_key_code, _moving_backward_right_key_code,
            _rotating_clockwise_key_code,_rotating_counterclockwise_key_code);

    // sprintf(_info_msg,"L %s",_info_msg);
    _info_msg_skip_keys_counter = _info_msg_skip_keys_counter_max+10;
}





OMNIBOT_PLATFORM_TELEOP_LIB::~OMNIBOT_PLATFORM_TELEOP_LIB()
{
}

int getch(void)
{
    int ch;
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

int OMNIBOT_PLATFORM_TELEOP_LIB::process()
{
    char ch = getch();
    
    if (_info_msg_skip_keys_counter > _info_msg_skip_keys_counter_max)
    {
        std::cout << _info_msg << std::endl;
        _info_msg_skip_keys_counter = 0;
    }
    else
        _info_msg_skip_keys_counter++;


    if (ch == _terminate_program_key)
    {
        return -1;
    } 
    else if (ch == _moving_forward_key_code)
    {
        _current_linear_x_velocity += (_linear_x_velocity_inverse ? -_velocity_step : _velocity_step);
    }
    else if (ch == _moving_backward_key_code)
    {
        _current_linear_x_velocity += (_linear_x_velocity_inverse ? _velocity_step : -_velocity_step);
    }
    else if (ch == _moving_left_key_code)
    {
        _current_linear_y_velocity += (_linear_y_velocity_inverse ? -_velocity_step : _velocity_step);
    }
    else if (ch == _moving_right_key_code)
    {
        _current_linear_y_velocity += (_linear_y_velocity_inverse ? _velocity_step : -_velocity_step);
    }
    else if (ch == _moving_forward_left_key_code)
    {
        _current_linear_x_velocity += (_linear_x_velocity_inverse ? -_velocity_step : _velocity_step);
        _current_linear_y_velocity += (_linear_y_velocity_inverse ? -_velocity_step : _velocity_step);
    }
    else if (ch == _moving_forward_right_key_code)
    {
        _current_linear_x_velocity += (_linear_x_velocity_inverse ? -_velocity_step : _velocity_step);
        _current_linear_y_velocity += (_linear_y_velocity_inverse ? _velocity_step : -_velocity_step);
    }
    else if (ch == _moving_backward_left_key_code)
    {
        _current_linear_x_velocity += (_linear_x_velocity_inverse ? _velocity_step : -_velocity_step);
        _current_linear_y_velocity += (_linear_y_velocity_inverse ? -_velocity_step : _velocity_step);
    }
    else if (ch == _moving_backward_right_key_code)
    {
        _current_linear_x_velocity += (_linear_x_velocity_inverse ? _velocity_step : -_velocity_step);
        _current_linear_y_velocity += (_linear_y_velocity_inverse ? _velocity_step : -_velocity_step);
    }
    else if (ch == _rotating_clockwise_key_code)
    {   
        _current_angular_z_velocity += (_angular_z_velocity_inverse ? -_velocity_step : _velocity_step);
    }
    else if (ch == _rotating_counterclockwise_key_code)
    {
        _current_angular_z_velocity += (_angular_z_velocity_inverse ? _velocity_step : +_velocity_step);
    }
    else if (ch == _stop_key_code)
    {
        _current_linear_x_velocity  = 0.0;
        _current_linear_y_velocity  = 0.0;
        _current_angular_z_velocity = 0.0;
    }

    _current_linear_x_velocity  = constrain(_current_linear_x_velocity,   _min_linear_x_velocity, _max_linear_x_velocity);
    _current_linear_y_velocity  = constrain(_current_linear_y_velocity,   _min_linear_x_velocity, _max_linear_x_velocity);
    _current_angular_z_velocity = constrain(_current_angular_z_velocity,  _min_linear_x_velocity, _max_linear_x_velocity);

    ROS_INFO("Publishing velocities to [%s] topic : linear_x: %f linear_y: %f angular_z: %f", 
        _output_topic.c_str(), _current_linear_x_velocity,_current_linear_y_velocity,_current_angular_z_velocity);

    _cmd_vel_msg.linear.x = _current_linear_x_velocity;
    _cmd_vel_msg.linear.y = _current_linear_y_velocity;
    _cmd_vel_msg.angular.z = _current_angular_z_velocity;

    _cmd_vel_pub.publish(_cmd_vel_msg);

    return 0;
}

double OMNIBOT_PLATFORM_TELEOP_LIB::getWorkRate()
{
    return _config["workRate"].as<double>();
}
