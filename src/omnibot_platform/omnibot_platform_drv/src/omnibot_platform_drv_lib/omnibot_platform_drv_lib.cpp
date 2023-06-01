#include "omnibot_platform_drv_lib/omnibot_platform_drv_lib.h"


OMNIBOT_PLATFORM_DRV_LIB::OMNIBOT_PLATFORM_DRV_LIB(ros::NodeHandle *nh, std::string config_path)
{
    _config = YAML::LoadFile(config_path);

    if (_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    _output_topic_frame_id = _config["output_topic_frame_id"].as<std::string>();
    _input_topic_name = _config["input_topic"].as<std::string>();
    _output_topic_name = _config["output_topic"].as<std::string>();
    _output_topic_rate = _config["output_topic_rate"].as<double>();

    _linear_x_velocity_min_limit = _config["velocities_limits"]["linear_x_min"].as<double>();
    _linear_y_velocity_min_limit = _config["velocities_limits"]["linear_y_min"].as<double>();
    _angular_z_velocity_min_limit = _config["velocities_limits"]["angular_z_min"].as<double>();

    _linear_x_velocity_max_limit = _config["velocities_limits"]["linear_x_max"].as<double>();
    _linear_y_velocity_max_limit = _config["velocities_limits"]["linear_y_max"].as<double>();
    _angular_z_velocity_max_limit = _config["velocities_limits"]["angular_z_max"].as<double>();    

    _wheel_base_lenth = _config["wheel_base_lenth"].as<double>();
    _wheel_base_width = _config["wheel_base_width"].as<double>();
    _wheels_deameter  = _config["wheels_deameter"].as<double>();
    _wheels_width     = _config["wheels_width"].as<double>();
    _wheels_max_velocity = _config["wheels_max_velocity"].as<double>();

    _wheels_joints_count = _config["wheels_joints_count"].as<int>();

    _joints_names.resize(_wheels_joints_count);
    _joints_reverse.resize(_wheels_joints_count);

    _joints_control_msg.name.resize(_wheels_joints_count);
    _joints_control_msg.effort.resize(_wheels_joints_count);
    _joints_control_msg.velocity.resize(_wheels_joints_count);
    _joints_control_msg.position.resize(_wheels_joints_count);
    
    inverse_linear_x = _config["cmd_vel_inverse"]["linear_x"].as<bool>();
    inverse_linear_y = _config["cmd_vel_inverse"]["linear_y"].as<bool>();
    inverse_angular_z = _config["cmd_vel_inverse"]["angular_z"].as<bool>();

    if (_check_is_node_IsSequence(_config,"wheels_joints_names"))
    {
        _joints_names = _config["wheels_joints_names"].as<std::vector<std::string>>();
        _joints_control_msg.name = _joints_names;
        


        ROS_INFO(R"(Recieve this joints: )"); 
        for (const auto &name : _joints_control_msg.name) std::cout << "\033[36m" << name << "  "; 
        std::cout <<"\033[0m"<< std::endl; 
    } 

    if (_check_is_node_IsSequence(_config,"wheels_joints_reverse"))
    {
        _joints_reverse = _config["wheels_joints_reverse"].as<std::vector<bool>>();

        for (int i  = 0; i < _wheels_joints_count; i++)
        {
            std::cout << "Joint \033[36m"  << _joints_names[i] << "\033[0m Reverse: "<< (_joints_reverse[i] ? "TRUE" : "FALSE") << std::endl;
        }
    }

    _joints_control_msg.header.frame_id = _output_topic_frame_id;
    _cmd_vel_sub = nh->subscribe<geometry_msgs::Twist>(_input_topic_name,10,&OMNIBOT_PLATFORM_DRV_LIB::_cmd_vel_sub_cb_f,this);
    _joint_control_pub = nh->advertise<sensor_msgs::JointState>(_output_topic_name,10);
}


void OMNIBOT_PLATFORM_DRV_LIB::_cmd_vel_sub_cb_f(const geometry_msgs::Twist::ConstPtr &twist)
{
    _cmd_vel_input_msg.linear.x = inverse_linear_x ? -twist->linear.x : twist->linear.x;
    _cmd_vel_input_msg.linear.y = inverse_linear_y ? -twist->linear.y : twist->linear.y; 
    _cmd_vel_input_msg.angular.z = inverse_angular_z ? -twist->angular.z : twist->angular.z;
}


OMNIBOT_PLATFORM_DRV_LIB::~OMNIBOT_PLATFORM_DRV_LIB()
{

}

void OMNIBOT_PLATFORM_DRV_LIB::process(const ros::TimerEvent&)
{
    double r = _wheels_deameter   / 2;
    double Lx = _wheel_base_lenth / 2;
    double Ly = _wheel_base_width / 2;

    _wheels_target_velocities.w1 = (1 / r) * (_cmd_vel_input_msg.linear.x - _cmd_vel_input_msg.linear.y - (Lx + Ly) * _cmd_vel_input_msg.angular.z);
    _wheels_target_velocities.w2 = (1 / r) * (_cmd_vel_input_msg.linear.x + _cmd_vel_input_msg.linear.y + (Lx + Ly) * _cmd_vel_input_msg.angular.z);
    _wheels_target_velocities.w3 = (1 / r) * (_cmd_vel_input_msg.linear.x + _cmd_vel_input_msg.linear.y - (Lx + Ly) * _cmd_vel_input_msg.angular.z);
    _wheels_target_velocities.w4 = (1 / r) * (_cmd_vel_input_msg.linear.x - _cmd_vel_input_msg.linear.y + (Lx + Ly) * _cmd_vel_input_msg.angular.z);

    _wheels_target_velocities.w1 = _joints_reverse[0] ? -_wheels_target_velocities.w1 : _wheels_target_velocities.w1;
    _wheels_target_velocities.w2 = _joints_reverse[1] ? -_wheels_target_velocities.w2 : _wheels_target_velocities.w2;
    _wheels_target_velocities.w3 = _joints_reverse[2] ? -_wheels_target_velocities.w3 : _wheels_target_velocities.w3;
    _wheels_target_velocities.w4 = _joints_reverse[3] ? -_wheels_target_velocities.w4 : _wheels_target_velocities.w4;

    if (_wheels_target_velocities.w1 > _wheels_max_velocity)
    {
        _wheels_target_velocities.w1 = _wheels_max_velocity;
    }
    else if (_wheels_target_velocities.w1 < -_wheels_max_velocity)
    {
        _wheels_target_velocities.w1 = -_wheels_max_velocity;
    }

    if (_wheels_target_velocities.w2 > _wheels_max_velocity)
    {
        _wheels_target_velocities.w2 = _wheels_max_velocity;
    }
    else if (_wheels_target_velocities.w2 < -_wheels_max_velocity)
    {
        _wheels_target_velocities.w2 = -_wheels_max_velocity;
    }

    if (_wheels_target_velocities.w3 > _wheels_max_velocity)
    {
        _wheels_target_velocities.w3 = _wheels_max_velocity;
    }
    else if (_wheels_target_velocities.w3 < -_wheels_max_velocity)
    {
        _wheels_target_velocities.w3 = -_wheels_max_velocity;
    }

    if (_wheels_target_velocities.w4 > _wheels_max_velocity)
    {
        _wheels_target_velocities.w4 = _wheels_max_velocity;
    }
    else if (_wheels_target_velocities.w4 < -_wheels_max_velocity)
    {
        _wheels_target_velocities.w4 = -_wheels_max_velocity;
    }

    

    _joints_control_msg.velocity[0] = _wheels_target_velocities.w1;
    _joints_control_msg.velocity[1] = _wheels_target_velocities.w2;
    _joints_control_msg.velocity[2] = _wheels_target_velocities.w3;
    _joints_control_msg.velocity[3] = _wheels_target_velocities.w4;

    for (int i = 0; i < _joints_control_msg.name.size(); i++)
    {
        std::string vel_str  = boost::lexical_cast<std::string>(_joints_control_msg.velocity[i]);
        
        if (vel_str.find((std::string)"e") != std::string::npos) 
            _joints_control_msg.velocity[i] = 0.0;
    }

    _joints_control_msg.header.stamp = ros::Time().now();
    _joints_control_msg.header.seq += 1;

    _joint_control_pub.publish(_joints_control_msg);
}      


double OMNIBOT_PLATFORM_DRV_LIB::getWorkRate()
{   
    return _output_topic_rate;
}


bool OMNIBOT_PLATFORM_DRV_LIB::_check_is_node_IsSequence(YAML::Node node, std::string name)
{
    if (!node[name].IsSequence())
    {
        ROS_FATAL("Node %s isn`t sequence!!! Stop Node!!!",name.c_str());
        ros::shutdown();
        return false;
        
    }
    else
    {
        ROS_INFO("Node %s is sequence!!!",name.c_str());
        return true;
    }

}

