#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "omnibot_platform_drv_lib/omnibot_platform_drv_lib.h"
#include "yaml-cpp/yaml.h"
#include <iostream>

YAML::Node omnibot_platform_config; 

omnibot_platform_drv_cfg_t config;
OMNIBOT_PLATFORM_DRV_LIB op_drv;
wheels_angular_vel_t wheels_vels;

ros::Subscriber cmd_vel_sub;

sensor_msgs::JointState joint_control_msg;
ros::Publisher joint_control_pub;
ros::Timer joint_control_pub_timer;

void LoadConfigFile(const std::string yaml_file);
void cmd_vel_sub_cb_f(const geometry_msgs::Twist::ConstPtr &cmd_vel);
void joint_control_pub_f(const ros::TimerEvent &e);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "omnibot_platform_drv");
    ros::NodeHandle nh;

    std::string omnibot_platform_config_yaml_file_path = nh.param<std::string>("omnibot_platform_config", " ");

    LoadConfigFile(omnibot_platform_config_yaml_file_path);

    YAML::Node joint_reverse = omnibot_platform_config["wheels_joints_reverse"];
    assert(joint_reverse.Type() == YAML::NodeType::Sequence);
    assert(joint_reverse.IsSequence());

    for (std::size_t i = 0; i < joint_reverse.size(); i++)
    {
        config.reverse_wheels[i] = joint_reverse[i].as<bool>();
    }
    config.max_wheel_veloicity = omnibot_platform_config["wheel_max_velocity"].as<double>();
    
    op_drv.setConfig(&config);

    

    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        omnibot_platform_config["input_topic_name"].as<std::string>(),
        10,
        cmd_vel_sub_cb_f);

    joint_control_pub = nh.advertise<sensor_msgs::JointState>(
        omnibot_platform_config["output_topic_name"].as<std::string>(),
        10);

    joint_control_msg.header.frame_id =  omnibot_platform_config["output_topic_frame_id"].as<std::string>();
    joint_control_msg.name.resize(omnibot_platform_config["wheels_joints_count"].as<int>());
    joint_control_msg.effort.resize(omnibot_platform_config["wheels_joints_count"].as<int>());
    joint_control_msg.position.resize(omnibot_platform_config["wheels_joints_count"].as<int>());
    joint_control_msg.velocity.resize(omnibot_platform_config["wheels_joints_count"].as<int>());

    wheels_vels.w1 = 0.0;
    wheels_vels.w2 = 0.0;
    wheels_vels.w3 = 0.0;
    wheels_vels.w4 = 0.0;
    
    YAML::Node joint_names = omnibot_platform_config["wheels_joints_names"];
    assert(joint_names.Type() == YAML::NodeType::Sequence);
    assert(joint_names.IsSequence());

    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
        joint_control_msg.name[i] = joint_names[i].as<std::string>();
    }


    joint_control_pub_timer = nh.createTimer(ros::Duration(ros::Rate(omnibot_platform_config["output_topic_hz"].as<double>())),joint_control_pub_f);


    ros::spin();
    ros::shutdown();

    return 0;
}

void LoadConfigFile(const std::string yaml_file)
{
    omnibot_platform_config = YAML::LoadFile(yaml_file);

    if (omnibot_platform_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    config.wheel_base_width = omnibot_platform_config["wheel_base_width"].as<double>();
    config.wheel_base_lenth = omnibot_platform_config["wheel_base_lenth"].as<double>();

    config.wheels_deameter = omnibot_platform_config["wheels_deameter"].as<double>();
    config.wheels_width = omnibot_platform_config["wheels_width"].as<double>();

    config.max_linear_x_velocity = omnibot_platform_config["max_linear_x_velocity"].as<double>();
    config.min_linear_x_velocity = omnibot_platform_config["min_linear_x_velocity"].as<double>();

    config.max_linear_y_velocity = omnibot_platform_config["max_linear_y_velocity"].as<double>();
    config.min_linear_y_velocity = omnibot_platform_config["min_linear_y_velocity"].as<double>();

    config.max_angular_z_velocity = omnibot_platform_config["max_angular_z_velocity"].as<double>();
    config.min_angular_z_velocity = omnibot_platform_config["min_angular_z_velocity"].as<double>();
}

void cmd_vel_sub_cb_f(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    if (omnibot_platform_config["input_x_y_remap"].as<bool>() == true)
    {
        op_drv.setXVelocity(cmd_vel->linear.y);
        op_drv.setYVelocity(cmd_vel->linear.x);
    }
    else
    {
        op_drv.setXVelocity(cmd_vel->linear.x);
        op_drv.setYVelocity(cmd_vel->linear.y);
    }
    
    
    op_drv.setZVelocity(cmd_vel->angular.z);

    op_drv.getWheelsVelocity(&wheels_vels);

    joint_control_msg.velocity[0] = wheels_vels.w1;
    joint_control_msg.velocity[1] = wheels_vels.w2;
    joint_control_msg.velocity[2] = wheels_vels.w3;
    joint_control_msg.velocity[3] = wheels_vels.w4;

    
}

void joint_control_pub_f(const ros::TimerEvent &e)
{
    joint_control_msg.header.stamp = ros::Time::now(); 
    joint_control_pub.publish(joint_control_msg);
}