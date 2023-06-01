#ifndef __OMNIBOT_PLATFORM_DRV_LIB_H__
#define __OMNIBOT_PLATFORM_DRV_LIB_H__
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "yaml-cpp/yaml.h"

class OMNIBOT_PLATFORM_DRV_LIB
{
private:
    typedef struct 
    {
        double w1,w2,w3,w4;
    }wheels_vels_output;
    


    ros::Subscriber _cmd_vel_sub;
    ros::Publisher  _joint_control_pub;
    YAML::Node _config;

    
    geometry_msgs::Twist _cmd_vel_input_msg;
    wheels_vels_output _wheels_target_velocities = {0.0, 0.0, 0.0, 0.0};

    double  _linear_x_velocity_min_limit, _linear_x_velocity_max_limit,
            _linear_y_velocity_min_limit, _linear_y_velocity_max_limit,
            _angular_z_velocity_min_limit,_angular_z_velocity_max_limit;

    int _wheels_joints_count;
    double _wheels_deameter, _wheels_width, _wheels_max_velocity;

    double _wheel_base_width, _wheel_base_lenth;

    std::vector<std::string> _joints_names;
    std::vector<bool>        _joints_reverse;
    sensor_msgs::JointState  _joints_control_msg;

    std::string _output_topic_frame_id, _output_topic_name, _input_topic_name;
    double _output_topic_rate;

    bool inverse_linear_x,inverse_linear_y,inverse_angular_z;

    void _cmd_vel_sub_cb_f(const geometry_msgs::Twist::ConstPtr &twist);
    bool _check_is_node_IsSequence(YAML::Node node, std::string name);


public:
    OMNIBOT_PLATFORM_DRV_LIB(ros::NodeHandle *nh, std::string config_path);
    double getWorkRate();
    void process(const ros::TimerEvent&);
    ~OMNIBOT_PLATFORM_DRV_LIB();
};



#endif // __OMNIBOT_PLATFORM_DRV_LIB_H__