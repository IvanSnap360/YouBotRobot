#ifndef __SENSOR_ODOM_MANAGER_LIB_H__
#define __SENSOR_ODOM_MANAGER_LIB_H__
#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "yaml-cpp/yaml.h"
// #include <cmath.h>

class SENSOR_ODOM_MANAGER_LIB
{
private:
    YAML::Node _config;

    ros::NodeHandle *_nh;
    ros::Publisher _odom_publisher;
    ros::Subscriber _odom_subscriber;


    geometry_msgs::Pose _sensor_position;
    std::string _odom_in_topic;
    std::string _odom_out_topic;
    std::string _odom_out_frame;
    std::string _base_frame;
    uint32_t _calc_rate;

    nav_msgs::Odometry _odom_in_msg;
    nav_msgs::Odometry _odom_out_msg;

    geometry_msgs::TransformStamped _tf_msg;
    tf2_ros::TransformBroadcaster *_tf_pub; 

    void _sensor_odom_cb_f(const nav_msgs::OdometryConstPtr &odom_msg);

    bool _first_frame_received = false;
    tf::Quaternion _qtAdjustedRot;

    bool _ignore_x,_ignore_y,_ignore_z;
        
    double _transl_x,_transl_y,_transl_z;

    ros::Timer _main_timer;
    void _main_f(const ros::TimerEvent &enent);

public:
    SENSOR_ODOM_MANAGER_LIB(ros::NodeHandle *_nh, std::string config_path);
    void run();
    void stop();
    ~SENSOR_ODOM_MANAGER_LIB();
};


#endif // __SENSOR_ODOM_MANAGER_LIB_H__