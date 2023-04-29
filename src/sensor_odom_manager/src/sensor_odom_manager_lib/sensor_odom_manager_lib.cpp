#include "sensor_odom_manager_lib/sensor_odom_manager_lib.h"

#define degreesToRadians(degrees) (degrees * M_PI / 180)

SENSOR_ODOM_MANAGER_LIB::SENSOR_ODOM_MANAGER_LIB(ros::NodeHandle *nh, std::string config_path)
{
    _nh = nh;

    _config = YAML::LoadFile(config_path);

    if (_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    _sensor_position.position.x = _config["sensor_offset"]["position"]["x"].as<double>();
    _sensor_position.position.y = _config["sensor_offset"]["position"]["y"].as<double>();
    _sensor_position.position.z = _config["sensor_offset"]["position"]["z"].as<double>();
    _sensor_position.orientation.x = _config["sensor_offset"]["rotation"]["x"].as<double>();
    _sensor_position.orientation.y = _config["sensor_offset"]["rotation"]["y"].as<double>();
    _sensor_position.orientation.z = _config["sensor_offset"]["rotation"]["z"].as<double>();

    _odom_in_topic = _config["odom_input_topic"].as<std::string>();
    _odom_out_topic = _config["odom_output_topic"].as<std::string>();
    _odom_out_frame = _config["odom_output_frame"].as<std::string>();
    _calc_rate = _config["calculation_rate_hz"].as<uint32_t>();
    _base_frame = _config["base_frame"].as<std::string>();

    _ignore_x = _config["ignore_x"].as<bool>();
    _ignore_y = _config["ignore_y"].as<bool>();
    _ignore_z = _config["ignore_z"].as<bool>();

    _odom_subscriber = nh->subscribe<nav_msgs::Odometry>(_odom_in_topic, 10, &SENSOR_ODOM_MANAGER_LIB::_sensor_odom_cb_f, this);
    _odom_publisher = nh->advertise<nav_msgs::Odometry>(_odom_out_topic, 10);
    _tf_pub = new tf2_ros::TransformBroadcaster();

    _main_timer = nh->createTimer(ros::Duration(ros::Rate(_calc_rate)), &SENSOR_ODOM_MANAGER_LIB::_main_f, this, false);

    /**
     * @brief odom output message setup
     *
     */
    _odom_out_msg.header.frame_id = _odom_out_frame;
    _odom_out_msg.child_frame_id = _base_frame;
    _odom_out_msg.pose.pose.position.x = 0.0;
    _odom_out_msg.pose.pose.position.y = 0.0;
    _odom_out_msg.pose.pose.position.z = 0.0;
    _odom_out_msg.pose.pose.orientation.x = 0.0;
    _odom_out_msg.pose.pose.orientation.y = 0.0;
    _odom_out_msg.pose.pose.orientation.z = 0.0;
    _odom_out_msg.pose.pose.orientation.w = 1.0;

    _odom_out_msg.twist.twist.linear.x = 0.0;
    _odom_out_msg.twist.twist.linear.y = 0.0;
    _odom_out_msg.twist.twist.linear.z = 0.0;
    _odom_out_msg.twist.twist.angular.x = 0.0;
    _odom_out_msg.twist.twist.angular.y = 0.0;
    _odom_out_msg.twist.twist.angular.z = 0.0;

    /**
     * @brief odom input message setup
     *
     */


    /**
     * @brief  transform message setup
     *
     */

    _tf_msg.header.frame_id = _odom_out_frame;
    _tf_msg.child_frame_id = _base_frame;

    _tf_msg.transform.translation.x = 0.0;
    _tf_msg.transform.translation.y = 0.0;
    _tf_msg.transform.translation.z = 0.0;

    _tf_msg.transform.rotation.x = 0.0;
    _tf_msg.transform.rotation.y = 0.0;
    _tf_msg.transform.rotation.z = 0.0;
    _tf_msg.transform.rotation.w = 0.0;

    //*********************
    _first_frame_received = false;
    //*********************
    _qtAdjustedRot.setX(0.0);
    _qtAdjustedRot.setY(0.0);
    _qtAdjustedRot.setZ(0.0);
    _qtAdjustedRot.setW(1.0);
}

void SENSOR_ODOM_MANAGER_LIB::_sensor_odom_cb_f(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    _first_frame_received = true;

    _odom_in_msg.header.frame_id = odom_msg->header.frame_id;
    _odom_in_msg.child_frame_id = odom_msg->child_frame_id;

    _odom_in_msg.pose.pose.position.x = odom_msg->pose.pose.position.x;
    _odom_in_msg.pose.pose.position.y = odom_msg->pose.pose.position.y;
    _odom_in_msg.pose.pose.position.z = odom_msg->pose.pose.position.z;

    _odom_in_msg.pose.pose.orientation.x = odom_msg->pose.pose.orientation.x;
    _odom_in_msg.pose.pose.orientation.y = odom_msg->pose.pose.orientation.y;
    _odom_in_msg.pose.pose.orientation.z = odom_msg->pose.pose.orientation.z;
    _odom_in_msg.pose.pose.orientation.w = odom_msg->pose.pose.orientation.w;

    _odom_in_msg.twist.twist.linear.x = odom_msg->twist.twist.linear.x;
    _odom_in_msg.twist.twist.linear.y = odom_msg->twist.twist.linear.y;
    _odom_in_msg.twist.twist.linear.z = odom_msg->twist.twist.linear.z;

    _odom_in_msg.twist.twist.angular.x = odom_msg->twist.twist.angular.x;
    _odom_in_msg.twist.twist.angular.y = odom_msg->twist.twist.angular.y;
    _odom_in_msg.twist.twist.angular.z = odom_msg->twist.twist.angular.z;
}

void SENSOR_ODOM_MANAGER_LIB::run()
{
    _main_timer.start();
}

void SENSOR_ODOM_MANAGER_LIB::_main_f(const ros::TimerEvent &enent)
{
    if (_first_frame_received == true)
    {
        _odom_out_msg.header.stamp = ros::Time::now();
        _odom_out_msg.header.seq = ros::Time::now().toSec();

        _tf_msg.header.stamp = ros::Time::now();
        _tf_msg.header.seq = ros::Time::now().toSec();
        

        _odom_out_msg.header.frame_id = _odom_out_frame;
        _odom_out_msg.child_frame_id = _base_frame;

        _odom_out_msg.twist.twist.linear.x = _odom_in_msg.twist.twist.linear.x;
        _odom_out_msg.twist.twist.linear.y = _odom_in_msg.twist.twist.linear.y;
        _odom_out_msg.twist.twist.angular.z = _odom_in_msg.twist.twist.angular.z;

        _qtAdjustedRot.setX(_odom_in_msg.pose.pose.orientation.x);
        _qtAdjustedRot.setY(_odom_in_msg.pose.pose.orientation.y);
        _qtAdjustedRot.setZ(_odom_in_msg.pose.pose.orientation.z);
        _qtAdjustedRot.setW(_odom_in_msg.pose.pose.orientation.w);
        _qtAdjustedRot.normalize();

        if (_sensor_position.orientation.x != 0.0 || _sensor_position.orientation.y != 0.0 || _sensor_position.orientation.z != 0.0)
        {
            auto qtSensorAdjustmentTool = tf::createQuaternionFromRPY(degreesToRadians(-_sensor_position.orientation.x),
                                                                      degreesToRadians(-_sensor_position.orientation.y),
                                                                      degreesToRadians(-_sensor_position.orientation.z));
            qtSensorAdjustmentTool.normalize();
            auto qtRot = qtSensorAdjustmentTool * _qtAdjustedRot;
            qtRot.normalize();

            _qtAdjustedRot = qtRot * qtSensorAdjustmentTool.inverse();
            _qtAdjustedRot.normalize();
        }

        tf::Matrix3x3 m(_qtAdjustedRot);
        tfScalar yaw, pitch, roll;

        m.getEulerYPR(yaw, pitch, roll);

        if (_ignore_x == false)
        {
            _transl_x = _odom_in_msg.pose.pose.position.x - ((_sensor_position.position.x * cos(yaw)) - (-_sensor_position.position.y * sin(yaw)));
        }
        else
        {
            _transl_x = 0.0;
        }

        if (_ignore_y == false)
        {
            _transl_y = _odom_in_msg.pose.pose.position.y - ((_sensor_position.position.x * sin(yaw)) + (-_sensor_position.position.y * cos(yaw)));
        }
        else
        {
            _transl_y = 0.0;
        }

        if (_ignore_z == false)
        {
            _transl_z = _odom_in_msg.pose.pose.position.z - _sensor_position.position.x;
        }
        else
        {
            _transl_z = 0.0;
        }

        _odom_out_msg.pose.pose.position.x = _transl_x;
        _odom_out_msg.pose.pose.position.y = _transl_y;
        _odom_out_msg.pose.pose.position.z = _transl_z;
        _odom_out_msg.pose.pose.orientation.x = _qtAdjustedRot.getX();
        _odom_out_msg.pose.pose.orientation.y = _qtAdjustedRot.getY();
        _odom_out_msg.pose.pose.orientation.z = _qtAdjustedRot.getZ();
        _odom_out_msg.pose.pose.orientation.w = _qtAdjustedRot.getW();


        _tf_msg.transform.translation.x = _transl_x;
        _tf_msg.transform.translation.y = _transl_y;
        _tf_msg.transform.translation.z = _transl_z;
        _tf_msg.transform.rotation.x = _qtAdjustedRot.getX();
        _tf_msg.transform.rotation.y = _qtAdjustedRot.getY();
        _tf_msg.transform.rotation.z = _qtAdjustedRot.getZ();
        _tf_msg.transform.rotation.w = _qtAdjustedRot.getW();


        _tf_pub->sendTransform(_tf_msg);
        _odom_publisher.publish(_odom_out_msg);
    }
}

void SENSOR_ODOM_MANAGER_LIB::stop()
{
    _main_timer.stop();
}

SENSOR_ODOM_MANAGER_LIB::~SENSOR_ODOM_MANAGER_LIB()
{
}
