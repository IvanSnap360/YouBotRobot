#ifndef __OMNIBOT_MANIPULATOR_HW_CONTROLLER_H__
#define __OMNIBOT_MANIPULATOR_HW_CONTROLLER_H__
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "yaml-cpp/yaml.h"
#include <sensor_msgs/JointState.h>
class OMNIBOT_MANIPULATOR_HW_CONTROLLER
{
private:
    YAML::Node _cfg;
    ros::NodeHandle *_nh;
    DynamixelWorkbench *_dxl;

    std::string _port_path;
    uint32_t _baudrate;

    std::string _manipulator_action_server_name;
    std::string _gripper_action_server_name;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *_manipulator_action_server;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *_gripper_action_server;

    void _manipulator_action_server_goal_cb_f();
    void _manipulator_action_server_preempt_cb_f();
    void _gripper_action_server_goal_cb_f();
    void _gripper_action_server_preempt_cb_f();

    ros::Publisher _normalized_manipulator_joint_state_data_publisher;
    ros::Subscriber _raw_manipulator_joint_state_data_subscriber;
    void _raw_manipulator_joint_state_data_subscriber_cb_f(const sensor_msgs::JointState::ConstPtr &in_data);


    void readConfig(std::string path);
    void initPublishers();
    void initSubscribers();
    void initActions();
    void initServices();
public:
    OMNIBOT_MANIPULATOR_HW_CONTROLLER(ros::NodeHandle *nh, std::string config_path);
    ~OMNIBOT_MANIPULATOR_HW_CONTROLLER();
};



#endif // __OMNIBOT_MANIPULATOR_HW_CONTROLLER_H__