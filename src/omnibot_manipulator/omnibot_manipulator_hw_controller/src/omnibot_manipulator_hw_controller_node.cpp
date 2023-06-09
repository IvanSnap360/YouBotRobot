#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "sensor_msgs/JointState.h"
#include <control_msgs/FollowJointTrajectoryAction.h>


ros::Subscriber dynamixel_controller_joint_state_subscriber;
ros::Publisher manipulator_joint_state_publisher;


float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void dynamixel_controller_joint_state_subscriber_cb_f(const sensor_msgs::JointState::ConstPtr &in_data)
{
    static sensor_msgs::JointState out_data;
    out_data = *in_data;
    out_data.position[0] = map(in_data->position[0], -0.4, 0.8, 0.0, 0.025);
    manipulator_joint_state_publisher.publish(out_data);
    // std::cout << "get joint_state" << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "omnibotmanipulator_hw_controller_node");
    ros::NodeHandle nh;

    dynamixel_controller_joint_state_subscriber =
        nh.subscribe<sensor_msgs::JointState>("/dynamixel_contoller/joint_states", 10,
                                              &dynamixel_controller_joint_state_subscriber_cb_f);

    manipulator_joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/omnibot_robot/manipulator_joint_states_correcter", 10);
    ros::spin();
    ros::shutdown();
    return 0;
}
