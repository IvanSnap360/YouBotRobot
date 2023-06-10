#include <ros/ros.h>

#include "action_server_lib/action_server_lib.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_server_node");
    ros::NodeHandle nh("~");
    std::string manipulator_action_name;
    std::string gripper_action_name;
    std::string output_topic_name;
    if (ros::param::has("~manipulator_action_name"))
    {
        ros::param::get("~manipulator_action_name", manipulator_action_name);
    }
    else
    {
        ROS_ERROR("No set param 'manipulator_action_name'!!!");
    }
    if (ros::param::has("~gripper_action_name"))
    {
        ros::param::get("~gripper_action_name", gripper_action_name);
    }
    else
    {
        ROS_ERROR("No set param 'gripper_action_name'!!!");
    }
    if (ros::param::has("~output_topic_name"))
    {
        ros::param::get("~output_topic_name", output_topic_name);
    }
    else
    {
        ROS_ERROR("No set param 'output_topic_name'!!!");
    }

    auto manipulator_server = RobotTrajectoryFollower(&nh, manipulator_action_name, output_topic_name);
    auto gripper_server = RobotTrajectoryFollower(&nh, gripper_action_name, output_topic_name);
    manipulator_server.setState(true);
    gripper_server.setState(true);
    ros::spin();
    ros::shutdown();
    return 0;
}
