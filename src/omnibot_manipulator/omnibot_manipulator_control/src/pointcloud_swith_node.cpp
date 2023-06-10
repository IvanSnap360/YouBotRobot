#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_srvs/SetBool.h"
ros::Subscriber input_topic_sub;
ros::Publisher  output_topic_pub;
ros::ServiceServer control_server;
bool publish_enable = false;
bool control_service_cb_f(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    publish_enable = (bool)req.data;
    res.message = "Pointcloud publisher is " + req.data ? "ENABLED" : "DISABLED";
    res.success = true;
    return true;
}

void input_topic_sub_cb_f(const sensor_msgs::PointCloud2::ConstPtr &in_data)
{
    if (publish_enable)
    {
        output_topic_pub.publish(in_data);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointcloud_swith_node");
    ros::NodeHandle nh("~");

    std::string input_topic;
    std::string output_topic;
    std::string control_service_name;

    if (ros::param::has("~input_topic"))
    {
        ros::param::get("~input_topic", input_topic);
    }
    else
    {
        ROS_ERROR("Param 'input_topic' is not set ");
    }
    if (ros::param::has("~output_topic"))
    {
        ros::param::get("~output_topic", output_topic);
    }
    else
    {
        ROS_ERROR("Param 'output_topic' is not set ");
    }
    if (ros::param::has("~control_service_name"))
    {
        ros::param::get("~control_service_name", control_service_name);
    }
    else
    {
        ROS_ERROR("Param 'control_service_name' is not set ");
    }

    input_topic_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, input_topic_sub_cb_f);
    output_topic_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    control_server  = nh.advertiseService(control_service_name,control_service_cb_f);
    ros::spin();
    ros::shutdown();
    return 0;
}
