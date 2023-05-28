#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

enum
{
    LEFT_FRONT,
    RIGHT_FRONT,
    LEFT_BACK,
    RIGHT_BACK,

    WHEELS_COUNT
} wheels_enum;

YAML::Node remap_config;

std_msgs::Float64 controller_msgs[WHEELS_COUNT];

ros::Publisher left_front_omniwheel_pub;
ros::Publisher right_front_omniwheel_pub;
ros::Publisher left_back_omniwheel_pub;
ros::Publisher right_back_omniwheel_pub;
ros::Subscriber joints_control_sub;

void joints_control_sub_cb_f(const sensor_msgs::JointState::ConstPtr &jc);

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "joint_control_to_contoller_translate_node");
    ros::NodeHandle nh;

    std::string config_yaml_file_path = nh.param<std::string>("omnibot_platform_retanslation_config", " ");

    remap_config = YAML::LoadFile(config_yaml_file_path);

    if (remap_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    
    YAML::Node output_topics = remap_config["to"];
    assert(output_topics.Type() == YAML::NodeType::Sequence);
    assert(output_topics.IsSequence());    

    left_front_omniwheel_pub  = nh.advertise<std_msgs::Float64>(output_topics[(std::size_t)LEFT_FRONT].as<std::string>(),10);        
    right_front_omniwheel_pub = nh.advertise<std_msgs::Float64>(output_topics[(std::size_t)RIGHT_FRONT].as<std::string>(),10);
    left_back_omniwheel_pub   = nh.advertise<std_msgs::Float64>(output_topics[(std::size_t)LEFT_BACK].as<std::string>(),10);
    right_back_omniwheel_pub  = nh.advertise<std_msgs::Float64>(output_topics[(std::size_t)RIGHT_BACK].as<std::string>(),10);

    joints_control_sub        = nh.subscribe<sensor_msgs::JointState>(remap_config["from"].as<std::string>(),
                10,
                joints_control_sub_cb_f);


    ros::spin();
    ros::shutdown();
    return 0;
    
}

void joints_control_sub_cb_f(const sensor_msgs::JointState::ConstPtr &state)
{   
    if (state->velocity.size() !=  WHEELS_COUNT)
    {
        ROS_WARN("Invalid input msg");
        return;
    }

    for (int i = 0; i < (std::size_t)state->velocity.size(); i++)
    {
        if(i == LEFT_FRONT || i == RIGHT_FRONT)
            controller_msgs[i].data = state->velocity[i];
        else if (i == LEFT_BACK || i == RIGHT_BACK)
            controller_msgs[i].data = -state->velocity[i];

    }

    left_front_omniwheel_pub.publish(controller_msgs[LEFT_FRONT]);
    right_front_omniwheel_pub.publish(controller_msgs[RIGHT_FRONT]);
    left_back_omniwheel_pub.publish(controller_msgs[LEFT_BACK]);
    right_back_omniwheel_pub.publish(controller_msgs[RIGHT_BACK]);
}
