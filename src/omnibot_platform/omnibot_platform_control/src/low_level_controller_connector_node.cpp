#include <ros/ros.h>
#include <ros/console.h>
#include "yaml-cpp/yaml.h"
#include "std_msgs/Float64.h"
#include <vector>
class LLCC_connector
{
private:
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    ros::NodeHandle *_nh;
    std::string _input_topic;
    std::string _output_topic;
    std_msgs::Float64 val;
    bool _reverse = false;
    void cb_f(const std_msgs::Float64::ConstPtr &indata)
    {
        val.data = _reverse ? -indata->data : indata->data;
        publisher.publish(val);
    }

public:
    LLCC_connector()
    {
    }
    void init(ros::NodeHandle *nh, std::string input_topic, std::string output_topic, bool reverse)
    {
        _nh = nh;
        publisher = _nh->advertise<std_msgs::Float64>(output_topic, 10);

        subscriber = _nh->subscribe<std_msgs::Float64>(input_topic, 10, &LLCC_connector::cb_f, this);
        _reverse = reverse;

        ROS_INFO_STREAM("Remaping from  \033[36m " << input_topic << "\033[0m to \033[36m" << output_topic << "\033[0m with " << (_reverse ? "reverse" : "no revese") << "\n");
    }
    ~LLCC_connector()
    {
    }
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "low_level_controller_connector_node");
    ros::NodeHandle nh;
    std::string config_yaml_file_path = nh.param<std::string>("omnibot_platform_llcc_config", " ");

    YAML::Node remap_config = YAML::LoadFile(config_yaml_file_path);

    if (remap_config == NULL)
        ROS_FATAL("CANT READ YAML FILE!!!!!");
    else
        ROS_INFO("SUCCESS TO READ YAML FILE");

    YAML::Node input_topics = remap_config["to"];
    assert(input_topics.Type() == YAML::NodeType::Sequence);
    assert(input_topics.IsSequence());

    YAML::Node output_topics = remap_config["from"];
    assert(output_topics.Type() == YAML::NodeType::Sequence);
    assert(output_topics.IsSequence());

    if (remap_config["to"].size() != remap_config["from"].size())
    {
        ROS_FATAL("Remaping config is wrong. Please check count of topics in 'to' list and 'from' list");
        return -1;
    }
    LLCC_connector connectors[remap_config["from"].size()];
    for (int i = 0; i < remap_config["from"].size(); i++)
    {
        connectors[i].init(&nh,
                            remap_config["from"][i].as<std::string>(),
                            remap_config["to"][i]["topic"].as<std::string>(),
                            remap_config["to"][i]["revese"].as<bool>());
    }

    ros::spin();
    ros::shutdown();
    return 0;
}
