#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class RobotTrajectoryFollower
{
private:
    ros::NodeHandle *_nh;
    ros::Publisher _trajectory_publisher;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *_as;
    bool _state;
public:
    RobotTrajectoryFollower(ros::NodeHandle *_nh,std::string action_server_name,std::string traj_pub_name);
    ~RobotTrajectoryFollower();
    void setState(bool state);
    void goalCB();
    void preemptCB();
};
