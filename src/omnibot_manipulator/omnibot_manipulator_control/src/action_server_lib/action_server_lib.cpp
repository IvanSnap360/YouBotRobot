#include "action_server_lib/action_server_lib.h"

RobotTrajectoryFollower::RobotTrajectoryFollower(ros::NodeHandle *_nh, std::string action_server_name, std::string traj_pub_name)
{

    _as = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(action_server_name, true);
    // Register callback functions:
    _as->registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this));
    _as->registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));
    // _as->start();

    _trajectory_publisher = _nh->advertise<trajectory_msgs::JointTrajectory>(traj_pub_name, 10);
}

RobotTrajectoryFollower::~RobotTrajectoryFollower() // Destructor
{
}

void RobotTrajectoryFollower::goalCB()
{
    std::cout << "goalCB!!!" << std::endl;
    if (_state)
    {
        _trajectory_publisher.publish(_as->acceptNewGoal()->trajectory);
    }
}

void RobotTrajectoryFollower::preemptCB()
{
    if (_state)
    {
        _as->setPreempted();
    }
}

void RobotTrajectoryFollower::setState(bool state)
{
    _state = state;
}
