#include "action_server_lib/action_server_lib.h"

RobotTrajectoryFollower::RobotTrajectoryFollower(ros::NodeHandle *_nh, std::string action_server_name, std::string traj_pub_name)
{

    _as = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(action_server_name, false);
    // Register callback functions:
    _as->registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this));
    _as->registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));
    _as->start();

    _trajectory_publisher = _nh->advertise<trajectory_msgs::JointTrajectory>(traj_pub_name, 10);

}

RobotTrajectoryFollower::~RobotTrajectoryFollower() // Destructor
{
}

void RobotTrajectoryFollower::goalCB()
{
    ROS_INFO("Recieved new goal!!!");
    // std::cout << (int) _as->acceptNewGoal()->trajectory.points.size() << std::endl;
    _current_goal = *_as->acceptNewGoal();
    control_msgs::FollowJointTrajectoryFeedback feedback;
    control_msgs::FollowJointTrajectoryResult result;
    _trajectory_publisher.publish(_current_goal.trajectory);


    result.error_code = result.SUCCESSFUL;
    result.error_string = "Goal is reached!!!";
    _as->setSucceeded(result);
}

void RobotTrajectoryFollower::preemptCB()
{
    if (_state)
    {
        ROS_INFO("Recieved kill cancel task!!!");
        _as->setPreempted();
    }
}

void RobotTrajectoryFollower::setState(bool state)
{
    _state = state;
}
