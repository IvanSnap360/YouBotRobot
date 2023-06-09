#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class RobotTrajectoryFollower
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    ros::Publisher _trajectory_publisher;
    std::string action_name_;

public:
    RobotTrajectoryFollower(std::string name,std::string traj_pub_name) : as_(nh_, name, false),
                                                action_name_(name)
    {
        // Register callback functions:
        as_.registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));

        as_.start();
        _trajectory_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>(traj_pub_name,10);
    }

    ~RobotTrajectoryFollower(void) // Destructor
    {
    }

    void goalCB()
    {
        // accept the new goal
        // goal_ = as_.acceptNewGoal()->samples;
        std::cout << "goalCB!!!" << std::endl;
        _trajectory_publisher.publish(as_.acceptNewGoal()->trajectory);
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");

    RobotTrajectoryFollower ManipulatorTrajectoryFollower("/maipulator_group_controller/follow_joint_trajectory","/dynamixel_contoller/joint_trajectory");
    RobotTrajectoryFollower GripperTrajectoryFollower("/gripper_group_controller/follow_joint_trajectory","/dynamixel_contoller/joint_trajectory");

    ros::spin();

    return 0;
}