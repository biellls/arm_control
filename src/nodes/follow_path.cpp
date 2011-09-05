#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <yaml-cpp/yaml.h>

#include "arm_control/MoveArmAction.h"

#include "melfa/robot_pose.h"
#include "melfa_ros/robot_path.h"
#include "melfa_ros/conversions.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");
    if (argc < 2)
    {
        std::cerr << "USAGE: " << argv[0] << " <path config file>" << std::endl;
        exit(1);
    }

    std::vector<melfa::ToolPose> robot_path = melfa_ros::readRobotPath(argv[1]);

    // insert data from RobotPath struct
    arm_control::MoveArmGoal goal;
    goal.path.poses.resize(robot_path.size());
    for (size_t i = 0; i < robot_path.size(); ++i)
    {
        melfa_ros::poseToolToMsg(robot_path[i], goal.path.poses[i].pose);
    }

    actionlib::SimpleActionClient<arm_control::MoveArmAction> action_client("robot_arm/arm_control_action_server");
    ROS_INFO("Waiting for move arm action server...");
    action_client.waitForServer();

    ROS_INFO("Server started, sending goal.");
    action_client.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(300.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = action_client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}

