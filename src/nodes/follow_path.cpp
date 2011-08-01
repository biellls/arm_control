#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <yaml-cpp/yaml.h>

#include "arm_control/MoveArmAction.h"

#include "melfa/robot_pose.h"
#include "melfa_ros/robot_path.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");
    if (argc < 2)
    {
        std::cerr << "USAGE: " << argv[0] << " <path config file>" << std::endl;
        exit(1);
    }

    melfa_ros::RobotPath robot_path = melfa_ros::readRobotPath(argv[1]);

    /*
    // insert data from RobotPath struct
    melfa_ros::poseRobotToMsg(robot_path.tool_pose, goal.tool_pose);
    goal.path.resize(robot_path.path_poses.size());
    for (size_t i = 0; i < robot_path.path_poses.size(); ++i)
    {
        melfa_ros::poseRobotToMsg(robot_path.path_poses[i], goal.path[i]);
    }

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }
    */

    return 0;
}

