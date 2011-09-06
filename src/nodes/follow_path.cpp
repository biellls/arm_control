#include <fstream>
#include <queue>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <yaml-cpp/yaml.h>

#include "arm_control/MoveToolAction.h"

#include "melfa/tool_pose.h"
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
    std::queue<melfa::ToolPose> way_points;
    for (size_t i = 0; i < robot_path.size(); ++i) way_points.push(robot_path[i]);

    actionlib::SimpleActionClient<arm_control::MoveToolAction> action_client("robot_arm/move_tool_action_server");
    ROS_INFO("Waiting for move tool action server...");
    action_client.waitForServer();
    ROS_INFO("Server found.");

    while (way_points.size() > 0)
    {

        arm_control::MoveToolGoal goal;
        melfa_ros::toolPoseToPoseMsg(way_points.front(), goal.target_pose);
        action_client.sendGoal(goal);
        way_points.pop();
        ROS_INFO("Sent next way point. %zu way points left.", way_points.size());
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
            break;
        }
    }

    return 0;
}

