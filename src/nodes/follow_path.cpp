#include <fstream>
#include <queue>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <yaml-cpp/yaml.h>

#include "arm_control/MoveToolAction.h"
#include "arm_control/MoveJointsAction.h"

#include "melfa/tool_pose.h"
#include "melfa_ros/robot_path.h"
#include "melfa_ros/conversions.h"


int main (int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");
    if (argc < 2)
    {
        std::cerr << "Reads joint states and tool poses from a YAML formatted\n"
                  << "file and sends corresponding move joints and move tool actions.\n"
                  << "First all move joint commands are sent, then the move tool commands are sent." << std::endl;
        std::cerr << "USAGE: " << argv[0] << " <path config file>" << std::endl;
        exit(1);
    }

    std::queue<melfa::JointState> joint_way_points = melfa_ros::readJointPath(argv[1]);
    std::queue<melfa::ToolPose> tool_way_points = melfa_ros::readToolPath(argv[1]);

    ROS_INFO("Read %zu joint states and %zu tool poses.", joint_way_points.size(),
            tool_way_points.size());

    actionlib::SimpleActionClient<arm_control::MoveJointsAction> move_joints_action_client("robot_arm/move_joints_action_server");
    ROS_INFO("Waiting for move joints action server...");
    move_joints_action_client.waitForServer();
    ROS_INFO("Server found.");

    while (joint_way_points.size() > 0)
    {
        arm_control::MoveJointsGoal goal;
        melfa_ros::jointStateToJointStateMsg(joint_way_points.front(), goal.target_joint_state);
        move_joints_action_client.sendGoal(goal);
        joint_way_points.pop();
        ROS_INFO("Sent next joint way point. %zu way points left.", joint_way_points.size());
        //wait for the action to return
        bool finished_before_timeout = move_joints_action_client.waitForResult(ros::Duration(300.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = move_joints_action_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
            break;
        }
    }

    actionlib::SimpleActionClient<arm_control::MoveToolAction> move_tool_action_client("robot_arm/move_tool_action_server");
    ROS_INFO("Waiting for move tool action server...");
    move_tool_action_client.waitForServer();
    ROS_INFO("Server found.");

    while (tool_way_points.size() > 0)
    {
        arm_control::MoveToolGoal goal;
        melfa_ros::toolPoseToPoseMsg(tool_way_points.front(), goal.target_pose);
        move_tool_action_client.sendGoal(goal);
        tool_way_points.pop();
        ROS_INFO("Sent next tool way point. %zu way points left.", tool_way_points.size());
        //wait for the action to return
        bool finished_before_timeout = move_tool_action_client.waitForResult(ros::Duration(300.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = move_tool_action_client.getState();
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

