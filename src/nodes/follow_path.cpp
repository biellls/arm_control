#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <yaml-cpp/yaml.h>

#include "arm_control/robot_pose.h"

namespace ac = arm_control;

void operator >> (const YAML::Node& node, ac::RobotPose& pose)
{
    node[0] >> pose.x;
    node[1] >> pose.y;
    node[2] >> pose.z;
    node[3] >> pose.roll;
    node[4] >> pose.pitch;
    node[5] >> pose.yaw;
    pose.roll = pose.roll / 180.0 * M_PI;
    pose.pitch = pose.pitch / 180.0 * M_PI;
    pose.yaw = pose.yaw / 180.0 * M_PI;
}

std::ostream& operator<< (std::ostream& ostr, ac::RobotPose& pose)
{
    ostr << "(" << pose.x << ", " << pose.y << ", " << pose.z << ") "
         << "(" << pose.roll << ", " << pose.pitch << ", " << pose.yaw << ")";
    return ostr;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");
    if (argc < 2)
    {
        std::cerr << "USAGE: " << argv[0] << " <path config file>" << std::endl;
        exit(1);
    }

    std::ifstream in(argv[1]);
    if (!in.is_open())
    {
        std::cerr << "Cannot open file " << argv[1] << std::endl;
        exit(2);
    }

    try
    {
        YAML::Parser parser(in);
        if (!parser)
        {
            std::cerr << "Cannot create YAML parser." << std::endl;
            exit(3);
        }

        YAML::Node doc;
        parser.GetNextDocument(doc);

        double acceleration;
        double maximum_velocity;
        doc["acceleration"] >> acceleration;
        doc["maximum_velocity"] >> maximum_velocity;
        std::cout << "acceleration = " << acceleration << std::endl;
        std::cout << "maximum_velocity = " << maximum_velocity << std::endl;
        ac::RobotPose tool_pose;
        doc["tool_pose"] >> tool_pose;
        std::cout << "tool_pose = " << tool_pose << std::endl;
        int num_poses = doc["path"].size();
        std::vector<ac::RobotPose> robot_poses(num_poses);
        std::cout << "Reading " << robot_poses.size() << " way points." << std::endl;
        for (size_t i = 0; i < doc["path"].size(); ++i)
        {
            doc["path"][i] >> robot_poses[i];
            std::cout << i << "\t" << robot_poses[i] << std::endl;
        }
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML exception: " << e.what() << std::endl;
        exit(4);
    }

    /*
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ac::MoveArmAction> ac("follow_path_action_client", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    ac::MoveArmGoal goal;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    */
    return 0;
}

