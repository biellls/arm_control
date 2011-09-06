#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "melfa_ros/robot_path.h"

void operator >> (const YAML::Node& node, melfa::ToolPose& pose)
{
    node[0] >> pose.x;
    node[1] >> pose.y;
    node[2] >> pose.z;
    node[3] >> pose.roll;
    node[4] >> pose.pitch;
    node[5] >> pose.yaw;
}

void operator >> (const YAML::Node& node, melfa::JointState& joint_state)
{
    node[0] >> joint_state.j1;
    node[1] >> joint_state.j2;
    node[2] >> joint_state.j3;
    node[3] >> joint_state.j4;
    node[4] >> joint_state.j5;
    node[5] >> joint_state.j6;
}

std::queue<melfa::ToolPose> melfa_ros::readToolPath(const std::string& file_name)
{
    std::ifstream in(file_name.c_str());
    if (!in.is_open())
    {
        ROS_ERROR_STREAM("Cannot open file " << file_name);
        return std::queue<melfa::ToolPose>();
    }
    try
    {
        YAML::Parser parser(in);
        if (!parser)
        {
            ROS_ERROR("Cannot create YAML parser.");
            return std::queue<melfa::ToolPose>();
        }

        YAML::Node doc;
        parser.GetNextDocument(doc);

        std::queue<melfa::ToolPose> tool_path;
        for (size_t i = 0; i < doc["tool_path"].size(); ++i)
        {
            melfa::ToolPose tool_pose;
            doc["tool_path"][i] >> tool_pose;
            tool_path.push(tool_pose);
        }
        return tool_path;
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR_STREAM("YAML exception: " << e.what());
        return std::queue<melfa::ToolPose>();
    }
}

std::queue<melfa::JointState> melfa_ros::readJointPath(const std::string& file_name)
{
    std::ifstream in(file_name.c_str());
    if (!in.is_open())
    {
        ROS_ERROR_STREAM("Cannot open file " << file_name);
        return std::queue<melfa::JointState>();
    }
    try
    {
        YAML::Parser parser(in);
        if (!parser)
        {
            ROS_ERROR("Cannot create YAML parser.");
            return std::queue<melfa::JointState>();
        }

        YAML::Node doc;
        parser.GetNextDocument(doc);

        std::queue<melfa::JointState> joint_path;
        for (size_t i = 0; i < doc["joint_path"].size(); ++i)
        {
            melfa::JointState joint_state;
            doc["joint_state"][i] >> joint_state;
            joint_path.push(joint_state);
        }
        return joint_path;
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR_STREAM("YAML exception: " << e.what());
        return std::queue<melfa::JointState>();
    }
}

