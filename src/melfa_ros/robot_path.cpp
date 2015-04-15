#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "melfa_ros/robot_path.h"

void operator >> (const YAML::Node& node, melfa::ToolPose& pose)
{
    pose.x     = node[0].as<double>();
    pose.y     = node[1].as<double>();
    pose.z     = node[2].as<double>();
    pose.roll  = node[3].as<double>();
    pose.pitch = node[4].as<double>();
    pose.yaw   = node[5].as<double>();
}

void operator >> (const YAML::Node& node, melfa::JointState& joint_state)
{
    joint_state.j1 = node[0].as<double>();
    joint_state.j2 = node[1].as<double>();
    joint_state.j3 = node[2].as<double>();
    joint_state.j4 = node[3].as<double>();
    joint_state.j5 = node[4].as<double>();
    joint_state.j6 = node[5].as<double>();
}

std::queue<melfa::ToolPose> melfa_ros::readToolPath(const std::string& file_name)
{
    try
    {
        YAML::Node doc = YAML::LoadFile(file_name);
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
    try
    {
        YAML::Node doc = YAML::LoadFile(file_name);
        std::queue<melfa::JointState> joint_path;
        for (size_t i = 0; i < doc["joint_path"].size(); ++i)
        {
            melfa::JointState joint_state;
            doc["joint_path"][i] >> joint_state;
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

