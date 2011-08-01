#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "melfa_ros/robot_path.h"

void operator >> (const YAML::Node& node, melfa::RobotPose& pose)
{
    node[0] >> pose.x;
    node[1] >> pose.y;
    node[2] >> pose.z;
    node[3] >> pose.roll;
    node[4] >> pose.pitch;
    node[5] >> pose.yaw;
}


melfa_ros::RobotPath melfa_ros::readRobotPath(const std::string& file_name)
{
    std::ifstream in(file_name.c_str());
    if (!in.is_open())
    {
        ROS_ERROR_STREAM("Cannot open file " << file_name);
        return melfa_ros::RobotPath();
    }
    try
    {
        YAML::Parser parser(in);
        if (!parser)
        {
            ROS_ERROR("Cannot create YAML parser.");
            return melfa_ros::RobotPath();
        }

        YAML::Node doc;
        parser.GetNextDocument(doc);

        melfa_ros::RobotPath robot_path;
        doc["acceleration"] >> robot_path.acceleration;
        doc["maximum_velocity"] >> robot_path.maximum_velocity;
        doc["tool_pose"] >> robot_path.tool_pose;
        robot_path.path_poses.resize(doc["path"].size());
        for (size_t i = 0; i < doc["path"].size(); ++i)
        {
            doc["path"][i] >> robot_path.path_poses[i];
        }
        return robot_path;
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR_STREAM("YAML exception: " << e.what());
        return melfa_ros::RobotPath();
    }
}

