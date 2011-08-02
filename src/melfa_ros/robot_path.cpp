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


std::vector<melfa::RobotPose> melfa_ros::readRobotPath(const std::string& file_name)
{
    std::ifstream in(file_name.c_str());
    if (!in.is_open())
    {
        ROS_ERROR_STREAM("Cannot open file " << file_name);
        return std::vector<melfa::RobotPose>();
    }
    try
    {
        YAML::Parser parser(in);
        if (!parser)
        {
            ROS_ERROR("Cannot create YAML parser.");
            return std::vector<melfa::RobotPose>();
        }

        YAML::Node doc;
        parser.GetNextDocument(doc);

        std::vector<melfa::RobotPose> robot_path;
        robot_path.resize(doc["path"].size());
        for (size_t i = 0; i < doc["path"].size(); ++i)
        {
            doc["path"][i] >> robot_path[i];
        }
        return robot_path;
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR_STREAM("YAML exception: " << e.what());
        return std::vector<melfa::RobotPose>();
    }
}

