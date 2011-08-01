#ifndef ROBOT_PATH_H_
#define ROBOT_PATH_H_

#include <geometry_msgs/Pose.h>

#include "melfa/robot_pose.h"

namespace melfa_ros
{
    struct RobotPath
    {
        double acceleration;
        double maximum_velocity;
        melfa::RobotPose tool_pose;
        std::vector<melfa::RobotPose> path_poses;
    };

    RobotPath readRobotPath(const std::string& file_name);
}

#endif

