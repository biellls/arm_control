#ifndef ROBOT_PATH_H_
#define ROBOT_PATH_H_

#include <geometry_msgs/Pose.h>

#include "melfa/robot_pose.h"

namespace melfa_ros
{
    std::vector<melfa::RobotPose> readRobotPath(const std::string& file_name);
}

#endif

