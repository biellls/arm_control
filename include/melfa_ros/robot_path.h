#ifndef ROBOT_PATH_H_
#define ROBOT_PATH_H_

#include <queue>

#include "melfa/tool_pose.h"
#include "melfa/joint_state.h"

namespace melfa_ros
{
    std::queue<melfa::ToolPose> readToolPath(const std::string& file_name);
    std::queue<melfa::JointState> readJointPath(const std::string& file_name);
}

#endif

