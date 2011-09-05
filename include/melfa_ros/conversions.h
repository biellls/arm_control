#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <geometry_msgs/Pose.h>

namespace melfa
{
    struct ToolPose;
}

namespace melfa_ros
{
    /**
    * Converts a pose msg to a robot pose
    */
    void poseMsgToTool(const geometry_msgs::Pose& pose_msg, melfa::ToolPose& tool_pose);

    /**
    * Converts a robot pose to a pose msg
    */
    void poseToolToMsg(const melfa::ToolPose& robot_pose, geometry_msgs::Pose& pose_msg);
}

#endif

