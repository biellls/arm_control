#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <geometry_msgs/Pose.h>

namespace melfa
{
    struct RobotPose;
}

namespace melfa_ros
{
    /**
    * Converts a pose msg to a robot pose
    */
    void poseMsgToRobot(const geometry_msgs::Pose& pose_msg, melfa::RobotPose& robot_pose);

    /**
    * Converts a robot pose to a pose msg
    */
    void poseRobotToMsg(const melfa::RobotPose& robot_pose, geometry_msgs::Pose& pose_msg);
}

#endif

