#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

namespace melfa
{
    struct ToolPose;
    struct JointState;
}

namespace melfa_ros
{
    /**
    * Converts a pose msg to a tool pose
    */
    void poseMsgToToolPose(const geometry_msgs::Pose& pose_msg, melfa::ToolPose& tool_pose);

    /**
    * Converts a tool pose to a pose msg
    */
    void toolPoseToPoseMsg(const melfa::ToolPose& tool_pose, geometry_msgs::Pose& pose_msg);

    /**
    * Converts a joint state msg to a joint state
    */
    void jointStateMsgToJointState(const sensor_msgs::JointState& joint_state_msg, melfa::JointState& joint_state);

    /**
    * Converts a joint state to a joint state msg
    */
    void jointStateToJointStateMsg(const melfa::JointState& joint_state, sensor_msgs::JointState& joint_state_msg);
}

#endif

