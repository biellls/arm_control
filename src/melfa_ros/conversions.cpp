#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "melfa/tool_pose.h"

#include "melfa_ros/conversions.h"

void melfa_ros::poseMsgToTool(const geometry_msgs::Pose& pose_msg, melfa::ToolPose& tool_pose)
{
    tool_pose.x = pose_msg.position.x;
    tool_pose.y = pose_msg.position.y;
    tool_pose.z = pose_msg.position.z;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose_msg.orientation, quat);
    btMatrix3x3(quat).getRPY(tool_pose.roll, tool_pose.pitch, tool_pose.yaw);
}

void melfa_ros::poseToolToMsg(const melfa::ToolPose& tool_pose, geometry_msgs::Pose& pose_msg)
{
    tf::Quaternion quat;
    quat.setRPY(tool_pose.roll, tool_pose.pitch, tool_pose.yaw);
    tf::Pose tf_pose(quat, tf::Vector3(tool_pose.x, tool_pose.y, tool_pose.z));
    tf::poseTFToMsg(tf_pose, pose_msg);
}

