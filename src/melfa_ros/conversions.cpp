#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "melfa/tool_pose.h"
#include "melfa/joint_state.h"

#include "melfa_ros/conversions.h"

void melfa_ros::poseMsgToToolPose(const geometry_msgs::Pose& pose_msg, melfa::ToolPose& tool_pose)
{
    tool_pose.x = pose_msg.position.x;
    tool_pose.y = pose_msg.position.y;
    tool_pose.z = pose_msg.position.z;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose_msg.orientation, quat);
    btMatrix3x3(quat).getRPY(tool_pose.roll, tool_pose.pitch, tool_pose.yaw);
}

void melfa_ros::toolPoseToPoseMsg(const melfa::ToolPose& tool_pose, geometry_msgs::Pose& pose_msg)
{
    tf::Quaternion quat;
    quat.setRPY(tool_pose.roll, tool_pose.pitch, tool_pose.yaw);
    tf::Pose tf_pose(quat, tf::Vector3(tool_pose.x, tool_pose.y, tool_pose.z));
    tf::poseTFToMsg(tf_pose, pose_msg);
}

void melfa_ros::jointStateMsgToJointState(const sensor_msgs::JointState& joint_state_msg, melfa::JointState& joint_state)
{
    joint_state.j1 = joint_state_msg.position[0];
    joint_state.j2 = joint_state_msg.position[1];
    joint_state.j3 = joint_state_msg.position[2];
    joint_state.j4 = joint_state_msg.position[3];
    joint_state.j5 = joint_state_msg.position[4];
    joint_state.j6 = joint_state_msg.position[5];
}

void melfa_ros::jointStateToJointStateMsg(const melfa::JointState& joint_state, sensor_msgs::JointState& joint_state_msg)
{
    joint_state_msg.name.resize(6);
    joint_state_msg.position.resize(6);
    joint_state_msg.name[0] = "J1";
    joint_state_msg.name[1] = "J2";
    joint_state_msg.name[2] = "J3";
    joint_state_msg.name[3] = "J4";
    joint_state_msg.name[4] = "J5";
    joint_state_msg.name[5] = "J6";
    joint_state_msg.position[0] = joint_state.j1;
    joint_state_msg.position[1] = joint_state.j2;
    joint_state_msg.position[2] = joint_state.j3;
    joint_state_msg.position[3] = joint_state.j4;
    joint_state_msg.position[4] = joint_state.j5;
    joint_state_msg.position[5] = joint_state.j6;
}

