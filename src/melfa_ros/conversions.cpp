#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "melfa/robot_pose.h"

#include "melfa_ros/conversions.h"

void melfa_ros::poseMsgToRobot(const geometry_msgs::Pose& pose_msg, melfa::RobotPose& robot_pose)
{
    robot_pose.x = pose_msg.position.x;
    robot_pose.y = pose_msg.position.y;
    robot_pose.z = pose_msg.position.z;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose_msg.orientation, quat);
    btMatrix3x3(quat).getRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
}

void melfa_ros::poseRobotToMsg(const melfa::RobotPose& robot_pose, geometry_msgs::Pose& pose_msg)
{
    tf::Quaternion quat;
    quat.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
    tf::Pose tf_pose(quat, tf::Vector3(robot_pose.x, robot_pose.y, robot_pose.z));
    tf::poseTFToMsg(tf_pose, pose_msg);
}

