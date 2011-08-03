#include <iostream>

#include "melfa/robot_pose.h"

std::ostream& operator<< (std::ostream& ostr, const melfa::RobotPose& robot_pose)
{
    ostr << "(" 
         << robot_pose.x << ", " 
         << robot_pose.y << ", "
         << robot_pose.z 
         << ") ("
         << robot_pose.roll << ", "
         << robot_pose.pitch << ", "
         << robot_pose.yaw
         << ") ("
         << robot_pose.j1 << ", "
         << robot_pose.j2 << ", "
         << robot_pose.j3 << ", "
         << robot_pose.j4 << ", "
         << robot_pose.j5 << ", "
         << robot_pose.j6 << ")";
    return ostr;
}

