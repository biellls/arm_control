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
         << ")";
    return ostr;
}

