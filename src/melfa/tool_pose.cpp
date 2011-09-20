#include <iostream>
#include <cmath>

#include "melfa/tool_pose.h"

std::ostream& operator<< (std::ostream& ostr, const melfa::ToolPose& tool_pose)
{
    ostr << "(" 
         << tool_pose.x << ", " 
         << tool_pose.y << ", "
         << tool_pose.z 
         << ") ("
         << tool_pose.roll << ", "
         << tool_pose.pitch << ", "
         << tool_pose.yaw
         << ") in degrees: ("
         << tool_pose.roll / M_PI * 180.0 << ", "
         << tool_pose.pitch / M_PI * 180.0 << ", "
         << tool_pose.yaw / M_PI * 180.0 
         << ")";
     return ostr;
}

