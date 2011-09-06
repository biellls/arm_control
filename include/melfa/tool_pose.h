#ifndef TOOL_POSE_H_
#define TOOL_POSE_H_

namespace melfa
{
    /**
    * \brief struct to hold a 6D pose of the robot tool
    * The position is given in meters (x, y, z)
    * and the orientation is given in radiants
    * (roll, pitch, yaw)
    */
    struct ToolPose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };
}

std::ostream& operator<< (std::ostream& ostr, const melfa::ToolPose& tool_pose);

#endif

