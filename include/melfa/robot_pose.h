#ifndef ROBOT_POSE_H_
#define ROBOT_POSE_H_

namespace melfa
{
    /**
    * \brief struct to hold a robot pose
    * The position is given in meters (x, y, z)
    * and the orientation is given in radiants
    * (roll, pitch, yaw)
    */
    struct RobotPose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };
}

std::ostream& operator<< (std::ostream& ostr, const melfa::RobotPose& robot_pose);

#endif

