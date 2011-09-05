#ifndef ROBOT_POSE_H_
#define ROBOT_POSE_H_

namespace melfa
{
    /**
    * \brief struct to hold a robot pose
    * The angles are given in radiants.
    */
    struct RobotPose
    {
        double j1;
        double j2;
        double j3;
        double j4;
        double j5;
        double j6;
    };
}

std::ostream& operator<< (std::ostream& ostr, const melfa::RobotPose& robot_pose);

#endif

