#include <iostream>
#include "melfa/robot_pose.h"
#include "melfa_ros/conversions.h"

int main(int argc, char* argv[])
{
    if (argc != 7)
    {
        std::cerr << "Converts robot poses as seen in the melfa display to ros pose messages." << std::endl;
        std::cerr << "Usage: " << argv[0] 
            << " <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
        std::cerr << "where x, y, z, are given in millimeters and roll, pitch, and yaw in degrees" << std::endl;
        return -1;
    }

    melfa::RobotPose robot_pose;
    robot_pose.x = atof(argv[1]) / 1000.0;
    robot_pose.y = atof(argv[2]) / 1000.0;
    robot_pose.z = atof(argv[3]) / 1000.0;
    robot_pose.roll = atof(argv[4]) / 180.0 * M_PI;
    robot_pose.pitch = atof(argv[5]) / 180.0 * M_PI;
    robot_pose.yaw = atof(argv[6]) / 180.0 * M_PI;

    geometry_msgs::Pose pose_msg;
    melfa_ros::poseRobotToMsg(robot_pose, pose_msg);
    std::cout << pose_msg;

    return 0;
}

