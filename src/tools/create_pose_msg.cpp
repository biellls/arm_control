#include <iostream>
#include "melfa/tool_pose.h"
#include "melfa_ros/conversions.h"

int main(int argc, char* argv[])
{
    if (argc != 7)
    {
        std::cerr << "Converts tool poses as seen in the melfa display to ros pose messages." << std::endl;
        std::cerr << "Usage: " << argv[0] 
            << " <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
        std::cerr << "where x, y, z, are given in millimeters and roll, pitch, and yaw in degrees" << std::endl;
        return -1;
    }

    melfa::ToolPose pose;
    pose.x = atof(argv[1]) / 1000.0;
    pose.y = atof(argv[2]) / 1000.0;
    pose.z = atof(argv[3]) / 1000.0;
    pose.roll = atof(argv[4]) / 180.0 * M_PI;
    pose.pitch = atof(argv[5]) / 180.0 * M_PI;
    pose.yaw = atof(argv[6]) / 180.0 * M_PI;

    geometry_msgs::Pose pose_msg;
    melfa_ros::poseToolToMsg(pose, pose_msg);
    std::cout << pose_msg;

    return 0;
}

