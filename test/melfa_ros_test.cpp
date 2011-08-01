#include <gtest/gtest.h>
#include "melfa_ros/conversions.h"
#include "melfa/robot_pose.h"

TEST(MelfaRos, conversions)
{

    melfa::RobotPose robot_pose;
    robot_pose.x = 0.1;
    robot_pose.y = 0.2;
    robot_pose.z = 0.3;
    robot_pose.roll = 10.0 * M_PI / 180.0;
    robot_pose.pitch = 20.0 * M_PI / 180.0;
    robot_pose.yaw = 30.0 * M_PI / 180.0;

    geometry_msgs::Pose ros_pose;
    melfa_ros::poseRobotToMsg(robot_pose, ros_pose);

    EXPECT_DOUBLE_EQ(robot_pose.x, ros_pose.position.x);
    EXPECT_DOUBLE_EQ(robot_pose.y, ros_pose.position.y);
    EXPECT_DOUBLE_EQ(robot_pose.z, ros_pose.position.z);

    melfa::RobotPose robot_pose2;
    melfa_ros::poseMsgToRobot(ros_pose, robot_pose2);
    EXPECT_NEAR(robot_pose.x, robot_pose2.x, 0.00001);
    EXPECT_NEAR(robot_pose.y, robot_pose2.y, 0.00001);
    EXPECT_NEAR(robot_pose.z, robot_pose2.z, 0.00001);
    EXPECT_NEAR(robot_pose.roll, robot_pose2.roll, 0.00001);
    EXPECT_NEAR(robot_pose.pitch, robot_pose2.pitch, 0.00001);
    EXPECT_NEAR(robot_pose.yaw, robot_pose2.yaw, 0.00001);


}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

