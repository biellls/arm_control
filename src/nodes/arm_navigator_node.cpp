#include <queue>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include "arm_control/MoveArmAction.h"

namespace ac = arm_control;

class ArmNavigatorNode
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer timer_;

    actionlib::SimpleActionServer<ac::MoveArmAction> action_server_;

    nav_msgs::Path current_path_;
    int current_target_pose_index_;

  public:
    ArmNavigatorNode() : nh_("robot_arm"), nh_private_("~"), action_server_(nh_, "arm_navigator_action_server", false)
    {
        init();
    }

    ~ArmNavigatorNode()
    {
    }

    void init()
    {
        timer_ = nh_.createTimer(ros::Duration(0.5), boost::bind(&ArmNavigatorNode::reportPath, this)); 
        action_server_.registerGoalCallback(boost::bind(&ArmNavigatorNode::goalCB, this));
        action_server_.start();
    }

    void goalCB()
    {
        ac::MoveArmGoalConstPtr goal = action_server_.acceptNewGoal();
        way_points_ = readWayPoints(goal->path);
        current_target_pose_index_ = 0;
        try
        {
            while (way_points_.size() > 0 && ros::ok())
            {
                try
                {
                    // get current pose as feedback
                    ac::MoveArmFeedback feedback;
                    tf::poseTFToMsg(retrievePose(), feedback.current_pose);
                    action_server_.publishFeedback(feedback);

                    if (action_server_.isPreemptRequested())
                    {
                        ROS_INFO("MoveArmAction preempted.");
                        action_server_.setPreempted();
                        melfa_.stop();
                        break;
                    }
                    melfa::RobotPose& next_target_pose = way_points_.front();
                    melfa_.moveTo(next_target_pose);
                    way_points_.pop();
                    ROS_INFO("Sent next way point to robot: %f %f %f, %f %f %f",
                            next_target_pose.x,
                            next_target_pose.y,
                            next_target_pose.z,
                            next_target_pose.roll,
                            next_target_pose.pitch,
                            next_target_pose.yaw);
                }
                catch (const melfa::MelfaRobotBusyException&)
                {
                    // move command failed because robot is busy, wait a little
                    ROS_INFO("Robot is busy, waiting 1 second to send next waypoint.");
                    sleep(1);
                }
            }
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when moving robot: %s", e.what());
            action_server_.setAborted();
        }
        // motion is finished
        ac::MoveArmResult result;
        tf::poseTFToMsg(retrievePose(), result.end_pose);
        action_server_.setSucceeded(result);
    }

    std::queue<melfa::RobotPose> readWayPoints(const geometry_msgs::PoseArray& pose_array) const
    {
        std::queue<melfa::RobotPose> way_points;
        for (size_t i = 0; i < pose_array.poses.size(); ++i)
        {
            const geometry_msgs::Pose& pose = pose_array.poses[i];
            melfa::RobotPose way_point;
            melfa_ros::poseMsgToRobot(pose, way_point);
            way_points.push(way_point);
        }
        return way_points;
    }

    /// reads the pose from the robot and fills the given tf struct
    tf::Pose retrievePose()
    {
        melfa::RobotPose robot_pose = melfa_.getPose();
        tf::Quaternion quat;
        quat.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
        tf::Pose tf_pose(quat, tf::Vector3(robot_pose.x, robot_pose.y, robot_pose.z));
        ROS_DEBUG("Retrieved pose: %f %f %f, %f %f %f", 
                robot_pose.x, 
                robot_pose.y, 
                robot_pose.z, 
                robot_pose.roll, 
                robot_pose.pitch, 
                robot_pose.yaw);
        return tf_pose;
    }


    void reportPose()
    {
        try
        {
            geometry_msgs::PoseStamped pose_stamped;
            melfa_ros::poseRobotToMsg(melfa_.getPose(), pose_stamped.pose);
            ros::Time timestamp = ros::Time::now();
            pose_stamped.header.stamp = timestamp;
            pose_stamped.header.frame_id = "arm_base";
            pose_pub_.publish(pose_stamped);

            tf::Pose tf_pose;
            tf::poseMsgToTF(pose_stamped.pose, tf_pose);
            tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(tf_pose),
                        timestamp, "arm_base", "arm_tool"));
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when trying to retrieve robot pose: %s", e.what());
            action_server_.setAborted();
        }
     }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControlNode arm_control_node;
    ros::spin();
    return 0;
}

