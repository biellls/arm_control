#include <queue>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>

#include "arm_control/MoveArmAction.h"

#include "melfa_ros/conversions.h"

#include "melfa/melfa.h"
#include "melfa/robot_pose.h"
#include "melfa/exceptions.h"

namespace ac = arm_control;

class ArmControlNode
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pose_pub_;

    tf::TransformBroadcaster tf_broadcaster_;
    melfa::Melfa melfa_;
    ros::Timer timer_;

    actionlib::SimpleActionServer<ac::MoveArmAction> action_server_;

    std::queue<melfa::RobotPose> way_points_;

  public:
    ArmControlNode() : nh_("robot_arm"), nh_private_("~"), action_server_(nh_, "arm_control_action_server", false)
    {
        init();
    }

    ~ArmControlNode()
    {
    }

    void init()
    {
        std::string device;
        nh_private_.param<std::string>("device", device, "/dev/ttyUSB0");
        ROS_INFO("using device %s", device.c_str());
        double acceleration;
        nh_private_.param<double>("acceleration", acceleration, 100);
        double maximum_velocity;
        nh_private_.param<double>("maximum_velocity", maximum_velocity, 0.05);
        melfa::RobotPose tool_pose;
        nh_private_.param<double>("tool_pose_x", tool_pose.x, 0.0);
        nh_private_.param<double>("tool_pose_y", tool_pose.x, 0.0);
        nh_private_.param<double>("tool_pose_z", tool_pose.x, 0.0);
        nh_private_.param<double>("tool_pose_roll", tool_pose.roll, 0.0);
        nh_private_.param<double>("tool_pose_pitch", tool_pose.pitch, 0.0);
        nh_private_.param<double>("tool_pose_yaw", tool_pose.yaw, 0.0);

        try
        {
            melfa::Melfa::ConfigParams params;
            params.device = device;
            melfa_.setParams(params);
            melfa_.connect();
            ROS_INFO("Robot connected.");
            melfa_.setAcceleration(acceleration);
            ROS_INFO("Acceleration set to %f", acceleration);
            melfa_.setMaximumVelocity(maximum_velocity);
            ROS_INFO("Maximum velocity set to %f", maximum_velocity);
            melfa_.setToolPose(tool_pose);
            ROS_INFO_STREAM("Tool pose set to " << tool_pose);
        }
        catch (melfa::SerialConnectionError& err)
        {
            ROS_ERROR("Serial Connection error: %s", err.what());
        }
        catch (melfa::RobotError& err)
        {
            ROS_ERROR("Robot error: %s", err.what());
        }

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);

        timer_ = nh_.createTimer(ros::Duration(5), boost::bind(&ArmControlNode::report, this)); 
        action_server_.registerGoalCallback(boost::bind(&ArmControlNode::goalCB, this));
        action_server_.start();
        ROS_INFO("Action server started.");
    }

    void goalCB()
    {
        ac::MoveArmGoalConstPtr goal = action_server_.acceptNewGoal();
        way_points_ = readWayPoints(goal->path.poses);
        ROS_INFO("Received new goal: path with %zu waypoints.", way_points_.size());
        try
        {
            while (way_points_.size() > 0 && ros::ok())
            {
                try
                {
                    // get current pose as feedback
                    ac::MoveArmFeedback feedback;
                    melfa_ros::poseRobotToMsg(melfa_.getPose(), feedback.current_pose);
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
                catch (const melfa::RobotBusyException&)
                {
                    // move command failed because robot is busy, wait a little
                    ROS_INFO("Robot is busy, waiting 1 second to send next waypoint.");
                    sleep(1);
                }
            }
            // all waypoints have been sent, wait for robot to finish
            ROS_INFO("Last waypoint sent, waiting for robot to finish");
            while (melfa_.isBusy() && ros::ok())
                sleep(1);

            // motion is finished
            ac::MoveArmResult result;
            melfa_ros::poseRobotToMsg(melfa_.getPose(), result.end_pose);
            action_server_.setSucceeded(result);
        }
        catch (const melfa::PoseUnreachableException&)
        {
            ROS_ERROR("Requested pose is unreachable, aborting.");
            action_server_.setAborted();
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when moving robot: %s", e.what());
            action_server_.setAborted();
        }
    }

    std::queue<melfa::RobotPose> readWayPoints(const std::vector<geometry_msgs::PoseStamped>& poses) const
    {
        std::queue<melfa::RobotPose> way_points;
        for (size_t i = 0; i < poses.size(); ++i)
        {
            const geometry_msgs::Pose& pose = poses[i].pose;
            melfa::RobotPose way_point;
            melfa_ros::poseMsgToRobot(pose, way_point);
            way_points.push(way_point);
        }
        return way_points;
    }

    void report()
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

