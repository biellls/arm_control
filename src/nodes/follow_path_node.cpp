#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

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
    ros::Publisher status_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    melfa::Melfa melfa_;
    ros::Timer timer_;

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
            ROS_INFO("Acceleration set to %d", acceleration);
            melfa_.setMaximumVelocity(maximum_velocity);
            ROS_INFO("Maximum velocity set to %d", maximum_velocity);
            melfa_.setToolPose(tool_pose);
            ROS_INFO_STREAM("Tool pose set to " << tool_pose);
        }
        catch (melfa::MelfaSerialConnectionError& err)
        {
            ROS_ERROR("Serial Connection error: %s", err.what());
        }
        catch (melfa::MelfaRobotError& err)
        {
            ROS_ERROR("Robot error: %s", err.what());
        }

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
        busy_status_pub_ = nh_.advertise<std_msgs::Bool>("busy_status", 1);
        timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&ArmControlNode::reportPose, this)); 
    }

    void goalCB()
    {
        ac::MoveArmGoalConstPtr goal = action_server_.acceptNewGoal();
        way_points_ = readWayPoints(goal->path);
        try
        {
            // set parameters
            melfa_.setMaximumVelocity(goal->maximum_velocity);
            melfa_.setAcceleration(goal->acceleration);
            melfa::RobotPose tool_pose;
            melfa_ros::poseMsgToRobot(goal->tool_pose, tool_pose);
            melfa_.setToolPose(tool_pose);
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

