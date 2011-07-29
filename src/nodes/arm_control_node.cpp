#include <queue>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include "arm_control/MoveArmAction.h"

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
    ArmControlNode() : nh_private_("~"), action_server_(nh_, "arm_control_action_server", false)
    {
        init();
    }

    ~ArmControlNode()
    {
    }

    void init()
    {
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);

        std::string device;
        nh_private_.param<std::string>("device", device, "/dev/ttyUSB0");
        ROS_INFO("using device %s", device.c_str());

        try
        {
            melfa::Melfa::ConfigParams params;
            params.device = device;
            melfa_.setParams(params);
            melfa_.connect();
        }
        catch (melfa::MelfaSerialConnectionError& err)
        {
            ROS_ERROR("Serial Connection error: %s", err.what());
        }
        catch (melfa::MelfaRobotError& err)
        {
            ROS_ERROR("Robot error: %s", err.what());
        }

        timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&ArmControlNode::reportPose, this)); 
        action_server_.registerGoalCallback(boost::bind(&ArmControlNode::goalCB, this));
        action_server_.start();
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
            poseMsgToRobot(goal->tool_pose, tool_pose);
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
            poseMsgToRobot(pose, way_point);
            way_points.push(way_point);
        }
        return way_points;
    }

    void poseMsgToRobot(const geometry_msgs::Pose& pose_msg, melfa::RobotPose& robot_pose) const
    {
        robot_pose.x = pose_msg.position.x;
        robot_pose.y = pose_msg.position.y;
        robot_pose.z = pose_msg.position.z;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose_msg.orientation, quat);
        btMatrix3x3(quat).getRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
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
            tf::Pose pose = retrievePose();
            ros::Time timestamp = ros::Time::now();
            pose_stamped.header.stamp = timestamp;
            pose_stamped.header.frame_id = "base_link";
            tf::poseTFToMsg(pose, pose_stamped.pose);
            pose_pub_.publish(pose_stamped);

            tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(pose),
                        timestamp, "base_link", "camera"));
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

