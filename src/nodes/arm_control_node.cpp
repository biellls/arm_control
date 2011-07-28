#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include "arm_control/MoveArmAction.h"

#include "melfa.h"
#include "exceptions.h"

namespace ac = arm_control;

class ArmControlNode
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pose_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    ac::Melfa melfa_;
    ros::Timer timer_;

    actionlib::SimpleActionServer<ac::MoveArmAction> action_server_;

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
            ac::Melfa::ConfigParams params;
            params.device = device;
            melfa_.setParams(params);
            melfa_.connect();
        }
        catch (ac::MelfaSerialConnectionError& err)
        {
            ROS_ERROR("Serial Connection error: %s", err.what());
        }
        catch (ac::MelfaRobotError& err)
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
        tf::Quaternion quat;
        tf::quaternionMsgToTF(goal->target_pose.orientation, quat);
        double x, y, z, roll, pitch, yaw;
        x = goal->target_pose.position.x;
        y = goal->target_pose.position.y;
        z = goal->target_pose.position.z;
        btMatrix3x3(quat).getRPY(roll, pitch, yaw);

        try
        {
            melfa_.moveTo(x, y, z, roll, pitch, yaw);
            while (melfa_.isBusy() && ros::ok())
            {
                // get current pose as feedback
                ac::MoveArmFeedback feedback;
                tf::poseTFToMsg(retrievePose(), feedback.current_pose);
                action_server_.publishFeedback(feedback);

                if (action_server_.isPreemptRequested())
                {
                    ROS_INFO("MoveArmAction preemted.");
                    action_server_.setPreempted();
                    melfa_.stop();
                    break;
                }
            }
        }
        catch (const ac::MelfaException& e)
        {
            ROS_ERROR("Exception occured when moving robot: %s", e.what());
            action_server_.setAborted();
        }
        // motion is finished
        ac::MoveArmResult result;
        tf::poseTFToMsg(retrievePose(), result.end_pose);
        action_server_.setSucceeded(result);
    }

    /// reads the pose from the robot and fills the given tf struct
    tf::Pose retrievePose()
    {
        double x, y, z, roll, pitch, yaw;
        melfa_.getPose(x, y, z, roll, pitch, yaw);
        tf::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        tf::Pose pose(quat, tf::Vector3(x, y, z));
        ROS_INFO("Retrieved pose: %f %f %f, %f %f %f", x, y, z, roll, pitch, yaw);
        return pose;
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
            ROS_INFO_STREAM("pose: " << pose_stamped.pose.position.x);
            pose_pub_.publish(pose_stamped);

            tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(pose),
                        timestamp, "base_link", "camera"));
        }
        catch (const ac::MelfaException& e)
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

