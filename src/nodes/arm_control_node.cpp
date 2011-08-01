#include <queue>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include "melfa_ros/conversions.h"

#include "melfa/melfa.h"
#include "melfa/robot_pose.h"
#include "melfa/exceptions.h"

class ArmControlNode
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pose_pub_;
    ros::Publisher busy_status_pub_;
    ros::Subscriber target_pose_sub_;

    tf::TransformBroadcaster tf_broadcaster_;
    melfa::Melfa melfa_;
    ros::Timer timer_;

  public:
    ArmControlNode() : nh_("robot_arm"), nh_private_("~")
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

        target_pose_sub_ = nh_.subscribe("target_pose", 1, &ArmControlNode::setTargetPose, this);

        timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&ArmControlNode::report, this)); 
    }

    void report()
    {
        std_msgs::Bool busy_msg;
        busy_msg.data = false;
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

            // try some execution command
            // and check if it throws to see if the robot is busy
            melfa_.execute("P1=P_CURR");
        }
        catch (const melfa::MelfaRobotBusyException&)
        {
            // robot is busy
            busy_msg.data = true;
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when trying to retrieve robot pose: %s", e.what());
        }
        busy_status_pub_.publish(busy_msg);
    }

    void setTargetPose(const geometry_msgs::PoseConstPtr& pose_msg)
    {
        ROS_INFO_STREAM("Received new pose request: " << *pose_msg);
        try
        {
            melfa::RobotPose robot_pose;
            melfa_ros::poseMsgToRobot(*pose_msg, robot_pose);
            melfa_.moveTo(robot_pose);
        }
        catch (const melfa::MelfaRobotBusyException&)
        {
            // robot is busy
            ROS_ERROR("Cannot fulfill pose request, robot is busy!");
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when trying to move robot: %s", e.what());
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

