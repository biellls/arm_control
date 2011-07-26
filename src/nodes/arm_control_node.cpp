#include <queue>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

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

    std::queue<std::string> command_queue_;

  public:
    ArmControlNode() : nh_private_("~")
    {
        init();
    }

    ~ArmControlNode()
    {
    }

    void init()
    {
        pose_pub_ = nh_.advertise<geometry_msgs::Pose>("pose", 1);

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

        timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&ArmControlNode::checkAction, this)); 
    }

    void checkAction()
    {
        if (command_queue_.size() > 0 && !melfa_.isBusy())
        {
            melfa_.execute(command_queue_.front());
            command_queue_.pop();
        }
        reportPose();
    }

    void reportPose()
    {
        double x, y, z, roll, pitch, yaw;
        ros::Time timestamp = ros::Time::now();
        melfa_.getPose(x, y, z, roll, pitch, yaw);

        tf::Vector3 translation(x, y, z);
        tf::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        tf::Pose tf_pose(quat, translation);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = timestamp;
        pose_stamped.header.frame_id = "base_link";

        tf::poseTFToMsg(tf_pose, pose_stamped.pose);

        pose_pub_.publish(pose_stamped);

        tf_broadcaster_.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf_pose),
                    timestamp, "base_link", "camera"));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControlNode arm_control_node;
    ros::spin();
    return 0;
}

