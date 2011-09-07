#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>

#include "arm_control/MoveToolAction.h"
#include "arm_control/MoveJointsAction.h"
#include "melfa_ros/conversions.h"

#include "melfa/melfa.h"
#include "melfa/joint_state.h"
#include "melfa/tool_pose.h"
#include "melfa/exceptions.h"

namespace ac = arm_control;

class ArmControlNode
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher tool_pose_pub_;
    ros::Publisher joint_state_pub_;

    tf::TransformBroadcaster tf_broadcaster_;
    melfa::Melfa melfa_;
    ros::Timer timer_;

    actionlib::SimpleActionServer<ac::MoveToolAction> move_tool_action_server_;
    actionlib::SimpleActionServer<ac::MoveJointsAction> move_joints_action_server_;

  public:
    ArmControlNode() : nh_("robot_arm"), nh_private_("~"), 
        move_tool_action_server_(nh_, "move_tool_action_server", false),
        move_joints_action_server_(nh_, "move_joints_action_server", false)
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
        int linear_override;
        nh_private_.param<int>("linear_override", linear_override, 50);
        int joint_override;
        nh_private_.param<int>("joint_override", joint_override, 50);
        double tool_x, tool_y, tool_z, tool_roll, tool_pitch, tool_yaw;
        nh_private_.param<double>("tool_x", tool_x, 0.0);
        nh_private_.param<double>("tool_y", tool_y, 0.0);
        nh_private_.param<double>("tool_z", tool_z, 0.0);
        nh_private_.param<double>("tool_roll", tool_roll, 0.0);
        nh_private_.param<double>("tool_pitch", tool_pitch, 0.0);
        nh_private_.param<double>("tool_yaw", tool_yaw, 0.0);

        try
        {
            melfa::Melfa::ConfigParams params;
            params.device = device;
            melfa_.setParams(params);
            melfa_.connect();
            ROS_INFO("Robot connected.");
            melfa_.setLinearOverride(linear_override);
            ROS_INFO("Linear override set to %i", linear_override);
            melfa_.setJointOverride(joint_override);
            ROS_INFO("Joint override set to %i", joint_override);
            melfa_.setTool(tool_x, tool_y, tool_z, tool_roll, tool_pitch, tool_yaw);
            ROS_INFO("Tool set to (%f, %f, %f) (%f, %f, %f)", 
                    tool_x, tool_y, tool_z, tool_roll, tool_pitch, tool_yaw);
        }
        catch (melfa::SerialConnectionError& err)
        {
            ROS_ERROR("Serial Connection error: %s", err.what());
        }
        catch (melfa::RobotError& err)
        {
            ROS_ERROR("Robot error: %s", err.what());
        }

        tool_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tool_pose", 1);
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);

        timer_ = nh_.createTimer(ros::Duration(0.01), boost::bind(&ArmControlNode::report, this)); 

        move_tool_action_server_.registerGoalCallback(boost::bind(&ArmControlNode::moveToolGoalCB, this));
        move_tool_action_server_.start();
        ROS_INFO("Move Tool Action server started.");

        move_joints_action_server_.registerGoalCallback(boost::bind(&ArmControlNode::moveJointsGoalCB, this));
        move_joints_action_server_.start();
        ROS_INFO("Move Joints Action server started.");
    }

    void moveToolGoalCB()
    {
        if (move_joints_action_server_.isActive())
        {
            ROS_ERROR("Cannot move tool, move joints action is active!");
            move_tool_action_server_.setAborted();
            return;
        }
        ac::MoveToolGoalConstPtr goal = move_tool_action_server_.acceptNewGoal();
        melfa::ToolPose target_tool_pose;
        melfa_ros::poseMsgToToolPose(goal->target_pose, target_tool_pose);
        ROS_INFO_STREAM("Received new goal: tool pose " << target_tool_pose);
        try
        {
            melfa_.moveTool(target_tool_pose);
            while (melfa_.isBusy())
            {
                usleep(200000);
                // get current pose as feedback
                ac::MoveToolFeedback feedback;
                melfa_ros::toolPoseToPoseMsg(melfa_.getToolPose(), feedback.current_pose);
                move_tool_action_server_.publishFeedback(feedback);

                if (move_tool_action_server_.isPreemptRequested())
                {
                    ROS_INFO("MoveToolAction preempted.");
                    move_tool_action_server_.setPreempted();
                    melfa_.stop();
                    break;
                }

                if (!ros::ok()) // node shutdown requested?
                {
                    melfa_.stop();
                    break;
                }
            }

            // motion is finished
            ac::MoveToolResult result;
            melfa_ros::toolPoseToPoseMsg(melfa_.getToolPose(), result.end_pose);
            move_tool_action_server_.setSucceeded(result);
        }
        catch (const melfa::PoseUnreachableException&)
        {
            ROS_ERROR("Requested pose is unreachable, aborting.");
            move_tool_action_server_.setAborted();
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when moving robot: %s", e.what());
            move_tool_action_server_.setAborted();
        }
    }

    void moveJointsGoalCB()
    {
        if (move_tool_action_server_.isActive())
        {
            ROS_ERROR("Cannot move joints, move tool action is active!");
            move_tool_action_server_.setAborted();
            return;
        }
        ac::MoveJointsGoalConstPtr goal = move_joints_action_server_.acceptNewGoal();
        melfa::JointState target_joint_state;
        melfa_ros::jointStateMsgToJointState(goal->target_joint_state, target_joint_state);
        ROS_INFO_STREAM("Received new goal: joint state " << target_joint_state);
        try
        {
            melfa_.moveJoints(target_joint_state);
            while (melfa_.isBusy())
            {
                usleep(200000);
                // get current pose as feedback
                ac::MoveJointsFeedback feedback;
                melfa_ros::jointStateToJointStateMsg(melfa_.getJointState(), feedback.current_joint_state);
                move_joints_action_server_.publishFeedback(feedback);

                if (move_joints_action_server_.isPreemptRequested())
                {
                    ROS_INFO("MoveJointsAction preempted.");
                    move_joints_action_server_.setPreempted();
                    melfa_.stop();
                    break;
                }

                if (!ros::ok()) // node shutdown requested?
                {
                    melfa_.stop();
                    break;
                }
            }

            // motion is finished
            ac::MoveJointsResult result;
            melfa_ros::jointStateToJointStateMsg(melfa_.getJointState(), result.end_joint_state);
            move_joints_action_server_.setSucceeded(result);
        }
        catch (const melfa::PoseUnreachableException&)
        {
            ROS_ERROR("Requested pose is unreachable, aborting.");
            move_joints_action_server_.setAborted();
        }
        catch (const melfa::MelfaException& e)
        {
            ROS_ERROR("Exception occured when moving robot: %s", e.what());
            move_joints_action_server_.setAborted();
        }
    }


    void report()
    {
        try
        {
            geometry_msgs::PoseStamped pose_stamped;
            melfa::ToolPose tool_pose = melfa_.getToolPose();
            melfa_ros::toolPoseToPoseMsg(tool_pose, pose_stamped.pose);
            ros::Time timestamp = ros::Time::now();
            pose_stamped.header.stamp = timestamp;
            pose_stamped.header.frame_id = "arm_base";
            tool_pose_pub_.publish(pose_stamped);

            sensor_msgs::JointState joint_state_msg;
            melfa::JointState joint_state = melfa_.getJointState();
            melfa_ros::jointStateToJointStateMsg(joint_state, joint_state_msg);
            joint_state_msg.header.stamp = timestamp;
            joint_state_msg.header.frame_id = "arm_base";
            joint_state_pub_.publish(joint_state_msg);

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
    // use multi threaded spinning to have callbacks
    // called while action is running
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    //ros::spin();
    return 0;
}

