#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include "melfa/melfa.h"
#include "melfa/exceptions.h"

/*
 * TODO include custom message to get device name and string
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
 
  std::string device_name("USB0");
  std::string command(msg->data.c_str());

  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connect();
      std::cout << "Robot connected." << std::endl;
      std::cout << "Executing command: " << command << std::endl;
      melfa.execute(command);
      std::cout << "Execution finished." << std::endl;
    }
  catch (melfa::SerialConnectionError& err)
    {
      std::cerr << "Serial Connection error: " << err.what() << std::endl;
    }
  catch (melfa::RobotError& err)
    {
      std::cerr << "Robot error: " << err.what() << std::endl;
    }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "execute_node");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("execute_instruction", 1000, executeCallback);

  ros::spin();
  
  return 0;
}

