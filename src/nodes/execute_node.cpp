#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "melfa/melfa.h"
#include "melfa/exceptions.h"

// Debug variable permits debugging without being connected to the robot
#define DEBUG true

// Control messages
#define CTL_EXECUTE "---SINGLE INSTRUCTION---"
#define CTL_PROGRAM_BEGIN "---LOAD PROGRAM BEGIN---"
#define CTL_PROGRAM_END "---LOAD PROGRAM END---"
#define CTL_POINTS_BEGIN "---LOAD POINTS BEGIN---"
#define CTL_POINTS_END "---LOAD POINTS END---"

/*
 * State machine states
 * S4, S5 and S6 are implicit/conceptual states because
 *  they perform an action and transition to a new state
 *  without waiting for input
 */

enum States
  {
    S0, // Initial state: wait for control message
        // * Transition: CTL_EXECUTE -> S1
        //               CTL_PROGRAM_BEGIN -> S2
        //               CTL_POINTS_BEGIN -> S3
    S1, // Execute state: wait for instruction to execute
        // * Transition: instr -> S4
    S2, // Load program state: wait for instructions and write them to file
        // * Transition: instr -> S2
        //               CTL_PROGRAM_END -> S7
    S3, // Load points state: wait for points and write them to file
        // * Transition: point -> S3
        //               CTL_POINTS_END -> S8
    //(S4), Execute instruction and transition to S0
        // * Transition: _ -> S0
    //(S5), Load program and transition to S0
        // * Transition: _ -> S0
    //(S6), Load points and transition to S0
        // * Transition: _ -> S0
  };

States state; // Current state

#define PROGRAM_FILE "program.mb4"
#define POINTS_FILE "points.pos"

std::ofstream program_file, points_file;

// Given a state returns a string representation
// Useful for printing errors
std::string stateName(States s)
{
  if (s == S0) return "S0";
  else if (s == S1) return "S1";
  else if (s == S2) return "S2";
  else return "S3";
}

bool isControlMessage(std::string msg)
{
  return
    msg == CTL_EXECUTE ||
    msg == CTL_PROGRAM_BEGIN ||
    msg == CTL_PROGRAM_END ||
    msg == CTL_POINTS_BEGIN ||
    msg == CTL_POINTS_END;
}

void clearProgramFile() {
}

void clearPointsFile() {
}

void loadProgram() {
}

void loadPoints() {
}

void nextState(std::string msg)
{
  ROS_INFO("Heard control message: %s", msg.c_str());
  if (state == S0) {
    //We will execute the next instruction
    if (msg == CTL_EXECUTE) {
      ROS_INFO("CTL_EXECUTE");
      state = S1;
      return;
    }
    //We will store the following instructions in a file
    //until we are told to load it to the robot
    if (msg == CTL_PROGRAM_BEGIN) {
      ROS_INFO("CTL_PROGRAM_BEGIN");
      //clearProgramFile();
      program_file.open(PROGRAM_FILE);
      state = S2;
      return;
    }
    //We will store the following points in a file
    // until we are told to load it to the robot
    if (msg == CTL_POINTS_BEGIN) {
      ROS_INFO("CTL_POINTS_BEGIN");
      points_file.open(PROGRAM_FILE);
      //clearPointsFile();
      state = S3;
      return;
    }
    //ERROR
    std::cerr << "Invalid control signal error: " << msg << std::endl;
    state = S0;
    return;
  } else if (state == S2 && msg == CTL_PROGRAM_END) {
      ROS_INFO("CTL_PROGRAM_END");
    //We load the program file to the robot
    loadProgram();
    program_file.close();
    state = S0;
    return;
  } else if (state == S3 && msg == CTL_POINTS_END) {
      ROS_INFO("CTL_POINTS_END");
    //We load the points file to the robot
    loadPoints();
    points_file.close();
    state = S0;
    return;
  }
  std::cerr << "Invalid control signal error: " << msg
            << " for state " << stateName(state) << std::endl;
  state = S0;
}

void writeToProgramFile(std::string msg) {
  program_file << msg << "\n";
}

void writeToPointsFile(std::string msg) {
  points_file << msg << "\n";
}

void execute_command(std::string command)
{
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
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

void handleMessage(std::string msg)
{
  ROS_INFO("Handling message: %s", msg.c_str());
  if (state == S1) {
    ROS_INFO("Execute command: %s", msg.c_str());
    execute_command(msg);
    state = S0;
  } else if (state == S2) {
    ROS_INFO("Write command: %s", msg.c_str());
    writeToProgramFile(msg);
  } else if (state == S3) {
    ROS_INFO("Write point: %s", msg.c_str());
    writeToPointsFile(msg);
  } else {
    std::cerr << "Invalid state error: " << stateName(state)
              << " for message " << msg << std::endl;
    state = S0;
  }
}

/*
 * TODO include custom message to get device name and string
 */
void executeCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Current state: %s", stateName(state).c_str());
  ROS_INFO("I heard: [%s]", msg->data.c_str());
 
  std::string message(msg->data.c_str());
  
  if (isControlMessage(message)) {
    nextState(message);
    return;
  }

  handleMessage(message);

}

int main(int argc, char* argv[])
{
  state = S0;

  ros::init(argc, argv, "execute_node");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("execute_instruction", 1000, executeCallback);

  ros::spin();
  
  return 0;
}
