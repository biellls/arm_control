#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ctime>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include "melfa/melfa.h"
#include "melfa/exceptions.h"
#include "melfa/tool_pose.h"
#include "melfa/joint_state.h"

// Debug variable permits debugging without being connected to the robot
#define DEBUG false

//Connection messages
//TODO chande CTL for CMD
#define CTL_CONNECT_FOR_LOAD "---CONNECT FOR LOAD---"
#define CTL_CONNECT_FOR_EXECUTE "---CONNECT FOR EXECUTE---"
#define CTL_DISCONNECT "---DISCONNECT---"

// Control messages
#define CTL_EXECUTE "---SINGLE INSTRUCTION---"
#define CTL_PROGRAM_BEGIN "---LOAD PROGRAM BEGIN---"
#define CTL_PROGRAM_END "---LOAD PROGRAM END---"
#define CTL_POINTS_BEGIN "---LOAD POINTS BEGIN---"
#define CTL_POINTS_END "---LOAD POINTS END---"
#define CTL_RUN_PROGRAM "---RUN PROGRAM---"
#define CTL_DELETE "---DELETE---"
#define CTL_MOVE_JOINT_STATE "---MOVE TOOL STATE---"

// Movement commands
#define MOV_TOOL_X_POS "---MOV TOOL +X---"
#define MOV_TOOL_X_NEG "---MOV TOOL -X---"
#define MOV_TOOL_Y_POS "---MOV TOOL +Y---"
#define MOV_TOOL_Y_NEG "---MOV TOOL -Y---"
#define MOV_TOOL_Z_POS "---MOV TOOL +Z---"
#define MOV_TOOL_Z_NEG "---MOV TOOL -Z---"

// Request messages
#define REQ_TOOL_POSE "---REQUEST TOOL POSE---"
#define REQ_JOINT_STATE "---REQUEST JOINT STATE---"

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
    S4
    //(S4), Execute instruction and transition to S0
        // * Transition: _ -> S0
    //(S5), Load program and transition to S0
        // * Transition: _ -> S0
    //(S6), Load points and transition to S0
        // * Transition: _ -> S0
  };

enum Connection_states
  {
    CONNECTED_FOR_LOAD,
    CONNECTED_FOR_EXECUTE,
    DISCONNECTED,
  };

States state; // Current state
Connection_states connection_state;

#define PROGRAM_FILE "program.mb4"
#define POINTS_FILE "points.POS"

// Constants
#define MAX_COMMAND_CHARS 256
#define TOOL_INCREMENT 0.02

std::fstream program_file, points_file;

// Global variables for multithreading (remote control)
#define NULL_COMMAND = "____NULL_COMMAND____"
bool remote_control_active = false;
boost::thread remote_control_thread;
time_t move_x_pos = -1;
time_t move_x_neg = -1;
time_t move_y_pos = -1;
time_t move_y_neg = -1;
time_t move_z_pos = -1;
time_t move_z_neg = -1;
// Time of last finished movement
time_t last_finished_move = -1;

void printToolPose(melfa::ToolPose toolPose) {
  std::cout << "Tool pose: (" <<
    toolPose.x << "," <<
    toolPose.y << "," <<
    toolPose.z << "," <<
    toolPose.roll << "," <<
    toolPose.pitch << "," <<
    toolPose.yaw << ")" << std::endl;
}

void remote_control_task() {
  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);
  
  melfa::ToolPose toolPose;
  
  melfa::Melfa melfa(params);
  try {
    melfa.connect();
    //melfa.initRobot();
    time_t now = time(0);
    std::cout << "Starting remote control task " << now << std::endl;
    melfa.setMaximumVelocity(0.5);
    melfa.setOverride(20);
    while (remote_control_active) {
      if (melfa.isBusy()) {
        sleep(0.1);
        continue;
      }
      time_t now = time(0);
      //// GETTING TOOL POSE
      toolPose = melfa.getToolPose();
      //// FINISHED GETTING TOOL POSE
      if (last_finished_move < move_x_pos) {
        std::cout << "Current timestamp now: " << now
                  << ", last timestamp X+: " << last_finished_move << std::endl;
        std::cout << "Remote control move X+" << std::endl;
        printToolPose(toolPose);
        toolPose.x += TOOL_INCREMENT;
      } else if (last_finished_move < move_x_neg) {
        std::cout << "Current timestamp now: " << now
                  << ", last timestamp X-: " << last_finished_move << std::endl;
        std::cout << "Remote control move X-" << std::endl;
        printToolPose(toolPose);
        toolPose.x -= TOOL_INCREMENT;
      } else if (last_finished_move < move_y_pos) {
        std::cout << "Current timestamp now: " << now
                  << ", last timestamp Y+: " << last_finished_move << std::endl;
        std::cout << "Remote control move Y+" << std::endl;
        printToolPose(toolPose);
        toolPose.y += TOOL_INCREMENT;
      } else if (last_finished_move < move_y_neg) {
        std::cout << "Current timestamp now: " << now 
                  << ", last timestamp Y-: " << last_finished_move << std::endl;
        std::cout << "Remote control move Y-" << std::endl;
        printToolPose(toolPose);
        toolPose.y -= TOOL_INCREMENT;
      } else if (last_finished_move < move_z_pos) {
        std::cout << "Current timestamp now: " << now
                  << ", last timestamp Z+: " << last_finished_move << std::endl;
        std::cout << "Remote control move Z+" << std::endl;
        printToolPose(toolPose);
        toolPose.z += TOOL_INCREMENT;
      } else if (last_finished_move < move_z_neg) {
        std::cout << "Current timestamp now: " << now
                  << ", last timestamp Z-: " << last_finished_move << std::endl;
        std::cout << "Remote control move Z-" << std::endl;
        printToolPose(toolPose);
        toolPose.z -= TOOL_INCREMENT;
      } else {
        //std::cout << "Not moving. Current timestamp now: " << now << std::endl;
        sleep(0.1);
        continue;
      }
      printToolPose(toolPose);
      melfa.moveTool(toolPose);
      last_finished_move = time(0);
      std::cout << "Finished move" << std::endl;
      sleep(0.1);
    }
  } catch (melfa::SerialConnectionError& err) {
    std::cerr << "Serial Connection error: " << err.what() << std::endl;
  } catch (melfa::RobotError& err) {
    std::cerr << "Robot error: " << err.what() << std::endl;
  }
  std::cout << "Exiting remote control task" << std::endl;
}

// Given a state returns a string representation
// Useful for printing errors
std::string stateName(States s)
{
  if (s == S0) return "S0";
  else if (s == S1) return "S1";
  else if (s == S2) return "S2";
  else if (s == S3) return "S3";
  else if (s == S4) return "S4";
  else return "Unknown State";
}

void send_command(std::string command)
{
  std::cout << "Sending command: " << command << std::endl;
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connect();
      //std::cout << "Robot connected." << std::endl;
      std::cout << "Executing command: " << command << std::endl;
      melfa.sendCommand(command);
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

void initSequence() {
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connectForLoad();
      std::cout << "Robot connected." << std::endl;
      //std::cout << "Executing command: " << command << std::endl;
      //melfa.sendCommand(command);
      //std::cout << "Execution finished." << std::endl;
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

void closeSequence() {
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.disconnectForLoad();
      std::cout << "Robot connected." << std::endl;
      //std::cout << "Executing command: " << command << std::endl;
      //melfa.sendCommand(command);
      //std::cout << "Execution finished." << std::endl;
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


void prepareLoad() {
  send_command("1;1;NEW");
  send_command("1;1;LOAD=1");
  send_command("1;1;PRTVERLISTL");
  send_command("1;1;PRTVEREMDAT");
}

std::string join(std::vector<std::string> v, std::string separator) {
  std::stringstream ss;
  for (size_t i = 0; i < v.size(); i++) {
    if (i != 0)
      ss << separator;
    ss << v[i];
  }
  return ss.str();
}

std::string getLineNum(std::string line) {
  return line.substr(0, line.find(" "));
}

void sendProgramLines() {
  send_command("1;9;LISTL<");
  if (program_file.is_open())
    program_file.close();
  program_file.open(PROGRAM_FILE);

  std::vector<std::string> v;
  std::string linenum;
  std::string line;
  while (std::getline(program_file, line)) {
    linenum = getLineNum(line);
    v.push_back(linenum);
  }
  std::string command = "1;9;EMDAT" + join(v, "<vt>");
  send_command(command);
}

void sendProgram() {
  if (program_file.is_open())
    program_file.close();
  program_file.open(PROGRAM_FILE);

  std::vector<std::string> v;
  std::string line;
  while (std::getline(program_file, line)) {
    //Strip \n
    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
    v.push_back(line);
  }
  std::string command = "1;9;EMDAT" + join(v, "<vt>");
  send_command(command);

  send_command("1;1;SAVE");
}

void sendPoints() {
  if (points_file.is_open())
    points_file.close();
  points_file.open(POINTS_FILE);

  std::vector<std::string> v;
  std::string line;
  while (std::getline(points_file, line)) {
    //Strip \n
    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
    v.push_back(line);
  }
  std::string command = "1;9;EMDAT" + join(v, "<vt>");
  send_command(command);

  send_command("1;1;SAVE");
}  

unsigned char val(char c) {
  if ('0' <= c && c <= '9') { return c      - '0'; }
  if ('a' <= c && c <= 'f') { return c + 10 - 'a'; }
  if ('A' <= c && c <= 'F') { return c + 10 - 'A'; }
  throw "Eeek";
}
 
std::string hexStringToString(std::string const & s) {
  if ((s.size() % 2) != 0) { throw "Eeek"; }
 
  std::string result;
  result.reserve(s.size() / 2);
 
  for (std::size_t i = 0; i < s.size(); i+=2)
    {
      unsigned char n = val(s[i]) * 16 + val(s[i + 1]);
      result += n;
    }
 
  return result;
}

void printResponse(std::vector<std::string> response) {
  std::stringstream ss;
  for (size_t i = 0; i < response.size(); i++) {
    //ss << hexStringToString(response[i]);
    ss << response[i];
    std::cout << "Response = " << ss << std::endl;
  }
}

void runProgram() {
  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connectForLoad();
      std::cout << "Robot connected." << std::endl;
      
      

      std::cout << "Sending command: " << "1;1;STATE" << std::endl;
      //melfa.sendCommand("1;1;STATE");
      std::cout << "Sending command: " << "1;1;CNTLON" << std::endl;
      melfa.sendCommand("1;1;CNTLON");
      std::cout << "Sending command: " << "1;1;SLOTINIT" << std::endl;
      melfa.sendCommand("1;1;SLOTINIT");
      std::cout << "Sending command: " << "1;1;RUN;1" << std::endl;
      melfa.sendCommand("1;1;RUN1;1");
      std::cout << "Sending command: " << "1;1;CNTLOFF" << std::endl;
      while (melfa.isBusy()) sleep(1);
      melfa.sendCommand("1;1;CNTLOFF");
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

void loadProgram() {
  //std::cout << hexStringToString("7ffc3ced7698") << std::endl;
  //Init sequence TODO mirar si hay que sacarlo fuera para sustituir a initrobot
  //initSequence();
  //prepareLoad();
  //sendProgramLines();
  //sendProgram();
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connectForLoad();
      std::cout << "Robot connected." << std::endl;

      //Prepare load
      melfa.sendCommand("1;1;NEW");
      melfa.sendCommand("1;1;LOAD=1");
      melfa.sendCommand("1;1;PRTVERLISTL");
      melfa.sendCommand("1;1;PRTVEREMDAT");
      std::cout << "Load prepared." << std::endl;

      std::cout << "Sending command: 1;9;LISTL<" << std::endl;
      std::vector<std::string> response = melfa.sendCommand("1;9;LISTL<");
      printResponse(melfa.sendCommand("1;9;LISTL<"));

      //Get program lines and numbers
      if (program_file.is_open())
        program_file.close();
      program_file.open(PROGRAM_FILE);

      std::vector<std::string> lineNums;
      std::vector<std::string> lines;
      std::string linenum;
      std::string line;
      while (std::getline(program_file, line)) {
        //Strip \n
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        lines.push_back(line);

        linenum = getLineNum(line);
        lineNums.push_back(linenum);
      }
      
      std::vector<std::string> commandsUploadLines;
      std::string commandLineNums = "1;9;EMDAT";
      std::string commandLines = "1;9;EMDAT";
      int commandLinesLength = commandLines.size();
      int begin_i = 0;
      for (int i = 0; i < lineNums.size(); i++) {
        if (i == lineNums.size() - 1) {  // Last line. We should upload it regardless of size
          std::vector<std::string> sub(&lineNums[begin_i], &lineNums[i]);
          commandLineNums = commandLineNums + join(sub, "\v");
          std::cout << "Sending command for line nums (command size = " << commandLineNums.size() << "): "
                    << commandLineNums << std::endl;
          std::cout << "Begin i = " << begin_i << " and i = " << i + 1 << std::endl;
          melfa.sendCommand(commandLineNums);
          std::vector<std::string> sub2(&lines[begin_i], &lines[i + 1]);
          commandLines = commandLines + join(sub2, "\v");
          commandsUploadLines.push_back(commandLines);
        } else if (commandLinesLength + lines[i].size() >= MAX_COMMAND_CHARS) {
          std::vector<std::string> sub(&lineNums[begin_i], &lineNums[i]);
          commandLineNums = commandLineNums + join(sub, "\v");
          std::cout << "Sending command for line nums (command size = " << commandLineNums.size() << "): "
                    << commandLineNums << std::endl;
          std::cout << "Begin i = " << begin_i << " and i = " << i << std::endl;
          melfa.sendCommand(commandLineNums);
          std::vector<std::string> sub2(&lines[begin_i], &lines[i]);
          commandLines = commandLines + join(sub2, "\v");
          commandsUploadLines.push_back(commandLines);
          
          // Reset values
          commandLineNums = "1;9;EMDAT";
          commandLines = "1;9;EMDAT";
          commandLinesLength = commandLines.size() + lines[i].size();
          begin_i = i;
        } else {
          std::cout << "Adding line = '" << lines[i] << "' of size = " << lines[i].size() << std::endl;
          commandLinesLength += lines[i].size() + 1;   //Add 1 because of vertical tab
          std::cout << "New size is " << commandLinesLength << std::endl;
        }
      }
      
      for (int i = 0; i < commandsUploadLines.size(); i ++) {
        std::cout << "Sending command for line (command size = " << commandsUploadLines[i].size() << "): "
                  << commandsUploadLines[i] << std::endl;
        melfa.sendCommand(commandsUploadLines[i]);
      }
      //Save file
      melfa.sendCommand("1;1;SAVE");
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

void loadPoints() {
  std::cout << "Load points started" << std::endl;
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connectForLoad();
      std::cout << "Robot connected." << std::endl;

      //Prepare load
      melfa.sendCommand("1;1;NEW");
      melfa.sendCommand("1;1;LOAD=1");
      melfa.sendCommand("1;1;PRTVERLISTL");
      melfa.sendCommand("1;1;PRTVEREMDAT");
      std::cout << "Load prepared." << std::endl;


      //Send program lines
      if (points_file.is_open())
        points_file.close();
      points_file.open(POINTS_FILE);

      std::string line;
      std::vector<std::string> pointLines;
      while (std::getline(points_file, line)) {
        if (line == "") {
          std::cout << "Skipping empty line" << std::endl;
          continue;
        }
        //Strip \n
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        //Remove DEF POS from start
        std::cout << "Line before: " << line << std::endl;
        line = line.substr(strlen("DEF POS "));
        std::cout << "Line after: " << line << std::endl;
        pointLines.push_back(line);
      }

      std::string commandLines = "1;9;EMDAT";
      int commandLinesLength = commandLines.size();
      int begin_i = 0;
      for (int i = 0; i < pointLines.size(); i++) {
        if (i == pointLines.size() - 1) { //Last line. We should upload it regardless of size
          std::vector<std::string> sub(&pointLines[begin_i], &pointLines[i + 1]);
          commandLines = commandLines + join(sub, "\v");
          std::cout << "Sending last command for points (command size = " << commandLines.size() << "): "
                    << commandLines << std::endl;
          std::cout << "Begin i = " << begin_i << " and i = " << i + 1 << std::endl;
          melfa.sendCommand(commandLines);
        } else if (commandLinesLength + pointLines[i].size() >= MAX_COMMAND_CHARS) {
          std::vector<std::string> sub(&pointLines[begin_i], &pointLines[i]);
          commandLines = commandLines + join(sub, "\v");
          std::cout << "Sending command for points (command size = " << commandLines.size() << "): "
                    << commandLines << std::endl;
          std::cout << "Begin i = " << begin_i << " and i = " << i << std::endl;
          melfa.sendCommand(commandLines);
          
          // Reset values
          commandLines = "1;9;EMDAT";
          commandLinesLength = commandLines.size() + pointLines[i].size();
          begin_i = i;
        } else {
          std::cout << "Adding line = '" << pointLines[i] << "' of size = " << pointLines[i].size() << std::endl;
          commandLinesLength += pointLines[i].size() + 1;   //Add 1 because of vertical tab
          std::cout << "New size is " << commandLinesLength << std::endl;
        }
      }
      //std::string command = "1;9;EMDAT" + join(v2, "\v");
      //std::cout << "Sending command: " << command << std::endl;
      //melfa.sendCommand(command);
      //std::cout << "Points sent." << std::endl;

      //Save file
      melfa.sendCommand("1;1;SAVE");
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

void deleteFromRobot() {
  std::cout << "Delete from robot started" << std::endl;
  //Init sequence TODO mirar si hay que sacarlo fuera para sustituir a initrobot
  //initSequence();
  //prepareLoad();
  //sendPoints();
  //closeSequence();
  if (DEBUG) return;

  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  melfa::Melfa melfa(params);
  try
    {
      melfa.connectForLoad();
      std::cout << "Robot connected." << std::endl;

      melfa.sendCommand("1;1;SAVE");
      melfa.sendCommand("1;1;CNTLON");
      melfa.sendCommand("1;1;RSTPRG");
      melfa.sendCommand("1;1;CNTLOFF");
      melfa.sendCommand("1;1;FDEL1");
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

void requestAndPublishJointState() {
  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);
  
  melfa::JointState jointState;
  
  melfa::Melfa melfa(params);
  try {
    melfa.connect();
    std::cout << "Robot connected." << std::endl;
    std::cout << "Requesting joint state: " << std::endl;
    jointState = melfa.getJointState();
    std::cout << "Joint state requested." << std::endl;
  } catch (melfa::SerialConnectionError& err) {
    std::cerr << "Serial Connection error: " << err.what() << std::endl;
  } catch (melfa::RobotError& err) {
    std::cerr << "Robot error: " << err.what() << std::endl;
  }
  std::cout << "Starting publish joint state" << std::endl;
  std::ostringstream stringStream;
  std::cout << jointState.j1 << std::endl;
  stringStream <<
    jointState.j1 << "," <<
    jointState.j2 << "," <<
    jointState.j3 << "," <<
    jointState.j4 << "," <<
    jointState.j5 << "," <<
    jointState.j6;
  std::cout << "Sending joint state: " << stringStream.str() << std::endl;
  std_msgs::String msg;
  msg.data = stringStream.str();
  ros::NodeHandle n;
  ros::Publisher joint_state_pub = n.advertise<std_msgs::String>("joint_state", 1000);
  std::cout << "Waiting for subscribers" << std::endl;
  while (joint_state_pub.getNumSubscribers() <= 0)
    sleep(1);
  joint_state_pub.publish(msg);
  ros::spinOnce();
  std::cout << "Joint state sent" << std::endl;
}

melfa::ToolPose requestToolPose() {
  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);
  
  melfa::ToolPose toolPose;
  
  melfa::Melfa melfa(params);
  try {
    melfa.connect();
    std::cout << "Robot connected." << std::endl;
    std::cout << "Requesting tool pose: " << std::endl;
    toolPose = melfa.getToolPose();
    std::cout << "Tool pose requested." << std::endl;
  } catch (melfa::SerialConnectionError& err) {
    std::cerr << "Serial Connection error: " << err.what() << std::endl;
  } catch (melfa::RobotError& err) {
    std::cerr << "Robot error: " << err.what() << std::endl;
  }
  return toolPose;
}

void publishToolPose(melfa::ToolPose toolPose) {
  std::cout << "Starting publish tool pose" << std::endl;
  std::ostringstream stringStream;
  stringStream <<
    toolPose.x << "," <<
    toolPose.y << "," <<
    toolPose.z << "," <<
    toolPose.roll << "," <<
    toolPose.pitch << "," <<
    toolPose.yaw;
  std::cout << "Sending tool pose: " << stringStream.str() << std::endl;
  std_msgs::String msg;
  msg.data = stringStream.str();
  ros::NodeHandle n;
  ros::Publisher tool_pose_pub = n.advertise<std_msgs::String>("tool_pose", 1000);
  std::cout << "Waiting for subscribers" << std::endl;
  while (tool_pose_pub.getNumSubscribers() <= 0)
    sleep(1);
  tool_pose_pub.publish(msg);
  ros::spinOnce();
  std::cout << "Tool pose sent" << std::endl;
}

melfa::JointState requestJointState() {
  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);
  
  melfa::JointState jointState;
  
  melfa::Melfa melfa(params);
  try {
    melfa.connect();
    std::cout << "Robot connected." << std::endl;
    std::cout << "Requesting joint state: " << std::endl;
    jointState = melfa.getJointState();
    std::cout << "Joint state requested." << std::endl;
  } catch (melfa::SerialConnectionError& err) {
    std::cerr << "Serial Connection error: " << err.what() << std::endl;
  } catch (melfa::RobotError& err) {
    std::cerr << "Robot error: " << err.what() << std::endl;
  }
  return jointState;
}

void moveTool(std::string msg) {
  std::string device_name("/dev/ttyUSB0");
  melfa::Melfa::ConfigParams params;
  params.device = std::string(device_name);

  double x, y, z, roll, pitch, yaw;
  if (sscanf(msg.c_str(), "(%lf,%lf,%lf,%lf,%lf,%lf)",
             &x, &y, &z, &roll, &pitch, &yaw) < 0) {
    std::cout << "Error parsing tool message" << msg << std::endl;
  }
  melfa::ToolPose toolPose;
  toolPose.x = x/1000.0;
  toolPose.y = y/1000.0;
  toolPose.z = z/1000.0;
  toolPose.roll = roll / 180 * M_PI;
  toolPose.pitch = pitch / 180 * M_PI;
  toolPose.yaw = yaw / 180 * M_PI;
  
  melfa::Melfa melfa(params);
  try {
    melfa.connect();
    melfa.setMaximumVelocity(1.0);
    melfa.setOverride(20);
    std::cout << "Robot connected." << std::endl;
    std::cout << "Moving Tool" << std::endl;
    std::cout << "X: " << toolPose.x << std::endl;
    std::cout << "Y: " << toolPose.y << std::endl;
    std::cout << "Z: " << toolPose.z << std::endl;
    std::cout << "Roll: " << toolPose.roll << std::endl;
    std::cout << "Pitch: " << toolPose.pitch << std::endl;
    std::cout << "Yaw: " << toolPose.yaw << std::endl;
    melfa.moveTool(toolPose);
    while (melfa.isBusy()) {
      sleep(0.1);
    }
    std::cout << "Tool moved." << std::endl;
  } catch (melfa::SerialConnectionError& err) {
    std::cerr << "Serial Connection error: " << err.what() << std::endl;
  } catch (melfa::RobotError& err) {
    std::cerr << "Robot error: " << err.what() << std::endl;
  }
}

void publishJointState(melfa::JointState jointState) {
  std::cout << "Starting publish joint state" << std::endl;
  std::ostringstream stringStream;
  std::cout << jointState.j1 << std::endl;
  stringStream <<
    jointState.j1 << "," <<
    jointState.j2 << "," <<
    jointState.j3 << "," <<
    jointState.j4 << "," <<
    jointState.j5 << "," <<
    jointState.j6;
  std::cout << "Sending joint state: " << stringStream.str() << std::endl;
  std_msgs::String msg;
  msg.data = stringStream.str();
  ros::NodeHandle n;
  ros::Publisher joint_state_pub = n.advertise<std_msgs::String>("joint_state", 1000);
  std::cout << "Waiting for subscribers" << std::endl;
  while (joint_state_pub.getNumSubscribers() <= 0)
    sleep(1);
  joint_state_pub.publish(msg);
  ros::spinOnce();
  std::cout << "Joint state sent" << std::endl;
}

time_t get_command_time(std::string msg) {
  std::string str_timestamp = msg.substr(17, msg.length());
  return atoi(str_timestamp.c_str());
}

bool startsWith(std::string arg, std::string prefix) {
  return !arg.compare(0, prefix.size(), prefix);
}

void executeCommandMessage(std::string msg) {
  if (!remote_control_active) {
    std::cout << "Activating remote control" << std::endl;
    remote_control_active = true;
    remote_control_thread = boost::thread(remote_control_task);
  }
  time_t command_time = get_command_time(msg);
  std::cout << "Remote control active. Command time: " << command_time << std::endl;
  if (startsWith(msg, MOV_TOOL_X_POS)) {
    std::cout << "MOV_TOOL_X_POS " << command_time << std::endl;
    move_x_pos = command_time;
  } else if (startsWith(msg, MOV_TOOL_X_NEG)) {
    std::cout << "MOV_TOOL_X_NEG " << command_time << std::endl;
    move_x_neg = command_time;
  } else if (startsWith(msg, MOV_TOOL_Y_POS)) {
    std::cout << "MOV_TOOL_Y_POS " << command_time << std::endl;
    move_y_pos = command_time;
  } else if (startsWith(msg, MOV_TOOL_Y_NEG)) {
    std::cout << "MOV_TOOL_Y_NEG " << command_time << std::endl;
    move_y_neg = command_time;
  } else if (startsWith(msg, MOV_TOOL_Z_POS)) {
    std::cout << "MOV_TOOL_Z_POS " << command_time << std::endl;
    move_z_pos = command_time;
  } else if (startsWith(msg, MOV_TOOL_Z_NEG)) {
    std::cout << "MOV_TOOL_Z_NEG " << command_time << std::endl;
    move_z_neg = command_time;
  } else {
    std::cerr << "Invalid command message error: " << stateName(state) << " for message " << msg << std::endl;
  }
}

bool isCommandMessage(std::string msg)
{
  return
    startsWith(msg, MOV_TOOL_X_POS) ||
    startsWith(msg, MOV_TOOL_X_NEG) ||
    startsWith(msg, MOV_TOOL_Y_POS) ||
    startsWith(msg, MOV_TOOL_Y_NEG) ||
    startsWith(msg, MOV_TOOL_Z_POS) ||
    startsWith(msg, MOV_TOOL_Z_NEG);
}

void handleMessage(std::string msg)
{
  if (isCommandMessage(msg)) {
    std::cout << "Executing command message: " << msg << std::endl;
    executeCommandMessage(msg);
    return;
  }
  //Connection messages
  if (msg == CTL_CONNECT_FOR_LOAD) {
    //Connect for load
    return;
  } else if (msg == CTL_CONNECT_FOR_EXECUTE) {
    //Connect for execute
    return;
  } else if (msg == CTL_DISCONNECT) {
    //Disconnect
    return;
  }

  //Execute and load messages
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
  } else if (state == S4) {
    ROS_INFO("Move tool: %s", msg.c_str());
    moveTool(msg);
    state = S0;
  } else {
    std::cerr << "Invalid state error: " << stateName(state)
              << " for message " << msg << std::endl;
    state = S0;
  }
}

bool isControlMessage(std::string msg)
{
  return
    msg == CTL_EXECUTE ||
    msg == CTL_RUN_PROGRAM ||
    msg == CTL_PROGRAM_BEGIN ||
    msg == CTL_PROGRAM_END ||
    msg == CTL_POINTS_BEGIN ||
    msg == CTL_POINTS_END ||
    msg == CTL_DELETE ||
    msg == CTL_MOVE_JOINT_STATE ||
    msg == REQ_JOINT_STATE ||
    msg == REQ_TOOL_POSE;
}

void nextState(std::string msg)
{
  ROS_INFO("Heard control message: %s", msg.c_str());
  if (!isCommandMessage(msg) && remote_control_active) {
    std::cout << "Deactivating remote control" << std::endl;
    remote_control_active = false;
    remote_control_thread.join();
    std::cout << "Joined remote control thread" << std::endl;
  }
  if (state == S0) {
    //We will execute the next instruction
    if (msg == CTL_EXECUTE) {
      ROS_INFO("CTL_EXECUTE");
      state = S1;
      return;
    }
    //We run the program in slot 1 and stay in S0
    if (msg == CTL_RUN_PROGRAM) {
      ROS_INFO("RUN_PROGRAM");
      runProgram();
      //send_command("1;1;SLOTINIT");
      //send_command("1;1;RUN;1");
      return;
    }
    //We will store the following instructions in a file
    //until we are told to load it to the robot
    if (msg == CTL_PROGRAM_BEGIN) {
      ROS_INFO("CTL_PROGRAM_BEGIN");
      if (program_file.is_open())
        program_file.close();
      program_file.open(PROGRAM_FILE, std::fstream::out);  //Will overwrite file contents
      state = S2;
      return;
    }
    //We will store the following points in a file
    // until we are told to load it to the robot
    if (msg == CTL_POINTS_BEGIN) {
      ROS_INFO("CTL_POINTS_BEGIN");
      if (points_file.is_open())
        points_file.close();
      points_file.open(POINTS_FILE, std::fstream::out);  //Will overwrite file contents
      state = S3;
      return;
    }
    //We will move to the point specified in the next message
    if (msg == CTL_MOVE_JOINT_STATE) {
      ROS_INFO("CTL_MOVE_JOINT_STATE");
      state = S4;
      return;
    }
    //We delete program and points file from robot and stay on S0
    if (msg == CTL_DELETE) {
      ROS_INFO("CTL_DELETE");
      deleteFromRobot();
      return;
    }
    if (msg == REQ_JOINT_STATE) {
      ROS_INFO("REQ_JOINT_STATE");
      melfa::JointState jointState = requestJointState();
      std::cout << "Calling publish joint state" << std::endl;
      publishJointState(jointState);
      return;
    }
    if (msg == REQ_TOOL_POSE) {
      ROS_INFO("REQ_TOOL_POSE");
      melfa::ToolPose toolPose = requestToolPose();
      publishToolPose(toolPose);
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

/*
 * TODO include custom message to get device name and string
 */
void executeCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Current state: %s", stateName(state).c_str());
  ROS_INFO("I heard: [%s]", msg->data.c_str());
 
  std::string message(msg->data.c_str());
  
  if (isControlMessage(message)) {
    ROS_INFO("Is control message");
    nextState(message);
    return;
  }

  handleMessage(message);
}

int main(int argc, char* argv[])
{
  state = S0;
  connection_state = DISCONNECTED;

  ros::init(argc, argv, "execute_node");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("execute_instruction", 1000, executeCallback);

  ros::spin();
  
  return 0;
}
