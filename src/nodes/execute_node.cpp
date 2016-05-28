#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <vector>
#include "melfa/melfa.h"
#include "melfa/exceptions.h"

// Debug variable permits debugging without being connected to the robot
#define DEBUG false

//Connection messages
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

std::fstream program_file, points_file;

// Given a state returns a string representation
// Useful for printing errors
std::string stateName(States s)
{
  if (s == S0) return "S0";
  else if (s == S1) return "S1";
  else if (s == S2) return "S2";
  else return "S3";
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

void printResponse(std::vector<std::string> response) {
  std::stringstream ss;
  for (size_t i = 0; i < response.size(); i++) {
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
      //std::vector<std::string> response = melfa.sendCommand("1;9;LISTL<");
      printResponse(melfa.sendCommand("1;9;LISTL<"));

      //Send program lines
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
      std::cout << "Sending program lines" << std::endl;
      std::string command = "1;9;EMDAT" + join(v, "\v");
      if (command.length() < MAX_COMMAND_CHARS) {
        std::cout << "Short program" << std::endl;
        std::cout << "Sending command: " << command << std::endl;
        melfa.sendCommand(command);
      } else {    //If the command is too long break it down
        std::cout << "Long program. Break into chunks and send over several instructions" << std::endl;
        std::size_t start = 0;
        std::size_t i = command.find("\v");
        while(i != std::string::npos) {
          std::size_t ni = command.find("\v", i+1);
          if (ni > start + MAX_COMMAND_CHARS - 1) {
            //send command substring
            std::string cs = command.substr(start, i - start + 1);
            std::cout << "Sending command: " << cs << std::endl;
            melfa.sendCommand(cs);
            start = i;
          }
          i = ni;
        }
      }
      //melfa.sendCommand("1;9;EMDAT10\v20\v30\v40");
      //melfa.sendCommand("1;9;EMDAT10 OVRD 30\v20 MOV P1\v30 MOV P5\v40 END");

      std::cout << "Program lines sent." << std::endl;

      //Send program
      if (program_file.is_open())
        program_file.close();
      program_file.open(PROGRAM_FILE);

      std::vector<std::string> v2;
      while (std::getline(program_file, line)) {
        //Strip \n
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        v2.push_back(line);
      }
      command = "1;9;EMDAT" + join(v2, "\v");
      std::cout << "Sending program" << std::endl;
      if (command.length() < MAX_COMMAND_CHARS) {
        std::cout << "Short program" << std::endl;
        std::cout << "Sending command: " << command << std::endl;
        melfa.sendCommand(command);
      } else {    //If the command is too long break it down
        std::cout << "Long program. Break into chunks and send over several instructions" << std::endl;
        std::size_t start = 0;
        std::size_t i = command.find("\v");
        while(i != std::string::npos) {
          std::size_t ni = command.find("\v", i+1);
          if (ni > start + MAX_COMMAND_CHARS - 1) {
            //send command substring
            std::string cs = command.substr(start, i - start + 1);
            std::cout << "Sending command: " << cs << std::endl;
            melfa.sendCommand(cs);
            start = i;
          }
          i = ni;
        }
      }
      std::cout << "Program sent." << std::endl;

      //Save file
      melfa.sendCommand("1;1;SAVE");
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

void loadPoints() {
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

      //Prepare load
      melfa.sendCommand("1;1;NEW");
      melfa.sendCommand("1;1;LOAD=1");
      melfa.sendCommand("1;1;PRTVERLISTL");
      melfa.sendCommand("1;1;PRTVEREMDAT");
      std::cout << "Load prepared." << std::endl;

      std::string line;
      std::vector<std::string> v2;
      while (std::getline(program_file, line)) {
        //Strip \n
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        v2.push_back(line);
      }
      std::string command = "1;9;EMDAT" + join(v2, "\v");
      std::cout << "Sending command: " << command << std::endl;
      melfa.sendCommand(command);
      std::cout << "Program sent." << std::endl;

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
      //melfa.connect();
      //std::cout << "Robot connected." << std::endl;
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
    msg == CTL_POINTS_END;
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
