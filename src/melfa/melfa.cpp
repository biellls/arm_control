#include <ros/ros.h>
#include <sstream>
#include <iomanip>

#include "melfa/robot_pose.h"
#include "melfa/tool_pose.h"
#include "melfa/exceptions.h"
#include "melfa/melfa.h"


// formats the value to have 2 digits after the comma
std::string format(double val)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << val;
    return ss.str();
}

melfa::Melfa::Melfa() : connected_(false)
{
}

melfa::Melfa::Melfa(const melfa::Melfa::ConfigParams& params) :
    params_(params), connected_(false)
{
}

void melfa::Melfa::setParams(const melfa::Melfa::ConfigParams& params)
{
    params_ = params;
}

melfa::Melfa::~Melfa()
{
    disconnect();
}

void melfa::Melfa::connect()
{
    boost::mutex::scoped_lock connection_lock(connection_mutex_);
    int status;
    {
        boost::mutex::scoped_lock lock(comm_mutex_);
        if (!comm_.openDevice(params_.device, status))
            throw SerialConnectionError("Melfa::connect(): could not open device.");

        std::string error("Melfa::connect():");
        bool ok;
        bool all_ok = true;
        ok = comm_.setBaudRate(9600);
        all_ok &= ok;
        if (!ok) error += " cannot set baud rate";
        ok = comm_.setDataBits(serial::SerialComm::DB8);
        all_ok &= ok;
        if (!ok) error += " cannot set data bits";
        ok = comm_.setStopBits(serial::SerialComm::TWO_STOP_BITS);
        all_ok &= ok;
        if (!ok) error += " cannot set stop bits";
        ok = comm_.setParity(serial::SerialComm::EVEN_PARITY); 
        all_ok &= ok;
        if (!ok) error += " cannot set parity";
        ok = comm_.initRawComm(status);
        all_ok &= ok;
        if (!ok) error += " cannot init comm";
        ok = comm_.setReadTimeout(2000);
        all_ok &= ok;
        if (!ok) error += " cannot set read timeout";
        if (!all_ok)
        {
            comm_.closeDevice(status);
            throw SerialConnectionError(error);
        }
        connected_ = true;
    }
    initRobot();
}

void melfa::Melfa::disconnect()
{
    boost::mutex::scoped_lock connection_lock(connection_mutex_);
    if (connected_)
    {
        deInitRobot();
        boost::mutex::scoped_lock lock(comm_mutex_);
        int status;
        comm_.closeDevice(status);
        connected_ = false;
    }
}

bool melfa::Melfa::isBusy()
{
    std::vector<std::string> state_msg = sendCommand("1;1;STATE");
    std::string busy_flag = state_msg[state_msg.size() - 1];
    if (busy_flag == "1")
    {
        return true;
    }
    else
    {
        return false;
    }
}

melfa::RobotPose melfa::Melfa::getPose()
{
    std::vector<std::string> joint_msg = sendCommand("1;1;JPOSF");
    melfa::RobotPose pose;
    pose.j1 = atof(joint_msg[1].c_str()) / 180.0 * M_PI;
    pose.j2 = atof(joint_msg[3].c_str()) / 180.0 * M_PI;
    pose.j3 = atof(joint_msg[5].c_str()) / 180.0 * M_PI;
    pose.j4 = atof(joint_msg[7].c_str()) / 180.0 * M_PI;
    pose.j5 = atof(joint_msg[9].c_str()) / 180.0 * M_PI;
    pose.j6 = atof(joint_msg[11].c_str()) / 180.0 * M_PI;
    return pose;
}

melfa::ToolPose melfa::Melfa::getToolPose()
{
    std::vector<std::string> pose_msg = sendCommand("1;1;PPOSF");
    if (pose_msg.size() < 12)
    {
        throw RobotError("Melfa::getToolPose(): Robot answer too small!");
    }
    melfa::ToolPose pose;
    pose.x = atof(pose_msg[1].c_str()) / 1000;
    pose.y = atof(pose_msg[3].c_str()) / 1000;
    pose.z = atof(pose_msg[5].c_str()) / 1000;
    pose.roll = atof(pose_msg[7].c_str()) / 180 * M_PI;
    pose.pitch = atof(pose_msg[9].c_str()) / 180 * M_PI;
    pose.yaw = atof(pose_msg[11].c_str()) / 180 * M_PI;
    return pose;
}

void melfa::Melfa::moveTo(const melfa::ToolPose& pose)
{
    execute("PCOSIROP=(" 
            + format(pose.x * 1000) + ","
            + format(pose.y * 1000) + ","
            + format(pose.z * 1000) + ","
            + format(pose.roll / M_PI * 180) + ","
            + format(pose.pitch / M_PI * 180) + ","
            + format(pose.yaw / M_PI * 180) + ")");
    execute("MVS PCOSIROP"); // MVS -> linear interpolation
}

void melfa::Melfa::moveTo(const melfa::RobotPose& pose)
{
    execute("JCOSIROP=(" 
            + format(pose.j1 / M_PI * 180) + ","
            + format(pose.j2 / M_PI * 180) + ","
            + format(pose.j3 / M_PI * 180) + ","
            + format(pose.j4 / M_PI * 180) + ","
            + format(pose.j5 / M_PI * 180) + ","
            + format(pose.j6 / M_PI * 180) + ")");
    execute("MOV JCOSIROP"); // MOV -> joint interpolation
}

void melfa::Melfa::stop()
{
    sendCommand("STOP");
}

void melfa::Melfa::setMaximumVelocity(double velocity)
{
    std::ostringstream command;
    command << "SPD " << format(velocity * 1000);
    execute(command.str());
}

void melfa::Melfa::setAcceleration(double percentage)
{
    std::ostringstream command;
    command << "ACCEL " << percentage;
    execute(command.str());
}

void melfa::Melfa::setTool(double x, double y, double z,
        double roll, double pitch, double yaw)
{
    std::ostringstream command;
    command << "TOOL (" 
            << format(x * 1000) << ","
            << format(y * 1000) << ","
            << format(z * 1000) << ","
            << format(roll / M_PI * 180) << ","
            << format(pitch / M_PI * 180) << ","
            << format(yaw / M_PI * 180) << ")";
    execute(command.str());
}

void melfa::Melfa::execute(const std::string& command)
{
    sendCommand("1;1;EXEC" + command);
}

std::vector<std::string> melfa::Melfa::sendCommand(const std::string& command)
{
    boost::mutex::scoped_lock lock(comm_mutex_);
    write(command + "\r");
    std::string answer = read();
    checkAnswer(answer);
    return parseAnswer(answer);
}

void melfa::Melfa::initRobot()
{
    sendCommand("1;1;OPEN=NARCUSR");
    sendCommand("1;1;CNTLON");
    sendCommand("1;1;RSTPRG"); // or SLOTINIT?
    sendCommand("1;1;PRGLOAD=COSIROP"); // we have to load an empty program
    sendCommand("1;1;OVRD=50");
    sendCommand("1;1;SRVON");
}

void melfa::Melfa::deInitRobot()
{
    sendCommand("1;1;SRVOFF");
    sendCommand("1;1;CNTLOFF");
    sendCommand("1;1;CLOSE");
}

void melfa::Melfa::write(const std::string& data)
{
    if (!connected_)
        throw SerialConnectionError("Melfa::write(): Robot not connected!");
    unsigned long num_bytes_written;
    int status;
    bool write_ok = comm_.writeData(data.length(),
            data.c_str(), num_bytes_written, status);
    if (!write_ok || num_bytes_written != data.length())
    {
        throw SerialConnectionError("Melfa::send(): error writing to device!");
    }
    std::cout << "written: " << data << std::endl;
}

std::string melfa::Melfa::read()
{
    if (!connected_)
        throw SerialConnectionError("Melfa::read(): Robot not connected!");
    int buffer_size = 4096;
    char buffer[buffer_size];
    long unsigned int total_num_bytes_read = 0;
    std::string answer;
    bool end_found = false;
    while (!end_found)
    {
        long unsigned int num_bytes_read;
        int status;
        bool read_ok = comm_.readData(buffer_size, buffer, num_bytes_read, status);
        if (!read_ok)
        {
            throw SerialConnectionError("Melfa::read(): error reading from device!");
        }
        std::string buffer_string(buffer, num_bytes_read);
        answer += buffer_string;
        total_num_bytes_read += num_bytes_read;
        // message end marker is '\r'
        if (answer.length() > 0 && answer[answer.length() - 1] == '\r')
            end_found = true;
    } 
    std::cout << "read: " << answer << std::endl;
    return answer;
}

void melfa::Melfa::checkAnswer(const std::string& answer)
{
    if (answer.size() < 4)
        throw RobotError("Melfa::checkAnswer(): Robot answer too small!");
    if (answer[answer.length() - 1] != '\r')
        throw RobotError("Melfa::checkAnswer(): no end marker in answer!");

    if (answer.substr(0, 3) == "QoK")
    {
        return;
    }
    std::string error("Melfa::checkAnswer(): ");
    if (answer[2] == 'k' || answer[2] == 'r')
    {
        error += " Robot in error state.";
        error += " (answer was " + answer + ")";
    }
    if (answer[2] == 'R' || answer[2] == 'r')
    {
        error += " Error no ";
        std::string errorno = answer.substr(3, 4);
        error += errorno;
        write("1;1;ERRORMES" + errorno + "\r");
        error += " " + read().substr(3);
        if (errorno == "5640")
        {
            throw RobotBusyException(error);
        }
        /* TODO: fill in right error number!
        else if (errorno == "")
        {
            throw TargetUnreachableException(error);
        }
        */
    }
    throw RobotError(error);
}

std::vector<std::string> melfa::Melfa::parseAnswer(const std::string& answer) const
{
    std::vector<std::string> elements;
    int start;
    if (answer[3] == ';')
        start = 4;
    else
        start = 3;
    // -1 here to strip the trailing '\r'
    std::stringstream ss(answer.substr(start, answer.length() - start - 1));
    std::string item;
    while(std::getline(ss, item, ';')) {
        elements.push_back(item);
    }
    return elements;
}

