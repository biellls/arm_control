#include <ros/ros.h>
#include <sstream>
#include <iomanip>

#include "robot_pose.h"
#include "exceptions.h"
#include "melfa.h"

namespace ac = arm_control;


// formats the value to have 2 digits after the comma
std::string format(double val)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << val;
    return ss.str();
}

ac::Melfa::Melfa() : connected_(false)
{
}

ac::Melfa::Melfa(const ac::Melfa::ConfigParams& params) :
    params_(params), connected_(false)
{
}

void ac::Melfa::setParams(const ac::Melfa::ConfigParams& params)
{
    params_ = params;
}

ac::Melfa::~Melfa()
{
    disconnect();
}

void ac::Melfa::connect()
{
    boost::mutex::scoped_lock connection_lock(connection_mutex_);
    int status;
    {
        boost::mutex::scoped_lock lock(comm_mutex_);
        if (!comm_.openDevice(params_.device, status))
            throw MelfaSerialConnectionError("Melfa::connect(): could not open device.");

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
            throw MelfaSerialConnectionError(error);
        }
        connected_ = true;
    }
    initRobot();
}

void ac::Melfa::disconnect()
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

bool ac::Melfa::isBusy()
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

ac::RobotPose ac::Melfa::getPose()
{
    std::vector<std::string> pose_msg = sendCommand("1;1;PPOSF");
    if (pose_msg.size() < 12)
    {
        throw MelfaRobotError("Melfa::getPose(): Robot answer too small!");
    }
    ac::RobotPose pose;
    pose.x = atof(pose_msg[1].c_str()) / 1000;
    pose.y = atof(pose_msg[3].c_str()) / 1000;
    pose.z = atof(pose_msg[5].c_str()) / 1000;
    pose.roll = atof(pose_msg[7].c_str()) / 180 * M_PI;
    pose.pitch = atof(pose_msg[9].c_str()) / 180 * M_PI;
    pose.yaw = atof(pose_msg[11].c_str()) / 180 * M_PI;
    return pose;
}

void ac::Melfa::moveTo(const ac::RobotPose& pose)
{
    execute("P1.X=" + format(pose.x * 1000));
    execute("P1.Y=" + format(pose.y * 1000));
    execute("P1.Z=" + format(pose.z * 1000));
    execute("P1.A=" + format(pose.roll / M_PI * 180));
    execute("P1.B=" + format(pose.pitch / M_PI * 180));
    execute("P1.C=" + format(pose.yaw / M_PI * 180));
    execute("MVS P1");
}

void ac::Melfa::stop()
{
    sendCommand("STOP");
}

void ac::Melfa::setMaximumVelocity(double velocity)
{
    std::ostringstream command;
    command << "SPD " << format(velocity * 1000);
    execute(command.str());
}

void ac::Melfa::setAcceleration(double percentage)
{
    std::ostringstream command;
    command << "ACCEL " << percentage;
    execute(command.str());
}

void ac::Melfa::setToolPose(const ac::RobotPose& pose)
{
    std::ostringstream command;
    command << "TOOL (" 
            << format(pose.x * 1000) << ","
            << format(pose.y * 1000) << ","
            << format(pose.z * 1000) << ","
            << format(pose.roll / M_PI * 180) << ","
            << format(pose.pitch / M_PI * 180) << ","
            << format(pose.yaw / M_PI * 180) << ")";
    execute(command.str());
}

void ac::Melfa::execute(const std::string& command)
{
    sendCommand("1;1;EXEC" + command);
}

std::vector<std::string> ac::Melfa::sendCommand(const std::string& command)
{
    boost::mutex::scoped_lock lock(comm_mutex_);
    write(command + "\r");
    std::string answer = read();
    checkAnswer(answer);
    return parseAnswer(answer);
}

void ac::Melfa::initRobot()
{
    sendCommand("1;1;OPEN=NARCUSR");
    sendCommand("1;1;CNTLON");
    sendCommand("1;1;RSTPRG"); // or SLOTINIT?
    sendCommand("1;1;PRGLOAD=COSIROP");
    sendCommand("1;1;OVRD=3");
}

void ac::Melfa::deInitRobot()
{
    sendCommand("1;1;CNTLOFF");
    sendCommand("1;1;CLOSE");
}

void ac::Melfa::write(const std::string& data)
{
    if (!connected_)
        throw MelfaSerialConnectionError("Melfa::write(): Robot not connected!");
    unsigned long num_bytes_written;
    int status;
    bool write_ok = comm_.writeData(data.length(),
            data.c_str(), num_bytes_written, status);
    if (!write_ok || num_bytes_written != data.length())
    {
        throw MelfaSerialConnectionError("Melfa::send(): error writing to device!");
    }
}

std::string ac::Melfa::read()
{
    if (!connected_)
        throw MelfaSerialConnectionError("Melfa::read(): Robot not connected!");
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
            throw MelfaSerialConnectionError("Melfa::read(): error reading from device!");
        }
        std::string buffer_string(buffer, num_bytes_read);
        answer += buffer_string;
        total_num_bytes_read += num_bytes_read;
        // message end marker is '\r'
        if (answer.length() > 0 && answer[answer.length() - 1] == '\r')
            end_found = true;
    } 
    return answer;
}

void ac::Melfa::checkAnswer(const std::string& answer)
{
    if (answer.size() < 4)
        throw MelfaRobotError("Melfa::checkAnswer(): Robot answer too small!");
    if (answer[answer.length() - 1] != '\r')
        throw MelfaRobotError("Melfa::checkAnswer(): no end marker in answer!");

    if (answer.substr(0, 3) == "QoK")
    {
        return;
    }
    std::string error("Melfa::checkAnswer(): ");
    if (answer[2] == 'k' || answer[2] == 'r')
    {
        error += " Robot in error state.";
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
            throw MelfaRobotBusyException(error);
        }
    }
    throw MelfaRobotError(error);
}

std::vector<std::string> ac::Melfa::parseAnswer(const std::string& answer) const
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

