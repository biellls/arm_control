#include <ros/ros.h>
#include <sstream>

#include "melfa.h"

namespace ac = arm_control;

ac::Melfa::Melfa(const ac::Melfa::ConfigParams& params) :
    params_(params), connected_(false)
{
}

ac::Melfa::~Melfa()
{
    disconnect();
}

bool ac::Melfa::connect()
{
    int status;
    bool ok = comm_.openDevice(params_.device, status);
    if (ok)
    {
        ROS_INFO("Melfa::connect(): device opened.");
    }
    else
    {
        ROS_ERROR_STREAM("Melfa::connect(): could not open device " << params_.device << ", returned status is " << status);
        return false;
    }

    ok &= comm_.setBaudRate(9600);
    ok &= comm_.setDataBits(serial::SerialComm::DB8);
    ok &= comm_.setStopBits(serial::SerialComm::TWO_STOP_BITS);
    ok &= comm_.setParity(serial::SerialComm::EVEN_PARITY);
    ok &= comm_.initRawComm(status);
    ok &= comm_.setReadTimeout(2000);
    if (ok)
    {
        ROS_INFO("Melfa::connect(): connection parameters set.");
    }
    else
    {
        ROS_ERROR("Melfa::connect(): Error setting connection parameters.");
        comm_.closeDevice(status);
        return false;
    }

    if (!sendRawCommand("CNTLON", 1))
    {
        ROS_ERROR("Melfa::connect(): Cannot gain control.");
        comm_.closeDevice(status);
        return false;
    }
    connected_ = true;
    return true;
}

void ac::Melfa::disconnect()
{
    if (connected_)
    {
        sendRawCommand("CNTLOFF", 1);
        int status;
        comm_.closeDevice(status);
        connected_ = false;
    }
}

bool ac::Melfa::execute(const std::string& command)
{
    if (!connected_)
        return false;
    return sendRawCommand("EXEC" + command, 9);
}

bool ac::Melfa::runProgram(const std::string& command)
{
    if (!connected_)
        return false;
    return false;
}

bool ac::Melfa::sendRawCommand(const std::string& command, int slot)
{
    std::stringstream stream;
    stream << "1;" << slot << ";" << command << "\r";
    std::string command_string = stream.str().c_str();
    long unsigned int num_bytes_to_write = command_string.length();
    long unsigned int num_bytes_written;
    int status;
    bool write_ok = comm_.writeData(num_bytes_to_write,
            command_string.c_str(), num_bytes_written, status);
    if (!write_ok || num_bytes_written != num_bytes_to_write)
    {
        ROS_ERROR("Melfa::sendRawCommand(): error writing to device!");
        return false;
    }


    // we only use a single read here because the answer is small.
    // there is no specific end marker in messages from the robot :-(

    // check for answer, must be Qok
    char buffer[4096];
    long unsigned int num_bytes_to_read = 4096;
    long unsigned int num_bytes_read;
    bool read_ok = comm_.readData(num_bytes_to_read, buffer, 
            num_bytes_read, status);
    if (!read_ok || num_bytes_read < 3)
    {
        ROS_ERROR("Melfa::sendRawCommand(): error reading answer!");
        return false;
    }
    if (buffer[0] == 'Q' && buffer[1] == 'o' && buffer[2] == 'K')
    {
        return true;
    }
    if (buffer[2] == 'k' || buffer[2] == 'r')
    {
        ROS_ERROR("Melfa::sendRawCommand(): Robot in error status!");
    }
    if (buffer[2] == 'R' || buffer[2] == 'r')
    {
        ROS_ERROR_STREAM("Melfa::sendRawCommand(): Robot answer is "
                "\"illigal data\" with error no " << buffer[3] 
                << buffer[4] << buffer[5] << buffer[6]);
    }
    return false;
}

