#include <ros/ros.h>
#include <sstream>

#include "exceptions.h"
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

void ac::Melfa::connect()
{
    int status;
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

void ac::Melfa::init()
{
    sendRawCommand("OPEN=NARCUSR", 1);
    sendRawCommand("CNTLON", 1);
    sendRawCommand("RSTPRG", 1);
    sendRawCommand("PRGLOAD=COSIROP", 1);
}

void ac::Melfa::disconnect()
{
    if (connected_)
    {
        sendRawCommand("CNTLOFF", 1);
        sendRawCommand("CLOSE", 1);
        int status;
        comm_.closeDevice(status);
        connected_ = false;
    }
}

bool ac::Melfa::isBusy()
{
    sendRawCommand("STATE", 1);
    std::string answer;
    readAnswer(answer, 53);
    checkAnswer(answer);
    if (answer[answer.length() - 1] == '0')
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ac::Melfa::execute(const std::string& command)
{
   sendRawCommand("EXEC" + command, 1);
}

void ac::Melfa::runProgram(const std::string& command)
{
    throw MelfaException("Melfa::runProgram(): not implemented!");
}

void ac::Melfa::sendRawCommand(const std::string& command, int slot)
{
    if (!connected_)
        throw MelfaRobotError("Melfa::sendRawCommand(): Robot not connected!");
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
        throw MelfaSerialConnectionError("Melfa::sendRawCommand(): error writing to device!");
    }
    else
    {
        ROS_INFO_STREAM("Melfa::sendRawCommand(): sent: " << command_string);
    }
}

void ac::Melfa::readAnswer(std::string& answer, long unsigned int min_num_bytes)
{
    if (!connected_)
        throw MelfaRobotError("Melfa::readAnswer(): Robot not connected!");
    answer = "";
    // there is no specific end marker in messages from the robot :-(
    int buffer_size = 4096;
    char buffer[buffer_size];
    long unsigned int total_num_bytes_read = 0;
    while (total_num_bytes_read < min_num_bytes)
    {
        long unsigned int num_bytes_to_read = buffer_size;
        long unsigned int num_bytes_read;
        int status;
        bool read_ok = comm_.readData(num_bytes_to_read, buffer, 
                num_bytes_read, status);
        if (!read_ok)
        {
            throw MelfaSerialConnectionError("Melfa::readAnswer(): error reading from device!");
        }
        std::string buffer_string(buffer, num_bytes_read);
        ROS_INFO_STREAM("Melfa::readAnswer(): received: " << buffer_string);
        answer += buffer_string;
        total_num_bytes_read += num_bytes_read;
    }
}

void ac::Melfa::checkAnswer(const std::string& answer)
{
    if (answer[0] == 'Q' && answer[1] == 'o' && answer[2] == 'K')
    {
        return;
    }
    std::string error;
    if (answer[2] == 'k' || answer[2] == 'r')
    {
        error += " Robot in error state.";
    }
    if (answer[2] == 'R' || answer[2] == 'r')
    {
        error += " Illegal data, error no ";
        error += answer.substr(3, 4);
    }
    throw MelfaRobotError(error);
}


