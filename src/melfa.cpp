#include <ros/ros.h>
#include <sstream>

#include "exceptions.h"
#include "melfa.h"

namespace ac = arm_control;

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
    initRobot();
}

void ac::Melfa::disconnect()
{
    if (connected_)
    {
        deInitRobot();
        int status;
        comm_.closeDevice(status);
        connected_ = false;
    }
}

bool ac::Melfa::isBusy()
{
    send("1;1;STATE\r");
    std::string answer = receive();
    checkAnswer(answer);
    if (answer[answer.length() - 1] == '1')
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ac::Melfa::getPose(double& x, double& y, double& z,
        double& roll, double& pitch, double& yaw)
{
    readPos("1", x);
    readPos("2", y);
    readPos("3", z);
    readPos("4", roll);
    readPos("5", pitch);
    readPos("6", yaw);
}

void ac::Melfa::execute(const std::string& command)
{
    sendRawCommand("EXEC" + command, 1);
}

void ac::Melfa::sendRawCommand(const std::string& command, int slot)
{
    std::stringstream stream;
    stream << "1;" << slot << ";" << command << "\r";
    send(stream.str());
    std::string answer = receive();
    checkAnswer(answer);
}

void ac::Melfa::initRobot()
{
    sendRawCommand("OPEN=NARCUSR", 1);
    sendRawCommand("CNTLON", 1);
    sendRawCommand("RSTPRG", 1);
    sendRawCommand("PRGLOAD=COSIROP", 1);
    sendRawCommand("OVRD=3", 1);
}

void ac::Melfa::deInitRobot()
{
    sendRawCommand("CNTLOFF", 1);
    sendRawCommand("CLOSE", 1);
}

void ac::Melfa::readPos(const std::string& id, double& pos)
{
    // answer of robot is
    // QoK<axis name>;<position>;;.....
    // example:
    // QoKX;526.34;;6.0;10;0.00;00000000
    send("1;1;PPOS" + id + "\r");
    std::string answer = receive();
    checkAnswer(answer);
    int start_pos = 5;
    int end_pos = answer.find(";", start_pos);
    int length = end_pos - start_pos;
    std::string num_string = answer.substr(start_pos, length);
    std::istringstream iss(num_string);
    iss >> pos;
}

void ac::Melfa::send(const std::string& command)
{
    if (!connected_)
        throw MelfaSerialConnectionError("Melfa::send(): Robot not connected!");
    long unsigned int num_bytes_to_write = command.length();
    long unsigned int num_bytes_written;
    int status;
    bool write_ok = comm_.writeData(num_bytes_to_write,
            command.c_str(), num_bytes_written, status);
    if (!write_ok || num_bytes_written != num_bytes_to_write)
    {
        throw MelfaSerialConnectionError("Melfa::send(): error writing to device!");
    }
    else
    {
        ROS_INFO_STREAM("Melfa::send(): sent: " << command);
    }
}

std::string ac::Melfa::receive()
{
    if (!connected_)
        throw MelfaSerialConnectionError("Melfa::receive(): Robot not connected!");
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
            throw MelfaSerialConnectionError("Melfa::readAnswer(): error reading from device!");
        }
        std::string buffer_string(buffer, num_bytes_read);
        answer += buffer_string;
        total_num_bytes_read += num_bytes_read;
        // message end marker is '\r'
        if (answer.length() > 0 && answer[answer.length() - 1] == '\r')
            end_found = true;
    } 
    return answer.substr(0, answer.length() - 1);
}

void ac::Melfa::checkAnswer(const std::string& answer)
{
    if (answer.size() < 3)
        throw MelfaRobotError("Robot answer too small!");
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
        std::string errorno = answer.substr(3, 4);
        error += errorno;
        send("1;1;ERRORMES" + errorno + "\r");
        error += " " + receive().substr(3);
    }
    throw MelfaRobotError(error);
}


