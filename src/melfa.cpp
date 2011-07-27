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
    std::vector<std::string> state_msg = sendCommand("1;1;STATE");
    if (state_msg[state_msg.size() - 1] == "1")
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
    std::vector<std::string> pose_msg = sendCommand("1;1;PPOSF");
    for (size_t i = 0; i < pose_msg.size(); ++i)
    {
        std::cout << i << ": " << pose_msg[i] << std::endl;
    }
}

void ac::Melfa::execute(const std::string& command)
{
    sendCommand("1;1;EXEC" + command);
}

std::vector<std::string> ac::Melfa::sendCommand(const std::string& command)
{
    write(command + "\r");
    std::string answer = read();
    checkAnswer(answer);
    return parseAnswer(answer);
}

void ac::Melfa::initRobot()
{
    sendCommand("1;1;OPEN=NARCUSR");
    sendCommand("1;1;CNTLON");
    sendCommand("1;1;RSTPRG");
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
    }
    throw MelfaRobotError(error);
}

std::vector<std::string> ac::Melfa::parseAnswer(const std::string& answer) const
{
    std::vector<std::string> elements;
    std::stringstream ss(answer.substr(4)); // skip leading "QoK"
    std::string item;
    while(std::getline(ss, item, ';')) {
        elements.push_back(item);
    }
    return elements;
}



