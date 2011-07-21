#include <ros/ros.h>

#include "melfa.h"

namespace ac = arm_control;

ac::Melfa::Melfa(const ac::Melfa::ConfigParams& params) :
    params_(params), connected_(false)
{
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

    connected_ = true;
    return true;
}

void ac::Melfa::disconnect()
{
    int status;
    comm_.closeDevice(status);
    connected_ = false;
}

void ac::Melfa::execute(const std::string& command)
{
    if (!connected_)
        return;
}

void ac::Melfa::runProgram(const std::string& command)
{
    if (!connected_)
        return;
}

