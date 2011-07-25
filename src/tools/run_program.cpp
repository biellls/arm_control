#include <fstream>
#include <iostream>

#include "melfa.h"
#include "exceptions.h"

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] 
            << " <device name> <program file name>" << std::endl;
        return -1;
    }

    std::string device_name(argv[1]);
    std::string program_file_name(argv[2]);

    arm_control::Melfa::ConfigParams params;
    params.device = std::string(argv[1]);

    arm_control::Melfa melfa(params);

    try
    {
        melfa.connect();
        melfa.init();
        melfa.runProgram(program_file_name);
    }
    catch (arm_control::MelfaSerialConnectionError& err)
    {
        std::cerr << "Serial Connection error: " << err.what() << std::endl;
    }
    catch (arm_control::MelfaRobotError& err)
    {
        std::cerr << "Robot error: " << err.what() << std::endl;
    }

    return 0;
}

