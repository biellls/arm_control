#include <fstream>
#include <iostream>

#include "melfa.h"

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

    std::ifstream in(program_file_name.c_str());
    if (!in.is_open())
    {
        std::cerr << "cannot open file " << program_file_name << "." << std::endl;
        return -3;
    }
 
    arm_control::Melfa::ConfigParams params;
    params.device = std::string(argv[1]);

    arm_control::Melfa melfa(params);
    
    if (!melfa.connect())
    {
        std::cerr << "Connection attempt failed!" << std::endl;
        return -2;
    }

    std::string command;
    while (getline(in, command))
    {
        std::cout << "executing: " << command << std::endl;
        if (!melfa.execute(command))
        {
            std::cerr << "error during execution!" << std::endl;
            break;
        }
    }

    return 0;
}

