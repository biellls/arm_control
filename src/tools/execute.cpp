
#include <iostream>

#include "melfa.h"

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] 
            << " <device name> <MELFA BASIC IV command>" << std::endl;
        std::cerr << "Use cuotes for the command." << std::endl;
        return -1;
    }

    std::string device_name(argv[1]);
    std::string command(argv[2]);

    arm_control::Melfa::ConfigParams params;
    params.device = std::string(argv[1]);

    arm_control::Melfa melfa(params);
    if (melfa.connect())
    {
        melfa.execute(command);
    }
    else
    {
        std::cerr << "Connection attempt failed!" << std::endl;
        return -2;
    }

    return 0;
}

