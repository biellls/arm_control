
#include <iostream>

#include "melfa/melfa.h"
#include "melfa/exceptions.h"

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

    melfa::Melfa::ConfigParams params;
    params.device = std::string(argv[1]);

    melfa::Melfa melfa(params);
    try
    {
        melfa.connect();
        std::cout << "Robot connected." << std::endl;
        std::cout << "Executing command: " << command << std::endl;
        melfa.execute(command);
        std::cout << "Execution finished." << std::endl;
    }
    catch (melfa::MelfaSerialConnectionError& err)
    {
        std::cerr << "Serial Connection error: " << err.what() << std::endl;
    }
    catch (melfa::MelfaRobotError& err)
    {
        std::cerr << "Robot error: " << err.what() << std::endl;
    }

    return 0;
}

