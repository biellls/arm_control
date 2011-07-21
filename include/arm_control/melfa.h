#ifndef MELFA_H
#define MELFA_H

#include <string>

#include "serialcomm.h"

namespace arm_control
{

/**
* \class Melfa
* \brief Communication with a melfa robot
*/
class Melfa
{
    public:

        /**
        * Defines configuration parameters
        */
        struct ConfigParams
        {
            std::string device;
        };

        Melfa(const ConfigParams& params);

        /**
        * \brief connects to the robot
        * \return true on success, false otherwise
        */
        bool connect();

        /**
        * \disconnects from the robot
        */
        void disconnect();

        /**
        * \brief runs a MELFA BASIC IV program given by file_name
        */
        void runProgram(const std::string& file_name);

        /**
        * \brief tells the robot to execute a MELFA BASIC IV command
        */
        void execute(const std::string& command);

    private:

        ConfigParams params_;
        bool connected_;
        serial::SerialComm comm_;
};

}

#endif

