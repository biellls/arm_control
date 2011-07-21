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

        /**
        * Create a new Melfa instance with given parameters
        */
        Melfa(const ConfigParams& params);

        /**
        * Destructor disconnects
        */
        ~Melfa();

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
        * \brief executes the given MELFA BASIC IV command
        */
        bool execute(const std::string& command);

        /**
        * \brief runs a MELFA BASIC IV program given by file_name
        */
        bool runProgram(const std::string& file_name);

        /**
        * \brief sends the give command directly to the robot on given slot
        * Carriage return at end of command is added inside this method.
        */
        bool sendRawCommand(const std::string& command, int slot);

    private:

        ConfigParams params_;
        bool connected_;
        serial::SerialComm comm_;
};

}

#endif

