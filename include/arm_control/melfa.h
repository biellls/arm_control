#ifndef MELFA_H
#define MELFA_H

#include <string>

#include "serialcomm.h"

namespace arm_control
{

/**
* \class Melfa
* \brief Communication with a melfa robot
* Errors are communicated via exceptions (see MelfaException and subclasses)
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
        */
        void connect();

        /**
        * \brief initializes the robot. call after connect.
        */
        void init();

        /**
        * \brief disconnects from the robot
        */
        void disconnect();

        /**
        * \brief asks the robot for its state and returns true if
        * it indicates that it is busy
        */
        bool isBusy();

        /**
        * \brief executes the given MELFA BASIC IV command
        */
        void execute(const std::string& command);

        /**
        * \brief runs a MELFA BASIC IV program given by file_name
        */
        void runProgram(const std::string& file_name);

        /**
        * \brief sends the give command directly to the robot on given slot
        * Carriage return at end of command is added inside this method.
        */
        void sendRawCommand(const std::string& command, int slot);

        /**
        * \brief reads an answer from the robot. This method blocks until
        * at least min_num_bytes are received.
        */
        void readAnswer(std::string& answer, long unsigned int min_num_bytes = 3);

        /**
        * \brief checks if the given answer is valid (starts with "QoK")
        */
        void checkAnswer(const std::string& answer);

    private:

        ConfigParams params_;
        bool connected_;
        serial::SerialComm comm_;
};

}

#endif

