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
        * \brief creates a new Melfa instance with empty parameters.
        * Don't forget to call setParams()!
        */
        Melfa();

        /**
        * Create a new Melfa instance with given parameters
        */
        Melfa(const ConfigParams& params);

        /**
        * \brief set parameters
        */
        void setParams(const ConfigParams& params);

        /**
        * Destructor disconnects
        */
        ~Melfa();

        /**
        * \brief connects to the robot
        */
        void connect();

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
        * \brief retrieve the current pose
        */
        void getPose(double& x, double& y, double& z, 
                double& roll, double& pitch, double& yaw);

        /**
        * \brief executes the given MELFA BASIC IV command
        */
        void execute(const std::string& command);

        /**
        * \brief sends the give command directly to the robot on given slot
        * Carriage return at end of command is added inside this method.
        * \return the answer from the robot
        */
        void sendRawCommand(const std::string& command, int slot);

    protected:

        /**
        * \brief initializes the robot. called in connect.
        */
        void initRobot();

        /**
        * \brief de-initializes the robot. called in disconnect.
        */
        void deInitRobot();

        /**
        * \brief reads a position
        */
        void readPos(const std::string& id, double& pos);

        /**
        * \brief sends given data through the serial connections
        */
        void send(const std::string& data);

        /**
        * \brief reads an answer from the robot. This method blocks until
        * the end marker '\r' was retrieved from the robot
        */
        std::string receive();

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

