#ifndef MELFA_H
#define MELFA_H

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#include "serialcomm.h"

namespace arm_control
{

/**
* \class Melfa
* \brief Communication with a melfa robot
* Errors are communicated via exceptions (see MelfaException and subclasses)
* This class is partly thread-safe, i.e. all serial communication is protected by a mutex
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
        * x, y, and z are given in meters, roll, pitch and yaw in radiants
        */
        void getPose(double& x, double& y, double& z, 
                double& roll, double& pitch, double& yaw);

        /**
        * \brief sends a move command to given pose
        * x, y, and z have to be given in meters, roll, pitch, and yaw in radiants
        */
        void moveTo(double x, double y, double z, 
                double roll, double pitch, double yaw);

        /**
        * \brief sends a stop command to the robot
        */
        void stop();

        /**
        * \brief executes the given MELFA BASIC IV command
        */
        void execute(const std::string& command);

        /**
        * \brief top-level low-level communication, calls the following
        *        three methods one after the other: write() read() checkAnswer() parseAnswer()
        * \return parsed checked answer of the robot
        */
        std::vector<std::string> sendCommand(const std::string& command);

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
        void write(const std::string& data);

        /**
        * \brief reads an answer from the robot. This method blocks until
        * the end marker '\r' was retrieved from the robot
        */
        std::string read();

        /**
        * \brief checks if the given answer is valid (starts with "QoK")
        * Throws an exception if the answer indicates an error.
        */
        void checkAnswer(const std::string& answer);

        /**
        * \brief parses the given answer by splitting it using delimiter ";"
        * \return the elements
        */
        std::vector<std::string> parseAnswer(const std::string& answer) const;

    private:

        boost::mutex comm_mutex_; // mutex to protect communication
        boost::mutex connection_mutex_; // mutex to protect connection/disconnection
        ConfigParams params_;
        bool connected_;
        serial::SerialComm comm_;
};

}

#endif

