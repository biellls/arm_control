#include <stdexcept>

namespace arm_control
{

/// \brief base exception
class MelfaException : public std::runtime_error
{
    public:
        MelfaException(const std::string& what) : std::runtime_error(what) {}
};

class MelfaSerialConnectionError : public MelfaException
{
    public:
        MelfaSerialConnectionError(const std::string& what) : MelfaException(what) {}
};

class MelfaRobotError : public MelfaException
{
    public:
        MelfaRobotError(const std::string& what) : MelfaException(what) {}
};

class MelfaRobotBusyException : public MelfaRobotError
{
    public:
        MelfaRobotBusyException(const std::string& what) : MelfaRobotError(what) {}
};


}
