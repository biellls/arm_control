#include <stdexcept>

namespace melfa
{

/// \brief base exception
class MelfaException : public std::runtime_error
{
    public:
        MelfaException(const std::string& what) : std::runtime_error(what) {}
};

class SerialConnectionError : public MelfaException
{
    public:
        SerialConnectionError(const std::string& what) : MelfaException(what) {}
};

class RobotError : public MelfaException
{
    public:
        RobotError(const std::string& what) : MelfaException(what) {}
};

class RobotBusyException : public RobotError
{
    public:
        RobotBusyException(const std::string& what) : RobotError(what) {}
};

class PoseUnreachableException : public RobotError
{
    public:
        PoseUnreachableException(const std::string& what) : RobotError(what) {}
};

}
