/******************************************************************************
 * Class: SerialComm
 * Author: Joan Pau on original version from ckonvalin
 * Created: 11/13/05
 *
 *
 *****************************************************************************/
#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <string>
#include <termios.h>    // brings in Unix serial API

namespace serial
{

class SerialComm
{
public:

    SerialComm(void);
    virtual ~SerialComm(void);

    enum E_StopBits
    {
        ONE_STOP_BIT,
        // ONE_5_STOP_BITS // Not handled on GNU/Linux
        TWO_STOP_BITS
    };

    enum E_Parity
    {
        EVEN_PARITY,
        MARK_PARITY,
        NO_PARITY,
        ODD_PARITY,
        SPACE_PARITY
    };

    enum E_DataBits
    {
        DB5,
        DB6,
        DB7,
        DB8
    };

    std::string getDeviceName();
    bool openDevice(const std::string& name, int & status);
    bool closeDevice(int & status);
    bool readData(unsigned long numBytesToRead, char * buffer,
                unsigned long & numBytesRead, int & status);
    bool writeData(unsigned long numBytesToWrite, const char * buffer,
                 unsigned long & numBytesWritten, int & status);
    bool flushBuffer(int & status);
    
    bool setBaudRate(unsigned long rate);
    bool setDataBits(E_DataBits bits);
    bool setStopBits(E_StopBits bits);
    bool setParity(E_Parity parity);
    bool setReadTimeout(unsigned long time_ms, unsigned long min_bytes=0);
    bool initRawComm(int & status);  // initialize the serial port for raw I/O
    bool getAttr(termios & tc);  // get low level configuration parameters
    bool setAttr(termios & tc);  // set low level configuration parameters

    static const int SERIAL_NO_ERROR; // defined in implementation
    bool checkStatus(int status);     // comparison to SERIAL_NO_ERROR

private:

    std::string m_deviceName;  // serial port device name (including full path)
    int  m_deviceFd;           // file descriptor to current device
    bool m_deviceOpen;         // true if a device is already open

};

} // namespace serial

#endif // SERIALCOMM_H
