/******************************************************************************
 * Class: SerialComm
 * Author: Joan Pau on original version from ckonvalin
 * Created: 11/13/05
 *
 * Purpose:
 		Establish Serial communication with device.
 *****************************************************************************/

#include "serialcomm.h"
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>

///////////////////////////////////////////////////////////////////////////////
using namespace serial;
///////////////////////////////////////////////////////////////////////////////

const int SerialComm::SERIAL_NO_ERROR = 0;

SerialComm::SerialComm(void) :
  m_deviceName(""),
  m_deviceFd(-1),
  m_deviceOpen(false)
{
}

SerialComm::~SerialComm(void)
{
    int status;

    closeDevice(status);
}

std::string SerialComm::getDeviceName()
{
    return m_deviceName;
}

bool SerialComm::openDevice(const std::string& name, int & status)
{
    bool success = true;

    status = SERIAL_NO_ERROR;

    m_deviceName = name;

    // see if a device is already open.  If so, close, then open new device.
    if(m_deviceOpen)
        closeDevice(status);

    /**
     * fd = open(char* name, O_RDWR | O_NOCTTY | O_NONBLOCK ) :
     * open device name for:
     *    1 read and write,
     *    2 no control terminal (keyboard does not affect communication)
     *    3 do not wait for device answer
     **/
    m_deviceFd = open(
        m_deviceName.c_str(),
        O_RDWR | O_NOCTTY );

    if(m_deviceFd < 0)
    {
        status = errno;
        success = checkStatus(status);
    }
    else
    {
        m_deviceOpen = true;
        success = initRawComm(status);
    }

    return(success);
}

bool SerialComm::initRawComm(int & status)
{
    bool success = false;

    termios tc;

    if ( getAttr(tc) )
    {
      tc.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
      tc.c_oflag &= ~OPOST;
      tc.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
      tc.c_cflag &= ~(CRTSCTS|HUPCL);
      tc.c_cflag |= (CREAD|CLOCAL);

      success = setAttr(tc);
    }
    status = errno;
    return(success);
}

bool SerialComm::closeDevice(int & status)
{
    status = SERIAL_NO_ERROR;

    if(m_deviceOpen)
    {
        if( close(m_deviceFd) < 0 )
            status = errno;

        m_deviceOpen = false;
    }

    return(checkStatus(status));
}

bool SerialComm::readData
(
    unsigned long numBytesToRead,
    char * buffer,
    unsigned long & numBytesRead,
    int & status
)
{
    bool success;
    status = SERIAL_NO_ERROR;
    long readResult;
    readResult = read(m_deviceFd, buffer, numBytesToRead);
    if( readResult < 0 )
    {
        numBytesRead = 0;
        status = errno;
        success = checkStatus(status);
    }
    else
    {
        numBytesRead = readResult;
        success = (numBytesToRead == numBytesRead);
    }

    return(success);
}

bool SerialComm::writeData
(
    unsigned long numBytesToWrite,
    const char * buffer,
    unsigned long & numBytesWritten,
    int & status
)
{
    bool success;
    status = SERIAL_NO_ERROR;
    long writeResult;

    writeResult = write(m_deviceFd, buffer, numBytesToWrite);
    if( writeResult < 0 )
    {
        numBytesWritten = 0;
        status = errno;
        success = checkStatus(status);
    }
    else
    {
        numBytesWritten = writeResult;
        success = (numBytesToWrite == numBytesWritten);
    }

    return(success);
}

bool SerialComm::flushBuffer(int & status)
{
    status = SERIAL_NO_ERROR;

    if( m_deviceOpen )
    {
        if( tcflush(m_deviceFd, TCIOFLUSH) < 0 ) // it might be tcdrain
            status = errno;
    }

    return(checkStatus(status));
}

bool SerialComm::setBaudRate(unsigned long rate)
{
    bool success = false;

    termios tc;
    if( getAttr(tc) && ( cfsetspeed(&tc, rate) == 0 ) )
    {
        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setDataBits(E_DataBits bits)
{
    bool success = false;

    termios tc;

    if( getAttr(tc) )
    {
        tc.c_cflag &= ~CSIZE;

        switch (bits)
        {
          case DB5:
            tc.c_cflag |= CS5;
            break;

          case DB6:
            tc.c_cflag |= CS6;
            break;

          case DB7:
            tc.c_cflag |= CS7;
            break;

          case DB8:
            tc.c_cflag |= CS8;
            break;
        }

        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setStopBits(E_StopBits bits)
{
    bool success = false;

    termios tc;
    
    if( getAttr(tc) )
    {
        switch(bits)
        {
          case ONE_STOP_BIT:
            tc.c_cflag &= ~CSTOPB;
            break;

          case TWO_STOP_BITS:
            tc.c_cflag |= CSTOPB;
            break;
        }

        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setParity(E_Parity parity)
{
    bool success = false;

    termios tc;

    if( getAttr(tc) )
    {   
        /**
         * Mark/Space parity is not well documented.
         * It may be set with the CMSPAR c_cflag
         * and the other parity flags (PARENB, PARODD).
         * In some systems it might not be available 
         * or not documented in termios.h man page.
         **/

        tc.c_cflag &= ~( PARODD | CMSPAR );

        switch(parity)
        {
        case EVEN_PARITY:
            tc.c_cflag |= PARENB;
            break;

        case MARK_PARITY:
            tc.c_cflag |= PARENB | PARODD | CMSPAR;
            break;

        case NO_PARITY:
            tc.c_cflag &= ~PARENB;
            break;

        case ODD_PARITY:
            tc.c_cflag |= PARENB | PARODD;
            break;

        case SPACE_PARITY:
            tc.c_cflag |= PARENB | CMSPAR;
            break;
        }

        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setReadTimeout(unsigned long time_ms, unsigned long min_bytes)
{
    termios tc;
    bool success = false;

    if(getAttr(tc))
    {
        /**
         * MIN == 0; TIME > 0: TIME specifies the limit for a timer in
         * tenths of a  second.
         * The  timer  is  started when read(2) is called.
         * read(2) returns either when at least one byte of data is
         * available,  or  when the  timer  expires.
         * If the timer expires without any input becoming available,
         * read(2) returns 0.
         * See termios.h man page, non-canonical mode input.
         * c_cc[VMIN] and c_cc[VTIME].
         **/
        tc.c_cc[VMIN] = min_bytes;
        tc.c_cc[VTIME] = time_ms/100;
        success = setAttr(tc);
    }

    return(success);
}


bool SerialComm::getAttr(termios & tc)
{
    bool success = true;
    if( tcgetattr(m_deviceFd, &tc) < 0 )
        success = false;

    return(success);
}

bool SerialComm::setAttr(termios & tc)
{
    bool success = true;
    if( tcsetattr(m_deviceFd, TCSANOW, &tc) < 0 )
        success = false;

    return(success);
}

bool SerialComm::checkStatus(int status)
{
    bool success = true;
    if(status != SERIAL_NO_ERROR)
        success = false;

    return(success);
}

