/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArSerialConnection.h"
#include "ArLog.h"
#include "ariaUtil.h"


AREXPORT ArSerialConnection::ArSerialConnection()
{
  myPort = INVALID_HANDLE_VALUE;
  myBaudRate = 9600;
  myStatus = STATUS_NEVER_OPENED;
  myHardwareControl = false;
  buildStrMap();
}

AREXPORT ArSerialConnection::~ArSerialConnection()
{
  if (myPort != INVALID_HANDLE_VALUE)
    close();
}

void ArSerialConnection::buildStrMap(void)
{
  myStrMap[OPEN_COULD_NOT_OPEN_PORT] = "Could not open serial port.";
  myStrMap[OPEN_COULD_NOT_SET_UP_PORT] = "Could not set up serial port.";
  myStrMap[OPEN_INVALID_BAUD_RATE] = "Baud rate invalid, could not set baud on serial port.";
  myStrMap[OPEN_COULD_NOT_SET_BAUD] = "Could not set baud rate on serial port.";
  myStrMap[OPEN_ALREADY_OPEN] = "Serial port already open.";
}

AREXPORT const char * ArSerialConnection::getOpenMessage(int messageNumber)
{
  return myStrMap[messageNumber].c_str();
}

AREXPORT bool ArSerialConnection::openSimple(void)
{
  if (internalOpen() == 0)
    return true;
  else
    return false;
}

/**
   @param port The serial port to connect to, or NULL which defaults to
   COM1 for windows and /dev/ttyS0 for linux
   @return 0 for success, otherwise one of the open enums
   @see getOpenMessage
*/
AREXPORT void ArSerialConnection::setPort(const char *port)
{
  if (port == NULL)
    myPortName = "COM1";
  else
    myPortName = port;
}

/**
   @return The seiral port to connect to
**/
AREXPORT const char * ArSerialConnection::getPort(void)
{
  return myPortName.c_str();
}

/**
   @param port The serial port to connect to, or NULL which defaults to
   COM1 for windows and /dev/ttyS0 for linux
   @return 0 for success, otherwise one of the open enums
   @see getOpenMessage
*/
AREXPORT int ArSerialConnection::open(const char *port)
{
  setPort(port);
  return internalOpen();
}



AREXPORT int ArSerialConnection::internalOpen(void)
{
  DCB dcb;


  if (myStatus == STATUS_OPEN)
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::open: Serial port already open");
    return OPEN_ALREADY_OPEN;
  }

  myPort = CreateFileA(myPortName.c_str(),
		      GENERIC_READ | GENERIC_WRITE,
		      0,	/* exclusive access  */
		      NULL,	/* no security attrs */
		      OPEN_EXISTING,
		      0,
		      NULL );

  if (myPort == INVALID_HANDLE_VALUE) {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::open: Could not open serial port '%s'",
	       myPortName.c_str());
    return OPEN_COULD_NOT_OPEN_PORT;
  }

  if ( !GetCommState(myPort, &dcb) )
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::open: Could not get port data to set up port");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_UP_PORT;
  }

  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = 0;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fNull = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX =  FALSE;

  if ( !SetCommState(myPort, &dcb) )
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::open: Could not set up port");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_UP_PORT;
  }

  myStatus = STATUS_OPEN;

  if (!setBaud(myBaudRate))
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::open: Could not set baud rate.");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_BAUD;
  }

  if (!setHardwareControl(myHardwareControl))
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::open: Could not set hardware control.");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_UP_PORT;
  }

  ArLog::log(ArLog::Verbose,
	     "ArSerialConnection::open: Successfully opened and configured serial port.");
  return 0;
}



AREXPORT bool ArSerialConnection::close(void)
{
  bool ret;

  if (myPort == INVALID_HANDLE_VALUE)
    return true;

  /* disable event notification  */
  SetCommMask( myPort, 0 ) ;
  /* drop DTR	*/
  EscapeCommFunction( myPort, CLRDTR ) ;
  /* purge any outstanding reads/writes and close device handle  */
  PurgeComm( myPort, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );

  myStatus = STATUS_CLOSED_NORMALLY;

  ret = CloseHandle( myPort ) ;
  if (ret)
    ArLog::log(ArLog::Verbose,
	       "ArSerialConnection::close: Successfully closed serial port.");
  else
    ArLog::log(ArLog::Verbose,
	       "ArSerialConnection::close: Unsuccessfully closed serial port.");
  myPort = (HANDLE) INVALID_HANDLE_VALUE;
  return ret;
}

AREXPORT int ArSerialConnection::getBaud(void)
{
   return myBaudRate;
}

AREXPORT bool ArSerialConnection::setBaud(int baud)
{
  DCB dcb;

  myBaudRate = baud;

  if (getStatus() != STATUS_OPEN)
    return true;

  if ( !GetCommState(myPort, &dcb) )
  {
    ArLog::log(ArLog::Terse, "ArSerialConnection::setBaud: Could not get port Data.");
    return false;
  }

  dcb.BaudRate = myBaudRate;

  if ( !SetCommState(myPort, &dcb) )
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::setBaud: Could not set port Data.");
    return false;
  }

  return true;
}

AREXPORT bool ArSerialConnection::getHardwareControl(void)
{
  return myHardwareControl;
}

AREXPORT bool ArSerialConnection::setHardwareControl(bool hardwareControl)
{
  DCB dcb;

  myHardwareControl = hardwareControl;

  if (getStatus() != STATUS_OPEN)
    return true;

  if ( !GetCommState(myPort, &dcb) )
  {
    ArLog::log(ArLog::Terse,
	       "ArSerialConnection::setBaud: Could not get port Data.");
    return false;
  }

  if (myHardwareControl == 0) /* set control lines */
  {
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
  }
  else
  {
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
  }

  if ( !SetCommState(myPort, &dcb) )
  {
    ArLog::log(ArLog::Terse, "ArSerialConnection::setBaud: Could not set port Data.");
    return false;
  }

  return true;
}

AREXPORT int ArSerialConnection::write(const char *data, unsigned int size)
{
  unsigned long ret;

  if (myPort != INVALID_HANDLE_VALUE && myStatus == STATUS_OPEN)
  {
    if (!WriteFile(myPort, data, size, &ret, NULL))
    {
      ArLog::log(ArLog::Terse, "ArSerialConnection::write: Error on writing.");
      perror("ArSerialConnection::write:");
      return -1;
    }
    return ret;
  }
  ArLog::log(ArLog::Terse, "ArSerialConnection::write: Connection invalid.");
  return -1;
}

AREXPORT int ArSerialConnection::read(const char *data, unsigned int size,
				      unsigned int msWait)
{
  COMSTAT stat;
  unsigned long ret;
  unsigned int numToRead;
  ArTime timeDone;

  if (myPort != INVALID_HANDLE_VALUE && myStatus == STATUS_OPEN)
  {
    if (msWait > 0)
    {
      timeDone.setToNow();
      timeDone.addMSec(msWait);
      while (timeDone.mSecTo() >= 0)
      {
	if (!ClearCommError(myPort, &ret, &stat))
	  return -1;
	if (stat.cbInQue < size)
	  ArUtil::sleep(2);
	else
	  break;
      }
    }
    if (!ClearCommError(myPort, &ret, &stat))
      return -1;
    if (stat.cbInQue == 0)
      return 0;
    if (stat.cbInQue > size)
      numToRead = size;
    else
      numToRead = stat.cbInQue;
    if (ReadFile( myPort, (void *)data, numToRead, &ret, NULL))
    {
      return (int)ret;
    }
    else
    {
      ArLog::log(ArLog::Terse, "ArSerialConnection::read:  Read failed.");
      return -1;
    }
  }
  ArLog::log(ArLog::Terse, "ArSerialConnection::read: Connection invalid.");
  return -1;
}


AREXPORT int ArSerialConnection::getStatus(void)
{
  return myStatus;
}

AREXPORT bool ArSerialConnection::isTimeStamping(void)
{
  return false;
}

AREXPORT ArTime ArSerialConnection::getTimeRead(int index)
{
  ArTime now;
  now.setToNow();
  return now;
}

AREXPORT bool ArSerialConnection::getCTS(void)
{
  DWORD modemStat;
  if (GetCommModemStatus(myPort, &modemStat))
  {
    return (bool) (modemStat & MS_CTS_ON);
  }
  else
  {
    fprintf(stderr, "problem with GetCommModemStatus\n");
    return false;
  }
}

AREXPORT bool ArSerialConnection::getDSR(void)
{
  DWORD modemStat;
  if (GetCommModemStatus(myPort, &modemStat))
  {
    return (bool) (modemStat & MS_DSR_ON);
  }
  else
  {
    fprintf(stderr, "problem with GetCommModemStatus\n");
    return false;
  }
}

AREXPORT bool ArSerialConnection::getDCD(void)
{
  DWORD modemStat;
  if (GetCommModemStatus(myPort, &modemStat))
  {
    return (bool) (modemStat & MS_RLSD_ON);
  }
  else
  {
    fprintf(stderr, "problem with GetCommModemStatus\n");
    return false;
  }
}

AREXPORT bool ArSerialConnection::getRing(void)
{
  DWORD modemStat;
  if (GetCommModemStatus(myPort, &modemStat))
  {
    return (bool) (modemStat & MS_RING_ON);
  }
  else
  {
    fprintf(stderr, "problem with GetCommModemStatus\n");
    return false;
  }
}

