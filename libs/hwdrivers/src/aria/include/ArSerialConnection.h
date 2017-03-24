/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSERIALCONNECTION_H
#define ARSERIALCONNECTION_H

#include <string>
#include "ariaTypedefs.h"
#include "ArDeviceConnection.h"

#ifndef WIN32
#define TIOGETTIMESTAMP         0x5480
#define TIOSTARTTIMESTAMP       0x5481
#endif

/// For connecting to devices through a serial port
class ArSerialConnection: public ArDeviceConnection
{
 public:
  /// Constructor
  AREXPORT ArSerialConnection();
  /// Destructor also closes the connection
  AREXPORT virtual ~ArSerialConnection();

  /** Opens the serial port
   *  @sa ArUtil::COM1, ArUtil::COM2, ArUtil::COM3, ArUtil::COM4
   */
  AREXPORT int open(const char * port = NULL);

  /** Sets the port this connection will use
   *  @sa ArUtil::COM1, ArUtil::COM2, ArUtil::COM3, ArUtil::COM4
   */
  AREXPORT void setPort(const char *port = NULL);

  /** Gets the port this is using
   *  @sa ArUtil::COM1, ArUtil::COM2, ArUtil::COM3, ArUtil::COM4
   */
  AREXPORT const char * getPort(void);
  
  AREXPORT virtual bool openSimple(void);  
  AREXPORT virtual int getStatus(void);
  AREXPORT virtual bool close(void);
  AREXPORT virtual int read(const char *data, unsigned int size, 
			    unsigned int msWait = 0);
  AREXPORT virtual int write(const char *data, unsigned int size);
  AREXPORT virtual const char * getOpenMessage(int messageNumber);

  /// Sets the baud rate on the connection
  AREXPORT bool setBaud(int baud);
  /// Gets what the current baud rate is set to
  AREXPORT int getBaud(void);

  /// Sets whether to enable or disable the hardware control lines
  AREXPORT bool setHardwareControl(bool hardwareControl);
  /// Gets whether the hardware control lines are enabled or disabled
  AREXPORT bool getHardwareControl(void);

  /// Sees how the CTS line is set (true = high)
  AREXPORT bool getCTS(void);

  /// Sees how the DSR line is set (true = high)
  AREXPORT bool getDSR(void);

  /// Sees how the DCD line is set (true = high)
  AREXPORT bool getDCD(void);

  /// Sees how the Ring line is set (true = high)
  AREXPORT bool getRing(void);

  // Internal open, for use by open and openSimple
  AREXPORT int internalOpen(void);

  enum Open { 
      OPEN_COULD_NOT_OPEN_PORT = 1, ///< Could not open the port
      OPEN_COULD_NOT_SET_UP_PORT, ///< Could not set up the port
      OPEN_INVALID_BAUD_RATE, ///< Baud rate is not valid
      OPEN_COULD_NOT_SET_BAUD, ///< Baud rate valid, but could not set it
      OPEN_ALREADY_OPEN ///< Connection was already open
  };
  AREXPORT virtual ArTime getTimeRead(int index);
  AREXPORT virtual bool isTimeStamping(void);

 protected:
  void buildStrMap(void);

#ifndef WIN32
  // these both return -1 for errors
  int rateToBaud(int rate);
  int baudToRate(int baud);
  // this just tries
  void startTimeStamping(void);
  bool myTakingTimeStamps;
#endif
    
  ArStrMap myStrMap;
  std::string myPortName;
  int myBaudRate;
  int myStatus;
  bool myHardwareControl;

#ifndef WIN32
  int myPort;
#endif // ifdef linux

#ifdef WIN32
  HANDLE myPort;
#endif // ifdef WIN32


};

#endif
