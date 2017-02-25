/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARDEVICECONNECTION_H
#define ARDEVICECONNECTION_H

#include <string>
#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArBasePacket.h"

/// Base class for device connections
/**
   Base class for device connections, this is mostly for connections to the
   robot or simulator but could also be used for a connection to a laser
   or other device

   Note that this is mostly a base class, so if you'll want to use one of the
   classes which inherit from this one... also note that in those classes
   is where you'll find setPort which sets the place the device connection
   will try to connect to... the inherited classes also have an open which
   returns more detailed information about the open attempt, and which takes
   the parameters for where to connect
*/
class ArDeviceConnection
{
 public:
  /// constructor
  AREXPORT ArDeviceConnection();
  /// destructor also forces a close on the connection
  AREXPORT virtual ~ArDeviceConnection();
  /// Reads data from connection
  /**
     Reads data from connection
     @param data pointer to a character array to read the data into
     @param size maximum number of bytes to read
     @param msWait read blocks for this many milliseconds (not at all for == 0)
     @return number of bytes read, or -1 for failure
     @see write, writePacket
  */
  AREXPORT virtual int read(const char *data, unsigned int size,
			    unsigned int msWait = 0) = 0;
  /// Writes data to connection
  /**
     Writes data to connection from a packet
     @param packet pointer to a packet to write the data from
     @return number of bytes written, or -1 for failure
     @see read, write
  */
  /*AREXPORT*/virtual int writePacket(ArBasePacket *packet)
    { if (packet == NULL || packet->getLength() == 0) return 0;
    return write(packet->getBuf(), packet->getLength()); }
  /// Writes data to connection
  /**
     Writes data to connection
     @param data pointer to a character array to write the data from
     @param size number of bytes to write
     @return number of bytes read, or -1 for failure
     @see read, writePacket
  */
  AREXPORT virtual int write(const char *data, unsigned int size) = 0;
  /// Gets the status of the connection, which is one of the enum status
  /**
     Gets the status of the connection, which is one of the enum status.
     If you want to get a string to go along with the number, use
     getStatusMessage
     @return the status of the connection
     @see getStatusMessage
  */
  AREXPORT virtual int getStatus(void) = 0;
  /// Gets the description string associated with the status
  /**
      @param messageNumber the int from getStatus you want the string for
      @return the description associated with the status
      @see getStatus
  */
  AREXPORT const char *getStatusMessage(int messageNumber) const;

  /// Opens the connection again, using the values from setLocation or
  // a previous open
  virtual bool openSimple(void) = 0;

  /// Closes the connection
  /**
     @return whether the close succeeded or not
  */
  virtual bool close(void) { return false; }
  /// Gets the string of the message associated with opening the device
  /**
     Each class inherited from this one has an open method which returns 0
     for success or an integer which can be passed into this function to
     obtain a string describing the reason for failure
     @param messageNumber the number returned from the open
     @return the error description associated with the messageNumber
  */
  AREXPORT virtual const char * getOpenMessage(int messageNumber) = 0;
  enum Status {
      STATUS_NEVER_OPENED = 1, ///< Never opened
      STATUS_OPEN,  ///< Currently open
      STATUS_OPEN_FAILED, ///< Tried to open, but failed
      STATUS_CLOSED_NORMALLY, ///< Closed by a close call
      STATUS_CLOSED_ERROR ///< Closed because of error
  };
  /// Gets the time data was read in
  /**
      @param index looks like this is the index back in the number of bytes
      last read in
      @return the time the last read data was read in
  */
  AREXPORT virtual ArTime getTimeRead(int index) = 0;
  /// sees if timestamping is really going on or not
  /** @return true if real timestamping is happening, false otherwise */
  AREXPORT virtual bool isTimeStamping(void) = 0;
 protected:
  void buildStrMap(void);
  static bool ourStrMapInited;
  static ArStrMap ourStrMap;
};

#endif


