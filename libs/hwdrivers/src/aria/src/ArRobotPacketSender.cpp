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
#include "ArDeviceConnection.h"
#include "ArRobotPacketSender.h"

/**
   @param sync1 first byte of the header this sender will send, this 
   should be left as the default in nearly all cases, ie don't mess with it
   @param sync2 second byte of the header this sender will send, this 
   should be left as the default in nearly all cases, ie don't mess with it
*/
AREXPORT ArRobotPacketSender::ArRobotPacketSender(unsigned char sync1,
						  unsigned char sync2) :
  myPacket(sync1, sync2)
{
  myDeviceConn = NULL;
}

/**
   @param sync1 first byte of the header this sender will send, this 
   should be left as the default in nearly all cases, ie don't mess with it
   @param sync2 second byte of the header this sender will send, this 
   should be left as the default in nearly all cases, ie don't mess with it
*/
AREXPORT ArRobotPacketSender::ArRobotPacketSender(
	ArDeviceConnection *deviceConnection, unsigned char sync1,
	unsigned char sync2) :
  myPacket(sync1, sync2)
{
  myDeviceConn = deviceConnection;
}

AREXPORT ArRobotPacketSender::~ArRobotPacketSender()
{

}

AREXPORT void ArRobotPacketSender::setDeviceConnection(
	ArDeviceConnection *deviceConnection)
{
  myDeviceConn = deviceConnection;
}

AREXPORT ArDeviceConnection *ArRobotPacketSender::getDeviceConnection(void)
{
  return myDeviceConn;
}

bool ArRobotPacketSender::connValid(void)
{
  return (myDeviceConn != NULL && 
	  myDeviceConn->getStatus() == ArDeviceConnection::STATUS_OPEN);
}

/**
   @param command the command number to send
   @return whether the command could be sent or not
*/
AREXPORT bool ArRobotPacketSender::com(unsigned char number)
{
  if (!connValid())
    return false;

  myPacket.empty();
  myPacket.setID(number);

  myPacket.finalizePacket();
  
  return myDeviceConn->write(myPacket.getBuf(), myPacket.getLength());
}

/**
   @param command the command number to send
   @param argument the integer argument to send with the command
   @return whether the command could be sent or not
*/
AREXPORT bool ArRobotPacketSender::comInt(unsigned char command, 
					  short int argument)
{

  if (!connValid())
    return false;

  myPacket.empty();
  myPacket.setID(command);
  if (argument >= 0) 
  {
    myPacket.uByteToBuf(INTARG);
  }
  else 
  {
    myPacket.uByteToBuf(NINTARG);
    argument = -argument;
  }
  myPacket.uByte2ToBuf(argument);

  myPacket.finalizePacket();

  if (myDeviceConn->write(myPacket.getBuf(), myPacket.getLength()) >= 0)
    return true;

  return false;

}

/**
   @param command the command number to send
   @param high the high byte to send with the command
   @param low the low byte to send with the command
   @return whether the command could be sent or not
*/
AREXPORT bool ArRobotPacketSender::com2Bytes(unsigned char command, char high,
					     char low)
{
  return comInt(command, ((high & 0xff)<<8) + (low & 0xff));
}

/**
 * Sends a length-prefixed string command.
   @param command the command number to send
   @param str NULL-terminated string to send with the command
   @return whether the command could be sent or not
*/
AREXPORT bool ArRobotPacketSender::comStr(unsigned char command, 
					  const char *argument)
{
  size_t size;
  if (!connValid())
    return false;
  size = strlen(argument);
  if (size > 199) // 200 - 1 byte for length
    return false;

  myPacket.empty();
  
  myPacket.setID(command);
  myPacket.uByteToBuf(STRARG);
  myPacket.uByteToBuf(size);
  myPacket.strToBuf(argument);
  
  myPacket.finalizePacket();

  if (myDeviceConn->write(myPacket.getBuf(), myPacket.getLength()) >= 0)
    return true;

  return false;
  
}

/**
 * Sends a packet containing the given command, and a length-prefixed string 
 * containing the specified number of bytes copied from the given source string.
   @param command the command number to send
   @param str the character array containing data to send with the command
   @param size number of bytes from the array to send; prefix the string with a byte containing this value as well. this size must be less than the maximum packet size of 200
   @return whether the command could be sent or not
*/
AREXPORT bool ArRobotPacketSender::comStrN(unsigned char command, 
					   const char *str, int size)
{
  if (!connValid())
    return false;

  if(size > 199) return false;   // 200 - 1 byte for length

  myPacket.empty();
  
  myPacket.setID(command);
  myPacket.uByteToBuf(STRARG);

  myPacket.uByteToBuf(size);
  myPacket.strNToBuf(str, size);
  
  myPacket.finalizePacket();

  if (myDeviceConn->write(myPacket.getBuf(), myPacket.getLength()) >= 0)
    return true;

  return false;
  
}

AREXPORT bool ArRobotPacketSender::comDataN(unsigned char command, const char* data, int size)
{
  if(!connValid()) return false;
  if(size > 200) return false;
  myPacket.empty();
  myPacket.setID(command);
  myPacket.uByteToBuf(STRARG);
  myPacket.strNToBuf(data, size);
  myPacket.finalizePacket();
  if(myDeviceConn->write(myPacket.getBuf(), myPacket.getLength()) >= 0)
      return true;
  return false;
}


