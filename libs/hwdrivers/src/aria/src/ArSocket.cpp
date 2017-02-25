/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <stdio.h>
#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArSocket.h"
#include "ArLog.h"

// JL: Added for MRPT to build in MSVC 64bit
#if defined(__BORLANDC__) || defined(_MSC_VER)
#pragma comment (lib,"WS2_32.LIB")
#endif

void ArSocket::internalInit(void)
{
  myCloseFunctor = NULL;
  myStringAutoEcho = true;
  myStringEcho = false;
  myStringPosLast = 0;
  myStringPos = 0;
  myStringGotComplete = false;
  myStringBufEmpty[0] = '\0';
  myStringGotEscapeChars = false;
  myStringHaveEchoed = false;
  sprintf(myIPString, "none");
  resetTracking();
}

AREXPORT int ArSocket::sendTo(const void *msg, int len)
{
  int ret;
  ret = ::sendto(myFD, (char*)msg, len, 0, (struct sockaddr*)&mySin,
		  sizeof(mySin));
  if (ret > 0)
  {
    mySends++;
    myBytesSent += ret;
  }
  return ret;
}

AREXPORT int ArSocket::sendTo(const void *msg, int len,
			      struct sockaddr_in *sin)
{
  int ret;
  ret = ::sendto(myFD, (char*)msg, len, 0, (struct sockaddr*)sin,
		  sizeof(struct sockaddr_in));
  if (ret > 0)
  {
    mySends++;
    myBytesSent += ret;
  }
  return ret;
}


AREXPORT int ArSocket::recvFrom(void *msg, int len, sockaddr_in *sin)
{

#ifdef WIN32
  int i=sizeof(sockaddr_in);
#else
  socklen_t i=sizeof(sockaddr_in);
#endif
  int ret;
  ret = ::recvfrom(myFD, (char*)msg, len, 0, (struct sockaddr*)sin, &i);
  if (ret > 0)
  {
    myRecvs++;
    myBytesRecvd += ret;
  }
  return ret;
}

/**
   @param buff buffer to write from
   @param len how many bytes to write
   @return number of bytes written
**/
AREXPORT int ArSocket::write(const void *buff, size_t len)
{

  if (myFD < 0)
  {
    ArLog::log(ArLog::Terse, "ArSocket::write: called after socket closed");
    return 0;
  }

  struct timeval tval;
  fd_set fdSet;
  tval.tv_sec = 0;
  tval.tv_usec = 0;
  FD_ZERO(&fdSet);
  FD_SET(myFD, &fdSet);


  if (select(myFD + 1, NULL, &fdSet, NULL, &tval) <= 0)
    return 0;

  int ret;
#ifdef WIN32
  ret = ::send(myFD, (char*)buff, len, 0);
#else
  ret = ::write(myFD, (char*)buff, len);
#endif

  if (ret > 0)
  {
    mySends++;
    myBytesSent += ret;
  }
  return ret;
}

/**
   @param buff buffer to read into
   @param len how many bytes to read
   @param msWait if 0, don't block, if > 0 wait this long for data
   @return number of bytes read
*/
AREXPORT int ArSocket::read(void *buff, size_t len, unsigned int msWait)
{
  if (myFD < 0)
  {
    ArLog::log(ArLog::Terse, "ArSocket::read: called after socket closed");
    return 0;
  }

  int ret;
  if (msWait != 0)
  {
    struct timeval tval;
    fd_set fdSet;
    tval.tv_sec = msWait / 1000;
    tval.tv_usec = (msWait % 1000) * 1000;
    FD_ZERO(&fdSet);
    FD_SET(myFD, &fdSet);
    if (select(myFD + 1, &fdSet, NULL, NULL, &tval) <= 0)
      return 0;
  }
  ret = ::recv(myFD, (char*)buff, len, 0);
  if (ret > 0)
  {
    myRecvs++;
    myBytesRecvd += ret;
  }
  return ret;
}



/*
   This cannot write more than 512 number of bytes
   @param str the string to write to the socket
   @return number of bytes written
**/
AREXPORT int ArSocket::writeString(const char *str, ...)
{
  char buf[1200];
  int len;
  int ret;
  myWriteStringMutex.lock();
  va_list ptr;
  va_start(ptr, str);
  vsnprintf(buf, sizeof(buf), str, ptr);
  va_end(ptr);
  if (myLogWriteStrings)
    ArLog::log(ArLog::Normal, "Sent to %s: %s", getIPString(), buf);
  len = strlen(buf);
  buf[len] = '\n';
  len++;
  buf[len] = '\r';
  len++;
  ret = write(buf, len);
  myWriteStringMutex.unlock();
  return ret;
}

void ArSocket::setIPString(void)
{
  unsigned char *bytes;
  bytes = (unsigned char *)inAddr();
  if (bytes != NULL)
    sprintf(myIPString, "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2], bytes[3]);
}


  /**
     @note This function can only read strings less than 512 characters
     long as it reads the characters into its own internal buffer (to
     compensate for some of the things the DOS telnet does).

     @return Data read, or an empty string (first character will be '\\0')
       if no data was read.  If there was an error reading from the socket,
       NULL is returned.
  **/

AREXPORT char *ArSocket::readString(void)
{
  size_t i;
  int n;

  myReadStringMutex.lock();
  myStringBufEmpty[0] = '\0';

  // read one byte at a time
  for (i = myStringPos; i < sizeof(myStringBuf); i++)
  {

    n = read(&myStringBuf[i], 1);
    if (n > 0)
    {
      if (i == 0 && myStringBuf[i] < 0)
      {
	myStringGotEscapeChars = true;
      }
      if (myStringBuf[i] == '\n' || myStringBuf[i] == '\r')
      {
	if (i != 0)
	  myStringGotComplete = true;
	myStringBuf[i] = '\0';
	myStringPos = 0;
	myStringPosLast = 0;
	// if we have leading escape characters get rid of them
	if (myStringBuf[0] < 0)
	{
	  int ei;
	  myStringGotEscapeChars = true;
	  // increment out the escape chars
	  for (ei = 0;
	       myStringBuf[ei] < 0 || (ei > 0 && myStringBuf[ei - 1] < 0);
	       ei++);
	  // okay now return the good stuff
	  doStringEcho();
	  myReadStringMutex.unlock();
	  return &myStringBuf[ei];
	}
	// if we don't return what we got
	doStringEcho();
	myReadStringMutex.unlock();
	return myStringBuf;
      }
      // if its not an ending character but was good keep going
      else
	continue;
    }
    // failed
    else if (n == 0)
    {
      myReadStringMutex.unlock();
      return NULL;
    }
    else // which means (n < 0)
    {
#ifdef WIN32
      if (WSAGetLastError() == WSAEWOULDBLOCK)
      {
	myStringPos = i;
	doStringEcho();
	myReadStringMutex.unlock();
	return myStringBufEmpty;
      }
#endif
#ifndef WIN32
      if (errno == EAGAIN)
      {
	myStringPos = i;
	doStringEcho();
	myReadStringMutex.unlock();
	return myStringBufEmpty;
      }
#endif
      perror("Error in reading from network");
      myReadStringMutex.unlock();
      return NULL;
    }
  }
  // if they want a 0 length string
  ArLog::log(ArLog::Normal, "Some trouble in ArSocket::readString to %s", getIPString());
  writeString("String too long");
  myReadStringMutex.unlock();
  return NULL;
}

void ArSocket::doStringEcho(void)
{
  //size_t to;

  if (!myStringAutoEcho && !myStringEcho)
    return;

  // if we're echoing complete thel ines
  if (myStringHaveEchoed && myStringGotComplete)
  {
    write("\n\r", 2);
    myStringGotComplete = false;
  }

  // if there's nothing to send we don't need to send it
  if (myStringPosLast == myStringPos)
    return;

  // we probably don't need it if its doing escape chars
  if (myStringAutoEcho && myStringGotEscapeChars)
    return;

  myStringHaveEchoed = true;
  //to =strchr(myStringBuf, '\0') - myStringBuf;
  write(&myStringBuf[myStringPosLast], myStringPos - myStringPosLast);
  myStringPosLast = myStringPos;
}

