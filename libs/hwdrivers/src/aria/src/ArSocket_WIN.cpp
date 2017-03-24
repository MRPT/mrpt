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
#include "ArSocket.h"
#include "ArLog.h"
#include <stdio.h>
#include <string.h>
#include "ArFunctor.h"

/*
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
*/

bool ArSocket::ourInitialized=false;


AREXPORT ArSocket::ArSocket() :
  myType(Unknown),
  myError(NoErr),
  myErrorStr(),
  myDoClose(true),
  myFD(-1),
  myNonBlocking(false),
  mySin()
{
  internalInit();
}

AREXPORT ArSocket::ArSocket(const char *host, int port, Type type) :
  myType(type),
  myError(NoErr),
  myErrorStr(),
  myDoClose(true),
  myFD(-1),
  myNonBlocking(false),
  mySin()
{
  internalInit();
  connect(host, port, type);
}

AREXPORT ArSocket::ArSocket(int port, bool doClose, Type type) :
  myType(type),
  myError(NoErr),
  myErrorStr(),
  myDoClose(doClose),
  myFD(-1),
  myNonBlocking(false),
  mySin()
{
  internalInit();
  open(port, type);
}

AREXPORT ArSocket::~ArSocket()
{
  close();
}

AREXPORT bool ArSocket::init()
{
  WORD wVersionRequested;
  WSADATA wsaData;

//  if (!ourInitialized)
  //{
  wVersionRequested=MAKEWORD( 2, 2 );
  
  if (WSAStartup(wVersionRequested, &wsaData) != 0)
  {
    ourInitialized=false;
    return(false);
  }
  
  ourInitialized=true;
  //}

  return(true);
}

AREXPORT void ArSocket::shutdown()
{
  if (ourInitialized)
  {
    WSACleanup();
    ourInitialized=false;
  }
}

AREXPORT bool ArSocket::hostAddr(const char *host, struct in_addr &addr)
{
  struct hostent *hp;

  if (!(hp=gethostbyname(host)))
  {
    perror("gethostbyname");
    memset(&addr, 0, sizeof(in_addr));
    return(false);
  }
  else
  {
    memcpy(&addr, hp->h_addr, hp->h_length);
    return(true);
  }
}

AREXPORT bool ArSocket::addrHost(struct in_addr &addr, char *host)
{
  struct hostent *hp;

  hp=gethostbyaddr((char*)&addr.s_addr, sizeof(addr.s_addr), AF_INET);
  if (hp)
    strcpy(host, hp->h_name);
  else
    strcpy(host, inet_ntoa(addr));

  return(true);
}

AREXPORT bool ArSocket::connect(const char *host, int port, Type type)
{
  char localhost[MAXGETHOSTSTRUCT];

  init();

  if (!host)
  {
    if (gethostname(localhost, sizeof(localhost)) == 1)
    {
      myError=ConBadHost;
      myErrorStr="Failure to locate host '";
      myErrorStr+=localhost;
      myErrorStr+="'";
      perror("gethostname");
      return(false);
    }
    host=localhost;
  }

  memset(&mySin, 0, sizeof(mySin));
  if ((mySin.sin_addr.s_addr = inet_addr(host)) == INADDR_NONE)
  {
    if (!hostAddr(host, mySin.sin_addr))
    {
      setIPString();
      myError = ConBadHost;
      myErrorStr = "Could not find the address of '";
      myErrorStr += host;
      myErrorStr += "'";
      return(false);
    }
  }

  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(port);

  if ((type == TCP) && ((myFD=socket(AF_INET, SOCK_STREAM, 0)) < 0))
  {
    myError=NetFail;
    myErrorStr="Failure to make TCP socket";
    perror("socket");
    return(false);
  }
  else if ((type == UDP) && ((myFD=socket(AF_INET, SOCK_DGRAM, 0)) < 0))
  {
    myError=NetFail;
    myErrorStr="Failure to make UDP socket";
    perror("socket");
    return(0);
  }

  myType=type;

  if (::connect(myFD, (struct sockaddr *)&mySin,
		sizeof(struct sockaddr_in)) < 0)
  {
    char buff[10];
    int err=WSAGetLastError();
    sprintf(buff, "%d", err);
    myErrorStr="Failure to connect socket";
    myErrorStr+=buff;
    switch (err)
    {
    case WSAEADDRNOTAVAIL:
      myError=ConBadHost;
      break;
    case WSAECONNREFUSED:
      myError=ConRefused;
      break;
    case WSAENETUNREACH:
      myError=ConNoRoute;
      break;
    default:
      myError=NetFail;
      break;
    }
    //perror("connect");
    ::shutdown(myFD, SD_BOTH);
    closesocket(myFD);
    myFD = -1;
    return(0);
  }

  return(1);
}

AREXPORT bool ArSocket::open(int port, Type type, const char *openOnIP)
{
  int ret;
  char localhost[MAXGETHOSTSTRUCT];

  if ((type == TCP) && ((myFD=socket(AF_INET, SOCK_STREAM, 0)) < 0))
  {
    ret=WSAGetLastError();
    myErrorStr="Failure to make TCP socket";
    perror("socket");
    return(false);
  }
  else if ((type == UDP) && ((myFD=socket(AF_INET, SOCK_DGRAM, 0)) < 0))
  {
    myErrorStr="Failure to make UDP socket";
    perror("socket");
    return(false);
  }

  /* MPL this is useless withw hat I Took out below
  if (gethostname(localhost, sizeof(localhost)) == 1)
  {
    myErrorStr="Failure to locate localhost";
    perror("gethostname");
    return(false);
  }
  */

  memset(&mySin, 0, sizeof(mySin));
  /* MPL took this out since it was just overriding it with the
     INADDR_ANY anyways and it could cause slowdowns if a machine wasn't
     configured so lookups are quick
  if (!hostAddr(localhost, mySin.sin_addr))
    return(false);
  */
  setIPString();
  if (openOnIP != NULL)
  {
    
    if (!hostAddr(openOnIP, mySin.sin_addr))
    {
      ArLog::log(ArLog::Normal, "Couldn't find ip of %s to open on", openOnIP);
      return(false); 
    }
    else
    {
      //printf("Opening on %s\n", openOnIP);
    }
  }
  else
  {
    mySin.sin_addr.s_addr=htonl(INADDR_ANY);
  }
  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(port);

  myType=type;

  if ((ret=bind(myFD, (struct sockaddr *)&mySin, sizeof(mySin))) < 0)
  {
    myErrorStr="Failure to bind socket to port ";
    sprintf(localhost, "%d", port);
    myErrorStr+=localhost;
    perror("socket");
    return(false);
  }

  if ((type == TCP) && (listen(myFD, 5) < 0))
  {
    myErrorStr="Failure to listen on socket";
    perror("listen");
    return(false);
  }

  return(true);
}

AREXPORT bool ArSocket::create(Type type)
{
  if ((type == TCP) && ((myFD=socket(AF_INET, SOCK_STREAM, 0)) < 0))
  {
    myErrorStr="Failure to make TCP socket";
    perror("socket");
    return(false);
  }
  else if ((type == UDP) && ((myFD=socket(AF_INET, SOCK_DGRAM, 0)) < 0))
  {
    myErrorStr="Failure to make UDP socket";
    perror("socket");
    return(false);
  }

/*
  int zero = 0;
  if (setsockopt(myFD, SOL_SOCKET, SO_SNDBUF, (char *)&zero, sizeof(zero)) != 0)
  {
    perror("setsockopt");
    ArLog::log(ArLog::Normal, "Could not set SNDBUF %d", WSAGetLastError());
    return(false);
  }

  if (setsockopt(myFD, SOL_SOCKET, SO_RCVBUF, (char *)&zero, sizeof(zero)) != 0)
  {
    perror("setsockopt");
    ArLog::log(ArLog::Normal, "Could not set SNDBUF %d", WSAGetLastError());
    return(false);
  }

  myType=type;
*/
  /*if (getSockName())
    return(true);
  else
    return(false);*/
  return(true);
}

AREXPORT bool ArSocket::findValidPort(int startPort, const char *openOnIP)
{
  //char localhost[MAXGETHOSTSTRUCT];

  /*
  if (gethostname(localhost, sizeof(localhost)) == 1)
  {
    myErrorStr="Failure to locate localhost";
    perror("gethostname");
    return(false);
  }
  */
  for (int i=0; i+startPort < 65000; ++i)
  {
    memset(&mySin, 0, sizeof(mySin));
    /*
    if (!hostAddr(localhost, mySin.sin_addr))
      return(false);
    */
    setIPString();
    if (openOnIP != NULL)
    {
      if (!hostAddr(openOnIP, mySin.sin_addr))
      {
	ArLog::log(ArLog::Normal, "Couldn't find ip of %s to open on", 
		   openOnIP);
	return(false); 
      }
      else
      {
	//printf("Opening on %s\n", openOnIP);
      }
    }
    else
    {
      mySin.sin_addr.s_addr=htonl(INADDR_ANY);
    }

    mySin.sin_family=AF_INET;
    //mySin.sin_addr.s_addr=htonl(INADDR_ANY);
    mySin.sin_port=hostToNetOrder(startPort+i);

    if (bind(myFD, (struct sockaddr *)&mySin, sizeof(mySin)) == 0)
      break;
  }

  return(true);
}

AREXPORT bool ArSocket::connectTo(const char *host, int port)
{
  char localhost[MAXGETHOSTSTRUCT];

  if (myFD < 0)
    return(false);

  if (!host)
  {
    if (gethostname(localhost, sizeof(localhost)) == 1)
    {
      myErrorStr="Failure to locate host '";
      myErrorStr+=localhost;
      myErrorStr+="'";
      perror("gethostname");
      return(false);
    }
    host=localhost;
  }

  memset(&mySin, 0, sizeof(mySin));
  if (!hostAddr(host, mySin.sin_addr))
    return(false);
  setIPString();
  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(port);

  return(connectTo(&mySin));
}

AREXPORT bool ArSocket::connectTo(struct sockaddr_in *sin)
{
  if (::connect(myFD, (struct sockaddr *)sin,
		sizeof(struct sockaddr_in)) < 0)
  {
    myErrorStr="Failure to connect socket";
    //perror("connect");
    return(0);
  }

  return(1);
}

AREXPORT bool ArSocket::close()
{
  if (myFD != -1)
    ArLog::log(ArLog::Verbose, "Closing socket");
  if (myCloseFunctor != NULL)
    myCloseFunctor->invoke();
  if (myDoClose && (myFD >= 0))
  {
    ::shutdown(myFD, SD_BOTH);
    closesocket(myFD);
    myFD = -1;
    return(true);
  }
  return(false);
}

AREXPORT bool ArSocket::setLinger(int time)
{
  struct linger lin;

  if (time)
  {
    lin.l_onoff=1;
    lin.l_linger=time;
  }
  else
  {
    lin.l_onoff=0;
    lin.l_linger=time;
  }

  if (setsockopt(myFD, SOL_SOCKET, SO_LINGER, (char*)&lin, sizeof(lin)) != 0)
  {
    myErrorStr="Failure to setsockopt LINGER";
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

AREXPORT bool ArSocket::setBroadcast()
{
  if (setsockopt(myFD, SOL_SOCKET, SO_BROADCAST, NULL, 0) != 0)
  {
    myErrorStr="Failure to setsockopt BROADCAST";
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

AREXPORT bool ArSocket::setReuseAddress()
{
  int opt=1;

  if (setsockopt(myFD, SOL_SOCKET, SO_REUSEADDR,
		 (char*)&opt, sizeof(opt)) != 0)
  {
    myErrorStr="Failure to setsockopt REUSEADDR";
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

AREXPORT bool ArSocket::setNonBlock()
{
  u_long arg=1;

  if (ioctlsocket(myFD, FIONBIO, &arg) != 0)
  {
    myErrorStr="Failure to fcntl O_NONBLOCK";
    perror("fcntl");
    return(false);
  }
  else
  {
    myNonBlocking = true;
    return(true);
  }
}

AREXPORT bool ArSocket::copy(int fd, bool doclose)
{
  int len;

  myFD=fd;
  myDoClose=doclose;

  len=sizeof(struct sockaddr_in);
  if (getsockname(myFD, (struct sockaddr*)&mySin, &len))
  {
    myErrorStr="Failed to getsockname on fd ";
    perror("getsockname");
    return(false);
  }
  else
    return(true);
}

AREXPORT bool ArSocket::accept(ArSocket *sock)
{
  int len;
  unsigned char *bytes;
  
  len=sizeof(struct sockaddr_in);
  sock->myFD=::accept(myFD, (struct sockaddr*)&(sock->mySin), &len);
  sock->myType=myType;
  bytes = (unsigned char *)sock->inAddr();
  sprintf(sock->myIPString, "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2], 
	  bytes[3]);
  if ((sock->myFD < 0 && !myNonBlocking) || 
      (sock->myFD < 0 && WSAGetLastError() != WSAEWOULDBLOCK && myNonBlocking))
  {
    myErrorStr="Failed to accept on socket";
    perror("accept");
    return(false);
  }

  return(true);
}

AREXPORT void ArSocket::inToA(struct in_addr *addr, char *buff)
{
  strcpy(buff, inet_ntoa(*addr));
}

AREXPORT bool ArSocket::getSockName()
{
  int size;

  if (myFD < 0)
  {
    myErrorStr="Trying to get socket name on an unopened socket";
    printf(myErrorStr.c_str());
    return(false);
  }

  size=sizeof(mySin);
  if (getsockname(myFD, (struct sockaddr *)&mySin, &size) != 0)
  {
    myErrorStr="Error getting socket name";
    switch (WSAGetLastError())
    {
    case WSAEINVAL:
      myErrorStr+=": inval";
      break;
    case WSANOTINITIALISED:
      myErrorStr+=": not init";
      break;
    }
    perror(myErrorStr.c_str());
    return(false);
  }

  return(true);
}

AREXPORT unsigned int ArSocket::hostToNetOrder(int i)
{
  return(htons(i));
}

AREXPORT unsigned int ArSocket::netToHostOrder(int i)
{
  return(ntohs(i));
}

AREXPORT std::string ArSocket::getHostName()
{
  char localhost[MAXGETHOSTSTRUCT];

  if (gethostname(localhost, sizeof(localhost)) == 1)
    return("");
  else
    return(localhost);
}

/** If this socket is a TCP socket, then set the TCP_NODELAY flag to 1,
 *  to disable the use of the Nagle algorithm (which waits until enough
 *  data is ready to send to fill a TCP frame, rather then sending the
 *  packet immediately).
 *  @return true of the flag was successfully set, false if there was an 
 *    error or this socket is not a TCP socket.
 */
AREXPORT bool ArSocket::setNoDelay(bool flag)
{
  if(myType != TCP) return false;
  int f = flag?1:0;
  int r = setsockopt(myFD, IPPROTO_TCP, TCP_NODELAY, (char*)&f, sizeof(f));
  return (r != -1);
}

