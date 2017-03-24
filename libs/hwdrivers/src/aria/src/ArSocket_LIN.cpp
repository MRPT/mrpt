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
#include <errno.h>
#include <stdio.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "ArFunctor.h"
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

/// We're always initialized in Linux
bool ArSocket::ourInitialized=true;


/**
   In Windows, the networking subsystem needs to be initialized and shutdown
   individyaly by each program. So when a program starts they will need to
   call the static function ArSocket::init() and call ArSocket::shutdown()
   when it exits. For programs that use Aria::init() and Aria::uninit()
   calling the ArSocket::init() and ArSocket::shutdown() is unnecessary. The
   Aria initialization functions take care of this. These functions do nothing
   in Linux.
*/
bool ArSocket::init()
{
  return(true);
}

/**
   In Windows, the networking subsystem needs to be initialized and shutdown
   individyaly by each program. So when a program starts they will need to
   call the static function ArSocket::init() and call ArSocket::shutdown()
   when it exits. For programs that use Aria::init() and Aria::uninit()
   calling the ArSocket::init() and ArSocket::shutdown() is unnecessary. The
   Aria initialization functions take care of this. These functions do nothing
   in Linux.
*/
void ArSocket::shutdown()
{
}

ArSocket::ArSocket() :
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

/**
   Constructs the socket and connects it to the given host.
   @param host hostname of the server to connect to
   @param port port number of the server to connect to
   @param type protocol type to use
*/
ArSocket::ArSocket(const char *host, int port, Type type) :
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

/**
   Constructs the socket and opens it as a server port.
   @param port port number to bind the socket to
   @param doClose automaticaly close the port if the socket is destructed
   @param type protocol type to use
*/
ArSocket::ArSocket(int port, bool doClose, Type type) :
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

ArSocket::~ArSocket()
{
  close();
}

bool ArSocket::hostAddr(const char *host, struct in_addr &addr)
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
    bcopy(hp->h_addr, &addr, hp->h_length);
    return(true);
  }
}

bool ArSocket::addrHost(struct in_addr &addr, char *host)
{
  struct hostent *hp;

  hp=gethostbyaddr((char*)&addr.s_addr, sizeof(addr.s_addr), AF_INET);
  if (hp)
    strcpy(host, hp->h_name);
  else
    strcpy(host, inet_ntoa(addr));

  return(true);
}

std::string ArSocket::getHostName()
{
  char localhost[100];  // maxHostNameLen()];

  if (gethostname(localhost, sizeof(localhost)) == 1)
    return("");
  else
    return(localhost);
}

bool ArSocket::connect(const char *host, int port, Type type)
{
  char localhost[100];  // maxHostNameLen()];

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

  bzero(&mySin, sizeof(mySin));
  // MPL taking out this next code line from the if since it makes
  // everything we can't resolve try to connect to localhost
  // &&  !hostAddr("localhost", mySin.sin_addr))
  if (!hostAddr(host, mySin.sin_addr))
    return(false);
  setIPString();
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
    return(false);
  }

  myType=type;

  if (::connect(myFD, (struct sockaddr *)&mySin,
		sizeof(struct sockaddr_in)) < 0)
  {
    myErrorStr="Failure to connect socket";
    switch (errno)
    {
    case ECONNREFUSED:
      myError=ConRefused;
      myErrorStr+="; Connection refused";
      break;
    case ENETUNREACH:
      myError=ConNoRoute;
      myErrorStr+="; No route to host";
      break;
    default:
      myError=NetFail;
      break;
    }
    //perror("connect");
    ::close(myFD);
    myFD = -1;
    return(false);
  }

  return(true);
}

bool ArSocket::open(int port, Type type, const char *openOnIP)
{
  int ret;
  char localhost[100];  // maxHostNameLen()];

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

  myType=type;

  /* MPL removed this since with what I Took out down below months ago
  if (gethostname(localhost, sizeof(localhost)) == 1)
  {
    myErrorStr="Failure to locate localhost";
    perror("gethostname");
    return(false);
  }
  */
  bzero(&mySin, sizeof(mySin));
  /* MPL took this out since it was just overriding it with the
     INADDR_ANY anyways and it could cause slowdowns if a machine wasn't
     configured so lookups are quick
  if (!hostAddr(localhost, mySin.sin_addr) &&
      !hostAddr("localhost", mySin.sin_addr))
    return(false); */

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

  setIPString();
  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(port);

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

bool ArSocket::create(Type type)
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

  myType=type;

  if (getSockName())
    return(true);
  else
    return(false);
}

bool ArSocket::findValidPort(int startPort, const char *openOnIP)
{
//  char localhost[100];  // maxHostNameLen()];

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
    bzero(&mySin, sizeof(mySin));
    /*
    if (!hostAddr(localhost, mySin.sin_addr) &&
	!hostAddr("localhost", mySin.sin_addr))
      return(false);
    */
    setIPString();

    if (openOnIP != NULL)
    {

      if (!hostAddr(openOnIP, mySin.sin_addr))
      {
	ArLog::log(ArLog::Normal, "Couldn't find ip of %s to open udp on", openOnIP);
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
    mySin.sin_port=hostToNetOrder(startPort+i);

    if (bind(myFD, (struct sockaddr *)&mySin, sizeof(mySin)) == 0)
      break;
  }

  return(true);
}

bool ArSocket::connectTo(const char *host, int port)
{
  char localhost[100];  // maxHostNameLen()];

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

  bzero(&mySin, sizeof(mySin));
  if (!hostAddr(host, mySin.sin_addr))
    return(false);
  setIPString();
  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(port);

  return(connectTo(&mySin));
}

bool ArSocket::connectTo(struct sockaddr_in *sin)
{
  if (::connect(myFD, (struct sockaddr *)sin,
		sizeof(struct sockaddr_in)) < 0)
  {
    myErrorStr="Failure to connect socket";
    perror("connect");
    return(0);
  }

  return(1);
}


bool ArSocket::close()
{
  if (myFD != -1)
    ArLog::log(ArLog::Verbose, "Closing socket");
  if (myCloseFunctor != NULL)
    myCloseFunctor->invoke();
  if (myDoClose && ::close(myFD))
  {
    myFD=-1;
    return(false);
  }
  else
  {
    myFD=-1;
    return(true);
  }
}

bool ArSocket::setLinger(int time)
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

  if (setsockopt(myFD, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin)) != 0)
  {
    myErrorStr="Failure to setsockopt LINGER";
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

bool ArSocket::setBroadcast()
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

bool ArSocket::setReuseAddress()
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

bool ArSocket::setNonBlock()
{
  if (fcntl(myFD, F_SETFL, O_NONBLOCK) != 0)
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

/**
   Copy socket structures. Copy from one Socket to another will still have
   the first socket close the file descripter when it is destructed.
*/
bool ArSocket::copy(int fd, bool doclose)
{
  socklen_t len;

  myFD=fd;
  myDoClose=doclose;
  myType=Unknown;

  len=sizeof(struct sockaddr_in);
  if (getsockname(myFD, (struct sockaddr*)&mySin, &len))
  {
    myErrorStr="Failed to getsockname on fd ";
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

/**
   @return true if there are no errors, false if there are
   errors... not that if you're in non-blocking mode and there is no
   socket to connect that is NOT an error, you'll want to check the
   getFD on the sock you pass in to see if it is actually a valid
   socket.
 **/
bool ArSocket::accept(ArSocket *sock)
{
  socklen_t len;
  unsigned char *bytes;

  len=sizeof(struct sockaddr_in);
  sock->myFD=::accept(myFD, (struct sockaddr*)&(sock->mySin), &len);
  sock->myType=myType;
  bytes = (unsigned char *)sock->inAddr();
  sprintf(sock->myIPString, "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2],
	  bytes[3]);
  if ((sock->myFD < 0 && !myNonBlocking) ||
      (sock->myFD < 0 && errno != EWOULDBLOCK && myNonBlocking))
  {
    myErrorStr="Failed to accept on socket";
    perror("accept");
    return(false);
  }

  return(true);
}

void ArSocket::inToA(struct in_addr *addr, char *buff)
{
  strcpy(buff, inet_ntoa(*addr));
}

bool ArSocket::getSockName()
{
  socklen_t size;

  if (myFD < 0)
  {
    myErrorStr="Trying to get socket name on an unopened socket";
    printf("%s",myErrorStr.c_str());
    return(false);
  }

  size=sizeof(mySin);
  if (getsockname(myFD, (struct sockaddr *)&mySin, &size) != 0)
  {
    myErrorStr="Error getting socket name";
    perror(myErrorStr.c_str());
    return(false);
  }

  return(true);
}

unsigned int ArSocket::hostToNetOrder(int i)
{
  return(htons(i));
}

unsigned int ArSocket::netToHostOrder(int i)
{
  return(ntohs(i));
}

/** If this socket is a TCP socket, then set the TCP_NODELAY flag,
 *  to disable the use of the Nagle algorithm (which waits until enough
 *  data is ready to send to fill a TCP frame, rather then sending the
 *  packet immediately).
 *  @param flag true to turn on NoDelay, false to turn it off.
 *  @return true of the flag was successfully set, false if there was an
 *    error or this socket is not a TCP socket.
 */
bool ArSocket::setNoDelay(bool flag)
{
  if(myType != TCP) return false;
  int f = flag?1:0;
  int r = setsockopt(myFD, IPPROTO_TCP, TCP_NODELAY, (char*)&f, sizeof(f));
  return (r != -1);
}

