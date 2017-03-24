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
#include "ArSignalHandler.h"
#include "ArLog.h"


ArSignalHandler *ArSignalHandler::ourSignalHandler=0;
ArStrMap ArSignalHandler::ourSigMap;
std::list<ArFunctor1<int>*> ArSignalHandler::ourHandlerList;


AREXPORT void ArSignalHandler::signalCB(int sig)
{
  std::list<ArFunctor1<int>*>::iterator iter;

  for (iter=ourHandlerList.begin(); iter != ourHandlerList.end(); ++iter)
    (*iter)->invoke(sig);
  if (ourHandlerList.begin() == ourHandlerList.end())
    ArLog::log(ArLog::Terse, "ArSignalHandler::runThread: No handler function. Unhandled signal '%s'", ourSigMap[sig].c_str());
}  

AREXPORT void ArSignalHandler::createHandlerNonThreaded()
{
}

AREXPORT void ArSignalHandler::createHandlerThreaded()
{
  getHandler()->create(false);
}

AREXPORT void ArSignalHandler::blockCommon()
{
}

AREXPORT void ArSignalHandler::unblockAll()
{
}

AREXPORT void ArSignalHandler::block(Signal sig)
{
}

AREXPORT void ArSignalHandler::unblock(Signal sig)
{
}

AREXPORT void ArSignalHandler::handle(Signal sig)
{
}

AREXPORT void ArSignalHandler::unhandle(Signal sig)
{
}

AREXPORT void ArSignalHandler::addHandlerCB(ArFunctor1<int> *func,
					    ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    ourHandlerList.push_front(func);
  else if (position == ArListPos::LAST)
    ourHandlerList.push_back(func);
  else
    ArLog::log(ArLog::Terse, 
	       "ArSignalHandler::addHandler: Invalid position.");
}

AREXPORT void ArSignalHandler::delHandlerCB(ArFunctor1<int> *func)
{
  ourHandlerList.remove(func);
}

/**
   Removes all of the signal handler callback from the list of callbacks. 
**/
AREXPORT void ArSignalHandler::delAllHandlerCBs(void)
{
  ourHandlerList.clear();
}


AREXPORT ArSignalHandler * ArSignalHandler::getHandler()
{
  if (!ourSignalHandler)
    ourSignalHandler=new ArSignalHandler;

  return(ourSignalHandler);
}

AREXPORT void ArSignalHandler::blockCommonThisThread()
{
}

AREXPORT void ArSignalHandler::blockAllThisThread()
{
}

ArSignalHandler::ArSignalHandler()
{
  setThreadName("ArSignalHandler");
  initSigMap();
}

ArSignalHandler::~ArSignalHandler()
{
}

AREXPORT void * ArSignalHandler::runThread(void *arg)
{
  threadStarted();
  return(0);
}

void ArSignalHandler::initSigMap()
{
  ourSigMap[SigHUP]="SIGHUP";
  ourSigMap[SigINT]="SIGINT";
  ourSigMap[SigQUIT]="SIGQUIT";
  ourSigMap[SigILL]="SIGILL";
  ourSigMap[SigTRAP]="SIGTRAP";
  ourSigMap[SigABRT]="SIGABRT";
  //ourSigMap[SigIOT]="SIGIOT";
  ourSigMap[SigBUS]="SIGBUS";
  ourSigMap[SigFPE]="SIGFPE";
  ourSigMap[SigKILL]="SIGKILL";
  ourSigMap[SigUSR1]="SIGUSR1";
  ourSigMap[SigSEGV]="SIGSEGV";
  ourSigMap[SigUSR2]="SIGUSR2";
  ourSigMap[SigPIPE]="SIGPIPE";
  ourSigMap[SigALRM]="SIGALRM";
  ourSigMap[SigTERM]="SIGTERM";
  //ourSigMap[SigSTKFLT]="SIGSTKFLT";
  ourSigMap[SigCHLD]="SIGCHLD";
  ourSigMap[SigCONT]="SIGCONT";
  ourSigMap[SigSTOP]="SIGSTOP";
  ourSigMap[SigTSTP]="SIGTSTP";
  ourSigMap[SigTTIN]="SIGTTIN";
  ourSigMap[SigTTOU]="SIGTTOU";
  ourSigMap[SigURG]="SIGURG";
  ourSigMap[SigXCPU]="SIGXCPU";
  ourSigMap[SigXFSZ]="SIGXFSZ";
  ourSigMap[SigVTALRM]="SIGVTALRM";
  ourSigMap[SigPROF]="SIGPROF";
  ourSigMap[SigWINCH]="SIGWINCH";
  ourSigMap[SigIO]="SIGIO";
  ourSigMap[SigPWR]="SIGPWR";
}

AREXPORT const char * ArSignalHandler::nameSignal(int sig)
{
  return(ourSigMap[sig].c_str());
}

AREXPORT void ArSignalHandler::logThread(void)
{
  if (ourSignalHandler != NULL)
    ourSignalHandler->logThreadInfo();
  else
    ArLog::log(ArLog::Normal, "No signal handler thread running");
}
