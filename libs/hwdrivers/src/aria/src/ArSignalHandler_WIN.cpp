/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/


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
