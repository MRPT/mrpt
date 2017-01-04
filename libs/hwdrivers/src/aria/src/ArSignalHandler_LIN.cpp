/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ArSignalHandler.h"
#include "ArLog.h"
#include "ariaInternal.h"
#include <mrpt/utils/mrpt_macros.h>

ArSignalHandler *ArSignalHandler::ourSignalHandler=0;
ArStrMap ArSignalHandler::ourSigMap;
sigset_t ArSignalHandler::ourBlockSigSet;
sigset_t ArSignalHandler::ourHandleSigSet;
std::list<ArFunctor1<int>*> ArSignalHandler::ourHandlerList;


void ArSignalHandler::signalCB(int sig)
{
  std::list<ArFunctor1<int>*>::iterator iter;

  ArLog::log(ArLog::Verbose,
	     "ArSignalHandler::runThread: Received signal '%s' Number %d ",
	     ourSigMap[sig].c_str(), sig);
  for (iter=ourHandlerList.begin(); iter != ourHandlerList.end(); ++iter)
    (*iter)->invoke(sig);
  if (ourHandlerList.begin() == ourHandlerList.end())
    ArLog::log(ArLog::Terse,
  "ArSignalHandler::runThread: No handler function. Unhandled signal '%s' Number %d", 
	       ourSigMap[sig].c_str(), sig);
}  

/**
   Sets up the signal handling for a non-threaded program. When the program
   This uses the system call signal(2). This should not be used if you have
   a threaded program.
   @see createHandlerThreaded
*/
AREXPORT void ArSignalHandler::createHandlerNonThreaded()
{
  int i;
  initSigMap();
  signal(SigSEGV, &signalCB);
  signal(SigFPE, &signalCB);
  for (i=1; i <= SigPWR; ++i)
  {
    if (sigismember(&ourBlockSigSet, i))
      signal(i, SIG_IGN);
    if (sigismember(&ourHandleSigSet, i))
      signal(i, &signalCB);
  }
  
}

/**
   Sets up the signal handling for a threaded program. This call is
   only useful for Linux. This will create a dedicated thread in which
   to handle signals. The thread calls sigwait(3) and waits for a
   signal to be sent.  By default all ArThread instances block all
   signals. Thus the signal is sent to the signal handler thread. This
   will allow the other threads to continue uninterrupted and not skew
   their timing loops.  
   @see createHandlerNonThreaded 
**/
AREXPORT void ArSignalHandler::createHandlerThreaded()
{
  signal(SigSEGV, &signalCB);
  signal(SigFPE, &signalCB);
  getHandler()->create(false);
}

/**
   Sets the signal handler to block all the common signals. The
   'common' signals are SIGHUP, SIGINT, SIGQUIT, SIGTERM, SIGSEGV, and
   SIGPIPE. Call this before calling createHandlerNonThreaded or
   createHandlerThreaded.  
**/
AREXPORT void ArSignalHandler::blockCommon()
{
  unblockAll();
  block(SigHUP);
  block(SigPIPE);
  block(SigINT);
  block(SigQUIT);
  block(SigTERM);
}

/**
   Unblock all the signals. Call this before calling createHandlerNonThreaded
   or createHandlerThreaded.
*/
AREXPORT void ArSignalHandler::unblockAll()
{
  sigemptyset(&ourBlockSigSet);
}

/**
   Block the given signal. Call this before calling createHandlerNonThreaded
   or createHandlerThreaded.
   @param sig the number of the signal
*/
AREXPORT void ArSignalHandler::block(Signal sig)
{
  sigaddset(&ourBlockSigSet, sig);
}

/**
   Unblock the given signal. Call this before calling createHandlerNonThreaded
   or createHandlerThreaded.
   @param sig the number of the signal
*/
AREXPORT void ArSignalHandler::unblock(Signal sig)
{
  sigdelset(&ourBlockSigSet, sig);
}

/**
   Handle the given signal. All the handler callbacks will be called with this
   signal when it is received. Call this before calling
   createHandlerNonThreaded or createHandlerThreaded.
   @param sig the number of the signal
*/
AREXPORT void ArSignalHandler::handle(Signal sig)
{
  unblock(sig);
  sigaddset(&ourHandleSigSet, sig);
}

/**
   Do not handle the given signal. Call this before calling
   createHandlerNonThreaded or createHandlerThreaded.
   @param sig the number of the signal
*/
AREXPORT void ArSignalHandler::unhandle(Signal sig)
{
  sigdelset(&ourHandleSigSet, sig);
}

/**
   Add a handler callback to the list of callbacks. When there is a signal
   sent to the process, the list of callbacks are invoked and passed the signal
   number.
   @param functor functor created from ArFunctorC1<int> which refers to the 
   function to call.
   @param position whether to place the functor first or last
*/
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

/**
   Remove a handler callback from the list of callbacks. 
   @param functor functor created from ArFunctorC1<int> which refers to the 
   function to call.
*/
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

/**
   Get a pointer to the single instance of the ArSignalHandler. The signal
   handler uses the singleton model, which means there can only be one
   instance of ArSignalHandler. If the single instance of ArSignalHandler
   has not been created, getHandler will create it. This is how the handler
   should be created.
   @return returns a pointer to the instance of the signal handler
*/
AREXPORT ArSignalHandler * ArSignalHandler::getHandler()
{
  if (!ourSignalHandler)
    ourSignalHandler=new ArSignalHandler;

  return(ourSignalHandler);
}

/**
   Block all the common signals for the calling thread. The calling thread
   will never receive the common signals which are SIGHUP, SIGINT, SIGQUIT,
   and SIGTERM. This function can be called at any time.
*/
AREXPORT void ArSignalHandler::blockCommonThisThread()
{
  sigset_t commonSet;
  sigemptyset(&commonSet);
  sigaddset(&commonSet, SigHUP);
  sigaddset(&commonSet, SigPIPE);
  sigaddset(&commonSet, SigINT);
  sigaddset(&commonSet, SigQUIT);
  sigaddset(&commonSet, SigTERM);
  //sigaddset(&commonSet, SigSEGV);
  pthread_sigmask(SIG_SETMASK, &commonSet, 0);
}

AREXPORT void ArSignalHandler::blockAllThisThread()
{
  sigset_t fullSet;
  sigfillset(&fullSet);
  pthread_sigmask(SIG_SETMASK, &fullSet, 0);
}


ArSignalHandler::ArSignalHandler() :
  ourIgnoreQUIT(false)
{
  setThreadName("ArSignalHandler");
  initSigMap();
}

ArSignalHandler::~ArSignalHandler()
{
}

AREXPORT void * ArSignalHandler::runThread(void *arg)
{
  MRPT_UNUSED_PARAM(arg);
  threadStarted();

  // I think the old code was broken in that it didn't block all the
  // signals it wanted to wait for, which sigwait is supposed to
  // do... it also didn't check the return... for some reason system
  // on a debian box (at least a newer one) causes sigwait to return
  // with an error state (return of 4)... the old sigwait from rh 7.x
  // said it never returned an error... I don't entirely understand
  // it, and thats why both blocks of code are here

  // old code
  /*
  int sig;
  
  pthread_sigmask(SIG_SETMASK, &ourBlockSigSet, 0);

  while (myRunning)
  {
    sigwait(&ourHandleSigSet, &sig);
    signalCB(sig);
  }

  return(0);
*/
  // new code
  int sig = 0;
  
  while (myRunning)
  {
    pthread_sigmask(SIG_SETMASK, &ourBlockSigSet, 0);
    pthread_sigmask(SIG_BLOCK, &ourHandleSigSet, 0);

    if (sigwait(&ourHandleSigSet, &sig) == 0)
      signalCB(sig);
  }
  return(0);
}

AREXPORT void ArSignalHandler::initSigMap()
{
  ourSigMap[SIGHUP]="SIGHUP";
  ourSigMap[SIGINT]="SIGINT";
  ourSigMap[SIGQUIT]="SIGQUIT";
  ourSigMap[SIGILL]="SIGILL";
  ourSigMap[SIGTRAP]="SIGTRAP";
  ourSigMap[SIGABRT]="SIGABRT";
#ifdef linux
  ourSigMap[SIGIOT]="SIGIOT";
#endif
  ourSigMap[SIGBUS]="SIGBUS";
  ourSigMap[SIGFPE]="SIGFPE";
  ourSigMap[SIGKILL]="SIGKILL";
  ourSigMap[SIGUSR1]="SIGUSR1";
  ourSigMap[SIGSEGV]="SIGSEGV";
  ourSigMap[SIGUSR2]="SIGUSR2";
  ourSigMap[SIGPIPE]="SIGPIPE";
  ourSigMap[SIGALRM]="SIGALRM";
  ourSigMap[SIGTERM]="SIGTERM";
  //ourSigMap[SIGSTKFLT]="SIGSTKFLT";
  ourSigMap[SIGCHLD]="SIGCHLD";
  ourSigMap[SIGCONT]="SIGCONT";
  ourSigMap[SIGSTOP]="SIGSTOP";
  ourSigMap[SIGTSTP]="SIGTSTP";
  ourSigMap[SIGTTIN]="SIGTTIN";
  ourSigMap[SIGTTOU]="SIGTTOU";
  ourSigMap[SIGURG]="SIGURG";
  ourSigMap[SIGXCPU]="SIGXCPU";
  ourSigMap[SIGXFSZ]="SIGXFSZ";
  ourSigMap[SIGVTALRM]="SIGVTALRM";
  ourSigMap[SIGPROF]="SIGPROF";
  ourSigMap[SIGWINCH]="SIGWINCH";
  ourSigMap[SIGIO]="SIGIO";
#ifdef linux
  ourSigMap[SIGPWR]="SIGPWR";
#endif
}

AREXPORT const char *ArSignalHandler::nameSignal(int sig)
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
