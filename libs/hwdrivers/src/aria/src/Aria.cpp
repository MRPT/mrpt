/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "Aria.h"
#include "ArSocket.h"
#include "ArSignalHandler.h"
#include "ArKeyHandler.h"
#include "ArJoyHandler.h"
#ifndef ARINTERFACE
#include "ArModuleLoader.h"
#include "ArRobotJoyHandler.h"
#endif // ARINTERFACE

ArGlobalFunctor1<int> Aria::ourSignalHandlerCB(&Aria::signalHandlerCB);
ArMutex Aria::ourShuttingDownMutex;
bool Aria::ourShuttingDown=false;
bool Aria::ourExiting=false;
std::string Aria::ourDirectory="";
std::list<ArFunctor*> Aria::ourInitCBs;
std::list<ArFunctor*> Aria::ourUninitCBs;
ArKeyHandler *Aria::ourKeyHandler = NULL;
ArJoyHandler *Aria::ourJoyHandler = NULL;
bool Aria::ourInited = false;
bool Aria::ourRunning = false;
ArMutex Aria::ourExitCallbacksMutex;
std::multimap<int, ArFunctor *> Aria::ourExitCallbacks;
bool Aria::ourSigHandleExitNotShutdown = true;
std::multimap<int, ArRetFunctor<bool> *> Aria::ourParseArgCBs;
ArLog::LogLevel Aria::ourParseArgsLogLevel = ArLog::Verbose;
std::multimap<int, ArFunctor *> Aria::ourLogOptionsCBs;

#ifndef ARINTERFACE
std::list<ArRobot*> Aria::ourRobots;
ArConfig Aria::ourConfig;
ArRobotJoyHandler *Aria::ourRobotJoyHandler = NULL;
ArStringInfoGroup Aria::ourInfoGroup;
#endif // ARINTERFACE

/**
   This must be called first before any other Aria functions.
   It initializes the thread layer and the signal handling method. For
   Windows it iniatializes the socket layer as well. This also sets the
   directory Aria is located in from the ARIA environmental variable,
   for a description of this see getDirectory and setDirectory.

   For Linux the default signal handling method is to cleanly close down the
   program, cause all the instances of ArRobot to stop their run loop and
   disconnect from their robot. The program will exit on the following signals:
   SigHUP, SigINT, SigQUIT, and SigTERM.

   For Windows, there is no signal handling.

   This method also adds the file /etc/Aria.args and the environment variable
   ARIAARGS as locations for ArArgumentParser to obtain default argument values
   from. 

   @param method the method in which to handle signals. Defaulted to SIGHANDLE_SINGLE.
   @param initSockets specify whether or not to initialize the socket layer. This is only meaningfull for Windows. Defaulted to true.

   @param sigHandleExitNotShutdown if this is true and a signal
   happens Aria will use exit to quit instead of shutdown, false will
   still use shutdown which is the old behavior

   @see ArSignalHandler
   @see ArSocket

 */
AREXPORT void Aria::init(SigHandleMethod method, bool initSockets, 
			 bool sigHandleExitNotShutdown)
{
  std::list<ArFunctor*>::iterator iter;
  std::string str;
  char buf[1024];

  if (ourInited == true)
    return;

  ourRunning = true;
#ifndef WIN32
  srand48(time(NULL));
#endif

  ArThread::init();

  char* overrideSigMethod = getenv("ARIA_SIGHANDLE_METHOD");
  if(overrideSigMethod)
  {
    ArLog::log(ArLog::Terse, "Overriding signal handler method with %s from ARIA_SIGHANDLE_METHOD environment variable.", overrideSigMethod);
    if(!strcmp(overrideSigMethod, "NONE"))
      method = SIGHANDLE_NONE;
    else if(!strcmp(overrideSigMethod, "SINGLE"))
      method = SIGHANDLE_SINGLE;
    else if(!strcmp(overrideSigMethod, "THREAD"))
      method = SIGHANDLE_THREAD;
  }

  if (method != SIGHANDLE_NONE)
  {
    ArSignalHandler::addHandlerCB(&ourSignalHandlerCB, ArListPos::LAST);
    ArSignalHandler::blockCommon();
    ArSignalHandler::handle(ArSignalHandler::SigHUP);
    ArSignalHandler::handle(ArSignalHandler::SigINT);
    ArSignalHandler::handle(ArSignalHandler::SigQUIT);
    ArSignalHandler::handle(ArSignalHandler::SigTERM);
    ArSignalHandler::handle(ArSignalHandler::SigPIPE);
    if (method == SIGHANDLE_SINGLE)
        ArSignalHandler::createHandlerNonThreaded();
    else if (method == SIGHANDLE_THREAD)
    {
      ArSignalHandler::blockCommonThisThread();
      ArSignalHandler::createHandlerThreaded();
    }
  }

  if (initSockets)
    ArSocket::init();

  if (ourDirectory.length() == 0)
  {
    if (getenv("ARIA") != NULL)
    {
      setDirectory(getenv("ARIA"));
    }
    else
    {
#ifndef WIN32
      ArUtil::getStringFromFile("/etc/Aria", buf, sizeof(buf));
      str = buf;
#else // WIN32
      if (ArUtil::findFirstStringInRegistry(
          "SOFTWARE\\MobileRobots\\Aria",
          "Install Directory", buf, 1024))
        str = buf;
      else
        if (ArUtil::findFirstStringInRegistry(
            "SOFTWARE\\ActivMedia Robotics\\Aria",
            "Install Directory", buf, 1024))
          str = buf;
        else
          str = "";
        
  #endif // WIN32
        if (str.length() > 0)
        {
    setDirectory(str.c_str());
        }
        else
        {
  #ifndef ARINTERFACE
    ArLog::log(ArLog::Terse, "NonCritical Error: ARIA could not find where it is located.");
  #else
    ArLog::log(ArLog::Verbose, "NonCritical Error: ARIA could not find where it is located.");
  #endif
        }
      }
    }
    ourSigHandleExitNotShutdown = sigHandleExitNotShutdown;

  ourInited = true;
  for (iter=ourInitCBs.begin(); iter !=  ourInitCBs.end(); ++iter)
    (*iter)->invoke();
  ArArgumentParser::addDefaultArgumentFile("/etc/Aria.args");
  ArArgumentParser::addDefaultArgumentEnv("ARIAARGS");
  
}

/**
   This must be called last, after all other Aria functions.
   For both Linux and Windows, it closes all the open ArModules. For Windows
   it deinitializes the socket layer as well.
*/
AREXPORT void Aria::uninit()
{
  std::list<ArFunctor*>::iterator iter;

  for (iter=ourUninitCBs.begin(); iter != ourUninitCBs.end(); ++iter)
    (*iter)->invoke();

#ifndef ARINTERFACE
  ArModuleLoader::closeAll();
#endif // ARINTERFACE
  ArSocket::shutdown();
}

/**
   This will add a callback to the list of callbacks to call when Aria
   has been initialized. It can be called before anything else.
*/
AREXPORT void Aria::addInitCallBack(ArFunctor *cb, ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    ourInitCBs.push_front(cb);
  else
    ourInitCBs.push_back(cb);
}

/**
   This will add a callback to the list of callbacks to call right before Aria
   is un-initialized. It can be called before anything else. This facilitates
   code that in operating system signal handlers simply calls Aria::uninit()
   and packages that are based on Aria are unitited as well. It simplifies
   the entire uninit process.
*/
AREXPORT void Aria::addUninitCallBack(ArFunctor *cb, ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    ourUninitCBs.push_front(cb);
  else
    ourUninitCBs.push_back(cb);
}

/**
   This calls stop() on all ArThread's and ArASyncTask's. It will
   block until all ArThread's and ArASyncTask's exit. It is expected
   that all the tasks will obey the ArThread::myRunning variable and
   exit when it is false.
*/
AREXPORT void Aria::shutdown()
{
  bool doExit=false;

  ourShuttingDownMutex.lock();
  ourRunning = false;
  if (ourShuttingDown)
    doExit=true;
  else
    ourShuttingDown=true;
  ourShuttingDownMutex.unlock();

  if (doExit)
    return;

  ArThread::stopAll();
  ArThread::joinAll();
  uninit();
}

/**
   This will call the list of Aria exit callbacks and then exit with
   the given exit code.  Note that this could be called from anywhere
   and things may be locked or unlocked when called and the exit
   callbacks MUST exit and cannot wait for a lock.
**/
AREXPORT void Aria::exit(int exitCode)
{
  bool doReturn = false;
  ourShuttingDownMutex.lock();
  ourRunning = false;
  if (ourExiting)
    doReturn=true;
  else
    ourExiting=true;
  ourShuttingDownMutex.unlock();

  if (doReturn)
    return;

  callExitCallbacks();
  ::exit(exitCode);
}

AREXPORT void Aria::callExitCallbacks(void)
{
  ourRunning = false;
  std::multimap<int, ArFunctor *>::reverse_iterator it;
 
  ourExitCallbacksMutex.lock();
  ArLog::log(ArLog::Verbose, "Aria::exit: Starting exit callbacks");
  for (it = ourExitCallbacks.rbegin(); it != ourExitCallbacks.rend(); it++)
  {
    ArLog::log(ArLog::Verbose, 
	       "Aria::exit: Calling callback at position %d with name '%s'",
	       (*it).first, (*it).second->getName());
    (*it).second->invoke();
  }
  ArLog::log(ArLog::Verbose, "Aria::exit: Finished exit callbacks");
  ourExitCallbacksMutex.unlock();
} 
 
AREXPORT void Aria::addExitCallback(ArFunctor *functor, int position)
{
  ourExitCallbacksMutex.lock();
  ArLog::log(ArLog::Verbose, 
	"Aria::addExitCallback: Adding callback at position %d with name '%s'",
	     position, functor->getName());
  ourExitCallbacks.insert(std::pair<int, ArFunctor *>(position, functor));
  ourExitCallbacksMutex.unlock();
}

/*
AREXPORT void Aria::remExitCallback(ArFunctor *functor)
{
  std::multimap<int, ArFunctor *>::iterator it;
  
  ourExitCallbacksMutex.lock();
  for (it = ourExitCallbacks.begin(); it != ourExitCallbacks.end(); it++)
  {
    if ((*it).second == functor)
    {
      ourExitCallbacks.erase(it);
      ourExitCallbacksMutex.unlock();
      return remExitCallback(functor);
    }
  }
  ourExitCallbacksMutex.unlock();
}
*/

/**
   This method is roughly obsolete, it forces all the threads to die
   and then exits... The new way is exit.

   This calls cancel() on all AtThread's and ArASyncTask's. It forces
   each thread to exit and should only be used in the case of a thread
   hanging or getting stuck in an infinite loop. This works fine in Linux.
   In Windows it is not recommended at all that this function be called.
   Windows can not handle cleanly killing off a thread. See the help in
   the VC++ compiler on the WIN32 function TerminateThread. The biggest
   problem is that the state of DLL's can be destroyed.  
**/

AREXPORT void Aria::exitOld(int exitCode)
{
  ourRunning = false;
  ArThread::cancelAll();
  uninit();
  ::exit(exitCode);
}

#ifndef ARINTERFACE
AREXPORT void Aria::addRobot(ArRobot *robot)
{
  ourRobots.push_back(robot);
}

AREXPORT void Aria::delRobot(ArRobot *robot)
{
  ourRobots.remove(robot);
}

/**
   @param name the name of the robot you want to find
   @return NULL if there is no robot of that name, otherwise the robot with 
   that name
*/
AREXPORT ArRobot *Aria::findRobot(char *name)
{
  std::string rname;
  std::list<ArRobot *>::iterator it;
  if (name == NULL)
    return NULL;

  rname = name;
  for (it = ourRobots.begin(); it != ourRobots.end(); it++)
  {
    if ((*it)->getName() == rname)
      return (*it);
  }
  return NULL;

}

AREXPORT std::list<ArRobot*> * Aria::getRobotList()
{
  return(&ourRobots);
}

#endif // ARINTERFACE

AREXPORT void Aria::signalHandlerCB(int sig)
{

  // if we want to exit instead of shutdown then do that ( call never returns)
  if (ourSigHandleExitNotShutdown)
  {
    ArLog::log(ArLog::Normal, "Aria: Received signal '%s'. Exiting.",
	       ArSignalHandler::nameSignal(sig));
    Aria::exit(0);
    // we shouldn't need this here, since the program should already
    // exited... but just in case
    ::exit(0);
  }


  ourShuttingDownMutex.lock();
  if (!ourRunning)
  {
    ourShuttingDownMutex.unlock();
    return;
  }
  ourShuttingDownMutex.unlock();


  ArLog::log(ArLog::Normal, "Aria: Received signal '%s'. Shutting down.",
	     ArSignalHandler::nameSignal(sig));

#ifndef ARINTERFACE
  std::list<ArRobot*>::iterator iter;
  if ((sig == ArSignalHandler::SigINT) || (sig == ArSignalHandler::SigHUP) ||
      (sig == ArSignalHandler::SigTERM))
  {
    for (iter=ourRobots.begin(); iter != ourRobots.end(); ++iter)
      (*iter)->stopRunning();
  }
#endif //ARINTERFACE

  // I'm disregarding this advice below since I can't seem to get
  // anything else to work well and haven't seen problems from it

  // dont do an Aria::shutdown() here because we want the main()
  // function to do the ArThread::joinAll(). Otherwise variables on
  // the stack frame of main() may get destructed if main() happens to
  // exit before other threads.  And some of those variables may be
  // used by those threads.
  shutdown();
}

/**
   This sets the directory that ARIA is located in, so ARIA can find param
   files and the like.  This can also be controlled by the environment variable
   ARIA, which this is set to (if it exists) when Aria::init is done.  So 
   for setDirectory to be effective, it must be done after the Aria::init.
   @param directory the directory Aria is located in
   @see getDirectory
*/
AREXPORT void Aria::setDirectory(const char *directory)
{
  int ind;
  if (directory != NULL)
  {
    ourDirectory = directory;
    ind = strlen(directory) - 1;
    if (ind < 0)
      ind = 0;
    if (directory[ind] != '/' && directory[ind] != '\\')
    {
#ifdef WIN32
      ourDirectory += "\\";
#else // win32
      ourDirectory += "/";
#endif // win32
    }
#ifndef ARINTERFACE
    ourConfig.setBaseDirectory(ourDirectory.c_str());
#endif // ARINTERFACE
  }
}

/**
   This gets the directory that ARIA is located in, this is so ARIA can find 
   param files and the like.  
   @return the directory ARIA is located in
   @see setDirectory
*/
AREXPORT const char *Aria::getDirectory(void)
{
  return ourDirectory.c_str();
}

/// Sets the key handler, so that other classes can find it
AREXPORT void Aria::setKeyHandler(ArKeyHandler *keyHandler)
{
  ourKeyHandler = keyHandler;
}

/// Gets the key handler if one has been set
AREXPORT ArKeyHandler *Aria::getKeyHandler(void)
{
  return ourKeyHandler;
}

/// Sets the joy handler, so that other classes can find it
AREXPORT void Aria::setJoyHandler(ArJoyHandler *joyHandler)
{
  ourJoyHandler = joyHandler;
}

/// Gets the joy handler if one has been set
AREXPORT ArJoyHandler *Aria::getJoyHandler(void)
{
  return ourJoyHandler;
}

#ifndef ARINTERFACE
/// Sets the robot joy handler, so that other classes can find it
AREXPORT void Aria::setRobotJoyHandler(ArRobotJoyHandler *robotJoyHandler)
{
  ourRobotJoyHandler = robotJoyHandler;
}

/// Gets the robot joy handler if one has been set
AREXPORT ArRobotJoyHandler *Aria::getRobotJoyHandler(void)
{
  return ourRobotJoyHandler;
}

/**
   This gets the global config aria uses.
 **/
AREXPORT ArConfig *Aria::getConfig(void)
{
  return &ourConfig;
}

/**
   This gets the global string group aria uses.
 **/
AREXPORT ArStringInfoGroup *Aria::getInfoGroup(void)
{
  return &ourInfoGroup;
}

#endif // ARINTERFACE

/**
   This returns if the ARIA stuff is running, which is defined as the
   time between Aria::init and any of Aria::shutdown, Aria::exit, or
   the signal handler kicking off.
**/
AREXPORT bool Aria::getRunning(void)
{
  return ourRunning;
}

AREXPORT bool Aria::parseArgs(void)
{
  std::multimap<int, ArRetFunctor<bool> *>::reverse_iterator it;
  ArRetFunctor<bool> *callback;

  ArLog::log(ourParseArgsLogLevel, "Aria: Parsing arguments");
  for (it = ourParseArgCBs.rbegin(); it != ourParseArgCBs.rend(); it++)
  {
    callback = (*it).second;
    if (callback->getName() != NULL && callback->getName()[0] != '\0')
      ArLog::log(ourParseArgsLogLevel, 
		 "Aria: Calling parse arg functor '%s' (%d)", 
		 callback->getName(), (*it).first);
    else
      ArLog::log(ourParseArgsLogLevel, 
		 "Aria: Calling unnamed parse arg functor (%d)", 
		 (*it).first);

    if (!callback->invokeR())
    {
      return false;
    }
  }
  return true;
}

AREXPORT void Aria::logOptions(void)
{
  std::multimap<int, ArFunctor *>::reverse_iterator it;

  for (it = ourLogOptionsCBs.rbegin(); it != ourLogOptionsCBs.rend(); it++)
  {
    (*it).second->invoke();
    ArLog::log(ArLog::Terse, "");
    ArLog::log(ArLog::Terse, "");
  }
}

AREXPORT void Aria::addParseArgsCB(ArRetFunctor<bool> *functor, 
					  int position)
{
  ourParseArgCBs.insert(std::pair<int, ArRetFunctor<bool> *>(position, 
							     functor));
}

AREXPORT void Aria::setParseArgLogLevel(ArLog::LogLevel level)
{
  ourParseArgsLogLevel = level;
}

AREXPORT void Aria::addLogOptionsCB(ArFunctor *functor, int position)
{
  ourLogOptionsCBs.insert(std::pair<int, ArFunctor *>(position, functor));
}


