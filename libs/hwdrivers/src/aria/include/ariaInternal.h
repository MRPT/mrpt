/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARIAINTERNAL_H
#define ARIAINTERNAL_H


#include "ArMutex.h"
#include "ArFunctor.h"
#include "ArConfig.h"

#ifndef ARINTERFACE
#include "ArStringInfoGroup.h"
class ArRobot;
class ArRobotJoyHandler;
#endif // ARINTERFACE

class ArKeyHandler;
class ArJoyHandler;


/// This class performs global initialization and deinitialization
class Aria
{
public:

  typedef enum {
    SIGHANDLE_SINGLE, ///< Setup signal handlers in a global, non-thread way
    SIGHANDLE_THREAD, ///< Setup a dedicated signal handling thread
    SIGHANDLE_NONE ///< Do no signal handling
  } SigHandleMethod;

  /// Performs OS-specific initialization
  AREXPORT static void init(SigHandleMethod method = SIGHANDLE_THREAD,
			    bool initSockets = true, 
			    bool sigHandleExitNotShutdown = true);

  /// Performs OS-specific deinitialization
  AREXPORT static void uninit();

  /// Adds a callback to call when Aria is inited
  AREXPORT static void addInitCallBack(ArFunctor *cb, ArListPos::Pos position);

  /// Adds a callback to call when Aria is uninited
  AREXPORT static void addUninitCallBack(ArFunctor *cb,
					 ArListPos::Pos position);

  /// Shutdown all Aria processes/threads
  AREXPORT static void shutdown();

  /// Force an exit of all Aria processes/threads
  AREXPORT static void exit(int exitCode = 0);

  /// Sees if Aria is still running (mostly for the thread in main)
  AREXPORT static bool getRunning(void);

  /// Sets the directory that ARIA resides in
  AREXPORT static void setDirectory(const char * directory);

  /// Gets the directory that ARIA resides in
  AREXPORT static const char *getDirectory(void);

  /// Parses the arguments for the program (calls all the parseArgCBs)
  AREXPORT static bool parseArgs(void);

  /// Logs all the options for the program (Calls all the logOptionsCBs)
  AREXPORT static void logOptions(void);

  /// Sets the key handler, so that other classes can find it
  AREXPORT static void setKeyHandler(ArKeyHandler *keyHandler);

  /// Gets the joystick handler if one has been set
  AREXPORT static ArKeyHandler *getKeyHandler(void);

  /// Sets the joystick handler, so that other classes can find it
  AREXPORT static void setJoyHandler(ArJoyHandler *joyHandler);

  /// Gets the joystick handler if one has been set
  AREXPORT static ArJoyHandler *getJoyHandler(void);

  /// Adds a functor to call on Aria::exit
  AREXPORT static void addExitCallback(ArFunctor *functor, int position = 50);

  /// Force an exit of all Aria processes/threads (the old way)
  AREXPORT static void exitOld(int exitCode = 0);

  /// Internal, the callback for the signal handling
  AREXPORT static void signalHandlerCB(int sig);

  /// Internal, calls the exit callbacks
  AREXPORT static void callExitCallbacks(void);

  /// Adds a callback for when we parse arguments 
  AREXPORT static void addParseArgsCB(ArRetFunctor<bool> *functor, 
				      int position = 50);

  /// Sets the log level for the parsing function
  AREXPORT static void setParseArgLogLevel(ArLog::LogLevel level);

  /// Adds a callback for when we log options
  AREXPORT static void addLogOptionsCB(ArFunctor *functor, int position = 50);

#ifndef ARINTERFACE
  /// Sets the robot joystick handler, so that other classes can find it
  AREXPORT static void setRobotJoyHandler(ArRobotJoyHandler *robotJoyHandler);

  /// Gets the robot joystick handler if one has been set
  AREXPORT static ArRobotJoyHandler *getRobotJoyHandler(void);

  /// Gets the ArConfig for this program
  AREXPORT static ArConfig *getConfig(void);

  /// Gets the ArStringInfoGroup for this program
  AREXPORT static ArStringInfoGroup *getInfoGroup(void);

  /// Add a robot to the global list of robots
  AREXPORT static void addRobot(ArRobot *robot);

  /// Remove a robot from the global list of robots
  AREXPORT static void delRobot(ArRobot *robot);

  /// Finds a robot in the global list of robots, by name
  AREXPORT static ArRobot *findRobot(char *name);

  /// Get a copy of the global robot list
  AREXPORT static std::list<ArRobot*> * getRobotList();
#endif // ARINTERFACE
protected:
  static bool ourInited;
  static ArGlobalFunctor1<int> ourSignalHandlerCB;
  static bool ourRunning;
  static ArMutex ourShuttingDownMutex;
  static bool ourShuttingDown;
  static bool ourExiting;
  static std::string ourDirectory;
  static std::list<ArFunctor*> ourInitCBs;
  static std::list<ArFunctor*> ourUninitCBs;
  static ArKeyHandler *ourKeyHandler;
  static ArJoyHandler *ourJoyHandler;
#ifndef ARINTERFACE
  static std::list<ArRobot*> ourRobots;
  static ArRobotJoyHandler *ourRobotJoyHandler;
  static ArConfig ourConfig;
  static ArStringInfoGroup ourInfoGroup;
#endif // ARINTERFACE
  static ArMutex ourExitCallbacksMutex;
  static std::multimap<int, ArFunctor *> ourExitCallbacks;
  static bool ourSigHandleExitNotShutdown;
  static std::multimap<int, ArRetFunctor<bool> *> ourParseArgCBs;
  static ArLog::LogLevel ourParseArgsLogLevel;
  static std::multimap<int, ArFunctor *> ourLogOptionsCBs;
};


#endif // ARIAINTERNAL_H
