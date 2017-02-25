/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSIGNALHANDLER_H
#define ARSIGNALHANDLER_H


#include <list>
#ifdef WIN32
#else
#include <signal.h>
#endif
#include "ariaTypedefs.h"
#include "ArASyncTask.h"
#include "ArFunctor.h"


/// Signal handling class
/**
   This is a signal handling class. It has both a threaded and non-threaded
   mode of operation. The non-threaded mode will work in a threaded application
   but it is best to use the threaded mode. The benefit of the threaded mode
   is that if the signal incures some processing, but does not shutdown the
   program (ie. SIGUSR1 or SIGUSR2), the threaded mode will handle the signal
   in its own thread and hopefully that will not hurt the performance of the
   tight loop robot control. Exaclty how much performance you get out of this
   depends on your machines physical hardware and exactly what the processing
   the signal handler does. For instance, a multi-processor machine has a much
   greater chance of the signal handler not interfering with the robot control
   loop.

   See the Aria main class for how to initialize a default setup of the
   signal handling.

   There are functions to block, unblock, handle and unhandle signals. These
   functions all must be called before creating the signalhandler. In either
   single or multi-threaded mode. The functions to block and handle signals
   creates a set of blocking and handling which is then used by the create
   functions to tell the Linux kernel what to do.

   In the threaded mode, there is a signal handler thread that is created.
   That thread is created in a detached state, which means it can not be
   joined on. When the program exits, the signal handler thread will be
   ignored and that thread will never exit its run loop. This is perfectly
   fine behavior. There is no state that can be messed up in this fashion.
   It is just easier to exit the program than to try to wake up that thread
   and get it to exit itself.

   This class is for Linux only. Windows has virtualy no support for signals
   and the little support that it does have is not realy usefull. There is
   an empty implementation of this class for Windows so that code can compile
   in both Linux and Windows. Just do not expect the code that uses this
   signal handling to do anything in Windows. This should not be a problem
   since signals are not used in Windows.
*/
class ArSignalHandler : public ArASyncTask
{
public:

  typedef enum
  {
    SigHUP=1, SigINT, SigQUIT, SigILL, SigTRAP, SigABRT, SigBUS, SigFPE, 
    SigKILL, SigUSR1, SigSEGV, SigUSR2, SigPIPE, SigALRM, SigTERM, SigSTKFLT, 
    SigCHLD, SigCONT, SigSTOP, SigTSTP, SigTTIN, SigTTOU, SigURG,
    SigXCPU, SigXFSZ, SigVTALRM, SigPROF, SigWINCH, SigIO, SigPWR
  } Signal;

  /// Setup the signal handling for a non-threaded program
  AREXPORT static void createHandlerNonThreaded();

  /// Setup the signal handling for a multi-threaded program
  AREXPORT static void createHandlerThreaded();

  /// Block all the common signals the kill a program
  AREXPORT static void blockCommon();

  /// Unblock all the signals
  AREXPORT static void unblockAll();

  /// Block the given signal
  AREXPORT static void block(Signal sig);

  /// Unblock the given signal
  AREXPORT static void unblock(Signal sig);

  /// Handle the given signal
  AREXPORT static void handle(Signal sig);

  /// Dont handle the given signal
  AREXPORT static void unhandle(Signal sig);

  /// Add a handler callback
  AREXPORT static void addHandlerCB(ArFunctor1<int> *func,
				    ArListPos::Pos position);

  /// Remove a handler callback
  AREXPORT static void delHandlerCB(ArFunctor1<int> *func);

  /// Removes all the handlers
  AREXPORT static void delAllHandlerCBs(void);

  /// Get a pointer to the single ArSignalHandler instance
  AREXPORT static ArSignalHandler * getHandler();

  /// Get the name of the given signal
  AREXPORT static const char * nameSignal(int sig);

  /// Block all the common signals for the calling thread only
  AREXPORT static void blockCommonThisThread();

  /// Block all the signals for the calling thread only
  AREXPORT static void blockAllThisThread();

  /// Destructor
  virtual ~ArSignalHandler();

  AREXPORT virtual void * runThread(void *arg);

  AREXPORT static void signalCB(int sig);

  AREXPORT static void logThread(void);
protected:

  ArSignalHandler();

  static void initSigMap();

  bool ourIgnoreQUIT;

  static ArSignalHandler *ourSignalHandler;
  static ArStrMap ourSigMap;
#ifdef WIN32
#else
  static sigset_t ourBlockSigSet;
  static sigset_t ourHandleSigSet;
#endif
  static std::list<ArFunctor1<int>*> ourHandlerList;
};


#endif // ARSIGNALHANDLER_H
