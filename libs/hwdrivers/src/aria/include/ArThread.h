/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARTHREAD_H
#define ARTHREAD_H


#include <map>
#ifndef WIN32
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include "ariaTypedefs.h"
#include "ArMutex.h"
#include "ArFunctor.h"
#include "ArLog.h"

/// POSIX/WIN32 thread wrapper class.
/**
  create() will create the thread. That thread will run the given Functor.

  A thread can either be in a detached state or a joinable state. If the
  thread is in a detached state, that thread can not be join()'ed upon. The
  thread will simply run until the program exits, or its function exits.
  A joinable thread means that another thread and call join() upon it. If
  this function is called, the caller will block until the thread exits
  its function. This gives a way to synchronize upon the lifespan of threads.

  Calling cancel() will cancel the thread.

  The static function self() will return a thread
*/
class ArThread
{
public:

#ifdef WIN32
  typedef HANDLE ThreadType;
#else
  typedef pthread_t ThreadType;
#endif

  typedef std::map<ThreadType, ArThread*> MapType;
  typedef enum {
    STATUS_FAILED=1, ///< Failed to create the thread
    STATUS_NORESOURCE, ///< Not enough system resources to create the thread
    STATUS_NO_SUCH_THREAD, ///< The thread can no longer be found
    STATUS_INVALID, ///< Thread is detached or another thread is joining on it
    STATUS_JOIN_SELF, ///< Thread is your own thread. Can't join on self.
    STATUS_ALREADY_DETATCHED ///< Thread is already detatched
  } Status;

  /// Constructor
  AREXPORT ArThread(bool blockAllSignals=true);
  /// Constructor - starts the thread
  AREXPORT ArThread(ThreadType thread, bool joinable,
		    bool blockAllSignals=true);
  /// Constructor - starts the thread
  AREXPORT ArThread(ArFunctor *func, bool joinable=true,
		    bool blockAllSignals=true);
  /// Destructor
  AREXPORT virtual ~ArThread();

  /// Initialize the internal book keeping structures
  AREXPORT static void init(void);
  /// Returns the instance of your own thread
  AREXPORT static ArThread * self(void);
  /// Stop all threads
  AREXPORT static void stopAll();
  /// Cancel all threads
  AREXPORT static void cancelAll(void);
  /// Join on all threads
  AREXPORT static void joinAll(void);
  /// Yield the processor to another thread
  AREXPORT static void yieldProcessor(void);
  /// Gets the logging level for thread information
  static ArLog::LogLevel getLogLevel(void) { return ourLogLevel; }
  /// Sets the logging level for thread information
  static void setLogLevel(ArLog::LogLevel level) { ourLogLevel = level; }

  /// Create and start the thread
  AREXPORT virtual int create(ArFunctor *func, bool joinable=true,
			      bool lowerPriority=true);
  /// Stop the thread
  /*AREXPORT*/ virtual void stopRunning(void) {myRunning=false;}
  /// Join on the thread
  AREXPORT virtual int join(void **ret=NULL);
  /// Detatch the thread so it cant be joined
  AREXPORT virtual int detach(void);
  /// Cancel the thread
  AREXPORT virtual void cancel(void);

  /// Get the running status of the thread
  /*AREXPORT*/ virtual bool getRunning(void) const {return(myRunning);}
  /// Get the running status of the thread, locking around the variable
  /*AREXPORT*/virtual bool getRunningWithLock(void)
    { bool running; lock(); running = myRunning; unlock(); return running; }
  /// Get the joinable status of the thread
  /*AREXPORT*/ virtual bool getJoinable(void) const {return(myJoinable);}
  /// Get the underlying thread type
  /*AREXPORT*/ virtual const ThreadType * getThread(void) const {return(&myThread);}
  /// Get the functor that the thread runs
  /*AREXPORT*/ virtual ArFunctor * getFunc(void) const {return(myFunc);}

  /// Set the running value on the thread
  /*AREXPORT*/ virtual void setRunning(bool running) {myRunning=running;}

  /// Lock the thread instance
  /*AREXPORT*/ int lock(void) {return(myMutex.lock());}
  /// Try to lock the thread instance without blocking
  /*AREXPORT*/ int tryLock(void) {return(myMutex.tryLock());}
  /// Unlock the thread instance
  /*AREXPORT*/ int unlock(void) {return(myMutex.unlock());}

  /// Do we block all process signals at startup?
  bool getBlockAllSignals(void) {return(myBlockAllSignals);}

  /// Gets the name of the functor
  virtual const char *getThreadName(void) { return myName.c_str();  }

  /// Sets the name of the thread
  virtual void setThreadName(const char *name) { myName = name; }

  /// Thread to call in the function that is this thread
  /**
     If you call this function in your functor (ie runThread) it'll
     then call some things for logging (to make debugging easier)
   **/
  AREXPORT virtual void threadStarted(void);

  /// Logs the information about this thread
  AREXPORT virtual void logThreadInfo(void);

protected:
  static ArMutex ourThreadsMutex;
  static MapType ourThreads;
  AREXPORT static ArLog::LogLevel ourLogLevel;

  AREXPORT virtual int doJoin(void **ret=NULL);

  std::string myName;

  ArMutex myMutex;
  /// State variable to denote when the thread should continue or exit
  bool myRunning;
  bool myJoinable;
  bool myBlockAllSignals;
  ArFunctor *myFunc;
  ThreadType myThread;
  ArStrMap myStrMap;

#ifndef WIN32
  pid_t myPID;
#endif

};


#endif // ARTHREAD_H
