/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARMUTEX_H
#define ARMUTEX_H

#ifndef WIN32
#include <pthread.h>
#endif
#include <string>
#include "ariaTypedefs.h"


/// Mutex wrapper class
/**
   This class wraps the operating system's mutex functions. It allows mutualy
   exclusive access to a critical section. This is extremely useful for
   multiple threads which want to use the same variable. On Linux, ArMutex simply
   uses the POSIX pthread interface in an object oriented manner. It also
   applies the same concept to Windows using Windows' own abilities to restrict
   access to critical sections.
*/
class ArMutex
{
public:

#ifdef WIN32
  typedef HANDLE MutexType;
#else
  typedef pthread_mutex_t MutexType;
#endif

  typedef enum {
    STATUS_FAILED_INIT=1, ///< Failed to initialize
    STATUS_FAILED, ///< General failure
    STATUS_ALREADY_LOCKED ///< Mutex already locked
  } Status;

  /// Constructor
  AREXPORT ArMutex();
  /// Destructor
  AREXPORT virtual ~ArMutex();

  /// Lock the mutex
  AREXPORT virtual int lock();
  /// Try to lock the mutex, but do not block
  AREXPORT virtual int tryLock();
  /// Unlock the mutex, allowing another thread to obtain the lock
  AREXPORT virtual int unlock();
  /// Get a human readable error message from an error code
  AREXPORT virtual const char * getError(int messageNumber) const;
  /// Sets a flag that will log out when we lock and unlock (not trylock)
  /*AREXPORT*/ void setLog(bool log) { myLog = log; } 
  /// Sets a name we'll use to log with
  /*AREXPORT*/ void setLogName(const char *logName) { myLogName = logName; } 
  /// Get a reference to the underlying mutex variable
  /*AREXPORT*/ virtual MutexType & getMutex() {return(myMutex);}

protected:

  bool myFailedInit;
  MutexType myMutex;
  ArStrMap myStrMap;
  bool myLog;
  std::string myLogName;
};


#endif // ARMUTEX_H
