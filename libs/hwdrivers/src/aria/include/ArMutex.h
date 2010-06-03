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
