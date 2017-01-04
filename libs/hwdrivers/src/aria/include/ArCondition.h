/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARCONDITION_H
#define ARCONDITION_H


#ifndef WIN32
#include <pthread.h>
#include "ArMutex.h"
#endif
#include "ariaTypedefs.h"


/// Threading condition wrapper class
class ArCondition
{
public:

  enum {
    STATUS_FAILED=1, ///< General failure
    STATUS_FAILED_DESTROY, ///< Another thread is waiting on this condition so it can not be destroyed
    STATUS_FAILED_INIT, ///< Failed to initialize thread. Requested action is imposesible
    STATUS_WAIT_TIMEDOUT, ///< The timedwait timed out before signaling
    STATUS_WAIT_INTR, ///< The wait was interupted by a signal
    STATUS_MUTEX_FAILED_INIT, ///< The underlying mutex failed to init
    STATUS_MUTEX_FAILED ///< The underlying mutex failed in some fashion
  };

  /** @internal */
#ifdef WIN32
  typedef HANDLE CondType;
#else
  typedef pthread_cond_t CondType;
#endif

  /// Constructor
  AREXPORT ArCondition();
  /// Desctructor
  AREXPORT virtual ~ArCondition();

  /// Signal the thread waiting
  AREXPORT int signal();
  /// Broadcast a signal to all threads waiting
  AREXPORT int broadcast();
  /** @brief Wait for a signal */
  AREXPORT int wait();
  /// Wait for a signal for a period of time in milliseconds
  AREXPORT int timedWait(unsigned int msecs);
  /// Translate error into string
  AREXPORT const char *getError(int messageNumber) const;

protected:

  static ArStrMap ourStrMap;

  bool myFailedInit;
  CondType myCond;
#ifdef WIN32
  int myCount;
#else
  ArMutex myMutex;
#endif
};


#endif // ARCONDITION_H
