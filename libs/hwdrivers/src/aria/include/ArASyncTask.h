/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Threadable.h -- Threadable interface class
#ifndef ARQASYNCTASK_H
#define ARQASYNCTASK_H


#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include "ArThread.h"


/// Asynchronous task (runs in its own thread)
/**
   The ArAsynTask is a task that runs in its own thread. This is a
   rather simple class. The user simply needs to derive their own
   class from ArAsyncTask and define the runThread() function. They
   then need to create an instance of their task and call run or
   runAsync. The standard way to stop a task is to call stopRunning()
   which sets ArThread::myRunning to false. In their run loop, they
   should pay attention to the getRunning() or the ArThread::myRunning
   variable. If this value goes to false, the task should clean up
   after itself and exit its runThread() function.  
**/
class ArASyncTask : public ArThread
{
public:

  /// Constructor
  AREXPORT ArASyncTask();
  /// Destructor
  AREXPORT virtual ~ArASyncTask();

  /// The main run loop
  /**
     Override this function and put your taskes run loop here. Check the
     value of getRunning() periodicly in your loop. If the value
     is false, the loop should exit and runThread() should return.
     The argument and return value are specific to the platform thread implementation, and 
     can be ignored.
     @swignote In the wrapper libraries, this method takes no arguments and has no return value.
  */
  AREXPORT virtual void * runThread(void *arg) = 0;

  /// Run in this thread
  /*AREXPORT*/ virtual void run(void) { runInThisThread(); }
  
  /// Run in its own thread
  /*AREXPORT*/ virtual void runAsync(void) { create(); }

  // reimplemented here just so its easier to see in the docs
  /// Stop the thread
  /*AREXPORT*/ virtual void stopRunning(void) {myRunning=false;}

  /// Create the task and start it going
  AREXPORT virtual int create(bool joinable=true, bool lowerPriority=true);

  /// Run the code of the task syncronously
  AREXPORT virtual void * runInThisThread(void *arg=0);

private:

  // Hide regular Thread::Create
  virtual int create(ArFunctor * /*func*/, bool /*joinable=true*/,
		     bool /*lowerPriority=true*/) {return(0);}


  ArRetFunctor1C<void*, ArASyncTask, void*> myFunc;
};


#endif // ARASYNCTASK_H
