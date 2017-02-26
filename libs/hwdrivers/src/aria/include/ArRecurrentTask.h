/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// ArRecurrentTask.h -- Recurrent async task interface class
#ifndef ARRECURASYNCTASK_H
#define ARRECURASYNCTASK_H


#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include "ArThread.h"
#include "ArASyncTask.h"

/// Recurrent task (runs in its own thread)
/**
   The ArRecurrentTask is a task that runs in its own thread.  Recurrent
   tasks are asynchronous tasks that complete in a finite amount of time,
   and need to be reinvoked recurrently.  A typical example is Saphira's
   localization task: it runs for a few hundred milliseconds, localizes the
   robot, and returns.  Then the cycle starts over.
   The user simply needs to derive their own class
   from ArRecurrentTask and define the task() function.  This is the user
   code that will be called to execute the task body.
   Then, create an object of the class, and call the go() function to 
   start the task.  The status of the task can be checked with the
   done() function, which returns 0 if running, 1 if completed, and 2 if
   killed.  
   go() can be called whenever the task is done to restart it.  To stop the
   task in midstream, call reset().  
   kill() kills off the thread, shouldn't be used unless exiting the 
   async task permanently
*/
class ArRecurrentTask : public ArASyncTask
{
public:
  /// Constructor
  AREXPORT ArRecurrentTask();
  /// Descructor
  AREXPORT ~ArRecurrentTask();	
  /// The main run loop
  /**
     Override this function and put your task here. 
  */
  virtual void task() = 0;
  /// Starts up on cycle of the recurrent task
  AREXPORT void go();		
  /// Check if the task is running or not
  /**
     0 = running, 1 = finished normally, 2 = canceled
  */
  AREXPORT int  done();	
  /// Cancel the task and reset for the next cycle
  AREXPORT void reset();	// stops the current thread and restarts it
  AREXPORT void kill();	        // kills the current thread

  AREXPORT void *runThread(void *ptr); // main task loop

private:
  bool running;			// true if currently running
  bool go_req;			// run request
  bool killed;			// did we get killed by request?
};


#endif // ARRECURASYNCTASK_H
