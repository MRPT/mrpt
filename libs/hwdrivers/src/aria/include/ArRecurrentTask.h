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
