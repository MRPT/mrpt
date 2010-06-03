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
		     bool /*lowerPriority=true*/) {return(false);}


  ArRetFunctor1C<void*, ArASyncTask, void*> myFunc;
};


#endif // ARASYNCTASK_H
