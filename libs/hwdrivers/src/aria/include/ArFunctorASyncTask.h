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

#ifndef ARFUNCTORASYNCTASK_H
#define ARFUNCTORASYNCTASK_H

#include "ariaTypedefs.h"
#include "ArASyncTask.h"
#include "ArFunctor.h"

/// This is like ArASyncTask, but instead of runThread it uses a functor to run
class ArFunctorASyncTask : public ArASyncTask
{
public:
  /// Constructor
  AREXPORT ArFunctorASyncTask(ArRetFunctor1<void *, void *> *functor);
  /// Destructor
  AREXPORT virtual ~ArFunctorASyncTask();
  /// Our reimplementation of runThread
  AREXPORT virtual void *runThread(void *arg);
protected:
  ArRetFunctor1<void *, void *> *myFunc;
};

#endif
