/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArFunctorASyncTask.h"

AREXPORT ArFunctorASyncTask::ArFunctorASyncTask(ArRetFunctor1<void *, void *> *functor)
{
  setThreadName(functor->getName());
  myFunc = functor;
}

AREXPORT ArFunctorASyncTask::~ArFunctorASyncTask()
{

}

AREXPORT void *ArFunctorASyncTask::runThread(void *arg)
{
  threadStarted();
  return myFunc->invokeR(arg);
}
