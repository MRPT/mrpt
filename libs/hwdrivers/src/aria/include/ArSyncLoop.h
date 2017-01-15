/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSYNCLOOP_H
#define ARSYNCLOOP_H


#include "ariaTypedefs.h"
#include "ArASyncTask.h"
#include "ArSyncTask.h"


class ArRobot;


class ArSyncLoop : public ArASyncTask
{
public:

  AREXPORT ArSyncLoop();
  AREXPORT virtual ~ArSyncLoop();

  AREXPORT void setRobot(ArRobot *robot);

  AREXPORT void stopRunIfNotConnected(bool stopRun);
  AREXPORT virtual void * runThread(void *arg);

protected:
  bool myStopRunIfNotConnected;
  ArRobot *myRobot;

};


#endif // ARSYNCLOOP_H
