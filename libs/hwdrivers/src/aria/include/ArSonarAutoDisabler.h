/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSONARAUTODISABLER_H
#define ARSONARAUTODISABLER_H

/// Class for automatically disabling sonar when the robot is stopped
/**
   If you create one of this class it will disable the sonar when the
   robot stops moving and then enable the sonar when the robot moves.
   Later this may get more parameters and the ability to be turned on
   and off and things like that (email on aria-users if you want
   them).

   Note that this class assumes it is the only class turning the sonar
   on or off and that the sonar start on.
 **/

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"

class ArRobot;

class ArSonarAutoDisabler
{
public:
  /// Constructor
  AREXPORT ArSonarAutoDisabler(ArRobot *robot);
  /// Destructor
  AREXPORT virtual ~ArSonarAutoDisabler();

protected:
  /// our user task
  AREXPORT void userTask(void);
  ArRobot *myRobot;
  ArTime myLastMoved;
  bool mySonarEnabled;
  ArFunctorC<ArSonarAutoDisabler> myUserTaskCB;
};

#endif // ARSONARAUTODISABLER
