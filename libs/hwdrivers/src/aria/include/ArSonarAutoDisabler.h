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
