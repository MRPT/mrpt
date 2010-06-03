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

#ifndef ARACTIONLIMITERTABLESNSOR_H
#define ARACTIONLIMITERTABLESNSOR_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to limit speed (and stop) based on whether the "table"-sensors see anything
/**
   This action limits speed to 0 if the table-sensors see anything in front
   of the robot.  The action will only work if the robot has table sensors,
   meaning that the robots parameter file has them listed as true.
*/
class ArActionLimiterTableSensor : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionLimiterTableSensor(const char *name = "TableSensorLimiter");
  /// Destructor
  AREXPORT virtual ~ArActionLimiterTableSensor();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArActionDesired myDesired;
};



#endif // ARACTIONLIMITERTABLESNSOR_H
