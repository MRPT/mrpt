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

#ifndef ARACTIONMOVEMENTPARAMTERS_H
#define ARACTIONMOVEMENTPARAMTERS_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArMap.h"

/// This is a class for setting max velocities and accels and decels
/**
 **/
class ArActionMovementParameters : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionMovementParameters(const char *name = "MovementParameters",
				      bool overrideFaster = true);
  /// Destructor
  AREXPORT virtual ~ArActionMovementParameters();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const
                                                        { return &myDesired; }
#endif
  /// Enables this action (separate from activating it)
  /*AREXPORT*/ void enable(void) { myEnabled = true; }
  /// Enables this action in a way that'll work from the sector callbacks
  /*AREXPORT*/void enableOnceFromSector(ArMapObject *mapObject)
    { myEnableOnce = true; }
  /// Disables this action (separate from deactivating it)
  /*AREXPORT*/ void disable(void) { myEnabled = false; }
  /// Sets the parameters (don't use this if you're using the addToConfig)
  AREXPORT void setParameters(double maxVel = 0, double maxNegVel = 0,
			      double transAccel = 0, double transDecel = 0,
			      double rotVelMax = 0, double rotAccel = 0,
			      double rotDecel = 0);
  /// Adds to the ArConfig given, in section, with prefix
  AREXPORT void addToConfig(ArConfig *config, const char *section,
			    const char *prefix = NULL);
protected:
  bool myEnabled;
  bool myEnableOnce;
  bool myOverrideFaster;
  double myMaxVel;
  double myMaxNegVel;
  double myTransAccel;
  double myTransDecel;
  double myMaxRotVel;
  double myRotAccel;
  double myRotDecel;

  ArActionDesired myDesired;


};

#endif // ARACTIONMOVEMENTPARAMTERS_H
