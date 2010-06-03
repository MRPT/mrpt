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

#ifndef ARACTIONDECELERATINGLIMITER_H
#define ARACTIONDECELERATINGLIMITER_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to limit the forwards motion of the robot based on range sensor readings
/**
   This action uses the robot's range sensors (e.g. sonar, laser) to find a 
   maximum speed at which to travel
   and will increase the deceleration so that the robot doesn't hit
   anything.  If it has to, it will trigger an estop to avoid a
   collision.

   Note that this cranks up the deceleration with a strong strength,
   but it checks to see if there is already something decelerating
   more strongly... so you can put these actions lower in the priority list so
   things will play together nicely.
**/
class ArActionDeceleratingLimiter : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionDeceleratingLimiter(const char *name = "limitAndDecel", 
				       bool forwards = true);
  /// Destructor
  AREXPORT virtual ~ArActionDeceleratingLimiter();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  /// Sets the parameters (don't use this if you're using the addToConfig)
  AREXPORT void setParameters(double clearance = 100,
			      double sideClearanceAtSlowSpeed = 50,
			      double paddingAtSlowSpeed = 50,
			      double slowSpeed = 200,
			      double sideClearanceAtFastSpeed = 400,
			      double paddingAtFastSpeed = 300,
			      double fastSpeed = 1000,
			      double preferredDecel = 600,
			      bool useEStop = false,
			      double maxEmergencyDecel = 0);
  /// Gets if this will control us when going forwards
  bool getForwards(void) { return myForwards; }
  /// Sets if this will control us when going forwards
  void setForwards(bool forwards) { myForwards = forwards; }
  /// Adds to the ArConfig given, in section, with prefix
  AREXPORT void addToConfig(ArConfig *config, const char *section,
			    const char *prefix = NULL);
  /// Sets if we're using locationDependent range devices or not
  bool getUseLocationDependentDevices(void) 
    { return myUseLocationDependentDevices; }
  /// Sets if we're using locationDependent range devices or not
  void setUseLocationDependentDevices(bool useLocationDependentDevices)
    { myUseLocationDependentDevices = useLocationDependentDevices; }
protected:
  bool myLastStopped;
  bool myForwards;
  double myClearance;
  double mySideClearanceAtSlowSpeed;
  double myPaddingAtSlowSpeed;
  double mySlowSpeed;
  double mySideClearanceAtFastSpeed;
  double myPaddingAtFastSpeed;
  double myFastSpeed;
  double myPreferredDecel;
  double myMaxEmergencyDecel;
  bool myUseEStop;
  bool myUseLocationDependentDevices;
//unused?  double myDecelerateDistance;
  ArActionDesired myDesired;
};

#endif // ARACTIONSPEEDLIMITER_H
