/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
