/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONDRIVEDISTANCE_H
#define ARACTIONDRIVEDISTANCE_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

/// This action goes to a given ArPose very naively

/**
   This action naively drives a fixed distance. The action stops the
   robot when it has travelled the appropriate distance. It
   travels at 'speed' mm/sec.

   You can give it a distance with setDistance(), cancel its movement
   with cancelDistance(), and see if it got there with
   haveAchievedDistance().

   You can tell it to go backwards by calling setDistance with a
   negative value.

   This doesn't avoid obstacles or anything, you could add have an
   limiting ArAction at a higher priority to try to do this (so you
   don't smash things). (For truly intelligent navigation, see
   ActivMedia's ARNL or SONARNL software.)
**/


class ArActionDriveDistance : public ArAction
{
public:
  AREXPORT ArActionDriveDistance(const char *name = "driveDistance", 
				double speed = 400, double deceleration = 200);
  AREXPORT virtual ~ArActionDriveDistance();

  /// Sees if the goal has been achieved
  AREXPORT bool haveAchievedDistance(void);
  /// Cancels the goal the robot has
  AREXPORT void cancelDistance(void);
  /// Sets a new goal and sets the action to go there
  AREXPORT void setDistance(double distance, bool useEncoders = true);
  /// Gets whether we're using the encoder position or the normal position
  bool usingEncoders(void) { return myUseEncoders; }
  /// Sets the speed the action will travel at (mm/sec)
  void setSpeed(double speed = 400) { mySpeed = speed; }
  /// Gets the speed the action will travel at (mm/sec)
  double getSpeed(void) { return mySpeed; }
  /// Sets the deceleration the action will use (mm/sec/sec)
  void setDeceleration(double deceleration = 200) 
    { myDeceleration = deceleration; }
  /// Gets the deceleration the action will use (mm/sec/sec)
  double getDeceleration(void) { return myDeceleration; }
  /// Sets if we're printing or not
  void setPrinting(bool printing) { myPrinting = printing; }
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  double myDistance;
  bool myUseEncoders;
  double mySpeed;
  double myDeceleration;
  ArActionDesired myDesired;
  bool myPrinting;
  double myLastVel;

  double myDistTravelled;
  ArPose myLastPose;
  
  enum State
  {
    STATE_NO_DISTANCE, 
    STATE_ACHIEVED_DISTANCE,
    STATE_GOING_DISTANCE
  };
  State myState;
};

#endif // ARACTIONDRIVE
