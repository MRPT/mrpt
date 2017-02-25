/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONCOLORFOLLOW_H
#define ARACTIONCOLORFOLLOW_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"
#include "ArAction.h"
#include "ArACTS.h"
#include "ArPTZ.h"

/// ArActionColorFollow is an action that moves the robot toward the
/// largest blob that appears in it's current field of view.
class ArActionColorFollow : public ArAction
{
  
public:
  // Constructor
  AREXPORT ArActionColorFollow(const char *name, 
			       ArACTS_1_2 *acts,
			       ArPTZ *camera,
			       double speed = 200, 
			       int width = 160, 
			       int height = 120);
  
  // Destructor
  AREXPORT virtual ~ArActionColorFollow(void);
  
  // The action
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);

  // Set the ACTS channel that we want to get blob info out of
  AREXPORT bool setChannel(int channel);

  // Set the camera that we will be controlling
  AREXPORT void setCamera(ArPTZ *camera);

  // Toggle whether we should try to acquire a blob
  // if one cannot be seen
  AREXPORT void setAcquire(bool acquire);

  // Stop moving alltogether
  AREXPORT void stopMovement(void);
  
  // Start moving
  AREXPORT void startMovement(void);

  // Return the channel that we are looking for blobs on
  AREXPORT int getChannel();
  
  // Return whether or not we are trying to acquire a blob
  // if we cannot see one
  AREXPORT bool getAcquire();
  
  // Return whether or not we are moving
  AREXPORT bool getMovement();

  // Return whether or not we can see a target
  AREXPORT bool getBlob();

  // The state of the action
  enum TargetState 
  {
    NO_TARGET,      // There is no target in view
    TARGET          // There is a target in view
  };

  // The state of movement
  enum MoveState
  {
    FOLLOWING,     // Following a blob
    ACQUIRING,     // Searching for a blob
    STOPPED        // Sitting still
  };

  // The last seen location of the blob
  enum LocationState
  {
    LEFT,           // The blob is on the left side of the screen
    RIGHT,          // The blob is on the right side of the screen
    CENTER          // The blob is relatively close to the center
  };
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArActionDesired myDesired;
  ArACTS_1_2 *myActs;
  ArPTZ *myCamera;
  ArTime myLastSeen;
  TargetState myState;
  MoveState myMove;
  LocationState myLocation;
  bool myAcquire;
  bool killMovement;
  int myChannel;
  int myMaxTime;
  int myHeight;
  int myWidth;
  double mySpeed;
};


#endif // ARACTIONCOLORFOLLOW_H
