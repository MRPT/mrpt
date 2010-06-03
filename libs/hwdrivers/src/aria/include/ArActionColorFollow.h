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
