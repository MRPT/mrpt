/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONGROUPS_H
#define ARACTIONGROUPS_H

#include "ariaTypedefs.h"
#include "ArActionGroup.h"
#include "ArActionColorFollow.h"
#include "ArACTS.h"
#include "ArPTZ.h"

class ArActionInput;
class ArActionJoydrive;
class ArActionDeceleratingLimiter;
class ArActionRatioInput;
class ArRatioInputKeydrive;
class ArRatioInputJoydrive;
class ArRatioInputRobotJoydrive;

/// Action group to use to drive the robot with input actions (keyboard, joystick, etc.)
/** 
   This class is just useful for teleoping the robot under your own
   joystick and keyboard control... Note that you the predefined
   ArActionGroups in ARIA are made only to be used exclusively... only
   one can be active at once.

   This class is largely now obsolete (it is used by ArServerModeDrive
   but that is now obsolete and was replaced by a class that just
   makes its own action group)

   ArActionGroupRatioDrive is better.
**/
class ArActionGroupInput : public ArActionGroup
{
public:
  AREXPORT ArActionGroupInput(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupInput();
  AREXPORT void setVel(double vel);
  AREXPORT void setRotVel(double rotVel);
  AREXPORT void setHeading(double heading);
  AREXPORT void deltaHeadingFromCurrent(double delta);
  AREXPORT void clear(void);
  AREXPORT ArActionInput *getActionInput(void);
protected:
  ArActionInput *myInput;
};

/// Action group to stop the robot
/** 
   This class is just useful for having the robot stopped... Note that
   you the predefined ArActionGroups in ARIA are made only to be used
   exclusively... they won't combine.
**/
class ArActionGroupStop : public ArActionGroup
{
public:
  AREXPORT ArActionGroupStop(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupStop();
};

/// Action group to teleopoperate the robot using ArActionJoydrive, and the Limiter actions to avoid collisions.
/** 
   This class is just useful for teleoping the robot and having these
   actions read the joystick and keyboard... Note that you the
   predefined ArActionGroups in ARIA are made only to be used
   exclusively... only one can be active at once.
**/
class ArActionGroupTeleop : public ArActionGroup
{
public:
  AREXPORT ArActionGroupTeleop(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupTeleop();
  AREXPORT void setThrottleParams(int lowSpeed, int highSpeed);
protected:
  ArActionJoydrive *myJoydrive;
};

/// Action group to teleoperate the robot using ArActionJoydrive, but without any Limiter actions to avoid collisions.
/** 
   This class is just useful for teleoping the robot in an unguarded
   and unsafe manner and having these actions read the joystick and
   keyboard... Note that you the predefined ArActionGroups in ARIA are
   made only to be used exclusively... only one can be active at once.
**/
class ArActionGroupUnguardedTeleop : public ArActionGroup
{
public:
  AREXPORT ArActionGroupUnguardedTeleop(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupUnguardedTeleop();
  AREXPORT void setThrottleParams(int lowSpeed, int highSpeed);
protected:
  ArActionJoydrive *myJoydrive;
};

/// Action group to make the robot wander, avoiding obstacles.
/** 
   This class is useful for having the robot wander... Note that
   you the predefined ArActionGroups in ARIA are made only to be used
   exclusively... only one can be active at once.
**/
class ArActionGroupWander : public ArActionGroup
{
public:
  AREXPORT ArActionGroupWander(ArRobot *robot, int forwardVel = 400, int avoidFrontDist = 450, int avoidVel = 200, int avoidTurnAmt = 15);
  AREXPORT virtual ~ArActionGroupWander();
};

/// Follows a blob of color
/** 
   This class has the robot follow a blob of color... Note that you the
   predefined ArActionGroups in ARIA are made only to be used
   exclusively... only one can be active at once.
**/

class ArActionGroupColorFollow : public ArActionGroup
{
public:
  AREXPORT ArActionGroupColorFollow(ArRobot *robot, ArACTS_1_2 *acts, ArPTZ *camera);
  AREXPORT virtual ~ArActionGroupColorFollow();
  AREXPORT void setCamera(ArPTZ *camera);
  AREXPORT void setChannel(int channel);
  AREXPORT void startMovement();
  AREXPORT void stopMovement();
  AREXPORT void setAcquire(bool acquire);
  AREXPORT int getChannel();
  AREXPORT bool getAcquire();
  AREXPORT bool getMovement();
  AREXPORT bool getBlob();
protected:
  ArActionColorFollow *myColorFollow;
};

/// Use keyboard and joystick input to to drive the robot, with Limiter actions to avoid obstacles.
/** 
   This class is just useful for teleoping the robot under your own
   joystick and keyboard control... Note that you the predefined
   ArActionGroups in ARIA are made only to be used exclusively (one at
   a time)... only one can be active at once.
**/
class ArActionGroupRatioDrive : public ArActionGroup
{
public:
  AREXPORT ArActionGroupRatioDrive(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupRatioDrive();
  AREXPORT ArActionRatioInput *getActionRatioInput(void);
  AREXPORT void addToConfig(ArConfig *config, const char *section);
protected:
  ArActionDeceleratingLimiter *myDeceleratingLimiterForward;
  ArActionDeceleratingLimiter *myDeceleratingLimiterBackward;
  ArActionRatioInput *myInput;
  ArRatioInputKeydrive *myKeydrive;
  ArRatioInputJoydrive *myJoydrive;
  ArRatioInputRobotJoydrive *myRobotJoydrive;

};


/// Use keyboard and joystick input to to drive the robot, but without Limiter actions to avoid obstacles.
/** 
   This class is just useful for teleoping the robot under your own
   joystick and keyboard control... Note that you the predefined
   ArActionGroups in ARIA are made only to be used exclusively (one at
   a time)... only one can be active at once.
**/
class ArActionGroupRatioDriveUnsafe : public ArActionGroup
{
public:
  AREXPORT ArActionGroupRatioDriveUnsafe(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupRatioDriveUnsafe();
  AREXPORT ArActionRatioInput *getActionRatioInput(void);
  AREXPORT void addToConfig(ArConfig *config, const char *section);
protected:
  ArActionRatioInput *myInput;
  ArRatioInputKeydrive *myKeydrive;
  ArRatioInputJoydrive *myJoydrive;
  ArRatioInputRobotJoydrive *myRobotJoydrive;

};

#endif // ARACTIONGROUPS_H
