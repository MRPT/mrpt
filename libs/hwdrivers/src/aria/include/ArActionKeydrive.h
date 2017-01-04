/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONKEYDRIVE_H
#define ARACTIONKEYDRIVE_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArFunctor.h"

class ArRobot;

/// This action will use the keyboard arrow keys for input to drive the robot
class ArActionKeydrive : public ArAction
{
public:
  /// Constructor
 AREXPORT ArActionKeydrive(const char *name = "keydrive",
			   double transVelMax = 400,
			   double turnAmountMax = 24,
			   double velIncrement = 25,
			   double turnIncrement = 8);
  /// Destructor
  AREXPORT virtual ~ArActionKeydrive();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /// For setting the maximum speeds
  AREXPORT void setSpeeds(double transVelMax, double turnAmountMax);
  /// For setting the increment amounts
  AREXPORT void setIncrements(double velIncrement, double turnIncrement);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT virtual void activate(void);
  AREXPORT virtual void deactivate(void);
  /// Takes the keys this action wants to use to drive
  AREXPORT void takeKeys(void);
  /// Gives up the keys this action wants to use to drive
  AREXPORT void giveUpKeys(void);
  /// Internal, callback for up arrow
  AREXPORT void up(void);
  /// Internal, callback for down arrow
  AREXPORT void down(void);
  /// Internal, callback for left arrow
  AREXPORT void left(void);
  /// Internal, callback for right arrow
  AREXPORT void right(void);
  /// Internal, callback for space key
  AREXPORT void space(void);

protected:
  ArFunctorC<ArActionKeydrive> myUpCB;
  ArFunctorC<ArActionKeydrive> myDownCB;
  ArFunctorC<ArActionKeydrive> myLeftCB;
  ArFunctorC<ArActionKeydrive> myRightCB;
  ArFunctorC<ArActionKeydrive> mySpaceCB;
  // action desired
  ArActionDesired myDesired;
  // full speed
  double myTransVelMax;
  // full amount to turn
  double myTurnAmountMax;
  // amount to increment vel by on an up arrow or decrement on a down arrow
  double myVelIncrement;
  // amount to increment turn by on a left arrow or right arrow
  double myTurnIncrement;
  // amount we want to speed up
  double myDeltaVel;
  // amount we want to turn 
  double myTurnAmount;
  // what speed we want to go
  double myDesiredSpeed;
  // if our speeds been reset
  bool mySpeedReset;
};


#endif // ARACTIONKEYDRIVE_H
