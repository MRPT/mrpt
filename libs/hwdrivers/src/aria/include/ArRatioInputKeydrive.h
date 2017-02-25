/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARRATIOINPUTKEYDRIVE_H
#define ARRATIOINPUTKEYDRIVE_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArFunctor.h"
#include "ArActionRatioInput.h"


/// This will use the keyboard arrow keys and the ArActionRatioInput to drive the robot
/**
   You have to make an ArActionRatioInput and add it to the robot like
   a normal action for this to work.
**/
class ArRatioInputKeydrive 
{
public:
  /// Constructor
  AREXPORT ArRatioInputKeydrive(ArRobot *robot, ArActionRatioInput *input, 
				int priority = 25, double velIncrement = 5);
  /// Destructor
  AREXPORT virtual ~ArRatioInputKeydrive();
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
  /// Internal, gets our firecb
  /*AREXPORT*/ ArFunctor *getFireCB(void) { return &myFireCB; }
protected:
  AREXPORT void activate(void);
  AREXPORT void deactivate(void);
  AREXPORT void fireCallback(void);
  ArFunctorC<ArRatioInputKeydrive> myUpCB;
  ArFunctorC<ArRatioInputKeydrive> myDownCB;
  ArFunctorC<ArRatioInputKeydrive> myLeftCB;
  ArFunctorC<ArRatioInputKeydrive> myRightCB;
  ArFunctorC<ArRatioInputKeydrive> mySpaceCB;

  double myPrinting;
  double myTransRatio;
  double myRotRatio;
  double myThrottle;
  ArRobot *myRobot;
  bool myHaveKeys;
  double myVelIncrement;
  ArActionRatioInput *myInput;
  ArFunctorC<ArRatioInputKeydrive> myFireCB;
  ArFunctorC<ArRatioInputKeydrive> myActivateCB;
  ArFunctorC<ArRatioInputKeydrive> myDeactivateCB;
};


#endif // ARRATIOINPUTKEYDRIVE_H
