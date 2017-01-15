/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARRATIOINPUTJOYDRIVE_H
#define ARRATIOINPUTJOYDRIVE_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArActionRatioInput.h"
#include "ArJoyHandler.h"

class ArRobot;

/// This action will use the joystick for input to drive the robot
/**
   This class creates its own ArJoyHandler to get input from the
   joystick.  Then it will scale the speed between 0 and the given max
   for velocity and turning, up and down on the joystick go
   forwards/backwards while right and left go right and left.  You
   must press in one of the two joystick buttons for the class to pay
   attention to the joystick.
   
   NOTE: The joystick does not save calibration information, so you
   must calibrate the joystick before each time you use it.  To do
   this, press the button for at least a half a second while the
   joystick is in the middle.  Then let go of the button and hold the
   joystick in the upper left for at least a half second and then in
   the lower right corner for at least a half second.
**/
class ArRatioInputJoydrive
{
public:
  /// Constructor
  AREXPORT ArRatioInputJoydrive(ArRobot *robot, ArActionRatioInput *input,
				int priority = 50,
				bool stopIfNoButtonPressed = false,
				bool useOSCalForJoystick = true);
  /// Destructor
  AREXPORT virtual ~ArRatioInputJoydrive();
  /// Whether the joystick is initalized or not
  AREXPORT bool joystickInited(void);
  /// Set if we'll stop if no button is pressed, otherwise just do nothing
  AREXPORT void setStopIfNoButtonPressed(bool stopIfNoButtonPressed);
  /// Get if we'll stop if no button is pressed, otherwise just do nothing
  AREXPORT bool getStopIfNoButtonPressed(void);
  /// Sets whether to use OSCalibration the joystick or not
  AREXPORT void setUseOSCal(bool useOSCal);
  /// Gets whether OSCalibration is being used for the joystick or not
  AREXPORT bool getUseOSCal(void);
  /// Gets the joyHandler
  /*AREXPORT*/ ArJoyHandler *getJoyHandler(void) { return myJoyHandler; }
protected:
  void fireCallback(void);
  ArRobot *myRobot;
  ArActionRatioInput *myInput;
  // if we're printing extra information for tracing and such
  bool myPrinting;
  // joystick handler
  ArJoyHandler *myJoyHandler;
  bool myFiredLast;
  // if we want to stop when no button is presesd
  bool myStopIfNoButtonPressed;
  // if we're using os cal for the joystick
  bool myUseOSCal;
  bool myPreviousUseOSCal;
  ArFunctorC<ArRatioInputJoydrive> myFireCB;
};

#endif //ARRATIOINPUTJOYDRIVE_H
