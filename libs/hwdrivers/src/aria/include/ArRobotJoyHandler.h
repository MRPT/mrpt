/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTJOYHANDLER_H
#define ARROBOTJOYHANDLER_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

class ArRobot;
class ArRobotPacket;

/// Interfaces to a joystick on the robot's microcontroller
/** 
    This is largely meant to be about the same as the normal joy
    handler but gets the data back from the robot about the joystick,
    but this sameness is why it reports things as it does.

    Also note that x is usually rotational velocity (since it right/left),
    whereas Y is translational (since it is up/down).
**/
class ArRobotJoyHandler
{
 public:
  /// Constructor
  AREXPORT ArRobotJoyHandler(ArRobot *robot);
  /// Destructor
  AREXPORT ~ArRobotJoyHandler();
  /// Gets the adjusted reading, as floats
  AREXPORT void getDoubles(double *x, double *y, double *z);
  /// Gets the first button 
  bool getButton1(void) { return myButton1; }
  /// Gets the second button 
  bool getButton2(void) { return myButton2; }
  /// Gets the time we last got information back
  /*AREXPORT*/ ArTime getDataReceivedTime(void) { return myDataReceived; }
  /// If we've ever gotten a packet back
  /*AREXPORT*/ bool gotData(void) { return myGotData; }
 protected:
  AREXPORT bool handleJoystickPacket(ArRobotPacket *packet);
  AREXPORT void connectCallback(void);

  ArRobot *myRobot;
  ArTime myDataReceived;
  bool myButton1;
  bool myButton2;
  double myJoyX;
  double myJoyY;
  double myThrottle;
  bool myGotData;
  
  ArTime myStarted;
  ArRetFunctor1C<bool, ArRobotJoyHandler,
      ArRobotPacket *> myHandleJoystickPacketCB;
  ArFunctorC<ArRobotJoyHandler> myConnectCB;

};


#endif // ARJOYHANDLER_H

