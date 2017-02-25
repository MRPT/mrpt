/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARJOYHANDLER_H
#define ARJOYHANDLER_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

#ifdef WIN32
#include <mmsystem.h>
#else // if not win32
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif
#ifdef linux
#include <linux/joystick.h>
#endif



/// Interfaces to a joystick
/**
    The joystick handler keeps track of the minimum and maximums for both
    axes, updating them to constantly be better calibrated.  The speeds set
    influence what is returned by getAdjusted...

    The joystick is not opened until init is called.  What should basically
    be done to use this class is to 'init' a joystick, do a 'setSpeed' so you
    can use 'getAdusted', then at some point do a 'getButton' to see if a
    button is pressed, and then do a 'getAdjusted' to get the values to act
    on.

    Also note that x is usually rotational velocity (since it right/left),
    whereas Y is translational (since it is up/down).

    You can also use this to do multiple uses with the joystick, for example to
    have button 1 drive the robot while to have button 2 move the camera,
    you can get the different values you want (don't want to move the camera
    as quickly or as far as the robot) by using setSpeed before doing
    getAdjusted since setSpeed is fast and won't take any time.
*/
class ArJoyHandler
{
 public:
  /// Constructor
  AREXPORT ArJoyHandler(bool useOSCal = true, bool useOldJoystick = false);
  /// Destructor
  AREXPORT ~ArJoyHandler();
  /// Intializes the joystick, returns true if successful
  AREXPORT bool init(void);
  /// Returns if the joystick was successfully initialized or not
  /*AREXPORT*/ bool haveJoystick(void) { return myInitialized; }
  /// Gets the adjusted reading, as floats, between -1.0 and 1.0
  AREXPORT void getDoubles(double *x, double *y, double *z = NULL);
  /// Gets the button
  AREXPORT bool getButton(unsigned int button);
  /// Returns true if we definitely have a Z axis (we don't know in windows unless it moves)
  /*AREXPORT*/ bool haveZAxis(void) { return myHaveZ; }

  /// Sets the max that X or Y will return
  /*AREXPORT*/void setSpeeds(int x, int y, int z = 0)
    { myTopX = x; myTopY = y; myTopZ = z; }
  /// Gets the adjusted reading, as integers, based on the setSpeed
  AREXPORT void getAdjusted(int *x, int *y, int *z = NULL);

  /// Gets the number of axes the joystick has
  AREXPORT unsigned int getNumAxes(void);
  /// Gets the floating (-1 to 1) location of the given joystick axis
  AREXPORT double getAxis(unsigned int axis);
  /// Gets the number of buttons the joystick has
  AREXPORT unsigned int getNumButtons(void);

  /// Sets whether to just use OS calibration or not
  AREXPORT void setUseOSCal(bool useOSCal);
  /// Gets whether to just use OS calibration or not
  AREXPORT bool getUseOSCal(void);
  /// Starts the calibration process
  AREXPORT void startCal(void);
  /// Ends the calibration process
  AREXPORT void endCal(void);
  /// Gets the unfilitered reading, mostly for internal use, maybe
  /// useful for Calibration
  AREXPORT void getUnfiltered(int *x, int *y, int *z = NULL);
  /// Gets the stats for the joystick, useful after calibrating to save values
  AREXPORT void getStats(int *maxX, int *minX, int *maxY, int *minY,
		 int *cenX, int *cenY);
  /// Sets the stats for the joystick, useful for restoring calibrated settings
  AREXPORT void setStats(int maxX, int minX, int maxY, int minY,
		int cenX, int cenY);
  /// Gets the speeds that X and Y are set to
  AREXPORT void getSpeeds(int *x, int *y, int *z);
 protected:
  // function to get the data for OS dependent part
  void getData(void);
  int myMaxX, myMinX, myMaxY, myMinY, myCenX, myCenY, myTopX, myTopY, myTopZ;
  bool myHaveZ;

  std::map<unsigned int, int> myAxes;
  std::map<unsigned int, bool> myButtons;

  int myPhysMax;
  bool myInitialized;
  bool myUseOSCal;
  bool myUseOld;
  bool myFirstData;
  ArTime myLastDataGathered;
#ifdef WIN32
  unsigned int myJoyID;
  int myLastZ;
  JOYINFO myJoyInfo;
  JOYCAPS myJoyCaps;
#else // if not win32
  int myJoyNumber;
  char myJoyNameTemp[512];
  ArTime myLastOpenTry;
  void getOldData(void);
  void getNewData(void);
  #ifdef linux
  struct JS_DATA_TYPE myJoyData; // structure for the buttons and x,y coords
  #else
  int myJoyData;
  #endif
  FILE * myOldJoyDesc;
  int myJoyDesc;
#endif // linux
};


#endif // ARJOYHANDLER_H

