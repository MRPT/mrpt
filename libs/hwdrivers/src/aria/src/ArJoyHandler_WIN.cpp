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

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArJoyHandler.h"

AREXPORT bool ArJoyHandler::init(void)
{

  myPhysMax = 1;
  myLastZ = 0;

  // first see if we can talk to the first joystick
  if (joyGetDevCaps(JOYSTICKID1,&myJoyCaps,sizeof(myJoyCaps)) == 
      JOYERR_NOERROR &&
      joyGetPos(JOYSTICKID1,&myJoyInfo) != JOYERR_UNPLUGGED) 
  {
    myJoyID = JOYSTICKID1;

    // So far, it seems that the x range is the same as that of y and
    // z, so only one is used
    myPhysMax  = myJoyCaps.wXmax - myJoyCaps.wXmin;

    myInitialized = true;
    startCal();
    endCal();
    return true;
  } 
  // we couldn't talk to the first one so try the second one
  else if (joyGetDevCaps(JOYSTICKID2,&myJoyCaps,sizeof(myJoyCaps)) == 
      JOYERR_NOERROR &&
      joyGetPos(JOYSTICKID2,&myJoyInfo) != JOYERR_UNPLUGGED) 
  {
    myJoyID = JOYSTICKID2;

    // So far, it seems that the x range is the same as that of y and
    // z, so only one is used
    myPhysMax = myJoyCaps.wXmax - myJoyCaps.wXmin;

    myInitialized = true;
    startCal();
    endCal();
    return true;
  } 
  // we couldn't talk to either one
  else
  {
    myInitialized = false;
    return false;
  }

  // Just to prevent any possible divide-by-zeros...
  if (myPhysMax == 0) {
    myPhysMax = 1;
  }

  getData();
}

void ArJoyHandler::getData(void)
{
  int x, y, z;
  if (!myFirstData && myLastDataGathered.mSecSince() < 5)
    return;

  myFirstData = false;
  myLastDataGathered.setToNow();
  MMRESULT joyResult = joyGetPos(myJoyID,&myJoyInfo);

  if (joyResult == JOYERR_NOERROR) 
  {
    // KMC: I don't understand this logic... The spec says that 
    // getAxis returns a number between -1 and 1; the getAxis method
    // multiplies the contents of myAxes by 128.  The logic below
    // however seems to double everything... 
/**/
    x = (int)(myJoyInfo.wXpos*256.0/myPhysMax)-128;
    y = (int)-((myJoyInfo.wYpos*256.0/myPhysMax)-128);
    z = (int)-((myJoyInfo.wZpos*256.0/myPhysMax)-128);

/***/
/***
    x = (int) 128 * ((2.0 * (double) myJoyInfo.wXpos / (double) myPhysMax) - 1);
    y = (int)-128 * ((2.0 * (double) myJoyInfo.wYpos / (double) myPhysMax) - 1);
    z = (int)-128 * ((2.0 * (double) myJoyInfo.wZpos / (double) myPhysMax) - 1);
**/

    if (myLastZ != z)
      myHaveZ = true;
    if (x > myMaxX)
      myMaxX = x;
    if (x < myMinX)
      myMinX = x;
    if (y > myMaxY)
      myMaxY = y;
    if (y < myMinY)
      myMinY = y;

    myAxes[1] = x;
    myAxes[2] = y;
    myAxes[3] = z;
	
    myLastZ = z;
	
    myButtons[1] = (myJoyInfo.wButtons & 1);
    myButtons[2] = (myJoyInfo.wButtons & 2);
    myButtons[3] = (myJoyInfo.wButtons & 4);
    myButtons[4] = (myJoyInfo.wButtons & 8);
  }
  else //(joyResult == JOYERR_UNPLUGGED) 
  {
    myAxes[1] = 0;
    myAxes[2] = 0;
    myAxes[3] = 0;
    myButtons[1] = 0;
    myButtons[2] = 0;
    myButtons[3] = 0;
    myButtons[4] = 0;
	
	// Reset the initialized flag so that the joystick button in the GUI
	// will be disabled.
    myInitialized = false;
  } 
}


