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

#ifndef ARACTS_H
#define ARACTS_H

#include "ariaTypedefs.h"
#include "ArTcpConnection.h"
#include "ariaUtil.h"
#include "ArRobot.h"


/// A class for the acts blob
class ArACTSBlob
{
public:
  /// Constructor
  ArACTSBlob() {}
  /// Destructor
  virtual ~ArACTSBlob() {}
  /// Gets the number of pixels (area) covered by the blob
  int getArea(void) { return myArea; }
  /// Gets the X Center of Gravity of the blob
  int getXCG(void) { return myXCG; }
  /// Gets the Y Center of Gravity of the blob
  int getYCG(void) { return myYCG; }
  /// Gets the left border of the blob
  int getLeft(void) { return myLeft; }
  /// Gets the right border of the blob
  int getRight(void) { return myRight; }
  /// Gets the top border of the blob
  int getTop(void) { return myTop; }
  /// Gets the bottom border of the blob
  int getBottom(void) { return myBottom; }
  /// Sets the number of pixels (area) covered by the blob
  void setArea(int area) { myArea = area; }
  /// Sets the X Center of Gravity of the blob
  void setXCG(int xcg) { myXCG = xcg; }
  /// Sets the Y Center of Gravity of the blob
  void setYCG(int ycg) { myYCG = ycg; }
  /// Sets the left border of the blob
  void setLeft(int left) { myLeft = left; }
  /// Sets the right border fo the blob
  void setRight(int right) { myRight = right; }
  /// Sets the top border of the blob
  void setTop(int top) { myTop = top; }
  /// Sets the bottom border of the blob
  void setBottom(int bottom) { myBottom = bottom; }
  /// Prints the stats of the blob
  void log(void)
    {
      ArLog::log(ArLog::Terse, "Area: %3d X: %3d Y: %3d l: %3d r: %3d t: %3d: b: %3d", 
		 myArea, myXCG, myYCG, myLeft, myRight, myTop, myBottom);
    }
protected:
  int myArea; 
  int myXCG;
  int myYCG;
  int myLeft;
  int myRight;
  int myTop;
  int myBottom;
};

/// Driver for ACTS
class ArACTS_1_2
{
public:
  /// Constructor
  AREXPORT ArACTS_1_2();
  /// Destructor
  AREXPORT virtual ~ArACTS_1_2();

  /// Opens the connection to ACTS
  AREXPORT bool openPort(ArRobot *robot, const char *host = "localhost", int port = 5001);
  /// Closes the connection
  AREXPORT bool closePort(void);

  /// Finds out whether there is connection
  AREXPORT bool isConnected(void);

  /// Gets the robot this class is connected to
  AREXPORT ArRobot *getRobot(void);
  /// Sets the robot this class is connected to
  AREXPORT void setRobot(ArRobot *robot);

  /// Requests another packet
  AREXPORT bool requestPacket(void);
  /// Requests that ACTS quits
  AREXPORT bool requestQuit(void);
  /// Gets the blob information from the connection to acts
  AREXPORT bool receiveBlobInfo(void);
  
  /// Gets the number of blobs for the given chanel
  AREXPORT int getNumBlobs(int channel);
  
  /// Gets the given blob from the given channel
  AREXPORT bool getBlob(int channel, int blobNumber, ArACTSBlob *blob);

  /// A function that reads information from acts and requests packets
  AREXPORT void actsHandler(void);
  
  enum ActsConstants
  {
    NUM_CHANNELS = 32, ///< Number of channels there are
    MAX_BLOBS = 10, ///< Number of blobs per channel
    ACTS_BLOB_DATA_SIZE = 16, ///< Size of the blob data
    DATA_HEADER = NUM_CHANNELS * 4, ///< Size of the data header
    MAX_DATA = 5300 ///< Maximum amount of data
  };
  /// This will make the image stats inverted (for use with an inverted camera)
  AREXPORT void invert(int width = 160, int height = 120);
protected:
  int invertX(int before);
  int invertY(int before);
  /// an iternal function to strip out the information from some bytes
  int getData(char *rawData);
  ArFunctorC<ArACTS_1_2> mySensorTaskCB;
  ArRobot *myRobot;
  ArTcpConnection myConn;
  int myBlobNum[NUM_CHANNELS];
  int myBlobIndex[NUM_CHANNELS];
  char myData[MAX_DATA];
  bool myBlobsBad;
  bool myRequested;
  // these are for flipping the image
  bool myInverted;
  int myHeight;
  int myWidth;
};

#endif
