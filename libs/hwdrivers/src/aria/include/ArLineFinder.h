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

#ifndef ARSICKLINEFINDER_H
#define ARSICKLINEFINDER_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"
#include "ariaUtil.h"
#include <vector>

class ArLineFinderSegment;

/// This class finds lines out of any range device with raw readings (ArSick for instance)
class ArLineFinder
{
public:
  /// Constructor
  AREXPORT ArLineFinder(ArRangeDevice *dev);
  /// Destructor
  AREXPORT virtual ~ArLineFinder();

  /// Finds the line and returns a map of them 
  AREXPORT std::map<int, ArLineFinderSegment *> *getLines(void);
  /// Gets the position the last lines were gotten at
  ArPose getLinesTakenPose(void) { return myPoseTaken; }
  /// Logs all the points and lines from the last getLines
  AREXPORT void saveLast(void);
  /// Gets the lines, then prints them
  AREXPORT void getLinesAndSaveThem(void);
  /// Whether to print verbose information about line decisions
  void setVerbose(bool verbose) { myPrinting = verbose; }
  /// Whether to print verbose information about line decisions
  bool getVerbose(void) { return myPrinting; }
  /// Sets some parameters for line creation
  void setLineCreationParams(int minLineLen = 40, int minLinePoints = 2)
    { myMakingMinLen = minLineLen; myMakingMinPoints = minLinePoints; }
  /// Sets some parameters for line combining
  void setLineCombiningParams(int angleTol = 30, int linesCloseEnough = 75) 
    { myCombiningAngleTol = angleTol; 
    myCombiningLinesCloseEnough = linesCloseEnough; }
  /// Filter out lines smaller than this
  void setLineFilteringParams(int minPointsInLine = 3, int minLineLength = 75)
    { myFilteringMinPointsInLine = minPointsInLine; 
    myFilteringMinLineLength = minLineLength; }
  /// Don't let lines happen that have points not close to it
  void setLineValidParams(int maxDistFromLine = 30, 
			  int maxAveDistFromLine = 20)
    { myValidMaxDistFromLine = maxDistFromLine; 
    myValidMaxAveFromLine = maxAveDistFromLine; }
  /// Sets the maximum distance between points that'll still be included in the same line
  void setMaxDistBetweenPoints(int maxDistBetweenPoints = 0)
    { myMaxDistBetweenPoints = maxDistBetweenPoints; }
protected:
  // where the readings were taken
  ArPose myPoseTaken;
  // our points
  std::map<int, ArPose> *myPoints;
  std::map<int, ArLineFinderSegment *> *myLines;

  // fills up the myPoints variable from sick laser
  AREXPORT void fillPointsFromLaser(void);
  // fills up the myLines variable from the myPoints
  AREXPORT void findLines(void);
  // cleans the lines and puts them into myLines 
  AREXPORT bool combineLines();
  // takes two segments and sees if it can average them
  AREXPORT ArLineFinderSegment *averageSegments(ArLineFinderSegment *line1, 
					  ArLineFinderSegment *line2);
  // removes lines that don't have enough points added in
  AREXPORT void filterLines();

  bool myFlippedFound;
  bool myFlipped;
  int myValidMaxDistFromLine;
  int myValidMaxAveFromLine;
  int myMakingMinLen;
  int myMakingMinPoints;
  int myCombiningAngleTol;
  int myCombiningLinesCloseEnough;
  int myFilteringMinPointsInLine;
  int myFilteringMinLineLength;
  int myMaxDistBetweenPoints;
  double mySinMultiplier;
  bool myPrinting;
  ArRangeDevice *myRangeDevice;
};

/// Class for ArLineFinder to hold more info than an ArLineSegment
class ArLineFinderSegment : public ArLineSegment
{
public:
  ArLineFinderSegment() {}
  ArLineFinderSegment(double x1, double y1, double x2, double y2, 
		      int numPoints, int startPoint, int endPoint)
    { newEndPoints(x1, y1, x2, y2, numPoints, startPoint, endPoint); }
  virtual ~ArLineFinderSegment() {}
  void newEndPoints(double x1, double y1, double x2, double y2, 
		    int numPoints, int startPoint, int endPoint)
    {
      ArLineSegment::newEndPoints(x1, y1, x2, y2);
      myLineAngle = ArMath::atan2(y2 - y1, x2 - x1);
      myLength = ArMath::distanceBetween(x1, y1, x2, y2);
      myNumPoints = numPoints;
      myStartPoint = startPoint;
      myEndPoint = endPoint;
      myAveDistFromLine = 0;
    }
  double getLineAngle(void) { return myLineAngle; }
  double getLength(void) { return myLength; }
  int getNumPoints(void) { return myNumPoints; }
  int getStartPoint(void) { return myStartPoint; }
  int getEndPoint(void) { return myEndPoint; }
  void setAveDistFromLine(double aveDistFromLine) 
    { myAveDistFromLine = aveDistFromLine; }
  double getAveDistFromLine(void) { return myAveDistFromLine; }
protected:
  double myLineAngle;
  double myLength;
  int myNumPoints;
  int myStartPoint;
  int myEndPoint;
  double myAveDistFromLine;
};

#endif // ARSICKLINEFINDER_H
