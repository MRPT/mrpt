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

#ifndef ARINTERPOLATION_H
#define ARINTERPOLATION_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

/** 
    This class takes care of storing in readings of position vs time, and then
    interpolating between them to find where the robot was at a particular 
    point in time.  It has two lists, one containing the times, and one 
    containing the positions at those same times (per position), they must be 
    walked through jointly to maintain cohesion.  The new entries are at the
    front of the list, while the old ones are at the back.  
    numberOfReadings and the setNumberOfReadings control the number of entries
    in the list.  If a size is set that is smaller than the current size, then
    the old ones are chopped off.
**/
class ArInterpolation
{
public:
  /// Constructor
  AREXPORT ArInterpolation(size_t numberOfReadings = 100);
  /// Destructor
  AREXPORT virtual ~ArInterpolation();
  /// Adds a new reading
  AREXPORT bool addReading(ArTime timeOfReading, ArPose position);
  /// Finds a position
  AREXPORT int getPose(ArTime timeStamp, ArPose *position);
  /// Sets the number of readings this instance holds back in time
  AREXPORT void setNumberOfReadings(size_t numberOfReadings);
  /// Gets the number of readings this instance holds back in time
  AREXPORT size_t getNumberOfReadings(void) const;
  /// Empties the interpolated positions
  AREXPORT void reset(void);
protected:
  std::list<ArTime> myTimes;
  std::list<ArPose> myPoses;
  size_t mySize;
};

#endif
