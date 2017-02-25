/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
