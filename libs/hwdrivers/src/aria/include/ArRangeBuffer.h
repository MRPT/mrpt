/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARRANGEBUFFER_H
#define ARRANGEBUFFER_H

#include "ariaUtil.h"
#include "ariaTypedefs.h"
#include "ArTransform.h"
#include <list>
#include <vector>

/// This class is a buffer that holds ranging information
class ArRangeBuffer
{
public:
  /// Constructor
  AREXPORT ArRangeBuffer(int size);
  /// Destructor
  AREXPORT virtual ~ArRangeBuffer();
  /// Gets the size of the buffer
  AREXPORT size_t getSize(void) const;
  /// Sets the size of the buffer
  AREXPORT void setSize(size_t size);
  /// Gets the pose of the robot when readings were taken
  AREXPORT ArPose getPoseTaken() const;
  /// Sets the pose of the robot when readings were taken
  AREXPORT void setPoseTaken(ArPose p);
  /// Gets the encoder pose of the robot when readings were taken
  AREXPORT ArPose getEncoderPoseTaken() const;
  /// Sets the pose of the robot when readings were taken
  AREXPORT void setEncoderPoseTaken(ArPose p);
  /// Adds a new reading to the buffer
  AREXPORT void addReading(double x, double y);  
  /// Begins a walk through the getBuffer list of readings
  AREXPORT void beginInvalidationSweep(void);
  /// While doing an invalidation sweep a reading to the list to be invalidated
  AREXPORT void invalidateReading(std::list<ArPoseWithTime*>::iterator readingIt);
  /// Ends the invalidation sweep
  AREXPORT void endInvalidationSweep(void);
#ifndef SWIG
  /** @brief Gets a pointer to a list of readings
   *  @swigomit
   */
  AREXPORT const std::list<ArPoseWithTime *> *getBuffer(void) const;
#endif
  /// Gets a pointer to a list of readings
  AREXPORT std::list<ArPoseWithTime *> *getBuffer(void);

  /// Gets the closest reading, on a polar system 
  AREXPORT double getClosestPolar(double startAngle, double endAngle, 
				  ArPose position, unsigned int maxRange,
				  double *angle = NULL) const;
  /// Gets the closest reading, from a rectangular box, in robot LOCAL coords
  AREXPORT double getClosestBox(double x1, double y1, double x2, double y2,
				ArPose position, unsigned int maxRange, 
				ArPose *readingPos = NULL,
				ArPose targetPose = ArPose(0, 0, 0)) const;
  /// Applies a transform to the buffer
  AREXPORT void applyTransform(ArTransform trans);
  /// Clears all the readings in the range buffer
  AREXPORT void clear(void);
  /// Resets the readings older than this many seconds
  AREXPORT void clearOlderThan(int milliSeconds);
  /// Resets the readings older than this many seconds
  AREXPORT void clearOlderThanSeconds(int seconds);
  /// same as clear, but old name
  AREXPORT void reset(void);
  /// This begins a redoing of the buffer
  AREXPORT void beginRedoBuffer(void);
  /// Add a reading to the redoing of the buffer
  AREXPORT void redoReading(double x, double y);   
  /// End redoing the buffer
  AREXPORT void endRedoBuffer(void);
  /// Gets the buffer as an array instead of as a std::list
  AREXPORT std::vector<ArPoseWithTime> *getBufferAsVector(void);
  /// Gets the closest reading, from an arbitrary buffer
  AREXPORT static double getClosestPolarInList(
	  double startAngle, double endAngle, ArPose position, 
	  unsigned int maxRange, double *angle, 
	  const std::list<ArPoseWithTime *> *buffer);
  /// Gets the closest reading, from an arbitrary buffer
  AREXPORT static double getClosestBoxInList(
	  double x1, double y1, double x2, double y2, ArPose position, 
	  unsigned int maxRange, ArPose *readingPos, 
	  ArPose targetPose, const std::list<ArPoseWithTime *> *buffer);
protected:
  std::vector<ArPoseWithTime> myVector;
  ArPose myBufferPose;		// where the robot was when readings were acquired
  ArPose myEncoderBufferPose;		// where the robot was when readings were acquired

  std::list<ArPoseWithTime *> myBuffer;
  std::list<ArPoseWithTime *> myInvalidBuffer;
  std::list<std::list<ArPoseWithTime *>::iterator > myInvalidSweepList;
  std::list<std::list<ArPoseWithTime *>::iterator >::iterator myInvalidIt;
  std::list<ArPoseWithTime *>::iterator myRedoIt;
  int myNumRedone;
  bool myHitEnd;
  
  size_t mySize;
  std::list<ArPoseWithTime *>::reverse_iterator myRevIterator;
  std::list<ArPoseWithTime *>::iterator myIterator;
  
  ArPoseWithTime * myReading;
};

#endif // ARRANGEBUFFER_H
