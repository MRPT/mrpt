/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSENSORREADING_H
#define ARSENSORREADING_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArTransform.h"

/// A class to hold a sensor reading, should be one instance per sensor
/** This class holds sensor data and a sensor reading... it can happen
    that it contains the data for a sonar, but not the reading, in
    which case the range (from getRange) will be -1, and the counter
    it was taken (from getCounterTaken) will be 0, also it will never
    be new (from isNew).  If ignoreThisReading returns true then
    ignore this reading (its still here since this is used for raw
    data).
*/
class ArSensorReading
{
public:
  /// Constructor, the three args are the physical location of the sensor
  AREXPORT ArSensorReading(double xPos = 0.0, double yPos = 0.0, double thPos = 0.0);
   /// Copy constructor
  AREXPORT ArSensorReading(const ArSensorReading & reading);
  /// Assignment operator
  AREXPORT ArSensorReading &operator=(const ArSensorReading &reading);
  /// Destructor
  AREXPORT virtual ~ArSensorReading();

  /// Gets the range of the reading
  /**
     @return the distance return from the sensor (how far from the robot)
  */
  int getRange(void) const { return myRange; }

  /// Given the counter from the robot, it returns whether the reading is new
  /**
     @param counter the counter from the robot at the current time
     @return true if the reading was taken on the current loop
     @see getCounter
  */
  bool isNew(unsigned int counter) const { return counter == myCounterTaken; }
  /// Gets the X location of the sensor reading
  double getX(void) const { return myReading.getX(); }
  /// Gets the Y location of the sensor reading
  double getY(void) const { return myReading.getY(); }
  /// Gets the position of the reading
  /// @return the position of the reading (ie where the sonar pinged back)
  ArPose getPose(void) const { return myReading; }

  /// Gets the X location of the sensor reading in local coords
  double getLocalX(void) const { return myLocalReading.getX(); }
  /// Gets the Y location of the sensor reading
  double getLocalY(void) const { return myLocalReading.getY(); }
  /// Gets the position of the reading
  /// @return the position of the reading (ie the obstacle where the sonar pinged back)
  ArPose getLocalPose(void) const { return myLocalReading; }

  /// Gets the pose of the robot at which the reading was taken
  ArPose getPoseTaken(void) const { return myReadingTaken; }

  /// Gets the robot's encoder pose the reading was taken at
  ArPose getEncoderPoseTaken(void) const { return myEncoderPoseTaken; }

  /// Gets the X location of the sonar on the robot
  double getSensorX(void) const { return mySensorPos.getX(); }
  /// Gets the Y location of the sensor on the robot
  double getSensorY(void) const { return mySensorPos.getY(); }
  /// Gets the heading of the sensor on the robot
  double getSensorTh(void) const { return mySensorPos.getTh(); }

  /// Gets whether this reading should be ignore or not. e.g. the sensor
  /// encountered an error or did not actually detect anything.
  bool getIgnoreThisReading(void) const { return myIgnoreThisReading; }

  /// Gets the extra int with this reading
  /**
     Some range devices provide extra device-dependent information
     with each reading.  What that means depends on the range device,
     if a range device doesn't give the meaning in its constructor
     description then it has no meaning at all.

     Note that for all laser like devices this should be a value
     between 0 - 255 which is the measure of reflectance.  It should
     be 0 if that device doesn't measure reflectance (the default).
   **/
  int getExtraInt(void) const { return myExtraInt; }


  /// Gets the sensor's position on the robot
  /**
      @return the position of the sensor on the robot
  */
  ArPose getSensorPosition(void) const { return mySensorPos; }

  /// Gets the cosine component of the heading of the sensor reading
  double getSensorDX(void) const { return mySensorCos; }
  /// Gets the sine component of the heading of the sensor reading
  double getSensorDY(void) const { return mySensorSin; }

  /// Gets the X locaiton of the robot when the reading was received
  double getXTaken(void) const { return myReadingTaken.getX(); }
  /// Gets the Y location of the robot when the reading was received
  double getYTaken(void) const { return myReadingTaken.getY(); }
  /// Gets the th (heading) of the robot when the reading was received
  double getThTaken(void) const { return myReadingTaken.getTh(); }

  /// Gets the counter from when the reading arrived
  /**
     @return the counter from the robot when the sonar reading was taken
     @see isNew
  */
  unsigned int getCounterTaken(void) const { return myCounterTaken; }

  ArTime getTimeTaken(void) const { return myTimeTaken; }

  /// Takes the data and makes the reading reflect it
  AREXPORT void newData(int range, ArPose robotPose, ArPose encoderPose,
			ArTransform trans, unsigned int counter,
			ArTime timeTaken, bool ignoreThisReading = false,
			int extraInt = 0);

 /// Takes the data and makes the reading reflect it
  AREXPORT void newData(int sx, int sy, ArPose robotPose,
			ArPose encoderPose,
			ArTransform trans,
			unsigned int counter,
			ArTime timeTaken,
			bool ignoreThisReading = false,
			int extraInt = 0);

  /// Resets the sensors idea of its physical location on the robot
  AREXPORT void resetSensorPosition(double xPos, double yPos, double thPos,
				    bool forceComputation = false);

  /// Sets that we should ignore this reading
  /*AREXPORT*/void setIgnoreThisReading(bool ignoreThisReading)
    { myIgnoreThisReading = ignoreThisReading; }

  /// Sets the extra int
  /*AREXPORT*/void setExtraInt(int extraInt)
    { myExtraInt = extraInt; }


  /// Applies a transform to the reading position, and where it was taken
  AREXPORT void applyTransform(ArTransform trans);
  /// Applies a transform to the encoder pose taken
  AREXPORT void applyEncoderTransform(ArTransform trans);
  /// Applies a transform to the reading position, and where it was taken
  /*AREXPORT*/ bool getAdjusted(void) { return myAdjusted; }
  /// Applies a transform to the reading position, and where it was taken
  /*AREXPORT*/ void setAdjusted(bool adjusted) { myAdjusted = adjusted; }
protected:
  unsigned int myCounterTaken;
  ArPose myReading;
  ArPose myLocalReading;
  ArPose myReadingTaken;
  ArPose myEncoderPoseTaken;
  ArPose mySensorPos;
  double mySensorCos, mySensorSin;
  double myDistToCenter;
  double myAngleToCenter;
  int myRange;
  ArTime myTimeTaken;
  bool myIgnoreThisReading;
  int myExtraInt;
  bool myAdjusted;
};

#endif
