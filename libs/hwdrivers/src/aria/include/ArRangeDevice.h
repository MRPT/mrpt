/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARRANGEDEVICE_H
#define ARRANGEDEVICE_H

#include "ariaTypedefs.h"
#include "ArRangeBuffer.h"
#include "ArSensorReading.h"
#include "ArDrawingData.h"
#include "ArMutex.h"

class ArRobot;

/** 
    @brief The base class for all sensing devices which return range
    information from the device (mounted on a robot) to an object in
    the environment.

    This class maintains two ArRangeBuffer objects: a current buffer
    for storing very recent readings, and a cumulative buffer for a
    longer history of readings.  The maximum sizes of each buffer can
    be set in the constructor or resized later. Range device readings
    are most often represented as a point in space (X,Y) where the
    sensor detected an object.  (Therefore an ArPose object may only
    have X and Y components set).  Some devices provide extra
    information in a "raw" buffer, or in lists of more detailed
    ArSensoReading objects.


    Subclasses are used for specific sensor implementations like
    ArSick for SICK lasers and ArSonarDevice for the Pioneer sonar
    array. It can also be useful to treat "virtual" objects like
    forbidden areas specified by the user in a map like range devices.
    Some of these subsclasses may use a separate thread to update the
    range reading buffers, and so this base class provides "lock" and
    "unlock" methods which you should use when accessing device data.

    A range device may have an ArRobot object associated with it. A
    range device may also be associated with an ArRobot by calling
    ArRobot::addRangeDevice().  ArRobot provides functions which
    operate on all such associated ArRangeDevice objects.  This is a
    convenient (and thread-safe) way to access all range device data
    without depending on a specific set of individual range
    devices. For example, you can find the closest reading in a box or
    a polar section, no matter if that reading originated from the
    sonar, a laser, or other device.  
**/

class ArRangeDevice
{
public:
  /// Constructor
  AREXPORT ArRangeDevice(size_t currentBufferSize, size_t cumulativeBufferSize,
			 const char *name, unsigned int maxRange,
			 int maxSecondsToKeepCurrent = 0,
			 int maxSecondsToKeepCumulative = 0,
			 double maxDistToKeepCumulative = 0,
			 bool locationDependent = false);
  /// Destructor
  AREXPORT virtual ~ArRangeDevice();
  /// Gets the name of the device
  AREXPORT virtual const char *getName(void) const;
  /// Sets the robot this device is attached to
  AREXPORT virtual void setRobot(ArRobot *robot);
  /// Gets the robot this device is attached to
  AREXPORT virtual ArRobot *getRobot(void);
  /// Sets the maximum size of the buffer for current readings
  AREXPORT virtual void setCurrentBufferSize(size_t size);
  /// Sets the maximum size of the buffer for cumulative readings
  AREXPORT virtual void setCumulativeBufferSize(size_t size);
  /// Adds a reading to the buffer
  AREXPORT virtual void addReading(double x, double y);
  /// Gets the closest current reading in the given polar region
  AREXPORT virtual double currentReadingPolar(double startAngle, 
					      double endAngle,
					      double *angle = NULL) const;
  /// Gets the closest cumulative reading in the given polar region
  AREXPORT virtual double cumulativeReadingPolar(double startAngle, 
						 double endAngle,
						 double *angle = NULL) const;
  /// Gets the closest current reading from the given box region
  AREXPORT virtual double currentReadingBox(double x1, double y1, double x2,
					    double y2, 
					    ArPose *readingPos = NULL) const;
  /// Gets the closest current reading from the given box region
  AREXPORT virtual double cumulativeReadingBox(double x1, double y1, double x2,
					       double y2, 
					       ArPose *readingPos = NULL) const;
#ifndef SWIG
  /** @brief Gets the current range buffer
   *  @swigomit See getCurrentBufferAsVector()
   */
  virtual const ArRangeBuffer *getCurrentRangeBuffer(void) const
    { return &myCurrentBuffer; }
  /** @brief Gets the cumulative range buffer
   *  @swigomit See getCumulativeBufferAsVector()
   */
  virtual const ArRangeBuffer *getCumulativeRangeBuffer(void) const
    { return &myCumulativeBuffer; }
  /** @brief Gets the current buffer of readings
   *  @swigomit See getCurrentBufferAsVector()
   */
  virtual const std::list<ArPoseWithTime *> *getCurrentBuffer(void) const
    { return myCurrentBuffer.getBuffer(); }
  /** @brief Gets the current buffer of readings
   *  @swigomit See getCumulativeBufferAsVector()
   */
  virtual const std::list<ArPoseWithTime *> *getCumulativeBuffer(void) const
    { return myCumulativeBuffer.getBuffer(); }
#endif // SWIG

  /// Gets the current range buffer
  virtual ArRangeBuffer *getCurrentRangeBuffer(void)
    { return &myCurrentBuffer; }
  /// Gets the cumulative range buffer
  virtual ArRangeBuffer *getCumulativeRangeBuffer(void) 
    { return &myCumulativeBuffer; }
  /// Gets the current buffer of readings
  virtual std::list<ArPoseWithTime *> *getCurrentBuffer(void) 
    { return myCurrentBuffer.getBuffer(); }
  /** @brief Gets the current buffer of readings as a vector
   *  @swignote The return type will be named 
   *   ArPoseWithTimeVector instead of the std::vector template.
   */
  virtual std::vector<ArPoseWithTime> *getCurrentBufferAsVector(void) 
    { return myCurrentBuffer.getBufferAsVector(); }
  /// Gets the current buffer of readings
  virtual std::list<ArPoseWithTime *> *getCumulativeBuffer(void) 
    { return myCumulativeBuffer.getBuffer(); }
  /** @brief Gets the cumulative buffer of readings as a vector
   *  @swignote The return type will be named ArPoseWithTimeVector
   *    instead of the std::vector template.
   */
  virtual std::vector<ArPoseWithTime> *getCumulativeBufferAsVector(void) 
    { return myCumulativeBuffer.getBufferAsVector(); }

  /// Gets the raw unfiltered readings from the device
  /** The raw readings are the full set of unfiltered readings from the device.
      They are the latest readings. You should not manipulate the list you get from
      this function, the only manipulation of this list should be done by
      the range device itself.  (Its only pointers for speed.)

      @note Only ArSick provides this data currently.  Sonar, bumpers,
      etc. do not provide raw readings.
      This method was added to this base class for future lasers or other
      similar devices.
      Other kinds of range devices are sufficiently different from lasers that
      any "raw" information provided would usually require very different interpretation.
  **/
  virtual const std::list<ArSensorReading *> *getRawReadings(void) const
    { return myRawReadings; }

  ///  Gets the raw unfiltered readings from the device into a vector 
  AREXPORT virtual std::vector<ArSensorReading> *getRawReadingsAsVector(void);

  /// Gets the raw unfiltered readings from the device (but pose takens are corrected)
  /** The raw readings are the full set of unfiltered readings from
      the device.  They are the latest readings. You should not
      manipulate the list you get from this function, the only
      manipulation of this list should be done by the range device
      itself.  (Its only pointers for speed.)
      
      This is like the raw readings but they were corrected for the
      robot odometry offset (just the pose taken, and encoder psoe
      taken).

      @note Only ArSick provides this data currently.  Sonar, bumpers,
      etc. do not provide raw readings.
      This method was added to this base class for future lasers or other
      similar devices.
      Other kinds of range devices are sufficiently different from lasers that
      any "raw" information provided would usually require very different interpretation.
  **/
  virtual const std::list<ArSensorReading *> *getAdjustedRawReadings(void) const
    { return myRawReadings; }

  ///  Gets the raw adjusted readings from the device into a vector 
  AREXPORT virtual std::vector<ArSensorReading> *getAdjustedRawReadingsAsVector(void);

  /// gets the maximum seconds to keep current readings around
  /**
   @param maxSecondsToKeepCurrent this is the number of seconds to
   keep current readings around, if less than 0 then they are not
   automatically removed because of this
  **/
  void setMaxSecondsToKeepCurrent(int maxSecondsToKeepCurrent)
    { myMaxSecondsToKeepCurrent = maxSecondsToKeepCurrent; }

  /// gets the maximum seconds to keep current readings around
  /**
   @return this is the number of seconds current readings are kept
   around for, if less than 0 then they are not automatically removed
   because of this
  **/
  int getMaxSecondsToKeepCurrent(void) { return myMaxSecondsToKeepCurrent; }


  /// gets the maximum seconds to keep cumulative readings around
  /**
   @param maxSecondsToKeepCumulative this is the number of seconds to keep
   cumulative readings around, if less than 0 then they are not automatically
   removed because of this
  **/
  void setMaxSecondsToKeepCumulative(int maxSecondsToKeepCumulative)
    { myMaxSecondsToKeepCumulative = maxSecondsToKeepCumulative; }
  /// gets the maximum seconds to keep current readings around
  /**
   @return this is the number of seconds cumulative readings are kept
   around for, if less than 0 then they are not automatically removed
   because of this
  **/
  int getMaxSecondsToKeepCumulative(void) 
    { return myMaxSecondsToKeepCumulative; }

  /// sets the maximum distance cumulative readings can be from current pose
  /**
   @param maxDistToKeepCumulative if cumulative readings are further than
   this from where the current pose they are removed, if this is less
   than 0 they are not removed because of this
  **/
  void setMaxDistToKeepCumulative(double maxDistToKeepCumulative) 
    { myMaxDistToKeepCumulative = maxDistToKeepCumulative; } 

  /// sets the maximum distance cumulative readings can be from current pose
  /**
   @return if cumulative readings are further than this from where the
   current pose they are removed, if this is less than 0 they are not
   removed because of this
  **/
  double getMaxDistToKeepCumulative(void) { return myMaxDistToKeepCumulative; }

  /// Gets if this device is location dependent or not
  bool isLocationDependent(void) { return myIsLocationDependent; }
  /// Clears all the current readings
  virtual void clearCurrentReadings(void) { myCurrentBuffer.clear(); }
  /// Clears all the cumulative readings
  virtual void clearCumulativeReadings(void) { myCumulativeBuffer.clear(); }
  /// Clears all the cumulative readings older than this number of milliseconds
  virtual void clearCumulativeOlderThan(int milliSeconds) 
    { myCumulativeBuffer.clearOlderThan(milliSeconds); }

  /// Clears all the cumulative readings older than this number of seconds
  virtual void clearCumulativeOlderThanSeconds(int seconds) 
    { myCumulativeBuffer.clearOlderThanSeconds(seconds); }
  
  /// Gets the maximum range for this device
  virtual unsigned int getMaxRange(void) { return myMaxRange; }
  /// Sets the maximum range for this device
  virtual void setMaxRange(unsigned int maxRange) 
    { myMaxRange = maxRange; }

  /// Applies a transform to the buffers
  AREXPORT virtual void applyTransform(ArTransform trans, 
				       bool doCumulative = true);

  /// Gets data used for visualizing the current buffer (see ArNetworking)
  virtual ArDrawingData *getCurrentDrawingData(void) 
    { return myCurrentDrawingData; }
  /// Gets data used for visualizing the cumulative buffer (see ArNetworking)
  virtual ArDrawingData *getCumulativeDrawingData(void) 
    { return myCumulativeDrawingData; }
  /// Sets data for visualizing the current buffer (and if we own it)
  AREXPORT virtual void setCurrentDrawingData(ArDrawingData *data, 
					      bool takeOwnershipOfData);
  /// Sets data for visualizing the cumulative buffer (and if we own it)
  AREXPORT virtual void setCumulativeDrawingData(ArDrawingData *data, 
						 bool takeOwnershipOfData);

  

  /// Lock this device
  /**
     @note If you are implementaing a subclass and also inheriting an ASyncTask,
     you MUST override this method to use the lock from the ArASyncTask
  */
  /*AREXPORT*/ virtual int lockDevice() { return(myDeviceMutex.lock());}
  /// Try to lock this device
  /**
     @note If you are implementaing a subclass and also inheriting an ASyncTask,
     you MUST override this method to use the lock from the ArASyncTask
  */
  /*AREXPORT*/ virtual int tryLockDevice() {return(myDeviceMutex.tryLock());}
  /// Unlock this device
  /**
     @note If you are implementaing a subclass and also inheriting an ASyncTask,
     you MUST override this method to use the lock from the ArASyncTask
  */
  /*AREXPORT*/ virtual int unlockDevice() {return(myDeviceMutex.unlock());}

protected:
  AREXPORT void filterCallback(void);
  /**
    This call should be called by the range device every robot cycle
    before the range device makes new readings (and even if it isn't
    adding any that cycle)... it will adjust the raw readings by the
    robot odometry offset.  The robot should be locked when this
    happens (which should be the case if you're doing it in the robot
    callback). The code currently assumes that all readings were taken
    at the same point, so if that isn't true with your device then you
    can't use this mechanism.
  **/
  AREXPORT void adjustRawReadings(bool interlaced);
  std::vector<ArSensorReading> myRawReadingsVector;
  std::vector<ArSensorReading> myAdjustedRawReadingsVector;
  std::string myName;
  ArRobot *myRobot;
  unsigned int myMaxRange; 
  ArRangeBuffer myCurrentBuffer;
  ArRangeBuffer myCumulativeBuffer;
  int myMaxSecondsToKeepCumulative;
  int myMaxSecondsToKeepCurrent;
  double myMaxDistToKeepCumulative;
  ArFunctorC<ArRangeDevice> myFilterCB;
  std::list<ArSensorReading *> *myRawReadings;
  std::list<ArSensorReading *> *myAdjustedRawReadings;
  ArDrawingData *myCurrentDrawingData;
  bool myOwnCurrentDrawingData;
  ArDrawingData *myCumulativeDrawingData;
  bool myOwnCumulativeDrawingData;
  ArMutex myDeviceMutex;
  bool myIsLocationDependent;
};

#endif // ARRANGEDEVICE_H
