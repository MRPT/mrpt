/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARTRANSFORM_H
#define ARTRANSFORM_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

/// A class to handle transforms between different coordinates

class ArTransform
{
public:
  /// Constructor
  ArTransform()
    {
      myX = 0;
      myY = 0;
      myTh = 0;
      myCos = ArMath::cos(myTh);
      mySin = ArMath::sin(myTh);
    }
  /// Constructor, Sets the transform so points in this coord system
  /// transform to abs world coords

  ArTransform(ArPose pose)
    {
      setTransform(pose);
    }
  /// Constructor, sets the transform so that pose1 will be
  /// transformed to pose2
  ArTransform(ArPose pose1, ArPose pose2)
    {
      setTransform(pose1, pose2);
    }
  /// Destructor
  virtual ~ArTransform() {}

  /// Take the source pose and run the transform on it to put it into abs
  /// coordinates
  /**
      @param source the parameter to transform
      @return the source transformed into absolute coordinates
  */
  ArPose doTransform(ArPose source)
    {
      ArPose ret;
      ret.setX(myX + myCos * source.getX() + mySin * source.getY());
      ret.setY(myY + myCos * source.getY() - mySin * source.getX());
      ret.setTh(ArMath::addAngle(source.getTh(),myTh));
      return ret;
    }
  /// Take the source pose and run the transform on it to put it into abs
  /// coordinates
  /**
      @param source the parameter to transform
      @return the source transformed into absolute coordinates
  */
  ArPoseWithTime doTransform(ArPoseWithTime source)
    {
      ArPoseWithTime ret;
      ret.setX(myX + myCos * source.getX() + mySin * source.getY());
      ret.setY(myY + myCos * source.getY() - mySin * source.getX());
      ret.setTh(ArMath::addAngle(source.getTh(),myTh));
      ret.setTime(source.getTime());
      return ret;
    }

  /// Take the source pose and run the inverse transform on it, taking it from
  /// abs coords to local
  /**
      The source and result can be the same
      @param source the parameter to transform
      @return the source transformed from absolute into local coords
  */
  /*AREXPORT*/ArPose doInvTransform(ArPose source)
    {
      ArPose ret;
      double tx = source.getX() - myX;
      double ty = source.getY() - myY;
      ret.setX(myCos * tx - mySin * ty);
      ret.setY(myCos * ty + mySin * tx);
      ret.setTh(ArMath::subAngle(source.getTh(),myTh));
      return ret;
    }

  /// Take the source pose and run the inverse transform on it, taking it from
  /// abs coords to local
  /**
      The source and result can be the same
      @param source the parameter to transform
      @return the source transformed from absolute into local coords
  */
  /*AREXPORT*/ArPoseWithTime doInvTransform(ArPoseWithTime source)
    {
      ArPoseWithTime ret;
      double tx = source.getX() - myX;
      double ty = source.getY() - myY;
      ret.setX(myCos * tx - mySin * ty);
      ret.setY(myCos * ty + mySin * tx);
      ret.setTh(ArMath::subAngle(source.getTh(),myTh));
      ret.setTime(source.getTime());
      return ret;
    }


  /// Take a std::list of sensor readings and do the transform on it
  AREXPORT void doTransform(std::list<ArPose *> *poseList);
  /// Take a std::list of sensor readings and do the transform on it
  AREXPORT void doTransform(std::list<ArPoseWithTime *> *poseList);
  /// Sets the transform so points in this coord system transform to abs world coords
  AREXPORT void setTransform(ArPose pose);
  /// Sets the transform so that pose1 will be transformed to pose2
  AREXPORT void setTransform(ArPose pose1, ArPose pose2);
  /// Gets the transform angle value (degrees)
  double getTh() { return myTh; }

protected:
  double myX;
  double myY;
  double myTh;
  double myCos;
  double mySin;
};


#endif // ARTRANSFORM_H
