/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArTransform.h"

AREXPORT void ArTransform::doTransform(std::list<ArPose *> *poseList)
{
  std::list<ArPose *>::iterator it;
  ArPose *pose;
  
  for (it = poseList->begin(); it != poseList->end(); it++)
  {
    pose = (*it);
    *pose = doTransform(*pose);
  }

}

AREXPORT void ArTransform::doTransform(std::list<ArPoseWithTime *> *poseList)
{
  std::list<ArPoseWithTime *>::iterator it;
  ArPoseWithTime *pose;
  
  for (it = poseList->begin(); it != poseList->end(); it++)
  {
    pose = (*it);
    *pose = doTransform(*pose);
  }

}

/**
   @param pose the coord system from which we transform to abs world coords
*/
AREXPORT void ArTransform::setTransform(ArPose pose) 
{ 
  myTh = pose.getTh();
  myCos = ArMath::cos(-myTh);
  mySin = ArMath::sin(-myTh);
  myX = pose.getX();
  myY = pose.getY();
}

/**
   @param pose1 transform this into pose2
   @param pose2 transform pose1 into this
*/
AREXPORT void ArTransform::setTransform(ArPose pose1, ArPose pose2)
{
  myTh = ArMath::subAngle(pose2.getTh(), pose1.getTh());
  myCos = ArMath::cos(-myTh);
  mySin = ArMath::sin(-myTh);
  myX = pose2.getX() - (myCos * pose1.getX() + mySin * pose1.getY());
  myY = pose2.getY() - (myCos * pose1.getY() - mySin * pose1.getX());
}
