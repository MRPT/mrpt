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
