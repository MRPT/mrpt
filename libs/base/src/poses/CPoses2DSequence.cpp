/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers 


#include <mrpt/poses/CPoses2DSequence.h>
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE( CPoses2DSequence, CSerializable ,mrpt::poses)

/*---------------------------------------------------------------
			Default constructor
  ---------------------------------------------------------------*/
CPoses2DSequence::CPoses2DSequence() : poses()
{

}

/*---------------------------------------------------------------
			Returns the poses count in the sequence:
  ---------------------------------------------------------------*/
size_t CPoses2DSequence::posesCount()
{
	return poses.size();
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPoses2DSequence::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	i,n;

		// The poses:
		n = poses.size();
		out << n;
		for (i=0;i<n;i++) out << poses[i];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPoses2DSequence::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n;

			// The poses:
			in >> n;
			poses.resize(n);
			for (i=0;i<n;i++)	in >> poses[i];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
Reads the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
 ---------------------------------------------------------------*/
void  CPoses2DSequence::getPose(unsigned int ind, CPose2D &outPose)
{
	if (ind>=poses.size()) THROW_EXCEPTION("Index out of range!!");

	outPose = poses[ind];
}

/*---------------------------------------------------------------
Changes the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
 ---------------------------------------------------------------*/
void  CPoses2DSequence::changePose(unsigned int ind, CPose2D &inPose)
{
	if (ind>=poses.size()) THROW_EXCEPTION("Index out of range!!");

	*((&(poses[ind])) + ind) = inPose;
}

/*---------------------------------------------------------------
	Appends a new pose at the end of sequence. Remember that poses are relative, incremental to the last one.
 ---------------------------------------------------------------*/
void  CPoses2DSequence::appendPose(CPose2D &newPose)
{
	poses.push_back( newPose );
}

/*---------------------------------------------------------------
			Clears the sequence.
 ---------------------------------------------------------------*/
void  CPoses2DSequence::clear()
{
	poses.clear();
}

/*---------------------------------------------------------------
 ---------------------------------------------------------------*/
/** Returns the absolute pose of a robot after moving "n" poses, so for "n=0" the origin pose (0,0,0deg) is returned, for "n=1" the first pose is returned, and for "n=posesCount()", the pose
 *  of robot after moving ALL poses is returned, all of them relative to the starting pose.
 */
CPose2D  CPoses2DSequence::absolutePoseOf(unsigned int n)
{
	CPose2D				ret(0,0,0);
	unsigned int		i;

	if (n>poses.size()) THROW_EXCEPTION("Index out of range!!");

	for (i=0;i<n;i++)	ret = ret + poses[i];

	return ret;
}


/*---------------------------------------------------------------
	A shortcut for "absolutePoseOf( posesCount() )".
 ---------------------------------------------------------------*/
CPose2D  CPoses2DSequence::absolutePoseAfterAll()
{
	return absolutePoseOf( posesCount() );
}

/*---------------------------------------------------------------
	Returns the traveled distance after moving "n" poses, so for "n=0" it returns 0, for "n=1" the first traveled distance, and for "n=posesCount()", the total
	distance after ALL movements.
 ---------------------------------------------------------------*/
float  CPoses2DSequence::computeTraveledDistanceAfter(unsigned int n)
{
	unsigned int		i;
	float	dist = 0;

	if (n>poses.size()) THROW_EXCEPTION("Index out of range!!");

	for (i=0;i<n;i++)	dist+= poses[i].norm();

	return dist;
}

/*---------------------------------------------------------------
	Returns the traveled distance after ALL movements.
	A shortcut for "computeTraveledDistanceAfter( posesCount() )".
 ---------------------------------------------------------------*/
float  CPoses2DSequence::computeTraveledDistanceAfterAll()
{
	return computeTraveledDistanceAfter( posesCount() );
}
