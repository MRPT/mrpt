/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/utils/CStream.h>

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
void  CPoses2DSequence::writeToStream(mrpt::utils::CStream &out,int *version) const
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
void  CPoses2DSequence::readFromStream(mrpt::utils::CStream &in, int version)
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
