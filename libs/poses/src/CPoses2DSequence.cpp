/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPoses2DSequence, CSerializable, mrpt::poses)

size_t CPoses2DSequence::posesCount() { return poses.size(); }
uint8_t CPoses2DSequence::serializeGetVersion() const { return 0; }
void CPoses2DSequence::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(poses.size());
	for (const auto& p : poses) out << p;
}
void CPoses2DSequence::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			poses.resize(in.ReadAs<uint32_t>());
			for (auto& p : poses) in >> p;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
Reads the stored pose at index "ind", where the first one is 0, the last
"posesCount() - 1"
 ---------------------------------------------------------------*/
void CPoses2DSequence::getPose(unsigned int ind, CPose2D& outPose)
{
	if (ind >= poses.size()) THROW_EXCEPTION("Index out of range!!");

	outPose = poses[ind];
}

/*---------------------------------------------------------------
Changes the stored pose at index "ind", where the first one is 0, the last
"posesCount() - 1"
 ---------------------------------------------------------------*/
void CPoses2DSequence::changePose(unsigned int ind, CPose2D& inPose)
{
	if (ind >= poses.size()) THROW_EXCEPTION("Index out of range!!");

	*((&(poses[ind])) + ind) = inPose;
}

/*---------------------------------------------------------------
	Appends a new pose at the end of sequence. Remember that poses are relative,
 incremental to the last one.
 ---------------------------------------------------------------*/
void CPoses2DSequence::appendPose(CPose2D& newPose)
{
	poses.push_back(newPose);
}

/*---------------------------------------------------------------
			Clears the sequence.
 ---------------------------------------------------------------*/
void CPoses2DSequence::clear() { poses.clear(); }
/*---------------------------------------------------------------
 ---------------------------------------------------------------*/
/** Returns the absolute pose of a robot after moving "n" poses, so for "n=0"
 * the origin pose (0,0,0deg) is returned, for "n=1" the first pose is returned,
 * and for "n=posesCount()", the pose
 *  of robot after moving ALL poses is returned, all of them relative to the
 * starting pose.
 */
CPose2D CPoses2DSequence::absolutePoseOf(unsigned int n)
{
	CPose2D ret(0, 0, 0);
	unsigned int i;

	if (n > poses.size()) THROW_EXCEPTION("Index out of range!!");

	for (i = 0; i < n; i++) ret = ret + poses[i];

	return ret;
}

/*---------------------------------------------------------------
	A shortcut for "absolutePoseOf( posesCount() )".
 ---------------------------------------------------------------*/
CPose2D CPoses2DSequence::absolutePoseAfterAll()
{
	return absolutePoseOf(posesCount());
}

/*---------------------------------------------------------------
	Returns the traveled distance after moving "n" poses, so for "n=0" it
 returns 0, for "n=1" the first traveled distance, and for "n=posesCount()", the
 total
	distance after ALL movements.
 ---------------------------------------------------------------*/
float CPoses2DSequence::computeTraveledDistanceAfter(unsigned int n)
{
	unsigned int i;
	float dist = 0;

	if (n > poses.size()) THROW_EXCEPTION("Index out of range!!");

	for (i = 0; i < n; i++) dist += poses[i].norm();

	return dist;
}

/*---------------------------------------------------------------
	Returns the traveled distance after ALL movements.
	A shortcut for "computeTraveledDistanceAfter( posesCount() )".
 ---------------------------------------------------------------*/
float CPoses2DSequence::computeTraveledDistanceAfterAll()
{
	return computeTraveledDistanceAfter(posesCount());
}
