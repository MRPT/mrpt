/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPoses3DSequence.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPoses3DSequence, CSerializable, mrpt::poses)

size_t CPoses3DSequence::posesCount() { return m_poses.size(); }
uint8_t CPoses3DSequence::serializeGetVersion() const { return 0; }
void CPoses3DSequence::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_poses.size());
	for (const auto& p : m_poses) out << p;
}
void CPoses3DSequence::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			m_poses.resize(in.ReadAs<uint32_t>());
			for (auto& p : m_poses) in >> p;
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
void CPoses3DSequence::getPose(unsigned int ind, CPose3D& outPose)
{
	if (ind >= m_poses.size()) THROW_EXCEPTION("getPose: Index out of range!!");

	outPose = CPose3D(m_poses[ind]);
}

/*---------------------------------------------------------------
Changes the stored pose at index "ind", where the first one is 0, the last
"posesCount() - 1"
 ---------------------------------------------------------------*/
void CPoses3DSequence::changePose(unsigned int ind, CPose3D& inPose)
{
	if (ind >= m_poses.size()) THROW_EXCEPTION("getPose: Index out of range!!");
	m_poses[ind] = inPose.asTPose();
}

/*---------------------------------------------------------------
	Appends a new pose at the end of sequence. Remember that poses are relative,
 incremental to the last one.
 ---------------------------------------------------------------*/
void CPoses3DSequence::appendPose(CPose3D& newPose)
{
	m_poses.push_back(newPose.asTPose());
}

/*---------------------------------------------------------------
			Clears the sequence.
 ---------------------------------------------------------------*/
void CPoses3DSequence::clear() { m_poses.clear(); }
/*---------------------------------------------------------------
 ---------------------------------------------------------------*/
/** Returns the absolute pose of a robot after moving "n" poses, so for "n=0"
 * the origin pose (0,0,0deg) is returned, for "n=1" the first pose is returned,
 * and for "n=posesCount()", the pose
 *  of robot after moving ALL poses is returned, all of them relative to the
 * starting pose.
 */
CPose3D CPoses3DSequence::absolutePoseOf(unsigned int n)
{
	CPose3D ret(0, 0, 0);
	unsigned int i;

	if (n > m_poses.size())
		THROW_EXCEPTION("absolutePoseOf: Index out of range!!");

	for (i = 0; i < n; i++) ret = ret + CPose3D(m_poses[i]);

	return ret;
}

/*---------------------------------------------------------------
	A shortcut for "absolutePoseOf( posesCount() )".
 ---------------------------------------------------------------*/
CPose3D CPoses3DSequence::absolutePoseAfterAll()
{
	return absolutePoseOf(posesCount());
}

/*---------------------------------------------------------------
	Returns the traveled distance after moving "n" poses, so for "n=0" it
 returns 0, for "n=1" the first traveled distance, and for "n=posesCount()", the
 total
	distance after ALL movements.
 ---------------------------------------------------------------*/
float CPoses3DSequence::computeTraveledDistanceAfter(unsigned int n)
{
	unsigned int i;
	float dist = 0;

	if (n > m_poses.size())
		THROW_EXCEPTION("computeTraveledDistanceAfter: Index out of range!!");

	for (i = 0; i < n; i++) dist += m_poses[i].norm();
	return dist;
}

/*---------------------------------------------------------------
	Returns the traveled distance after ALL movements.
	A shortcut for "computeTraveledDistanceAfter( posesCount() )".
 ---------------------------------------------------------------*/
float CPoses3DSequence::computeTraveledDistanceAfterAll()
{
	return computeTraveledDistanceAfter(posesCount());
}
