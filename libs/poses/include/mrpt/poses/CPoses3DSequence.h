/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::poses
{
/** This class stores a sequence of relative, incremental 3D poses. It is useful
 * as the bases storing unit for more complex probability particles and for
 * computing the absolute pose of any intermediate pose.
 *
 * \sa CPose3D, CMultiMetricMap
 * \ingroup poses_grp
 */
class CPoses3DSequence : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CPoses3DSequence)
   public:
	/** Returns the poses count in the sequence:
	 */
	size_t posesCount();

	/** Reads the stored pose at index "ind", where the first one is 0, the last
	 * "posesCount() - 1"
	 * \exception std::exception On invalid index value
	 */
	void getPose(unsigned int ind, CPose3D& outPose);

	/** Changes the stored pose at index "ind", where the first one is 0, the
	 * last "posesCount() - 1"
	 * \exception std::exception On invalid index value
	 */
	void changePose(unsigned int ind, CPose3D& inPose);

	/** Appends a new pose at the end of sequence. Remember that poses are
	 * relative, incremental to the last one.
	 */
	void appendPose(CPose3D& newPose);

	/** Clears the sequence.
	 */
	void clear();

	/** Returns the absolute pose of a robot after moving "n" poses, so for
	 * "n=0" the origin pose (0,0,0deg) is returned, for "n=1" the first pose is
	 * returned, and for "n=posesCount()", the pose
	 *  of robot after moving ALL poses is returned, all of them relative to the
	 * starting pose.
	 * \exception std::exception On invalid index value
	 * \sa absolutePoseAfterAll
	 */
	CPose3D absolutePoseOf(unsigned int n);

	/** A shortcut for "absolutePoseOf( posesCount() )".
	 * \sa absolutePoseOf, posesCount
	 */
	CPose3D absolutePoseAfterAll();

	/** Returns the traveled distance after moving "n" poses, so for "n=0" it
	 * returns 0, for "n=1" the first traveled distance, and for
	 * "n=posesCount()", the total
	 *  distance after ALL movements.
	 * \exception std::exception On invalid index value
	 * \sa computeTraveledDistanceAfterAll
	 */
	float computeTraveledDistanceAfter(unsigned int n);

	/** Returns the traveled distance after ALL movements.
	 *   A shortcut for "computeTraveledDistanceAfter( posesCount() )".
	 * \sa computeTraveledDistanceAfter
	 */
	float computeTraveledDistanceAfterAll();

   private:
	/** The internal sequence of poses, stored as relative, incremental poses,
	 * thus each one is situated just at the end point of last one, where the
	 * first one is referenced to (0,0,0deg)
	 */
	std::vector<mrpt::math::TPose3D> m_poses;

};  // End of class def.
}  // namespace mrpt::poses
