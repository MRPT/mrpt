/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPoses2DSequence_H
#define CPoses2DSequence_H

#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
namespace poses
{

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoses2DSequence, mrpt::utils::CSerializable )

	/** This class stores a sequence of relative, incremental 2D poses. It is useful as the bases storing unit for more complex probability particles and for computing the absolute pose of any intermediate pose.
	 *
	 * \sa CPose2D, CMultiMetricMap
	 * \ingroup poses_grp
	 */
	class BASE_IMPEXP CPoses2DSequence : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPoses2DSequence )
	public:
		/** Default constructor
		 */
		CPoses2DSequence();

		/** Returns the poses count in the sequence:
		 */
		size_t	 posesCount();

		/** Reads the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
		 * \exception std::exception On invalid index value
		 */
		void  getPose(unsigned int ind, CPose2D &outPose);

		/** Changes the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
		 * \exception std::exception On invalid index value
		 */
		void  changePose(unsigned int ind, CPose2D &inPose);

		/** Appends a new pose at the end of sequence. Remember that poses are relative, incremental to the last one.
		 */
		void  appendPose(CPose2D &newPose);

		/** Clears the sequence.
		 */
		void  clear();

		/** Returns the absolute pose of a robot after moving "n" poses, so for "n=0" the origin pose (0,0,0deg) is returned, for "n=1" the first pose is returned, and for "n=posesCount()", the pose
		 *  of robot after moving ALL poses is returned, all of them relative to the starting pose.
		 * \exception std::exception On invalid index value
		 * \sa absolutePoseAfterAll
		 */
		CPose2D  absolutePoseOf(unsigned int n);

		/** A shortcut for "absolutePoseOf( posesCount() )".
		 * \sa absolutePoseOf, posesCount
		 */
		CPose2D  absolutePoseAfterAll();

		/** Returns the traveled distance after moving "n" poses, so for "n=0" it returns 0, for "n=1" the first traveled distance, and for "n=posesCount()", the total
		 *  distance after ALL movements.
		 * \exception std::exception On invalid index value
		 * \sa computeTraveledDistanceAfterAll
		 */
		float  computeTraveledDistanceAfter(unsigned int n);

		/** Returns the traveled distance after ALL movements.
		 *   A shortcut for "computeTraveledDistanceAfter( posesCount() )".
		 * \sa computeTraveledDistanceAfter
		 */
		float  computeTraveledDistanceAfterAll();

	private:
		/** The internal sequence of poses, stored as relative, incremental poses, thus each one is situated just at the end point of last one, where the first one is referenced to (0,0,0deg)
		 */
		mrpt::aligned_containers<CPose2D>::vector_t	poses;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPoses2DSequence, mrpt::utils::CSerializable )


	} // End of namespace
} // End of namespace

#endif
