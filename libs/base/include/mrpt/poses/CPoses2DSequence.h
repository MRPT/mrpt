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


	} // End of namespace
} // End of namespace

#endif
