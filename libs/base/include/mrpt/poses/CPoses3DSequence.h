/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CPoses3DSequence_H
#define CPoses3DSequence_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
namespace poses
{

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoses3DSequence, mrpt::utils::CSerializable )

	/** This class stores a sequence of relative, incremental 3D poses. It is useful as the bases storing unit for more complex probability particles and for computing the absolute pose of any intermediate pose.
	 *
	 * \sa CPose3D, CMultiMetricMap
	 * \ingroup poses_grp
	 */
	class BASE_IMPEXP CPoses3DSequence : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPoses3DSequence )
	public:
		/** Default constructor
		 */
		CPoses3DSequence();


		/** Returns the poses count in the sequence:
		 */
		size_t posesCount();

		/** Reads the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
		 * \exception std::exception On invalid index value
		 */
		void  getPose(unsigned int ind, CPose3D &outPose);

		/** Changes the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
		 * \exception std::exception On invalid index value
		 */
		void  changePose(unsigned int ind, CPose3D &inPose);

		/** Appends a new pose at the end of sequence. Remember that poses are relative, incremental to the last one.
		 */
		void  appendPose(CPose3D &newPose);

		/** Clears the sequence.
		 */
		void  clear();

		/** Returns the absolute pose of a robot after moving "n" poses, so for "n=0" the origin pose (0,0,0deg) is returned, for "n=1" the first pose is returned, and for "n=posesCount()", the pose
		 *  of robot after moving ALL poses is returned, all of them relative to the starting pose.
		 * \exception std::exception On invalid index value
		 * \sa absolutePoseAfterAll
		 */
		CPose3D  absolutePoseOf(unsigned int n);

		/** A shortcut for "absolutePoseOf( posesCount() )".
		 * \sa absolutePoseOf, posesCount
		 */
		CPose3D  absolutePoseAfterAll();

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
		std::vector<mrpt::math::TPose3D>	m_poses;

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
