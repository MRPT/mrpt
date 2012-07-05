/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef CRobotPosesGraph_H
#define CRobotPosesGraph_H

#include <mrpt/utils/CSerializable.h>

#include <mrpt/hmtslam/HMT_SLAM_common.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/slam/CMultiMetricMap.h>


namespace mrpt
{
	namespace hmtslam
	{
		using namespace mrpt::slam;

		/** Information kept for each robot pose used in CRobotPosesGraph */
		struct HMTSLAM_IMPEXP TPoseInfo
		{
			CSensoryFrame			sf;   //!< The observations
			CPose3DPDFParticles		pdf;  //!< The robot pose PDF
		};

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRobotPosesGraph, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** Auxiliary class used in mrpt::slam::CLocalMetricHypothesis for HMT-SLAM; this class stores a set of robot poses and its sensory frames and pose PDF, for being stored in a HMT-map as a serializable object in annotation NODE_ANNOTATION_POSES_GRAPH.
		  * \ingroup mrpt_hmtslam_grp
		  */
		class HMTSLAM_IMPEXP CRobotPosesGraph : public  mrpt::utils::CSerializable, public std::map<TPoseID,TPoseInfo>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CRobotPosesGraph )
		public:

			/** Insert all the observations in the map (without erasing previous contents). */
			void insertIntoMetricMap( CMultiMetricMap	&metricMap ) const;

			/** Converts the contents of this object into a 'simplemap' (mrpt::slam::CSimpleMap) object. */
			void convertIntoSimplemap( CSimpleMap &out_simplemap) const;

		}; // end of class


	} // End of namespace
} // End of namespace
#endif
