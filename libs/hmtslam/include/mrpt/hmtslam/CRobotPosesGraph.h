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
