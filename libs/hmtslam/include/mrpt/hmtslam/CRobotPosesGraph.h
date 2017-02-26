/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRobotPosesGraph_H
#define CRobotPosesGraph_H

#include <mrpt/utils/CSerializable.h>

#include <mrpt/hmtslam/HMT_SLAM_common.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CMultiMetricMap.h>


namespace mrpt
{
	namespace hmtslam
	{
		/** Information kept for each robot pose used in CRobotPosesGraph */
		struct HMTSLAM_IMPEXP TPoseInfo
		{
			mrpt::obs::CSensoryFrame			sf;   //!< The observations
			mrpt::poses::CPose3DPDFParticles		pdf;  //!< The robot pose PDF
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
			void insertIntoMetricMap( mrpt::maps::CMultiMetricMap	&metricMap ) const;

			/** Converts the contents of this object into a 'simplemap' (mrpt::maps::CSimpleMap) object. */
			void convertIntoSimplemap( mrpt::maps::CSimpleMap &out_simplemap) const;

		}; // end of class
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CRobotPosesGraph, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )


	} // End of namespace
} // End of namespace
#endif
