/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>

#include <mrpt/hmtslam/HMT_SLAM_common.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CMultiMetricMap.h>

namespace mrpt::hmtslam
{
/** Information kept for each robot pose used in CRobotPosesGraph */
struct TPoseInfo
{
	/** The observations */
	mrpt::obs::CSensoryFrame sf;
	/** The robot pose PDF */
	mrpt::poses::CPose3DPDFParticles pdf;
};

/** Auxiliary class used in mrpt::slam::CLocalMetricHypothesis for HMT-SLAM;
 * this class stores a set of robot poses and its sensory frames and pose PDF,
 * for being stored in a HMT-map as a serializable object in annotation
 * NODE_ANNOTATION_POSES_GRAPH.
 * \ingroup mrpt_hmtslam_grp
 */
class CRobotPosesGraph : public mrpt::serialization::CSerializable,
						 public std::map<TPoseID, TPoseInfo>
{
	DEFINE_SERIALIZABLE(CRobotPosesGraph)
   public:
	/** Insert all the observations in the map (without erasing previous
	 * contents). */
	void insertIntoMetricMap(mrpt::maps::CMultiMetricMap& metricMap) const;

	/** Converts the contents of this object into a 'simplemap'
	 * (mrpt::maps::CSimpleMap) object. */
	void convertIntoSimplemap(mrpt::maps::CSimpleMap& out_simplemap) const;

};  // end of class

}  // namespace mrpt::hmtslam
