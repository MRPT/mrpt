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
#include <mrpt/containers/list_searchable.h>
#include <set>

#define COMMON_TOPOLOG_HYP static_cast<THypothesisID>(0)

#define NODE_ANNOTATION_METRIC_MAPS "metricMaps"  // CMultiMetricMap
#define NODE_ANNOTATION_REF_POSEID "refPoseID"  // TPoseID
#define NODE_ANNOTATION_POSES_GRAPH "posesGraph"  // CRobotPosesGraph

#define NODE_ANNOTATION_PLACE_POSE "placePose"  // mrpt::poses::CPoint2D

#define ARC_ANNOTATION_DELTA \
	"Delta"  // CPose3DPDF (for the current implementation, it's a
// CPose3DPDFParticles)
#define ARC_ANNOTATION_DELTA_SRC_POSEID \
	"Delta_poseID_src"  // TPoseID (elemental datatype)
#define ARC_ANNOTATION_DELTA_TRG_POSEID \
	"Delta_poseID_trg"  // TPoseID (elemental datatype)

/**  Used in constructor of mrpt::hmtslam::CHMHMapArc */
#define ARC_TYPES "Membership,Navegability,RelativePose,Location"
#define DEFAULT_ARC_TYPE "Membership"

/**  Used in constructor of mrpt::hmtslam::CHMHMapNode */
#define NODE_TYPES "Place,Area,TopologicalMap,Object"
#define DEFAULT_NODE_TYPE "Place"

// Used as current robot pose when initializing an empty HMT-map.
#define POSEID_INVALID static_cast<TPoseID>(-1)

#define AREAID_INVALID static_cast<uint64_t>(-1)

#define MSG_SOURCE_LSLAM 1
#define MSG_SOURCE_AA 2

namespace mrpt::hmtslam
{
class CHMHMapArc;
class CHMHMapNode;

/** An integer number uniquely identifying each of the concurrent hypotheses for
 * the robot topological path (& possibly local metric clusters) in HMT-SLAM.
 *   The number 0 has the special meaning of "that part of the map/robot path
 * in which all hypotheses agree".
 *  They can be generated from CHMTSLAM::generateHypothesisID()
 */
using THypothesisID = int64_t;

/** An integer number uniquely identifying each robot pose stored in HMT-SLAM.
 * They can be generated from CHMTSLAM::generatePoseID()
 */
using TPoseID = uint64_t;

using TPairPoseIDs = std::pair<TPoseID, TPoseID>;

using TPoseIDList = std::vector<TPoseID>;
using TPoseIDSet = std::set<TPoseID>;

/** A set of hypothesis IDs, used for arcs and nodes in multi-hypothesis hybrid
 * maps.
 *  \sa THypothesisID, CHierarchicalMHMap
 * \ingroup mrpt_hmtslam_grp
 */
class THypothesisIDSet : public mrpt::serialization::CSerializable,
						 public std::set<THypothesisID>
{
	DEFINE_SERIALIZABLE(THypothesisIDSet)

   public:
	/** Default constructor
	 */
	THypothesisIDSet() = default;
	/** Constructor with one initial element
	 */
	THypothesisIDSet(const THypothesisID& val) { insert(val); }
	~THypothesisIDSet() override = default;
	/** Returns true if the hypothesis is into the set.
	 */
	bool has(const THypothesisID& val) const
	{
		return find(val) != end() || find(COMMON_TOPOLOG_HYP) != end();
	}

	/** Dump to console.
	 */
	void debugDump() const;
};

/** A class for storing a sequence of arcs (a path).
 * \sa CHMTSLAM
 */
class TArcList : public mrpt::containers::list_searchable<
					 std::shared_ptr<mrpt::hmtslam::CHMHMapArc>>
{
   private:
	using BASE = mrpt::containers::list_searchable<
		std::shared_ptr<mrpt::hmtslam::CHMHMapArc>>;

   public:
	void debugDump();
	void read(mrpt::serialization::CArchive& in);
	void write(mrpt::serialization::CArchive& out) const;
};

}  // namespace mrpt::hmtslam
