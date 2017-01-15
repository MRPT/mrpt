/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef HMT_SLAM_common_H
#define HMT_SLAM_common_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/list_searchable.h>
#include <set>

#include <mrpt/hmtslam/link_pragmas.h>


#define COMMON_TOPOLOG_HYP					static_cast<THypothesisID>(0)

#define NODE_ANNOTATION_METRIC_MAPS 		"metricMaps"         // CMultiMetricMap
#define NODE_ANNOTATION_REF_POSEID 			"refPoseID"			 // TPoseID
#define NODE_ANNOTATION_POSES_GRAPH 		"posesGraph"         // CRobotPosesGraph

#define NODE_ANNOTATION_PLACE_POSE 			"placePose"          // mrpt::poses::CPoint2D

#define ARC_ANNOTATION_DELTA				"Delta"				 // CPose3DPDF (for the current implementation, it's a CPose3DPDFParticles)
#define ARC_ANNOTATION_DELTA_SRC_POSEID		"Delta_poseID_src"	 // TPoseID (elemental datatype)
#define ARC_ANNOTATION_DELTA_TRG_POSEID		"Delta_poseID_trg"	 // TPoseID (elemental datatype)

/**  Used in constructor of mrpt::hmtslam::CHMHMapArc */
#define ARC_TYPES			"Membership,Navegability,RelativePose,Location"
#define DEFAULT_ARC_TYPE	"Membership"

/**  Used in constructor of mrpt::hmtslam::CHMHMapNode */
#define NODE_TYPES			"Place,Area,TopologicalMap,Object"
#define DEFAULT_NODE_TYPE	"Place"

// Used as current robot pose when initializing an empty HMT-map.
#define POSEID_INVALID	static_cast<TPoseID>(-1)

#define AREAID_INVALID	static_cast<uint64_t>(-1)

#define MSG_SOURCE_LSLAM    1
#define MSG_SOURCE_AA       2

namespace mrpt
{
	namespace hmtslam
	{
		class HMTSLAM_IMPEXP CHMHMapArc;
		class HMTSLAM_IMPEXP CHMHMapNode;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CHMHMapArc,mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** An integer number uniquely identifying each of the concurrent hypotheses for the robot topological path (& possibly local metric clusters) in HMT-SLAM.
		  *   The number 0 has the special meaning of "that part of the map/robot path in which all hypotheses agree".
		  *  They can be generated from CHMTSLAM::generateHypothesisID()
		  */
		typedef int64_t  THypothesisID;

		/** An integer number uniquely identifying each robot pose stored in HMT-SLAM.
		  * They can be generated from CHMTSLAM::generatePoseID()
		  */
		typedef uint64_t  TPoseID;

		typedef std::pair<TPoseID,TPoseID>  TPairPoseIDs;

		typedef std::vector<TPoseID> TPoseIDList;
		typedef std::set<TPoseID> TPoseIDSet;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( THypothesisIDSet,mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** A set of hypothesis IDs, used for arcs and nodes in multi-hypothesis hybrid maps.
		  *  \sa THypothesisID, CHierarchicalMHMap
		  * \ingroup mrpt_hmtslam_grp
		  */
		class HMTSLAM_IMPEXP THypothesisIDSet : public mrpt::utils::CSerializable, public std::set<THypothesisID>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( THypothesisIDSet )

		public:
			/** Default constructor
			  */
			THypothesisIDSet()
			{
			}

			/** Constructor with one initial element
			  */
			THypothesisIDSet( const THypothesisID& val)
			{
				insert( val );
			}

			virtual ~THypothesisIDSet()
			{
			}

			/** Returns true if the hypothesis is into the set.
			  */
			bool has(  const THypothesisID& val ) const
			{
				return find(val)!=end() || find(COMMON_TOPOLOG_HYP)!=end();
			}

			/** Dump to console.
			  */
			void debugDump() const;

		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( THypothesisIDSet,mrpt::utils::CSerializable, HMTSLAM_IMPEXP )


		/** A class for storing a sequence of arcs (a path).
		  * \sa CHMTSLAM
		  */
		class HMTSLAM_IMPEXP TArcList : public mrpt::utils::list_searchable<mrpt::hmtslam::CHMHMapArcPtr>
		{
		private:
			typedef mrpt::utils::list_searchable<mrpt::hmtslam::CHMHMapArcPtr> BASE;

		public:
			void  debugDump();
			void read( utils::CStream &in );
			void write( utils::CStream &out ) const;

		};


	} // End of namespace
} // End of namespace



#endif
