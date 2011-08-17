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
#ifndef HMT_SLAM_common_H
#define HMT_SLAM_common_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/stl_extensions.h>

#include <mrpt/hmtslam/link_pragmas.h>


#define COMMON_TOPOLOG_HYP					static_cast<THypothesisID>(0)

#define NODE_ANNOTATION_METRIC_MAPS 		"metricMaps"         // CMultiMetricMap
#define NODE_ANNOTATION_REF_POSEID 			"refPoseID"			 // TPoseID
#define NODE_ANNOTATION_POSES_GRAPH 		"posesGraph"         // CRobotPosesGraph

#define NODE_ANNOTATION_PLACE_POSE 			"placePose"          // CPoint2D

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
		using namespace mrpt::utils;

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


		/** A class for storing a sequence of arcs (a path).
		  * \sa CHMTSLAM
		  */
		class HMTSLAM_IMPEXP TArcList : public list_searchable<mrpt::hmtslam::CHMHMapArcPtr>
		{
		private:
			typedef list_searchable<mrpt::hmtslam::CHMHMapArcPtr> BASE;

		public:
			void  debugDump();
			void read( utils::CStream &in );
			void write( utils::CStream &out ) const;

		};


	} // End of namespace
} // End of namespace



#endif
