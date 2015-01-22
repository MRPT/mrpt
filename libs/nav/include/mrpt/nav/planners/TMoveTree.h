/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/graphs/CDirectedTree.h>
#include <mrpt/utils/traits_map.h>
#include <mrpt/math/wrap2pi.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		/** Generic base for metrics */
		template<class NODE_TYPE>
		struct PoseDistanceMetric; 
		
		/** This class contains motions and motions tree structures for the hybrid navigation algorithm
		*
		*  <b>Usage:</b><br>
		*      \note: this class inheredit mrpt::graphs::CDirectedTree, please refer to inheritance for detail about generic tree methods
		*
		*      - initialize a motions tree using .initializeMoveTree()
		*      - addEdge (from, to)
		*      - add here more instructions
		*
		* <b>Changes history</b>
		*      - 06/MAR/2014: Creation (MB)
		*      - 21/JAN/2015: Refactoring (JLBC)
		*
		*  \ingroup mrpt_nav_grp
		*/
		template<
			class NODE_TYPE, 
			class EDGE_TYPE,
			class MAPS_IMPLEMENTATION = mrpt::utils::map_traits_map_as_vector // Use std::map<> vs. std::vector<>
		>
		class TMoveTree : public mrpt::graphs::CDirectedTree<EDGE_TYPE>
		{
		public:
			typedef typename MAPS_IMPLEMENTATION::template map<mrpt::utils::TNodeID,NODE_TYPE>  node_map_t;  //!< Map: TNode_ID => Node info

			/** Finds the nearest node to a given pose, using the given metric */
			mrpt::utils::TNodeID getNearestNode(
				const NODE_TYPE &query_pt,
				PoseDistanceMetric<NODE_TYPE> &distanceMetricEvaluator,
				double *out_distance = NULL) const
			{
				MRPT_TODO("Optimize this query!")
				ASSERT_(!m_nodes.empty())

				double min_d = std::numeric_limits<double>::max();
				mrpt::utils::TNodeID min_id;
				for (typename node_map_t::const_iterator it=m_nodes.begin();it!=m_nodes.end();++it)
				{
					double d = distanceMetricEvaluator.distance(query_pt,it->second);
					if (d<min_d) {
						min_d = d;
						min_id = it->first;
					}
				}
				if (out_distance) *out_distance = min_d;
				return min_id;
			}

			void insertNode( const mrpt::utils::TNodeID id, const NODE_TYPE &node ) {
				m_nodes[id] = node;
			}


			const node_map_t & getAllNodes() const { return m_nodes; }

		private:
			node_map_t  m_nodes;  //!< Info per node

		}; // end TMoveTree

		/** An edge for the move tree used for planning in SE2 and TP-space */
		struct NAV_IMPEXP TMoveEdgeSE2_TP
		{
			const mrpt::utils::TNodeID parent_id;  //!< The ID of the parent node in the tree
			const mrpt::math::TPose2D  end_state;  //!< state in SE2 as 2D pose (x, y, phi)  - \note: it is not possible to initialize a motion without a state
			double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
			int ptg_index;          //!< indicate the type of trajectory used for this motion
			int ptg_K;              //!< identify the trajectory number K of the type ptg_index
			double ptg_dist;        //!< identify the lenght of the trajectory for this motion

			TMoveEdgeSE2_TP ( const mrpt::utils::TNodeID parent_id_, const mrpt::math::TPose2D end_pose_ ) :
				parent_id (parent_id_),
				end_state( end_pose_ ),
				cost( 0.0 ),
				ptg_index ( 0 ), ptg_K ( 0 ), ptg_dist ( 0.0 )   //these are all PTGs parameters
			{}
		};

		struct NAV_IMPEXP TNodeSE2
		{
			mrpt::math::TPose2D  state;  //!< state in SE2 as 2D pose (x, y, phi)

			TNodeSE2( const mrpt::math::TPose2D &state_) : state(state_) 
			{
			}

			TNodeSE2() {}
		};

		/** Pose metric for SE(2) */
		template<>
		struct PoseDistanceMetric<TNodeSE2>
		{
			double distance(const TNodeSE2 &a, const TNodeSE2& b) const 
			{
				return mrpt::math::square(a.state.x-b.state.x) +
				       mrpt::math::square(a.state.y-b.state.y) + 
					   mrpt::math::square( mrpt::math::angDistance(a.state.phi,b.state.phi) );
			}
		};

		typedef TMoveTree<TNodeSE2,TMoveEdgeSE2_TP> TMoveTreeSE2_TP;  //!< tree data structure for planning in SE2 with TP-space

	}
}
