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
		*
		* <b>Changes history</b>
		*      - 06/MAR/2014: Creation (MB)
		*      - 21/JAN/2015: Refactoring (JLBC)
		*
		*  \ingroup mrpt_nav_grp
		*/
		template<
			class NODE_TYPE_DATA, 
			class EDGE_TYPE,
			class MAPS_IMPLEMENTATION = mrpt::utils::map_traits_map_as_vector // Use std::map<> vs. std::vector<>
		>
		class TMoveTree : public mrpt::graphs::CDirectedTree<EDGE_TYPE>
		{
		public:
			struct NODE_TYPE : public NODE_TYPE_DATA
			{
				mrpt::utils::TNodeID parent_id; //!< INVALID_NODEID for the root, a valid ID otherwise
				mrpt::utils::TNodeID node_id;   //!< Duplicated ID (it's also in the map::iterator->first), but put here to make it available in path_t
				EDGE_TYPE * edge_to_parent; //!< NULL for root, a valid edge otherwise
				NODE_TYPE(mrpt::utils::TNodeID node_id_, mrpt::utils::TNodeID parent_id_, EDGE_TYPE * edge_to_parent_, const NODE_TYPE_DATA &data) : 
					node_id(node_id_),parent_id(parent_id_),
					edge_to_parent(edge_to_parent_),
					NODE_TYPE_DATA(data) 
				{
				}
				NODE_TYPE() {}
			};

			typedef public mrpt::graphs::CDirectedTree<EDGE_TYPE> base_t;
			typedef typename MAPS_IMPLEMENTATION::template map<mrpt::utils::TNodeID,NODE_TYPE>  node_map_t;  //!< Map: TNode_ID => Node info
			typedef std::list<NODE_TYPE> path_t; //!< A topological path up-tree

			/** Finds the nearest node to a given pose, using the given metric */
			mrpt::utils::TNodeID getNearestNode(
				const NODE_TYPE_DATA &query_pt,
				const PoseDistanceMetric<NODE_TYPE_DATA> &distanceMetricEvaluator,
				double *out_distance = NULL) const
			{
				MRPT_TODO("Optimize this query with KD-tree!")
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

			void insertNodeAndEdge(
				const mrpt::utils::TNodeID parent_id, 
				const mrpt::utils::TNodeID new_child_id, 
				const NODE_TYPE_DATA &new_child_node_data, 
				const EDGE_TYPE &new_edge_data ) 
			{
				// edge:
				typename base_t::TListEdges & edges_of_parent = base_t::edges_to_children[parent_id];
				edges_of_parent.push_back( typename base_t::TEdgeInfo(new_child_id,false/*direction_child_to_parent*/, new_edge_data ) );
				// node:
				m_nodes[new_child_id] = NODE_TYPE(new_child_id,parent_id, &edges_of_parent.back().data, new_child_node_data);
			}

			/** Insert a node without edges (should be used only for a tree root node) */
			void insertNode(const mrpt::utils::TNodeID node_id, const NODE_TYPE_DATA &node_data) 
			{
				m_nodes[node_id] = NODE_TYPE(node_id,INVALID_NODEID, NULL, node_data);
			}

			mrpt::utils::TNodeID getNextFreeNodeID() const { return m_nodes.size(); }

			const node_map_t & getAllNodes() const { return m_nodes; }

			/** Builds the path (sequence of nodes, with info about next edge) up-tree from a `target_node` towards the root 
			  * Nodes are ordered in the direction ROOT -> start_node
			  */
			void backtrackPath(
				const mrpt::utils::TNodeID target_node, 
				path_t &out_path
				) const 
			{
				out_path.clear();
				typename node_map_t::const_iterator it_src = m_nodes.find(target_node);
				if (it_src == m_nodes.end()) throw std::runtime_error("backtrackPath: target_node not found in tree!");
				const NODE_TYPE * node = &it_src->second;
				for (;;)
				{
					out_path.push_front(*node);

					mrpt::utils::TNodeID next_node_id = node->parent_id;
					if (next_node_id==INVALID_NODEID) {
						break; // finished
					}
					else {
						typename node_map_t::const_iterator it_next = m_nodes.find(next_node_id);
						if (it_next == m_nodes.end()) throw std::runtime_error("backtrackPath: Node ID not found during tree traversal!");
						node = &it_next->second;
					}
				}
				
			}

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
