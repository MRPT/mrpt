/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_ENGINE_H
#define GRAPH_SLAM_ENGINE_H

#include <mrpt/graphslam/types.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/traits_map.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/math/KDTreeCapable.h>

#include <mrpt/graphslam/solvers.h>  // Implementations of GraphSlamEngine<...,GRAPHSLAM_SOLVER,...>
#include <mrpt/graphslam/deciders.h>  // Implementations of GraphSlamEngine<...,UPDATE_DECIDER,...>


#include <mrpt/slam/CSensoryFrame.h>

namespace mrpt
{
	namespace graphslam
	{
		/** \addtogroup mrpt_graphslam_grp
		  *  @{ */

		/** A generic engine for online Graph SLAM algorithms. Its behavior and map type can be customized via template arguments.
		  *
		  * \tparam GRAPH_T         Type of graph of constraints: any of mrpt::graphs::CNetworkOfPoses<...>
		  * \tparam F2F_MATCH       Frame-to-frame matching method: for creating new edges, detecting loop closures, etc.
		  * \tparam UPDATE_DECIDER  Behavior for deciding when to insert or discard nodes in the graph, which parts of the graph to update, etc.
		  * \tparam GRAPHSLAM_SOLVER Batch solver algorithm. Default: mrpt::graphslam::solvers::GSS_LevMarq
		  * \tparam MAPS_IMPLEMENTATION Implementation of std::map<> as a real std::map<> or a wrapper with vector<> container.
		  */
		template <
			class GRAPH_T,          // Type of graph of constraints: any of mrpt::graphs::CNetworkOfPoses<...>
			class F2F_MATCH,        // Frame-to-frame matching method: for creating new edges, detecting loop closures, etc.
			class UPDATE_DECIDER = mrpt::graphslam::deciders::GS_GenericDecider,  // Behavior for deciding when to insert or discard nodes in the graph, which parts of the graph to update,, etc.
			class MAPS_IMPLEMENTATION = mrpt::utils::map_traits_stdmap, // Use std::map<> vs. std::vector<>
			class GRAPHSLAM_SOLVER = mrpt::graphslam::solvers::GS_SolverLevMarq // Batch solver algorithm. 
			>
		class GraphSlamEngine
		{
		public:
			typedef GraphSlamEngine<GRAPH_T,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER> self_t; //!< Myself type

			struct TNodeAnnotations;
			struct TEdgeAnnotations;

			/** The type of the graph (the "SLAM map") for this kind of problem depending on template arguments */
			typedef mrpt::graphs::CNetworkOfPoses<
				typename GRAPH_T::constraint_t, // Type of edges
				MAPS_IMPLEMENTATION, // Use std::map<> vs. std::vector<>
				TNodeAnnotations,
				TEdgeAnnotations>	graph_t;

			typedef graphslam_traits<graph_t>       gst; //!< Traits
			enum { rotation_dimensions = gst::edge_poses_type::rotation_dimensions };  //!< 2 for SE(2), 3 for SE(3)
			typedef typename graph_t::constraint_t         constraint_t; //!< This will the type of edge constraints: the CPOSE template argument of CNetworkOfPoses<> in GRAPH_T
			typedef typename graph_t::constraint_no_pdf_t  pose_t;   //!< A plain pose type (CPose2D or CPose3D), without uncertainty information.

			/** Additional info saved in each graph node, apart from its global pose (accesible via "global_poses_t  nodes;" in m_graph) */
			struct TNodeAnnotations 
			{
				mrpt::slam::CSensoryFrame  nodeAnnotation_observations; //!< All the sensory observations grabbed at this GraphSLAM node.
				typename graph_t::edge_t   *nodeAnnotation_odometryEdge; //!< !=NULL only if this node has an odometry edge from a previous node to this one.
				inline TNodeAnnotations() : nodeAnnotation_odometryEdge(NULL) {}
			};

			struct TEdgeAnnotations
			{
				bool is_odometry_edge; //!< True only in odometry edges. The difference is that odometry edges are the only ones updated incrementally.
				bool is_kf2kf_sensor_constraint; //!< True if the edge has been created by this GraphSLAM engine's "k2k_match" class.
				inline TEdgeAnnotations() : is_odometry_edge(false),is_kf2kf_sensor_constraint(false) { }
			};

			struct KeyFramesKDTree;

			// --------------------------------------------------
			/** \name Public API for Online Graph-SLAM Engine
			  * @{ */ 

			/** Default constructor */
			GraphSlamEngine(); 

			/** Reset the SLAM engine to an initial state (as after default constructor), excepting all parameters which are maintained. */
			void resetState(); 

			/** Main entry point (alternative 1): Process a pair "action"+"sensory observations" to update the map. */
			void processActionObservation (const mrpt::slam::CActionCollection &action, const mrpt::slam::CSensoryFrame &in_SF);

			/** Main entry point (alternative 2): Process one odometry OR sensor observation. */
			void processObservation (const mrpt::slam::CObservationPtr &obs);

			/** Returns the latest estimate of the current robot pose (the global pose for the current keyframe in the graph) */
			void getCurrentPose(pose_t &pose) const;

			/** Returns a const ref to the "SLAM map" (not threadsafe: use critical sections and make a copy of this as needed in your code) */
			const graph_t & getGraph() const { return m_graph; }

			/** Returns a KD-Tree object on which to perform queries to search keyframes closest to any location. */
			const KeyFramesKDTree & getKeyFrameKDTree() const { return m_keyframes_kdtree; }

			/** @} */ // --------------------------------------------------

			typename F2F_MATCH::TParams        m_f2f_match_params;
			typename UPDATE_DECIDER::TParams   m_update_decider_params; 
			//typename GRAPHSLAM_SOLVER::TParams m_solver_params; 

			struct TTimestampedNode
			{
				mrpt::utils::TNodeID     nodeID;
				mrpt::system::TTimeStamp timestamp; 

				inline TTimestampedNode() : nodeID(INVALID_NODEID), timestamp(INVALID_TIMESTAMP) {}
				inline TTimestampedNode(const mrpt::utils::TNodeID _nodeID, mrpt::system::TTimeStamp _timestamp) : nodeID(_nodeID), timestamp(_timestamp) {}
			};

			/** Struct: a KD-tree for querying the Euclidean coordinates (X,Y or X,Y,Z) of keyframes */
			struct KeyFramesKDTree : public mrpt::math::KDTreeCapable<typename self_t::KeyFramesKDTree,double>
			{
				// Must return the number of data points
				inline size_t kdtree_get_point_count() const { return m_graph.nodeCount(); }
				// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
				inline double kdtree_distance(const double *p1, const size_t idx_p2,size_t size) const {
					double ret=0.0;
					const typename graph_t::global_pose_t & pose = m_graph.nodes[idx_p2];
					for (size_t i=0;i<size;i++) ret+=mrpt::utils::square( p1[i] - pose[i]);
					return ret;
				}
				// Returns the dim'th component of the idx'th point in the class:
				inline double kdtree_get_pt(const size_t idx, int dim) const {
					const typename graph_t::global_pose_t & pose = m_graph.nodes[idx];
					return pose[dim];
				}
				template <class BBOX> bool kdtree_get_bbox(BBOX &bb) const {return false; }

				void markAsOutdated() { this->kdtree_mark_as_outdated(); } 
				KeyFramesKDTree(graph_t &graph) : m_graph(graph) {}
				graph_t &m_graph;
			};

		protected:
			graph_t            m_graph;   //!< The "SLAM map", the graph of poses and pose constraints
			TTimestampedNode   m_current_frame; //!< The current (latest) node at which the robot is at (invalid only at start up with an empty map).
			F2F_MATCH          m_f2f_match; 
			UPDATE_DECIDER     m_update_decider; 
			GRAPHSLAM_SOLVER   m_solver; 
			KeyFramesKDTree    m_keyframes_kdtree;  //!< a KD-tree for querying the Euclidean coordinates (X,Y or X,Y,Z) of keyframes.

			mrpt::poses::CPose3D     m_auxFakeOdo; //!< Aux for simulating odometry in obs-only datasets
			typename graph_t::constraint_no_pdf_t m_lastOdometry; //!< Last odometry reading, so we can compute odometry increments
			mrpt::system::TTimeStamp              m_lastOdometry_timestamp; 
			
		}; // end class GraphSlamEngine<>

	/**  @} */  // end of grouping
	} // End of namespace
} // End of namespace

// Template method implementations:
#include <mrpt/graphslam/GraphSlamEngine_impl.h>

#endif
