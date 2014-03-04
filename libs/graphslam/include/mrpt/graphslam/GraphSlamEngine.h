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

#include <mrpt/graphslam/solvers.h>  // Implementations of GraphSlamEngine<...,GRAPHSLAM_SOLVER,...>
#include <mrpt/graphslam/deciders.h>  // Implementations of GraphSlamEngine<...,UPDATE_DECIDER,...>


#include <mrpt/slam/CSensoryFrame.h>

namespace mrpt
{
	namespace graphslam
	{
		/** \addtogroup mrpt_graphslam_grp
		  *  @{ */

		namespace internal {
			/** Additional info saved in each graph node, apart from its global pose (accesible via "global_poses_t  nodes;" in m_graph) */
			struct TNodeAnnotations {
				mrpt::slam::CSensoryFrame nodeAnnotation_observations; //!< All the sensory observations grabbed at this GraphSLAM node.				
			};
		};


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
			class GRAPHSLAM_SOLVER = mrpt::graphslam::solvers::GS_SolverLevMarq< mrpt::graphs::CNetworkOfPoses<typename GRAPH_T::constraint_t,MAPS_IMPLEMENTATION, internal::TNodeAnnotations> > // Batch solver algorithm. 
			>
		class GraphSlamEngine
		{
		public:
			typedef graphslam_traits<GRAPH_T>       gst; //!< Traits
			typedef typename GRAPH_T::constraint_t  constraint_t; //!< This will the type of edge constraints: the CPOSE template argument of CNetworkOfPoses<> in GRAPH_T
			
			/** The type of the graph (the "SLAM map") for this kind of problem depending on template arguments */
			typedef mrpt::graphs::CNetworkOfPoses<
				constraint_t, // Type of edges
				MAPS_IMPLEMENTATION, // Use std::map<> vs. std::vector<>
				internal::TNodeAnnotations>	graph_t;

			// --------------------------------------------------
			/** \name Public API for Online Graph-SLAM Engine
			  * @{ */ 

			/** Default constructor */
			GraphSlamEngine(); 

			/** Returns a const ref to the "SLAM map" (not threadsafe: use critical sections and make a copy of this as needed in your code) */
			const graph_t & getGraph() const { return m_graph; }

			/** Main entry point (alternative 1): Process a pair "action"+"sensory observations" to update the map. */
			void 	processActionObservation (const mrpt::slam::CActionCollection &action, const mrpt::slam::CSensoryFrame &in_SF);

			/** Main entry point (alternative 2): Process one odometry OR sensor observation. */
			void 	processObservation (const mrpt::slam::CObservationPtr &obs);

			/** @} */ // --------------------------------------------------


			struct TTimestampedNode
			{
				mrpt::utils::TNodeID     nodeID;
				mrpt::system::TTimeStamp timestamp; 

				inline TTimestampedNode() : nodeID(INVALID_NODEID), timestamp(INVALID_TIMESTAMP) {}
			};


		protected:
			graph_t            m_graph;   //!< The "SLAM map", the graph of poses and pose constraints
			TTimestampedNode   m_current_frame; //!< The current (latest) node at which the robot is at (invalid only at start up with an empty map).
			F2F_MATCH          m_f2f_match; 
			UPDATE_DECIDER     m_update_decider; 
			GRAPHSLAM_SOLVER   m_solver; 


			mrpt::poses::CPose3D  m_auxAccumOdometry; //!< Aux for simulating odometry in obs-only datasets
			
		}; // end class GraphSlamEngine<>

	/**  @} */  // end of grouping
	} // End of namespace
} // End of namespace

// Template method implementations:
#include <mrpt/graphslam/GraphSlamEngine_impl.h>

#endif
