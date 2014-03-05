/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_ENGINE_IMPL_H
#define GRAPH_SLAM_ENGINE_IMPL_H
// Only to be included from within <mrpt/graphslam/GraphSlamEngine.h>

namespace mrpt { namespace graphslam {

	// Default ctor:
	template <class graph_t,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	GraphSlamEngine<graph_t,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::GraphSlamEngine() :
		m_keyframes_kdtree(m_graph),
		m_lastOdometry_timestamp(INVALID_TIMESTAMP)
	{
	}

	// Reset the SLAM engine to an initial state (as after default constructor)
	template <class graph_t,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	void GraphSlamEngine<graph_t,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::resetState()
	{
		MRPT_TODO("Uncomment when all TParams...")
		typename F2F_MATCH::TParams        f2f_bckup = m_f2f_match_params;
		typename UPDATE_DECIDER::TParams   dec_bckup = m_update_decider_params; 
		//typename GRAPHSLAM_SOLVER::TParams sol_bckup = m_solver_params; 
		*this = self_t(); // Reset
		m_f2f_match_params = f2f_bckup;
		m_update_decider_params = dec_bckup; 
		//m_solver_params = sol_bckup; 
	}

	// Returns the latest estimate of the current robot pose (the global pose for the current keyframe in the graph)
	template <class graph_t,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	void GraphSlamEngine<graph_t,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::getCurrentPose(pose_t &pose) const
	{
		const TNodeID curNodeID = m_current_frame.nodeID;
		ASSERTMSG_(curNodeID!=INVALID_NODEID,"Error: the graph is empty!")
		typename graph_t::global_poses_t::const_iterator itCurNode = m_graph.nodes.find(curNodeID);
		ASSERT_(itCurNode != m_graph.nodes.end())
		typename const graph_t::global_pose_t & curNode = itCurNode->second;
		pose = *static_cast<const typename graph_t::constraint_no_pdf_t *>(& curNode );
	}

	// Main entry point (alternative 1): Process a pair "action"+"sensory observations" to update the map.
	template <class graph_t,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	void GraphSlamEngine<graph_t,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::processActionObservation (
		const mrpt::slam::CActionCollection &action, 
		const mrpt::slam::CSensoryFrame &in_SF)
	{
		using namespace mrpt::slam; 
		using namespace mrpt::poses; 

		// 1) process action:
		CActionRobotMovement2DPtr movEstimation = action.getBestMovementEstimation();
		if (movEstimation)
		{
			m_auxFakeOdo.composeFrom( m_auxFakeOdo, CPose3D(movEstimation->poseChange->getMeanVal()) );

			CObservationOdometryPtr obs = CObservationOdometry::Create();
			obs->timestamp = movEstimation->timestamp;
			obs->odometry = CPose2D(m_auxFakeOdo);
			this->processObservation(obs);
		}
		// 2) Process observations one by one:
		for (CSensoryFrame::const_iterator i=in_SF.begin();i!=in_SF.end();++i)
			this->processObservation(*i);
	}

	/** Main entry point (alternative 2): Process one odometry OR sensor observation. */
	template <class graph_t,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	void GraphSlamEngine<graph_t,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::processObservation(
		const mrpt::slam::CObservationPtr &obs)
	{
		using namespace mrpt::utils;
		using namespace mrpt::poses;

		ASSERT_(obs.present())
		ASSERT_(obs->timestamp!=INVALID_TIMESTAMP)

		// STAGE 1: Process observation, update sensor data in current KeyFrame or 
		//           start a new Keyframe, as needed.
		// -----------------------------------------------------------------------------------
		bool is_first_obs_in_empty_graph = false; // Is the first obs in a totally empty graph?
		if (m_current_frame.nodeID==INVALID_NODEID)
		{
			ASSERTMSG_(m_graph.nodes.empty(), "I expected an empty graph in an uninitialized GraphSLAM engine!")
			is_first_obs_in_empty_graph=true;
		}

		// Is it an odometry observation??
		if (IS_CLASS(obs,CObservationOdometry))
		{
			if (!is_first_obs_in_empty_graph) // Ignore odometry until the map isn't empty so we have some reference observations to compare with!
			{
				// Convert an odometry (global pose) reading into incremental odometry:
				const CObservationOdometryPtr odo = CObservationOdometryPtr(obs);
				typename graph_t::constraint_no_pdf_t odo_increment;  // Default (NULL increment) 

				if (m_lastOdometry_timestamp!=INVALID_TIMESTAMP) {
					odo_increment.inverseComposeFrom(typename graph_t::constraint_no_pdf_t(odo->odometry), m_lastOdometry);
				}
				m_lastOdometry = typename graph_t::constraint_no_pdf_t(odo->odometry);
				m_lastOdometry_timestamp = odo->timestamp;

				// Are we far enough from last node as for creating a new node now?
				bool createNewFrame = m_update_decider.shouldCreateNewFrame(*this,obs);

				// (1/2): Update current estimate of keyframe global pose:
				// Pose composition: new pose = cur_pose (+) increment
				const TNodeID prevNodeID = m_current_frame.nodeID;
				typename graph_t::global_pose_t & prevNode = m_graph.nodes[prevNodeID];
				typename graph_t::constraint_no_pdf_t &prevPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&prevNode);
				
				// Move to a new keyframe?
				if (createNewFrame) {
					// Create new frame ID and set as current:
					const TNodeID newNodeID = m_graph.nodes.size();
					m_current_frame = TTimestampedNode(newNodeID, obs->timestamp);
					m_keyframes_kdtree.markAsOutdated();
				}

				typename graph_t::global_pose_t & curNode = m_graph.nodes[m_current_frame.nodeID];
				
				// Create new odometry edge?
				if (createNewFrame)
				{
					const TPairNodeIDs edge_ids(prevNodeID,m_current_frame.nodeID);
					typename graph_t::iterator & odoEdgeIt = m_graph.edges.insert(std::pair<TPairNodeIDs,typename graph_t::edge_t>(edge_ids,typename graph_t::edge_t()));
					curNode.nodeAnnotation_odometryEdge = & odoEdgeIt->second;
					curNode.nodeAnnotation_odometryEdge->is_odometry_edge = true;
				}
				// Pose composition:
				typename graph_t::constraint_no_pdf_t &newPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&curNode);
				newPose.composeFrom(prevPose, odo_increment);

				// (2/2): Update odometry edge constraint:
				if (curNode.nodeAnnotation_odometryEdge!=NULL) // If this node have an odometry edge:
				{
					typename graph_t::edge_t & odoEdge = *curNode.nodeAnnotation_odometryEdge;
					odoEdge+= odo_increment; // Pose composition
				}
			} // end it's not an empty map
		} // end it's odometry
		else
		{
			MRPT_TODO("CHECK OBS TYPES NOT TO IGNORE!")

			if (is_first_obs_in_empty_graph)
			{
				const TNodeID firstNodeID = 0;
				typename graph_t::global_pose_t & newNode = m_graph.nodes[firstNodeID]; // Create empty pose in the map<>
				newNode.nodeAnnotation_observations.insert(obs);  // Add obs to node
				m_current_frame = TTimestampedNode(firstNodeID, obs->timestamp); // Set as current position 
				m_keyframes_kdtree.markAsOutdated();
			}
			else
			{
				// Before updating node observations, are we far enough from last node as for creating a new node now?
				bool createNewFrame = m_update_decider.shouldCreateNewFrame(*this,obs);

				const TNodeID prevNodeID = m_current_frame.nodeID;
				typename graph_t::global_pose_t & prevNode = m_graph.nodes[prevNodeID];
				typename graph_t::constraint_no_pdf_t &prevPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&prevNode);

				if (createNewFrame)
				{
					// Create new frame ID and set as current:
					const TNodeID newNodeID = m_graph.nodes.size();
					m_current_frame = TTimestampedNode(newNodeID, obs->timestamp);
				}

				// Update observations for the current node: 
				typename graph_t::global_pose_t & curNode = m_graph.nodes[m_current_frame.nodeID]; // Create or get current frame in the map<>
				curNode.nodeAnnotation_observations.clear();
				curNode.nodeAnnotation_observations.insert(obs);  // Add obs to node

				// If this was a newly created KeyFrame, assigns to it an initial guess of its global pose: the same than the previous KF.
				if (createNewFrame)
				{
					typename graph_t::constraint_no_pdf_t &newPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&curNode);
					newPose = prevPose;
				}

			} // end normal case (not the very first observation)
		} // end it's a sensor observation

		if (!m_graph.nodes.empty())
		{
			// STAGE 2: Look for neighbors, candidate to create new edges:
			// ------------------------------------------------------------
			std::set<mrpt::utils::TNodeID> covisible_KFs;
			m_f2f_match.getCovisibleKeyframes(*this, m_current_frame.nodeID, covisible_KFs);
	
			// STAGE 3: Match neighboring KeyFrames and establish (or refine) edges among them:
			// ------------------------------------------------------------------------------
			// Skip odo edges!
			// Skip m_current_frame.nodeID

			// STAGE 4: Optimize the global pose of the current keyframe only:
			// ------------------------------------------------------------------------------

		} // end if map not empty

	} // end of processObservation()



} } // End of namespaces

#endif
