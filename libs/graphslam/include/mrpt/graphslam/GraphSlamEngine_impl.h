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

#include <mrpt/slam/CObservationOdometry.h>

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
//		typename F2F_MATCH::TParams        f2f_bckup = m_f2f_match_params;
//		typename UPDATE_DECIDER::TParams   dec_bckup = m_update_decider_params;
		//typename GRAPHSLAM_SOLVER::TParams sol_bckup = m_solver_params;
		*this = self_t(); // Reset
//		m_f2f_match_params = f2f_bckup;
//		m_update_decider_params = dec_bckup;
		//m_solver_params = sol_bckup;
	}

	// Returns the latest estimate of the current robot pose (the global pose for the current keyframe in the graph)
	template <class graph_t,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	bool GraphSlamEngine<graph_t,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::getCurrentPose(pose_t &pose) const
	{
		const TNodeID curNodeID = m_current_frame.nodeID;
		if (curNodeID==INVALID_NODEID)
			return false; // The graph is empty
		typename graph_t::global_poses_t::const_iterator itCurNode = m_graph.nodes.find(curNodeID);
		ASSERT_(itCurNode != m_graph.nodes.end())
		const typename graph_t::global_pose_t & curNode = itCurNode->second;
		pose = *static_cast<const typename graph_t::constraint_no_pdf_t *>(& curNode );
		return true;
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
		using namespace mrpt::slam;

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

		// Empty map:
		if (is_first_obs_in_empty_graph)
		{
			// Is it an odometry observation??
			if (IS_CLASS(obs,CObservationOdometry))
			{
				// Ignore odometry until the map isn't empty so we have some reference observations to compare with!
			}
			else
			{
				const TNodeID firstNodeID = 0;
				typename graph_t::global_pose_t & newNode = m_graph.nodes[firstNodeID]; // Create empty pose in the map<>
				newNode.nodeAnnotation_observations.insert(obs);  // Add obs to node
				m_current_frame = TTimestampedNode(firstNodeID, obs->timestamp); // Set as current position
				m_keyframes_kdtree.markAsOutdated();
			}
		}
		else
		{
			typename graph_t::constraint_no_pdf_t odo_increment;  // Default (NULL increment)
			if (IS_CLASS(obs,CObservationOdometry))
			{
				// Convert an odometry (global pose) reading into incremental odometry:
				const CObservationOdometryPtr odo = CObservationOdometryPtr(obs);

				if (m_lastOdometry_timestamp!=INVALID_TIMESTAMP) {
					odo_increment.inverseComposeFrom(typename graph_t::constraint_no_pdf_t(odo->odometry), m_lastOdometry);
				}
				m_lastOdometry = typename graph_t::constraint_no_pdf_t(odo->odometry);
				m_lastOdometry_timestamp = odo->timestamp;
			}

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

			// Create new odometry edge:
			if (createNewFrame)
			{
				const TPairNodeIDs edge_ids(prevNodeID,m_current_frame.nodeID);
				typename graph_t::iterator & odoEdgeIt = m_graph.edges.insert(std::pair<TPairNodeIDs,typename graph_t::edge_t>(edge_ids,typename graph_t::edge_t()));
				curNode.nodeAnnotation_odometryEdge = & odoEdgeIt->second;
				curNode.nodeAnnotation_odometryEdge->is_odometry_edge = true;
				curNode.nodeAnnotation_odometryEdge->odo_from_node = &m_graph.nodes[prevNodeID];
				curNode.nodeAnnotation_odometryEdge->odo_to_node = &m_graph.nodes[m_current_frame.nodeID];
			}
			// Pose composition:
			typename graph_t::constraint_no_pdf_t &newPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&curNode);
			newPose.composeFrom(prevPose, odo_increment);

			// Update observations for the current node (overwrite by sensor label):
			for (mrpt::slam::CSensoryFrame::iterator it=curNode.nodeAnnotation_observations.begin();it!=curNode.nodeAnnotation_observations.end();++it)
			{
				if ( (*it)->sensorLabel==obs->sensorLabel)
				{
					curNode.nodeAnnotation_observations.erase(it);
					break;
				}
			}
			curNode.nodeAnnotation_observations.insert(obs);  // Add obs to node

			// If this was a newly created KeyFrame, assigns to it an initial guess of its global pose: the same than the previous KF.
			if (createNewFrame)
			{
				typename graph_t::constraint_no_pdf_t &newPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&curNode);
				newPose = prevPose;
			}

			// (2/2): Update odometry edge constraint:
			// This is done from the CObservationOdometry entries in both the "from" and "to" ends of an odometry edge:
			if (curNode.nodeAnnotation_odometryEdge!=NULL) // If this node have an odometry edge:
			{
				typename graph_t::edge_t & odoEdge = *curNode.nodeAnnotation_odometryEdge;
				if (odoEdge.odo_from_node && odoEdge.odo_to_node)
				{
					mrpt::slam::CObservationOdometryPtr odoFrom = odoEdge.odo_from_node->nodeAnnotation_observations.template getObservationByClass<mrpt::slam::CObservationOdometry>();
					mrpt::slam::CObservationOdometryPtr odoTo = odoEdge.odo_to_node->nodeAnnotation_observations.template getObservationByClass<mrpt::slam::CObservationOdometry>();
					if (odoFrom.present() && odoTo.present())
					{
						odoEdge = odoTo->odometry - odoFrom->odometry;

//#ifdef _DEBUG
						std::cout << "[GSE] Updating odo constraint edge of node " << m_current_frame.nodeID << " to " << odoEdge << std::endl;
//#endif
					}
				}
			}
		} // end it's not an empty map


		if (!m_graph.nodes.empty())
		{
			// STAGE 2: Look for neighbors, candidate to create new edges:
			// ------------------------------------------------------------
			std::set<TNodeID> covisible_KFs;
			m_f2f_match.getCovisibleKeyframes(*this, m_current_frame.nodeID, covisible_KFs);

			// STAGE 3: Match neighboring KeyFrames and establish (or refine) edges among them:
			// ------------------------------------------------------------------------------
			bool some_edge_modified = false;
			{
				// Observations in "current":
				const CSensoryFrame &cur_sf = m_graph.nodes[m_current_frame.nodeID].nodeAnnotation_observations;
				typename graph_t::global_pose_t & curNode = m_graph.nodes[m_current_frame.nodeID];
				typename graph_t::constraint_no_pdf_t &curPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&curNode);

				for (std::set<TNodeID>::const_iterator itKF=covisible_KFs.begin();itKF!=covisible_KFs.end();++itKF)
				{
					// Observations in "other":
					const CSensoryFrame &other_sf = m_graph.nodes[*itKF].nodeAnnotation_observations;

					// Estimate the current relative pose (an initial estimate for the registration method)
					typename graph_t::global_pose_t & otherNode = m_graph.nodes[*itKF];
					typename graph_t::constraint_no_pdf_t &otherPose = *static_cast<typename graph_t::constraint_no_pdf_t *>(&otherNode);

					typename graph_t::constraint_no_pdf_t init_otherFromCur;
					init_otherFromCur.inverseComposeFrom( otherPose, curPose );

					constraint_t poseOtherFromCur;
					bool valid = m_f2f_match.matchTwoKeyframes(m_current_frame.nodeID,*itKF,cur_sf,other_sf, init_otherFromCur, poseOtherFromCur);

					// Look for edges in both directions: cur->other, other->cur
					// The idea: odometry and the user may add additinal edges from those arising from sensor KF-to-KF matching, and we must respect them.
					const TPairNodeIDs edge_ids_direct(*itKF,m_current_frame.nodeID), edge_ids_reverse(m_current_frame.nodeID,*itKF);
					std::pair<typename graph_t::edges_map_t::iterator,typename graph_t::edges_map_t::iterator> its_direct  = m_graph.edges.equal_range( edge_ids_direct );
					std::pair<typename graph_t::edges_map_t::iterator,typename graph_t::edges_map_t::iterator> its_reverse = m_graph.edges.equal_range( edge_ids_reverse );
					typename graph_t::edges_map_t::iterator it_the_edge = m_graph.edges.end();  // The kf-to-kf sensor-based constraint we are looking for, or we have created (below)
					bool the_edge_is_reverse = false;
					// Look in direct edges:
					for (typename graph_t::edges_map_t::iterator it=its_direct.first;it!=its_direct.second;++it) {
						if (it->second.is_kf2kf_sensor_constraint) {
							it_the_edge = it;
							break;
						}
					}
					// Look in reverse edges:
					for (typename graph_t::edges_map_t::iterator it=its_reverse.first;it!=its_reverse.second && it_the_edge!=m_graph.edges.end();++it)
					{
						if (it->second.is_kf2kf_sensor_constraint) {
							it_the_edge = it;
							the_edge_is_reverse = true;
							break;
						}
					}

					if (valid)
					{
						// Overwrite or create the edge cur -> other (apart from odometry edges!)
						// If not found yet: create new fresh edge!
						if (it_the_edge==m_graph.edges.end())
						{
							it_the_edge = m_graph.edges.insert(std::pair<TPairNodeIDs,typename graph_t::edge_t>(edge_ids_direct,typename graph_t::edge_t()));
							it_the_edge->second.is_kf2kf_sensor_constraint = true;
							the_edge_is_reverse=false;
						}
						// Store the constraint in the edge:
						constraint_t & newEdgeRelativePose = *static_cast<constraint_t*>(&it_the_edge->second);
						newEdgeRelativePose = the_edge_is_reverse ? (poseOtherFromCur) : (-poseOtherFromCur);
#ifdef _DEBUG
						std::cout << "[GSE] Updating obs constraint edge " << edge_ids_direct.first << "=>" << edge_ids_direct.second << " to " << newEdgeRelativePose << " (Reverse:" << (the_edge_is_reverse ? "YES":"NO") <<  std::endl;
#endif
						some_edge_modified=true;

					} // end valid "match" found.
					else
					{
						// We had an edge OLD_NODE ==> CURRENT_NODE but now the registration seems invalid:
						// The edge must be deleted, since it no longer reflects accurate localization data
						// for the current (MOVING) keyframe:
						if (it_the_edge!=m_graph.edges.end())
						{
							m_graph.edges.erase( it_the_edge );
							std::cout << "[GSE] Removing constraint edge " << edge_ids_direct.first << "=>" << edge_ids_direct.second << " due to non match." <<  std::endl;
						}
					}
				} // end for each covisible KF
			}

			// STAGE 4: Optimize the global pose of the current keyframe only:
			// ------------------------------------------------------------------------------
			if (some_edge_modified)
			{
				m_solver.optimizeSingle(m_graph, m_current_frame.nodeID);
				// NOTE: Don't need to call "m_keyframes_kdtree.markAsOutdated();" if we only modify the last (current) KeyFrame.

				MRPT_TODO("Add proper param!")
				static int cnt = 0;
				if (++cnt>10)
				{
					cnt=0;
					m_solver.optimizeFull(m_graph);
				}

			}

		} // end if map not empty

		/*{
			static int i=0;
			m_graph.saveToTextFile(mrpt::format("graph_%06i.txt",i++));
		}*/


	} // end of processObservation()



} } // End of namespaces

#endif
