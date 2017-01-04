/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/stl_containers_utils.h>
#include <mrpt/random.h>
#include <mrpt/math/distributions.h>
#include <mrpt/hmtslam/CRobotPosesGraph.h>

#include <mrpt/utils/CFileOutputStream.h>

#include <mrpt/system/os.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

#include <limits>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

/*---------------------------------------------------------------

						CHMTSLAM_LSLAM

	Local SLAM process within HMT-SLAM

  ---------------------------------------------------------------*/
void CHMTSLAM::thread_LSLAM()
{
	CHMTSLAM	*obj = this;
	CTicTac							tictac;
	unsigned int					nIter = 0;			// For logging purposes only

	// Seems that must be called in each thread??
	if (obj->m_options.random_seed)
			randomGenerator.randomize( obj->m_options.random_seed );
	else	randomGenerator.randomize();

	try
	{
		// Start thread:
		// -------------------------
		obj->logFmt(mrpt::utils::LVL_DEBUG,"[thread_LSLAM] Thread started (ID=0x%08lX)\n", mrpt::system::getCurrentThreadId() );

		// --------------------------------------------
		//    The main loop
		//  Executes until termination is signaled
		// --------------------------------------------
		while ( !obj->m_terminateThreads )
		{
			if (obj->m_options.random_seed)
				randomGenerator.randomize(obj->m_options.random_seed);

			// Process pending message?
			{
				CMessage *recMsg;
				do
				{
					recMsg = obj->m_LSLAM_queue.get();
					if (recMsg)
					{
						obj->LSLAM_process_message( *recMsg );
						delete recMsg;
					}
				} while (recMsg);
			}

			// There are pending elements?
			if (!obj->isInputQueueEmpty() )
			{
				if (obj->m_options.random_seed)
					randomGenerator.randomize(obj->m_options.random_seed);

				// Get the next object from the queue:
				CSerializablePtr nextObject = obj->getNextObjectFromInputQueue();
				ASSERT_(nextObject);

				// Clasify the new object:
				CActionCollectionPtr	actions;
				CSensoryFramePtr		observations;

				if (nextObject->GetRuntimeClass() == CLASS_ID(CActionCollection) )
						actions = CActionCollectionPtr( nextObject );
				else
				if (nextObject->GetRuntimeClass() == CLASS_ID(CSensoryFrame) )
						observations = CSensoryFramePtr( nextObject );
				else	THROW_EXCEPTION("Element in the queue is neither CActionCollection nor CSensoryFrame!!");


				// Process them, for each LMH:
				// -----------------------------------------
				ASSERT_(!obj->m_LMHs.empty());

				aligned_containers<THypothesisID, CLocalMetricHypothesis>::map_t::iterator  it;

				ASSERT_(obj->m_LSLAM_method);

				{
					CCriticalSectionLocker LMHs_cs_locker( & obj->m_LMHs_cs );

					for (it=obj->m_LMHs.begin();it!=obj->m_LMHs.end();it++)
					{
						CCriticalSectionLocker  LMH_individual_locker( & it->second.m_lock );

						// ----------------------------------------------
						// 1) Process acts & obs by Local SLAM method:
						// ----------------------------------------------
						obj->m_LSLAM_method->processOneLMH(
							&it->second,		// The LMH
							actions,
							observations
							);

						// ----------------------------------------------
						// 2) Invoke Area Abstraction (AA) method
						// ----------------------------------------------
						if (it->second.m_posesPendingAddPartitioner.size()>5) // Option: Do this only one out of N new added poses:
						{
							static CTicTac	tictac;
							tictac.Tic();

							unsigned nPosesToInsert = it->second.m_posesPendingAddPartitioner.size();
							TMessageLSLAMfromAAPtr msgFromAA = CHMTSLAM::areaAbstraction( &it->second, it->second.m_posesPendingAddPartitioner );

							obj->logFmt(mrpt::utils::LVL_DEBUG,"[AreaAbstraction] Took %.03fms to insert %u new poses.               AA\n", 1000*tictac.Tac(), nPosesToInsert );

							// Empty the list, it's done for now:
							it->second.m_posesPendingAddPartitioner.clear();


							if (obj->m_options.random_seed)
								randomGenerator.randomize(obj->m_options.random_seed);
						// -----------------------------------------------------------------------
						// 3) Process the new grouping, which is a quite complex process:
						//     Create new areas, joining, adding/deleting arcs and nodes, etc...
						// -----------------------------------------------------------------------
							obj->LSLAM_process_message_from_AA( *msgFromAA);
						}

						// ----------------------------------------------
						// 4) Invoke TBI method
						// ----------------------------------------------
						if (!it->second.m_areasPendingTBI.empty())
						{
							for (TNodeIDList::const_iterator areaID=it->second.m_areasPendingTBI.begin();areaID!=it->second.m_areasPendingTBI.end();++areaID)
							{

								static CTicTac	tictac;
								tictac.Tic();
								if (obj->m_options.random_seed)
									randomGenerator.randomize(obj->m_options.random_seed);

								TMessageLSLAMfromTBIPtr msgFromTBI = CHMTSLAM::TBI_main_method(
									&it->second,
									*areaID );

								obj->logFmt(mrpt::utils::LVL_DEBUG,"[TBI] Took %.03fms	                     TBI\n", 1000*tictac.Tac() );

								// -----------------------------------------------------------------------
								//   Process the set of (potentially) several topological hypotheses:
								// -----------------------------------------------------------------------
								obj->LSLAM_process_message_from_TBI( *msgFromTBI);

							} // for each pending area.

							it->second.m_areasPendingTBI.clear();		// Done here

						} // end of areas pending TBI

					} // end for each LMH

				} // end of LMHs_cs_locker


				// Free the object.
				nextObject.clear_unique();


				// -----------------------------------------------------------
				//					SLAM: Save log files
				// -----------------------------------------------------------
				if (obj->m_options.LOG_OUTPUT_DIR.size() && (nIter % obj->m_options.LOG_FREQUENCY)==0)
					obj->generateLogFiles(nIter);

				nIter++;

			} // End if queue isn't empty
			else
			{
				// Wait for new data:
				mrpt::system::sleep(5);
			}
		};	// end while execute thread

		// Finish thread:
		// -------------------------
		time_t timCreat,timExit; double timCPU=0;
		try { mrpt::system::getCurrentThreadTimes( timCreat,timExit,timCPU); } catch(...) {};
		obj->logFmt(mrpt::utils::LVL_DEBUG,"[thread_LSLAM] Thread finished. CPU time used:%.06f secs \n",timCPU);
		obj->m_terminationFlag_LSLAM = true;

	}
	catch(std::exception &e)
	{
		obj->m_terminationFlag_LSLAM = true;

		// Release semaphores:

		if (e.what())  obj->logFmt(mrpt::utils::LVL_DEBUG,"%s", e.what() );

		// DEBUG: Terminate application:
		obj->m_terminateThreads	= true;


	}
	catch(...)
	{
		obj->m_terminationFlag_LSLAM = true;

		obj->logFmt(mrpt::utils::LVL_DEBUG,"\n---------------------- EXCEPTION CAUGHT! ---------------------\n");
		obj->logFmt(mrpt::utils::LVL_DEBUG," In CHierarchicalMappingFramework::thread_LSLAM. Unexpected runtime error!!\n");

		// Release semaphores:

		// DEBUG: Terminate application:
		obj->m_terminateThreads	= true;
	}
}

/*---------------------------------------------------------------
						LSLAM_process_message
  ---------------------------------------------------------------*/
void CHMTSLAM::LSLAM_process_message( const CMessage &msg )
{
	MRPT_UNUSED_PARAM(msg);
	MRPT_START

/*	switch(msg.type)
	{
*/
	/* =============================
	          MSG FROM AA
	   ============================= */
/*	case MSG_SOURCE_AA:
	{
		CHMTSLAM::TMessageLSLAMfromAA  *MSG = reinterpret_cast<CHMTSLAM::TMessageLSLAMfromAA*> ( msg.getContentAsPointer() );
		LSLAM_process_message_from_AA( *MSG );
		delete MSG;  // Free memory

	} break; // end msg from AA
	default: THROW_EXCEPTION("Invalid msg type");
	}
*/

	MRPT_END
}


/*---------------------------------------------------------------
					LSLAM_process_message_from_AA
  ---------------------------------------------------------------*/
void CHMTSLAM::LSLAM_process_message_from_AA( const TMessageLSLAMfromAA &myMsg )
{
		MRPT_START

		CTicTac    tictac;
		tictac.Tic();
		logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Beginning of Msg from AA processing...              [\n" );

		// Get the corresponding LMH:
		aligned_containers<THypothesisID, CLocalMetricHypothesis>::map_t::iterator  itLMH = m_LMHs.find( myMsg.hypothesisID );
		ASSERT_( itLMH!=m_LMHs.end() );
		CLocalMetricHypothesis	*LMH = &itLMH->second;

		// Sanity checks:
		{
			// All poses in the AA's partitions must exist in the current LMH
			for (vector< TPoseIDList >::const_iterator	it= myMsg.partitions.begin(); it!=myMsg.partitions.end();++it)
				for (TPoseIDList::const_iterator itPose = it->begin();itPose!=it->end();++itPose)
					if (LMH->m_SFs.find( *itPose ) == LMH->m_SFs.end() )
						THROW_EXCEPTION_CUSTOM_MSG1("PoseID %i in AA's partition but not in LMH.\n", (int)*itPose );


			// All poses in the LMH must be in the AA's partitions:
			for (map<TPoseID,CHMHMapNode::TNodeID>::const_iterator itA=LMH->m_nodeIDmemberships.begin();itA!=LMH->m_nodeIDmemberships.end();++itA)
			{
				if ( LMH->m_currentRobotPose != itA->first ) // The current pose is not included in the AA method
				{
					bool found = false;
					for (vector< TPoseIDList >::const_iterator	it= myMsg.partitions.begin(); !found && it!=myMsg.partitions.end();++it)
						for (TPoseIDList::const_iterator itPose = it->begin();itPose!=it->end();++itPose)
							if ( itA->first == *itPose )
							{
								found=true;
								break;
							}
					if (!found)	THROW_EXCEPTION_CUSTOM_MSG1("LMH's pose %i not found in AA's partitions.", (int)itA->first );
				}
			}
		}

		// Neighbors BEFORE:
		TNodeIDSet  neighbors_before( LMH->m_neighbors );

		// Get current coords origin:
		TPoseID poseID_origin;
		{
			CCriticalSectionLocker  locker( &m_map_cs );

			map<TPoseID,CHMHMapNode::TNodeID>::const_iterator itCur = LMH->m_nodeIDmemberships.find( LMH->m_currentRobotPose );
			ASSERT_(itCur != LMH->m_nodeIDmemberships.end() );

			if (!m_map.getNodeByID( itCur->second)->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID,poseID_origin, LMH->m_ID ) )
				THROW_EXCEPTION("Current area reference pose not found");
		}

		// ---------------------------------------------------------------------
		// Process the partitioning:
		//   The goal is to obtain a mapping "int --> CHMHMapNode::TNodeID" from
		//    indexes in "myMsg.partitions" to topological areas.
		// To do this, we establish a voting scheme: each robot pose votes for
		//  its current area ID in the particle data, then the maximum is kept.
		// ---------------------------------------------------------------------
		// map<TPoseID,CHMHMapNode::TNodeID>: LMH->m_nodeIDmemberships
		map< unsigned int, map<CHMHMapNode::TNodeID, unsigned int> >		votes;
		unsigned int									i;

		static int DEBUG_STEP = 0;
		DEBUG_STEP++;
		logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] DEBUG_STEP=%i\n",DEBUG_STEP);
		if (DEBUG_STEP==3)
		{
			CMatrix	A(3,3);
			DEBUG_STEP = DEBUG_STEP + 0;
		}
		if (0)
		{
			CCriticalSectionLocker  locker( &m_map_cs );
			utils::CStringList s;
			m_map.dumpAsText(s);
			s.saveToFile( format("%s/HMAP_txt/HMAP_%05i_before.txt",m_options.LOG_OUTPUT_DIR.c_str(), DEBUG_STEP ) );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Saved HMAP_%05i_before.txt\n",DEBUG_STEP);
		}

		vector< TPoseIDList >::const_iterator	it;
		for ( i=0, it= myMsg.partitions.begin(); it!=myMsg.partitions.end();++it, i++)
			for (TPoseIDList::const_iterator itPose = it->begin();itPose!=it->end();++itPose)
			{
				map<TPoseID,CHMHMapNode::TNodeID>::const_iterator itP = LMH->m_nodeIDmemberships.find( *itPose );
				ASSERT_(itP != LMH->m_nodeIDmemberships.end() );

				votes[i][ itP->second ]++;
			}

		// The goal: a mapping from partition index -> area IDs:
		vector<CHMHMapNode::TNodeID>  partIdx2Areas( myMsg.partitions.size(), AREAID_INVALID );

		map<CHMHMapNode::TNodeID, pair<size_t, unsigned int> >  mostVotedFrom;  // ID -> (index, votes)
		ASSERT_(votes.size() == myMsg.partitions.size());

		// 1) For partitions voting for just one area, assign them that area if they are the most voted partitions:
		for (size_t k=0;k<myMsg.partitions.size();k++)
		{
			if (votes[k].size()==1)
			{ // map< unsigned int, map<CHMHMapNode::TNodeID, unsigned int> >		votes;			recall!
				if (votes[k].begin()->second>mostVotedFrom[ votes[k].begin()->first ].second)
				{
					mostVotedFrom[ votes[k].begin()->first ].first = k;
					mostVotedFrom[ votes[k].begin()->first ].second = votes[k].begin()->second;
				}
			}
		}

		// To the winners, assign very high votes so the rest of votes do not interfere in what has been
		//  already decided above:
		for (map<CHMHMapNode::TNodeID, pair<size_t, unsigned int> >::iterator v =mostVotedFrom.begin();v!=mostVotedFrom.end();++v)
			v->second.second = std::numeric_limits<unsigned int>::max();

		// 2) Assign each area ID to the partition that votes it most:

		for (size_t k=0;k<myMsg.partitions.size();k++)
		{
			for (map<CHMHMapNode::TNodeID, unsigned int>::iterator it=votes[k].begin();it!=votes[k].end();++it)
			{
				// Recall:
				// "votes"         is  index -> ( map: ID -> #votes )
				// "mostVotedFrom" is  ID -> (index, votes)
				map<CHMHMapNode::TNodeID, pair<size_t, unsigned int> >::iterator mostVotesIt;
				mostVotesIt = mostVotedFrom.find( it->first );
				if (mostVotesIt == mostVotedFrom.end())
				{
					// First time: add
					mostVotedFrom[it->first].first  = k;
					mostVotedFrom[it->first].second = it->second;
				}
				else
				{
					// compare:
					if ( it->second > mostVotesIt->second.second )
					{
						mostVotesIt->second.first  = k;
						mostVotesIt->second.second = it->second;
					}
				}
			}
		}

		// Fill out "partIdx2Areas" from "mostVotedFrom":
		for (map<CHMHMapNode::TNodeID, pair<size_t, unsigned int> >::iterator it=mostVotedFrom.begin();it!=mostVotedFrom.end();++it)
			partIdx2Areas[ it->second.first ] = it->first;


		// Create new area IDs for new areas (ie, partIdx2Areas[] still unassigned):
		for (i=0;i<partIdx2Areas.size();i++)
		{
			if (partIdx2Areas[i]== AREAID_INVALID)
			{
				// Create new area in the H-MAP:
				CCriticalSectionLocker  locker( &m_map_cs );

				CHMHMapNodePtr newArea = CHMHMapNode::Create( &m_map );

				// For now, the area exists in this hypothesis only:
				newArea->m_hypotheses.insert( LMH->m_ID );
				newArea->m_nodeType.setType( "Area" );
				newArea->m_label = generateUniqueAreaLabel();

				CMultiMetricMapPtr emptyMap = CMultiMetricMapPtr( new CMultiMetricMap(&m_options.defaultMapsInitializers) );
				newArea->m_annotations.setMemoryReference( NODE_ANNOTATION_METRIC_MAPS,     emptyMap, 		LMH->m_ID );

				CRobotPosesGraphPtr emptyPoseGraph = CRobotPosesGraph::Create();
				newArea->m_annotations.setMemoryReference( NODE_ANNOTATION_POSES_GRAPH, emptyPoseGraph, LMH->m_ID );

				// Set ID in list:
				partIdx2Areas[i] = newArea->getID();
			}
		} // end for i


		{
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] partIdx2Areas:\n");
			for (size_t i=0;i<partIdx2Areas.size();i++)
				logFmt(mrpt::utils::LVL_INFO,"       Partition #%i -> AREA_ID  %i ('%s')\n",(int)i,(int)partIdx2Areas[i], m_map.getNodeByID(partIdx2Areas[i])->m_label.c_str() );
		}


		// --------------------------------------------------------
		// Set the new area memberships into the LMH, and rebuild
		//   the list of neighbors:
		// --------------------------------------------------------
		LMH->m_neighbors.clear();
		for (i=0;i<partIdx2Areas.size();i++)
		{
			CHMHMapNode::TNodeID  nodeId = partIdx2Areas[i];

			// Add only if unique:
			LMH->m_neighbors.insert(nodeId);
			//if (LMH->m_neighbors.find(nodeId)==LMH->m_neighbors.end()) LMH->m_neighbors.push_back(nodeId);

			for (TPoseIDList::const_iterator it=myMsg.partitions[i].begin();it!=myMsg.partitions[i].end();++it)
				LMH->m_nodeIDmemberships[ *it ] = nodeId;  // Bind robot poses -> area IDs.
		} // end for i


		// ------------------------------------------------------------------------
		// The current robot pose is set as the membership of the closest pose:
		// ------------------------------------------------------------------------
		TMapPoseID2Pose3D  lstPoses;
		LMH->getMeans( lstPoses );
		TPoseID  closestPose = POSEID_INVALID;
		double   minDist=0;
		const CPose3D *curPoseMean = & lstPoses[ LMH->m_currentRobotPose ];

		for ( TMapPoseID2Pose3D::const_iterator it=lstPoses.begin();it!=lstPoses.end();++it )
		{
			if ( it->first != LMH->m_currentRobotPose ) // Only compare to OTHER poses!
			{
				double dist = curPoseMean->distanceEuclidean6D( it->second );
				if (closestPose==POSEID_INVALID || dist<minDist)
				{
					closestPose = it->first;
					minDist = dist;
				}
			}
		}
		ASSERT_( closestPose != POSEID_INVALID );


		// Save old one:
		const CHMHMapNode::TNodeID oldAreaID = LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ];

		// set it:
		LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ] = LMH->m_nodeIDmemberships[closestPose];

		// Save old one:
		const CHMHMapNode::TNodeID curAreaID = LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ];

		if (curAreaID!=oldAreaID)
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Current area has changed: %i -> %i\n",(int)oldAreaID,(int)curAreaID );


		// --------------------------------------------------------
		// Check for areas that have disapeared
		// --------------------------------------------------------
		for (TNodeIDSet::const_iterator pBef=neighbors_before.begin();pBef!=neighbors_before.end();++pBef)
		{
			if ( LMH->m_neighbors.find(*pBef) == LMH->m_neighbors.end() )
			{

#if 1
			{
				logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Old  neighbors: ");
				for (TNodeIDSet::const_iterator it=neighbors_before.begin();it!=neighbors_before.end();++it)
					logFmt(mrpt::utils::LVL_INFO,"%i ",(int)*it);
				logFmt(mrpt::utils::LVL_INFO,"\n");
			}
			{
				logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Cur. neighbors: ");
				for (TNodeIDSet::const_iterator it=LMH->m_neighbors.begin();it!=LMH->m_neighbors.end();++it)
					logFmt(mrpt::utils::LVL_INFO,"%i ",(int)*it);
				logFmt(mrpt::utils::LVL_INFO,"\n");
			}
#endif

				CCriticalSectionLocker  locker( &m_map_cs );

				// A node has dissappeared:
				// Delete the node from the HMT map:
				CHMHMapNodePtr node = m_map.getNodeByID( *pBef );

				if (!node)
				{
					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Area %i has been removed from the neighbors & no longer exists in the HMAP.\n", (int)*pBef);
				}
				else
				{
					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Deleting area %i\n", (int)node->getID() );

					// ----------------------------------------------------------------------------
					//  Check if arcs to nodes out of the LMH must be modified coz this deletion
					// ----------------------------------------------------------------------------
					TArcList  arcs;
					node->getArcs( arcs );

					// 1) First, make a list of nodes WITHIN the LMH with arcs to "a":
					typedef map<CHMHMapNodePtr,CHMHMapArcPtr> TListNodesArcs;
					TListNodesArcs	lstWithinLMH;

					for (TArcList::const_iterator a=arcs.begin();a!=arcs.end();++a)
					{
						CHMHMapNodePtr nodeB;

						if ( (*a)->getNodeFrom() == *pBef )
						{ // node to delete is: "from"
							nodeB = m_map.getNodeByID( (*a)->getNodeTo() );
						}
						else
						{// node to delete is: "to"
							nodeB = m_map.getNodeByID( (*a)->getNodeFrom() );
						}

						bool inNeib = LMH->m_neighbors.find( nodeB->getID() ) != LMH->m_neighbors.end();
						bool inBefNeib = neighbors_before.find( nodeB->getID() ) != neighbors_before.end();

						if ( inNeib && inBefNeib )
							lstWithinLMH[nodeB] = *a;  // Add to list:

					} // end for each arc

					// 2) Now, process:
					for (TArcList::const_iterator a=arcs.begin();a!=arcs.end();++a)
					{
						CHMHMapNodePtr nodeB;
						bool         dirA2B;

						CHMHMapArcPtr arc = *a;

						if ( arc->getNodeFrom() == *pBef )
						{ // node to delete is: "from"
							nodeB = m_map.getNodeByID( (*a)->getNodeTo() );
							dirA2B = true;
						}
						else
						{// node to delete is: "to"
							nodeB = m_map.getNodeByID( (*a)->getNodeFrom() );
							dirA2B = false;
						}

						bool inNeib = LMH->m_neighbors.find( nodeB->getID() ) != LMH->m_neighbors.end();
						bool inBefNeib = neighbors_before.find( nodeB->getID() ) != neighbors_before.end();

						if ( inNeib && inBefNeib )
						{
							// Target was and is in the LMH, nothing extra to do here.
						}
						else // The target was into the LMH, but not anymore.
						{
							// Target is outside of the LMH:
							// --------------------------------------------------------------
							//  Since we are deleting this node, we must readjust the
							//   arcs "a"<->"b" containing relative poses so they
							//   refer to valid reference poses.
							// --------------------------------------------------------------
							for (TListNodesArcs::iterator na=lstWithinLMH.begin();na!=lstWithinLMH.end();++na)
							{
								CHMHMapNodePtr node_c  = na->first;
								const CHMHMapArcPtr arc_c_a = na->second;

								// Now we have the arc "arc" from "node"<->"nodeB" in the direction "dirA2B", which will be deleted next.
								// The arc "a<->c", being "node_c" a node within the LMH, is in "arc_c_a".
								//   node_b -> outside LMH
								//   node_c -> within LMH
								// Then:
								//  A new arc "b->c" will be created with the Delta:
								//    Delta_b_c = [ a (-) b ] (+) [ c (-) a ]  = c (-) b
								//                \----v----/     \----v----/
								//                 Delta_b_a       Delta_a_c
								//

								// Get "Delta_b_a":
								CPose3DPDFGaussian Delta_b_a;
								TPoseID  refPoseAt_b;
								{
									CPose3DPDFGaussianPtr pdf  = arc->m_annotations.getAs<CPose3DPDFGaussian>( ARC_ANNOTATION_DELTA, LMH->m_ID, false );
									TPoseID  refPoseAt_a;
									if (!dirA2B)
									{
										Delta_b_a.copyFrom( *pdf );

										// Check valid reference poseIDs:
										arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseAt_b, LMH->m_ID, true );
										arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseAt_a, LMH->m_ID, true );
									}
									else
									{
										pdf->inverse( Delta_b_a );

										// Check valid reference poseIDs:
										arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseAt_b, LMH->m_ID, true );
										arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseAt_a, LMH->m_ID, true );
									}

									TPoseID  node_refPoseAt_b;
									nodeB->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, node_refPoseAt_b, LMH->m_ID, true);
									ASSERT_(node_refPoseAt_b==refPoseAt_b);

									TPoseID  node_refPoseAt_a;
									node->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, node_refPoseAt_a, LMH->m_ID, true);
									ASSERT_(node_refPoseAt_a==refPoseAt_a);
								}

								// Get "Delta_a_c":
								CPose3DPDFGaussian	Delta_a_c;
								TPoseID  refPoseAt_c;
								{
									CPose3DPDFGaussianPtr pdf = arc_c_a->m_annotations.getAs<CPose3DPDFGaussian>( ARC_ANNOTATION_DELTA, LMH->m_ID, false );
									TPoseID  refPoseAt_a;
									if ( arc_c_a->getNodeTo()==node_c->getID() )
									{
										Delta_a_c.copyFrom( *pdf );

										// Check valid reference poseIDs:
										arc_c_a->m_annotations.getElemental( ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseAt_a, LMH->m_ID, true );
										arc_c_a->m_annotations.getElemental( ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseAt_c, LMH->m_ID, true );
									}
									else
									{
										pdf->inverse( Delta_a_c );

										// Check valid reference poseIDs:
										arc_c_a->m_annotations.getElemental( ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseAt_a, LMH->m_ID, true );
										arc_c_a->m_annotations.getElemental( ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseAt_c, LMH->m_ID, true );
									}

									TPoseID  node_refPoseAt_c;
									node_c->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, node_refPoseAt_c, LMH->m_ID, true);
									ASSERT_(node_refPoseAt_c==refPoseAt_c);

									TPoseID  node_refPoseAt_a;
									node->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, node_refPoseAt_a, LMH->m_ID, true);
									ASSERT_(node_refPoseAt_a==refPoseAt_a);
								}

								// Compose:
								//  Delta_b_c = Delta_b_a (+) Delta_a_c
								CPose3DPDFGaussian	Delta_b_c( Delta_b_a + Delta_a_c );
								Delta_b_c.cov.zeros();  // *********** DEBUG !!!!!!!!!!!
								Delta_b_c.cov(0,0)=Delta_b_c.cov(1,1)=square(0.04);
								Delta_b_c.cov(3,3)=square(DEG2RAD(1));

								MRPT_LOG_DEBUG_STREAM << "b_a: " << Delta_b_a.mean << endl << "a_c: " << Delta_a_c.mean << endl << "b_a + a_c: " << Delta_b_c.mean << endl;

								// ------------------------------------------------
								// Finally, add the new annotation to arc "b->c":
								// ------------------------------------------------
								// Did an arc already exist? Look into existing arcs, in both directions:
								bool		 arcDeltaIsInverted;
								CHMHMapArcPtr newArc = m_map.findArcOfTypeBetweenNodes(
									nodeB->getID(),  // Source
									node_c->getID(),		// Target
									LMH->m_ID,	// Hypos
									"RelativePose",
									arcDeltaIsInverted );

								if (!newArc)
								{
									// Create a new one:
									newArc = CHMHMapArc::Create(
										nodeB,  // Source
										node_c,		// Target
										LMH->m_ID,	// Hypos
										&m_map			// The graph
										);
									newArc->m_arcType.setType("RelativePose");
									arcDeltaIsInverted = false;
								}

								if (!arcDeltaIsInverted)
								{  // arc: b->c
									newArc->m_annotations.set(ARC_ANNOTATION_DELTA, CPose3DPDFGaussianPtr( new CPose3DPDFGaussian(Delta_b_c) ),LMH->m_ID );
									MRPT_LOG_DEBUG_STREAM << "[LSLAM_proc_msg_AA] Setting arc " << nodeB->getID() << " -> " << node_c->getID() << " : " << Delta_b_c.mean << " cov = " << Delta_b_c.cov.inMatlabFormat() << endl;
									newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseAt_b ,LMH->m_ID );
									newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseAt_c ,LMH->m_ID );
								}
								else
								{  // arc: c->b
									CPose3DPDFGaussian Delta_b_c_inv;
									Delta_b_c.inverse( Delta_b_c_inv );

									MRPT_LOG_DEBUG_STREAM << "[LSLAM_proc_msg_AA] Setting arc " << nodeB->getID() << " <- " << node_c->getID() << " : " << Delta_b_c_inv.mean << " cov = " << Delta_b_c_inv.cov.inMatlabFormat() << endl;
									newArc->m_annotations.set(ARC_ANNOTATION_DELTA,CPose3DPDFGaussianPtr( new CPose3DPDFGaussian(Delta_b_c_inv)),LMH->m_ID );
									newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseAt_c ,LMH->m_ID );
									newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseAt_b ,LMH->m_ID );
								}
							} // end for each arc-node

							// Remove arc data for this hypothesis:
							arc->m_annotations.removeAll( LMH->m_ID );

							if (!arc->m_annotations.size())
							{
								arc.clear();
							}

						} // end of adjust arcs

					} // end for each arc

					// Make sure we delete all its arcs as well first:
					{
						TArcList arcs;
						node->getArcs(arcs);
						for (TArcList::iterator a=arcs.begin();a!=arcs.end();++a)
							a->clear();
					}

					node.clear();  // And finally, delete the node.
				} // end of "node" still exist in HMAP.
			}

		} // end for each before beigbors


		// ---------------------------------------------------------------------------------------
		//  Add areas to be considered by the TBI to launch potential loop-closure hypotheses.
		//   One option: To add the just newly entered area
		// ---------------------------------------------------------------------------------------
		//const CHMHMapNode::TNodeID new_curAreaID = LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ];
		if ( curAreaID != oldAreaID )
		{
			LMH->m_areasPendingTBI.insert(curAreaID); // Add to TBI list
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Current area changed: enqueing area %i for TBI.\n", (int)curAreaID );
		}
		else
		{
			static size_t cntAddTBI = 0;
			if (++cntAddTBI>4)
			{
				cntAddTBI = 0;
				LMH->m_areasPendingTBI.insert(curAreaID); // Add to TBI list
				logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Current area %i enqued for TBI (routine check).\n", (int)curAreaID );
			}
		}

		// ---------------------------------------------------------------------------------------
		// Create arcs between areas and the closest partition's area,
		//  and keep in order reference poses for each area, etc...
		//  This block of code also:
		//   - Update the arcs' deltas between all the pairs of connected areas within the LMH
		// ---------------------------------------------------------------------------------------
		// List of all LMH's internal arcs to create or update:
		//  Each entry is a pair of areas to create an arc between, and the "->second" is
		//  the corresponding reference pose IDs of each area.
		//map<TPairNodeIDs,TPairPoseIDs>		lstInternalArcsToCreate;
		list_searchable<TPairNodeIDs>			lstInternalArcsToCreate;

//		const CHMHMapNode::TNodeID curAreaID = LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ];

		if (partIdx2Areas.size()>1)
		{
			CCriticalSectionLocker	locker( &m_map_cs );
			//THypothesisIDSet   theArcHypos( LMH->m_ID );

			set<CHMHMapNode::TNodeID>	areasWithLink; // All the areas with at least one internal arc.

			// The closest distance between areas (only when above the threshold)
			map<CHMHMapNode::TNodeID, pair< CHMHMapNode::TNodeID, float > >	lstClosestDoubtfulNeigbors;

			for (size_t  idx_area_a=0;idx_area_a<partIdx2Areas.size();idx_area_a++)
			{
				// Get the area for this partition from the graph:
				// ------------------------------------------------------
				CHMHMapNode::TNodeID area_a_ID = partIdx2Areas[idx_area_a];
				CHMHMapNodePtr area_a =  m_map.getNodeByID(area_a_ID);
				ASSERT_(area_a);

				// Look for the closest area & it's reference pose:
				// -------------------------------------------------------------

				ASSERT_(myMsg.partitions[idx_area_a].size()>0);
				TPoseID 	poseID_trg;

				// Add the "trg" pose as reference in its area, or take the current reference, or change it
				//  if the pose id is no longer on the partition:

				if ( !area_a->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, poseID_trg, LMH->m_ID ) )
				{
					// No reference pose annotation yet: add it now:
					poseID_trg = myMsg.partitions[idx_area_a][0];

					area_a->m_annotations.setElemental( NODE_ANNOTATION_REF_POSEID, poseID_trg, LMH->m_ID );
					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Changing reference poseID of area '%i' to pose '%i'\n", (int)area_a_ID,(int)poseID_trg);

					// Reconsider the arcs of this area again, since the ref. poseID has changed:
					/*for ( list_searchable<TPairNodeIDs>::iterator it=lstAlreadyUpdated.begin();it!=lstAlreadyUpdated.end();  )
					{
						if (it->first == area_a_ID || it->second==area_a_ID)
								it = lstAlreadyUpdated.erase( it);
						else	it++;
					}*/

				}
				else
				{
					// Check if "poseID_trg" is still in the partition:
					bool found = false;
					TPoseID poseID_trg_old = poseID_trg;
					for (TPoseIDList::const_iterator p=myMsg.partitions[idx_area_a].begin();!found && p!=myMsg.partitions[idx_area_a].end();++p)
						if (poseID_trg==*p)
						{
							found = true;
							break;
						}

					if (!found)
					{
						// We must overwrite the anotation with a new reference pose:
						poseID_trg = myMsg.partitions[idx_area_a][0];
						area_a->m_annotations.setElemental( NODE_ANNOTATION_REF_POSEID, poseID_trg, LMH->m_ID );

						logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Changing reference poseID of area '%i' to pose '%i'\n", (int)area_a_ID,(int)poseID_trg);

						// ------------------------------------------------------------------------
						// Look for existing arcs from "area_a"<->other areas outside the LMH to
						//  fix the "Delta" annotations due to the change in reference poseID from
						//   the old "poseID_trg_old" to the new "poseID_trg".
						// ------------------------------------------------------------------------
						TArcList  arcs;
						area_a->getArcs( arcs );
						for (TArcList::const_iterator a=arcs.begin();a!=arcs.end();++a)
						{
							CHMHMapArcPtr theArc = *a;
							CHMHMapNode::TNodeID nodeFrom = theArc->getNodeFrom();
							CHMHMapNode::TNodeID nodeTo   = theArc->getNodeTo();

							// --------------------------------------------------------------------------------------------
							// Ok... we are here updating an existing arc "nodeFrom"->"nodeTo", with only one of the
							//  extremes being within the LMH, to account for a change in the reference pose of the area
							//  within the LMH.
							//  The old "poseID_trg_old" --> the new "poseID_trg".
							// --------------------------------------------------------------------------------------------
							if (nodeFrom==area_a_ID)
							{
								// Is nodeTo out of the LMH?
								if (LMH->m_neighbors.find( nodeTo )==LMH->m_neighbors.end())
								{	// nodeTo is outside the LMH:
									// The source area is into the LMH.
									CPose3DPDFParticles	 Anew_old_parts;
									LMH->getRelativePose(poseID_trg, poseID_trg_old, Anew_old_parts);

									CPose3DPDFGaussian Anew_old;
									Anew_old.copyFrom( Anew_old_parts );

									CPose3DPDFGaussian		newDelta;
									CPose3DPDFGaussianPtr	oldDelta = theArc->m_annotations.getAs<CPose3DPDFGaussian>(ARC_ANNOTATION_DELTA,LMH->m_ID, false );

									newDelta = Anew_old + *oldDelta;
									newDelta.cov.zeros();    // *********** DEBUG !!!!!!!!!!!
									newDelta.cov(0,0)=newDelta.cov(1,1)=square(0.04);
									newDelta.cov(3,3)=square(DEG2RAD(1));

									MRPT_LOG_DEBUG_STREAM<< "[LSLAM_proc_msg_AA] Updating arc " << nodeFrom << " -> " << nodeTo << " OLD: " << oldDelta->mean << " cov = " << oldDelta->cov.inMatlabFormat() << endl;
									MRPT_LOG_DEBUG_STREAM<< "[LSLAM_proc_msg_AA] Updating arc " << nodeFrom << " -> " << nodeTo << " NEW: " << newDelta.mean << " cov = " << newDelta.cov.inMatlabFormat() << endl;

									theArc->m_annotations.set(ARC_ANNOTATION_DELTA, CPose3DPDFGaussianPtr( new CPose3DPDFGaussian(newDelta)),LMH->m_ID );
									theArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_SRC_POSEID, poseID_trg ,LMH->m_ID );
								}
							}
							else
							{
								// Is nodeFrom out of the LMH?
								if (LMH->m_neighbors.find( nodeFrom )==LMH->m_neighbors.end())
								{	// nodeFrom is outside the LMH:
									// The target area is into the LMH:
									CPose3DPDFParticles	 Aold_new_parts;
									LMH->getRelativePose(poseID_trg_old, poseID_trg, Aold_new_parts);

									CPose3DPDFGaussian Aold_new;
									Aold_new.copyFrom( Aold_new_parts );

									CPose3DPDFGaussianPtr oldDelta = theArc->m_annotations.getAs<CPose3DPDFGaussian>(ARC_ANNOTATION_DELTA,LMH->m_ID, false );
									CPose3DPDFGaussian		newDelta;

									newDelta = *oldDelta + Aold_new;

									newDelta.cov.zeros();    // *********** DEBUG !!!!!!!!!!!
									newDelta.cov(0,0)=newDelta.cov(1,1)=square(0.04);
									newDelta.cov(3,3)=square(DEG2RAD(1));

									MRPT_LOG_DEBUG_STREAM<< "[LSLAM_proc_msg_AA] Updating arc " << nodeFrom << " <- " << nodeTo << " OLD: " << oldDelta->mean << " cov = " << oldDelta->cov.inMatlabFormat() << endl;
									MRPT_LOG_DEBUG_STREAM<< "[LSLAM_proc_msg_AA] Updating arc " << nodeFrom << " <- " << nodeTo << " NEW: " << newDelta.mean << " cov = " << newDelta.cov.inMatlabFormat() << endl;

									theArc->m_annotations.set(ARC_ANNOTATION_DELTA,CPose3DPDFGaussianPtr(new CPose3DPDFGaussian(newDelta)),LMH->m_ID );
									theArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_TRG_POSEID, poseID_trg ,LMH->m_ID );
								}
							}

						} // end for each arc
					}
				}

				// Now, go thru all other areas to check whether they are neighbors of "area_a":
				for (size_t idx_area_b=0;idx_area_b<myMsg.partitions.size();idx_area_b++)
				{
					if (idx_area_a==idx_area_b) continue; // Look for poses in a different area only!

					TPoseID 	poseID_closests = POSEID_INVALID;
					double      closestDistPoseSrc=0;

					// Get the "trg" pose at "area_a": Sweep over all the poses in the "area_a", to find the closests poses to other clusters:
					for (TPoseIDList::const_iterator itP0 = myMsg.partitions[idx_area_a].begin();itP0 != myMsg.partitions[idx_area_a].end();itP0++)
					{
						const CPose3D  &pose_trg= lstPoses[*itP0];  // Get its pose

						for (TPoseIDList::const_iterator itP = myMsg.partitions[idx_area_b].begin();itP != myMsg.partitions[idx_area_b].end();++itP)
						{
							const CPose3D  &otherPose = lstPoses[*itP];
							double dst = otherPose.distanceTo( pose_trg );
							if (dst<closestDistPoseSrc || poseID_closests == POSEID_INVALID)
							{
								poseID_closests = *itP;
								closestDistPoseSrc = dst;
								//closestAreaID = partIdx2Areas[k];
							}
						}
					} // end for itP0

					ASSERT_(poseID_closests != POSEID_INVALID);

					// Should we create an arc between area_a <-> area_b ??
					CHMHMapNode::TNodeID   area_b_ID = partIdx2Areas[idx_area_b];
					if ( closestDistPoseSrc< 5*m_options.SLAM_MIN_DIST_BETWEEN_OBS )
					{
						CHMHMapNodePtr area_b = m_map.getNodeByID( area_b_ID );
						ASSERT_(area_b);

						TPoseID 	poseID_src = POSEID_INVALID;
						if (!area_b->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, poseID_src, LMH->m_ID ) )
						{
							// Add 'poseID_closests': this should happen when the closest area is a new one:
							area_b->m_annotations.setElemental( NODE_ANNOTATION_REF_POSEID, poseID_closests, LMH->m_ID );
							poseID_src = poseID_closests;
							logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Changing reference poseID of area '%i' to pose '%i' (creat. annot)\n", (int)area_b_ID,(int)poseID_closests);
						}
						ASSERT_(poseID_src != POSEID_INVALID);

						// Add to the list of arcs to be computed after this loop:
						if ( lstInternalArcsToCreate.end() == lstInternalArcsToCreate.find(TPairNodeIDs( area_b_ID,area_a_ID )) &&
							 lstInternalArcsToCreate.end() == lstInternalArcsToCreate.find(TPairNodeIDs( area_a_ID, area_b_ID)) )
						{
							lstInternalArcsToCreate.insert( TPairNodeIDs( area_b_ID,area_a_ID ) );
							areasWithLink.insert( area_a_ID );
							areasWithLink.insert( area_b_ID );
						}
					}
					else
					{
						if (lstClosestDoubtfulNeigbors.find(area_b_ID) == lstClosestDoubtfulNeigbors.end() ||
							closestDistPoseSrc < lstClosestDoubtfulNeigbors[area_b_ID].second )
						{
							lstClosestDoubtfulNeigbors[area_b_ID].first = area_a_ID;
							lstClosestDoubtfulNeigbors[area_b_ID].second = closestDistPoseSrc ;
						}
					}

				} // end for idx_area_b

			} // end for each idx_area_a

			// ------------------------------------------------------------------------------------------------------
			// If two areas are neighbors but above the distance threshold, no link will be created between them:
			//  Check this situation by looking for doubtful neighbors for areas without any link:
			// ------------------------------------------------------------------------------------------------------
			for (size_t idx_area=0;idx_area<myMsg.partitions.size();idx_area++)
			{
				CHMHMapNode::TNodeID   area_ID = partIdx2Areas[idx_area];
				if (areasWithLink.find(area_ID) == areasWithLink.end())
				{
					// OK, this area does not have neighbor.. this cannot be so!
					if (lstClosestDoubtfulNeigbors.find(area_ID) != lstClosestDoubtfulNeigbors.end() )
					{
						// Add link to closest area:
						lstInternalArcsToCreate.insert( TPairNodeIDs( area_ID,lstClosestDoubtfulNeigbors[area_ID].first ) );

						// Now they have a link:
						areasWithLink.insert( area_ID );
						areasWithLink.insert( lstClosestDoubtfulNeigbors[area_ID].first );
					}
					else
					{
						THROW_EXCEPTION_CUSTOM_MSG1("Area %i seems unconnected??", (int)area_ID);
					}
				}
			}

		} // end if # partitions >= 2  && lock on m_map

#if	1
		{
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] lstInternalArcsToCreate contains %i entries:\n",(int)lstInternalArcsToCreate.size());
			for ( list_searchable<TPairNodeIDs>::const_iterator arcCreat=lstInternalArcsToCreate.begin();arcCreat!=lstInternalArcsToCreate.end();++arcCreat)
			{
				// Get the reference pose IDs:
				CHMHMapNode::TNodeID	closestAreaID	= arcCreat->first;
				CHMHMapNode::TNodeID	newAreaID		 = arcCreat->second;
				logFmt(mrpt::utils::LVL_INFO,"  AREA %i <-> AREA %i\n", (int)closestAreaID, (int)newAreaID );
			}
		}
#endif

		// -------------------------------------------------------------------------------------
		// Now, create or update all the internal arcs in the list "lstInternalArcsToCreate"
		//  The relative pose between the two referencePoseId's is computed and stored
		//  in the corresponding arc in the HMT-map:
		// -------------------------------------------------------------------------------------
		{
			CCriticalSectionLocker	locker( &m_map_cs );
			THypothesisIDSet   theArcHypos( LMH->m_ID );

			for ( list_searchable<TPairNodeIDs>::const_iterator arcCreat=lstInternalArcsToCreate.begin();arcCreat!=lstInternalArcsToCreate.end();++arcCreat)
			{
				// Get the reference pose IDs:
				CHMHMapNode::TNodeID	area_a_ID	= arcCreat->first;
				TPoseID					area_a_poseID_src;
				m_map.getNodeByID(area_a_ID)->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, area_a_poseID_src, LMH->m_ID, true );

				CHMHMapNode::TNodeID	area_b_ID	= arcCreat->second;
				TPoseID					area_b_poseID_trg;
				m_map.getNodeByID(area_b_ID)->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, area_b_poseID_trg, LMH->m_ID, true );

				// Get relative pose PDF according to this LMH:
				CPose3DPDFParticles		relPoseParts;
				LMH->getRelativePose(area_a_poseID_src, area_b_poseID_trg, relPoseParts );

				// Pass to gaussian PDF:
				CPose3DPDFGaussian		relPoseGauss;
				relPoseGauss.copyFrom(relPoseParts);

				relPoseGauss.cov.zeros();    // *********** DEBUG !!!!!!!!!!!
				relPoseGauss.cov(0,0)=relPoseGauss.cov(1,1)=square(0.04);
				relPoseGauss.cov(3,3)=square(DEG2RAD(1));

				logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Creating arc %i[ref:%i] -> %i[ref:%i] = (%.03f,%.03f,%.03fdeg)\n",
					(int)area_a_ID,(int)area_a_poseID_src,
					(int)area_b_ID,(int)area_b_poseID_trg,
					relPoseGauss.mean.x(),relPoseGauss.mean.y(),RAD2DEG(relPoseGauss.mean.yaw())
					);

				// Did an arc already exist?
				// Look into existing arcs, in both directions:
				bool		 arcDeltaIsInverted;
				CHMHMapArcPtr newArc = m_map.findArcOfTypeBetweenNodes(
					area_a_ID,
					area_b_ID,
					LMH->m_ID,
					"RelativePose",
					arcDeltaIsInverted );

				// If not found, create it now:
				if (!newArc)
				{
					newArc = CHMHMapArc::Create(
						area_a_ID,  // Source
						area_b_ID,		// Target
						theArcHypos,	// Hypos
						&m_map			// The graph
						);
					newArc->m_arcType.setType("RelativePose");
					arcDeltaIsInverted = false;
				}

				// Add data to arc:
				if (!arcDeltaIsInverted)
				{
					MRPT_LOG_DEBUG_STREAM<< "[LSLAM_proc_msg_AA] Updating int. arc " << area_a_ID << " -> " << area_b_ID << " : " << relPoseGauss.mean << " cov = " << relPoseGauss.cov.inMatlabFormat() << endl;
					newArc->m_annotations.set(ARC_ANNOTATION_DELTA, CPose3DPDFGaussianPtr(new CPose3DPDFGaussian(relPoseGauss)),LMH->m_ID );
					newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_SRC_POSEID, area_a_poseID_src ,LMH->m_ID );
					newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_TRG_POSEID, area_b_poseID_trg ,LMH->m_ID );
				}
				else
				{
					CPose3DPDFGaussian	relPoseInv;
					relPoseGauss.inverse(relPoseInv);

					MRPT_LOG_DEBUG_STREAM<< "[LSLAM_proc_msg_AA] Updating int. arc " << area_a_ID << " <- " << area_b_ID << " : " << relPoseInv.mean << " cov = " << relPoseInv.cov.inMatlabFormat() << endl;
					newArc->m_annotations.set(ARC_ANNOTATION_DELTA,CPose3DPDFGaussianPtr( new CPose3DPDFGaussian(relPoseInv)),LMH->m_ID );

					newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_SRC_POSEID, area_b_poseID_trg,LMH->m_ID );
					newArc->m_annotations.setElemental(ARC_ANNOTATION_DELTA_TRG_POSEID, area_a_poseID_src,LMH->m_ID );
				}

			} // end for each arc in lstInternalArcsToCreate

		} // end lock m_map

		// ----------------------------------------------------------------
		//   Remove arcs between areas that now do not need to have
		//    an arcs between them: we know by seeing if there is not
		//    an entry in 'lstAlreadyUpdated' but the arc actually exists:
		// ----------------------------------------------------------------
		{
			CCriticalSectionLocker	locker( &m_map_cs );

			for (TNodeIDSet::const_iterator pNei=LMH->m_neighbors.begin();pNei!=LMH->m_neighbors.end();++pNei)
			{
				const CHMHMapNode::TNodeID nodeFromID = *pNei;

				// Follow all arcs of this node:
				CHMHMapNodePtr nodeFrom=m_map.getNodeByID(nodeFromID);
				ASSERT_(nodeFrom);
				TArcList lstArcs;
				nodeFrom->getArcs( lstArcs, "RelativePose", LMH->m_ID );

				// Look for arcs to be removed:
				//   A) Arcs to areas within the LMH but which are not in "lstAlreadyUpdated"
				for (TArcList::const_iterator a=lstArcs.begin();a!=lstArcs.end();++a)
				{
					const CHMHMapNode::TNodeID nodeToID = (*a)->getNodeFrom()==nodeFromID ? (*a)->getNodeTo():(*a)->getNodeFrom();

					if (LMH->m_neighbors.find( nodeToID )!=LMH->m_neighbors.end())
					{
						CHMHMapArcPtr arc = *a;

						// Do exist a corresponding entry in "lstAlreadyUpdated"?
						if ( lstInternalArcsToCreate.end() == lstInternalArcsToCreate.find(TPairNodeIDs( nodeFromID,nodeToID )) &&
							 lstInternalArcsToCreate.end() == lstInternalArcsToCreate.find(TPairNodeIDs( nodeToID, nodeFromID)) )
						{
							// it doesn't! Delete this arc:
							arc->m_annotations.remove(ARC_ANNOTATION_DELTA,LMH->m_ID );
							logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Deleting annotation of arc: %lu-%lu\n", (long unsigned)nodeFromID, (long unsigned) nodeToID);
							// Any other ARC_ANNOTATION_DELTA? If not, delete the entire arc:
							if (! arc->m_annotations.getAnyHypothesis(ARC_ANNOTATION_DELTA) )
							{
								logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Deleting empty arc: %lu-%lu\n", (long unsigned)nodeFromID, (long unsigned) nodeToID);
								arc.clear();
							}
						}
					}
				} // end for each arc in lstArcs

			} // end for each neighbor
		} // end lock m_map_cs

		if (0)
		{
			CCriticalSectionLocker  locker( &m_map_cs );
			utils::CStringList s;
			m_map.dumpAsText(s);
			s.saveToFile( format("%s/HMAP_txt/HMAP_%05i_mid.txt",m_options.LOG_OUTPUT_DIR.c_str(), DEBUG_STEP ) );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Saved HMAP_%05i_mid.txt\n",DEBUG_STEP);
		}

		// -----------------------------------------------------------------------------
		//  Remove areas from LMH if they are at a topological distance of 2 or more
		//   We can quickly check this by identifying areas without a direct arc between
		//   them and the current area.
		// -----------------------------------------------------------------------------
//		const CHMHMapNode::TNodeID curAreaID = LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ];

		for (TNodeIDSet::iterator pNei1=LMH->m_neighbors.begin();pNei1!=LMH->m_neighbors.end();  )
		{
			if (*pNei1 != curAreaID )
			{
				TArcList lstArcs;
				{
					CCriticalSectionLocker  locker( &m_map_cs );
					m_map.findArcsOfTypeBetweenNodes( *pNei1, curAreaID , LMH->m_ID, "RelativePose", lstArcs);
				}
				if (lstArcs.empty())
				{
					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Getting area '%u' out of LMH\n", static_cast<unsigned>(*pNei1) );

					// Remove from list first:
					CHMHMapNode::TNodeID id = *pNei1;

					pNei1 = erase_return_next(LMH->m_neighbors,pNei1);

					// Now: this calls internally to "updateAreaFromLMH"
					double ESS_bef = LMH->ESS();
					LMH->removeAreaFromLMH( id );
					double ESS_aft = LMH->ESS();
					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] ESS: %f -> %f\n", ESS_bef, ESS_aft );
				}
				else pNei1++; // Go next:
			}
			else pNei1++; // Go next:
		} // end for pNei1



		// This list contains those areas just inserted into the LMH, so their poses have been added
		//  to the particles, etc... but not their observations into the metric maps: this is delayed
		//  since in the case we would need to change coordinate origin, it would had been pointless.
		TNodeIDSet	 areasDelayedMetricMapsInsertion;

		// -------------------------------------------------------------
		//  Recompose LMH by bringing in all areas with an arc to the
		//   current area:
		// -------------------------------------------------------------
		CHMHMapNodePtr currentArea;
		{
			CCriticalSectionLocker  locker( &m_map_cs );

			const CHMHMapNode::TNodeID curAreaID = LMH->m_nodeIDmemberships[ LMH->m_currentRobotPose ];
			currentArea = m_map.getNodeByID( curAreaID );

			TPoseID refPoseCurArea_accordingAnnot;
			currentArea->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, refPoseCurArea_accordingAnnot, LMH->m_ID, true );

			TArcList  arcsToCurArea;
			currentArea->getArcs(arcsToCurArea,"RelativePose",LMH->m_ID);
			for (TArcList::iterator a=arcsToCurArea.begin();a!=arcsToCurArea.end();++a)
			{
				const CHMHMapArcPtr arc = (*a);
				const CHMHMapNode::TNodeID otherAreaID = arc->getNodeFrom()==curAreaID ? arc->getNodeTo():arc->getNodeFrom();

				// If otherArea is out of the LMH, we must bring it in!
				if ( LMH->m_neighbors.find(otherAreaID)==LMH->m_neighbors.end() )
				{
					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Bringing in LMH area %i\n",(int)otherAreaID );

					CHMHMapNodePtr area = m_map.getNodeByID( otherAreaID );
					ASSERT_(area);

					CRobotPosesGraphPtr pg = area->m_annotations.getAs<CRobotPosesGraph>( NODE_ANNOTATION_POSES_GRAPH, LMH->m_ID, false );

					// Find the coordinate transformation between areas "currentArea"->"area" = Delta_c2a
					CPose3D	Delta_c2a;		// We are just interested in the mean
					{
						CPose3DPDFGaussianPtr pdf = arc->m_annotations.getAs<CPose3DPDFGaussian>( ARC_ANNOTATION_DELTA, LMH->m_ID, false );

						pdf->getMean( Delta_c2a );
					}

					TPoseID refPoseIDAtOtherArea, refPoseIDAtCurArea;

					if (arc->getNodeTo()==curAreaID)
					{
						// It is inverted:
						logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Arc is inverted: (%.03f,%.03f,%.03fdeg) -> ",
							Delta_c2a.x(),Delta_c2a.y(),RAD2DEG(Delta_c2a.yaw()) );

						Delta_c2a = CPose3D(0,0,0) - Delta_c2a;

						logFmt(mrpt::utils::LVL_INFO,"(%.03f,%.03f,%.03fdeg)\n",
							Delta_c2a.x(),Delta_c2a.y(),RAD2DEG(Delta_c2a.yaw()) );

						arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseIDAtOtherArea, LMH->m_ID, true );
						arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseIDAtCurArea, LMH->m_ID, true );
					}
					else
					{
						// It is NOT inverted.
						arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_TRG_POSEID, refPoseIDAtOtherArea, LMH->m_ID, true );
						arc->m_annotations.getElemental( ARC_ANNOTATION_DELTA_SRC_POSEID, refPoseIDAtCurArea, LMH->m_ID, true );
					}

					logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Bringing in: refPoseCur=%i refPoseOther=%i -> Delta_c2a:(%.03f,%.03f,%.03fdeg)\n",
						(int)refPoseIDAtCurArea, (int)refPoseIDAtOtherArea,
						Delta_c2a.x(),Delta_c2a.y(),RAD2DEG(Delta_c2a.yaw()) );

					// Assure the arc's references are OK:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					{
						TPoseID refPoseOtherArea_accordingAnnot;
						area->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, refPoseOtherArea_accordingAnnot, LMH->m_ID, true );
						ASSERT_( refPoseIDAtOtherArea == refPoseOtherArea_accordingAnnot );

						ASSERT_( refPoseIDAtCurArea == refPoseCurArea_accordingAnnot );
					}
#endif
					// Given the above checks: the new particles' poses are simply:
					//  POSE_i' = refPoseCurrentArea (+) Delta_cur_area (+) POSE_i

					// Create new poses within the particles:
					// --------------------------------------------
					TPoseIDList		lstNewPoseIDs;
					lstNewPoseIDs.reserve( pg->size() );
					for (CRobotPosesGraph::iterator p=pg->begin();p!=pg->end();++p)
					{
						const TPoseID   &poseID = p->first;
						const TPoseInfo &poseInfo = p->second;

						lstNewPoseIDs.push_back(poseID);

						// Add the particles:
						ASSERT_( poseInfo.pdf.m_particles.size() == LMH->m_particles.size() );

						CPose3DPDFParticles::CParticleList::const_iterator  itSrc;
						CLocalMetricHypothesis::CParticleList::iterator     itTrg;

						for (itSrc=poseInfo.pdf.m_particles.begin(), itTrg=LMH->m_particles.begin(); itTrg!=LMH->m_particles.end(); itSrc++,itTrg++)
						{
							// log_w: not modified since diff. areas are independent...
							itTrg->d->robotPoses[ poseID ] = itTrg->d->robotPoses[ refPoseIDAtCurArea ] + Delta_c2a + *itSrc->d;
						}

						// Update m_nodeIDmemberships
						LMH->m_nodeIDmemberships[ poseID ] = otherAreaID;

						// Update m_SFs
						LMH->m_SFs[ poseID ] = poseInfo.sf;

						// Add area to neighbors:
						LMH->m_neighbors.insert( otherAreaID );

					} // for each pose in the new area (Crobotposesgraph)

					// Update m_robotPosesGraph: This will be done in the next iteration of the LSLAM thread,
					//  now just add to the list of pending pose IDs:
					LMH->m_posesPendingAddPartitioner.insert( LMH->m_posesPendingAddPartitioner.end(),lstNewPoseIDs.begin(),lstNewPoseIDs.end() );

					// Mark this new area as to pending for updating the metric map at the end of this method:
					areasDelayedMetricMapsInsertion.insert( otherAreaID );

				} // end if the area is out of LMH
			} // end for each arc
		} // end of lock on m_map_cs


		if (0)
		{
			utils::CStringList s;
			LMH->dumpAsText(s);
			s.saveToFile( format("%s/HMAP_txt/HMAP_%05i_LMH_mid.txt",m_options.LOG_OUTPUT_DIR.c_str(), DEBUG_STEP ) );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Saved HMAP_%05i_LMH_mid.txt\n",DEBUG_STEP);
		}
		if (0)
		{
			COpenGLScene	sceneLSLAM;
			// Generate the metric maps 3D view...
			opengl::CSetOfObjectsPtr maps3D = opengl::CSetOfObjects::Create();
			maps3D->setName( "metric-maps" );
			LMH->getMostLikelyParticle()->d->metricMaps.getAs3DObject( maps3D );
			sceneLSLAM.insert( maps3D );

			// ...and the robot poses, areas, etc:
			opengl::CSetOfObjectsPtr LSLAM_3D = opengl::CSetOfObjects::Create();
			LSLAM_3D->setName("LSLAM_3D");
			LMH->getAs3DScene( LSLAM_3D );
			sceneLSLAM.insert( LSLAM_3D );

			sceneLSLAM.enableFollowCamera(true);

			string filLocalAreas = format("%s/HMAP_txt/HMAP_%05i_LMH_mid.3Dscene",m_options.LOG_OUTPUT_DIR.c_str(), DEBUG_STEP );
			logFmt(mrpt::utils::LVL_INFO,"[LOG] Saving %s\n", filLocalAreas.c_str());
			CFileOutputStream(filLocalAreas) << sceneLSLAM;
		}

		// -------------------------------------------------------------
		//  Change local coordinate system, as required
		//  This regenerates the metric maps as well.
		// -------------------------------------------------------------
		TPoseID new_poseID_origin;

		if (! currentArea->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID,new_poseID_origin, LMH->m_ID ) )
			THROW_EXCEPTION("New coordinate origin not found");

		if ( new_poseID_origin != poseID_origin )
		{	// Change coords AND rebuild metric maps
			CTicTac tictac;
			tictac.Tic();
			LMH->changeCoordinateOrigin( new_poseID_origin );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] LMH->changeCoordinateOrigin %lu->%lu took %f ms\n", poseID_origin,new_poseID_origin,tictac.Tac()*1000 );
		}
		else if (areasDelayedMetricMapsInsertion.size())
		{
			CTicTac tictac;
			tictac.Tic();
			// We haven't rebuilt the whole metric maps, so just insert the new observations as needed:
			for (TNodeIDSet::iterator areaID=areasDelayedMetricMapsInsertion.begin();areaID!=areasDelayedMetricMapsInsertion.end();++areaID)
			{
				// For each posesID within this areaID:
				for(map<TPoseID,CHMHMapNode::TNodeID>::const_iterator pn=LMH->m_nodeIDmemberships.begin();pn!=LMH->m_nodeIDmemberships.end();++pn)
				{
					if (pn->second==*areaID)
					{
						// We must add this poseID:
						const TPoseID & poseToAdd = pn->first;
						const CSensoryFrame &SF = LMH->m_SFs.find(poseToAdd)->second;

						// Process the poses in the list for each particle:
						for (CLocalMetricHypothesis::CParticleList::iterator partIt=LMH->m_particles.begin();partIt!=LMH->m_particles.end();++partIt)
						{
							TMapPoseID2Pose3D::const_iterator pose3D = partIt->d->robotPoses.find( poseToAdd );
							ASSERT_(pose3D!=partIt->d->robotPoses.end());
							SF.insertObservationsInto(  &partIt->d->metricMaps, & pose3D->second );
						} // end for each particle
					}
				} // end for each m_nodeIDmemberships
			} // end for each areasDelayedMetricMapsInsertion

			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] areasDelayedMetricMapsInsertion took %f ms\n", tictac.Tac()*1000 );
		}

		if (0)
		{
			CCriticalSectionLocker  locker( &m_map_cs );
			utils::CStringList s;
			m_map.dumpAsText(s);
			s.saveToFile( format("%s/HMAP_txt/HMAP_%05i_after.txt",m_options.LOG_OUTPUT_DIR.c_str(), DEBUG_STEP ) );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Saved HMAP_%05i_after.txt\n",DEBUG_STEP);
		}
		if (0)
		{
			utils::CStringList s;
			LMH->dumpAsText(s);
			s.saveToFile( format("%s/HMAP_txt/HMAP_%05i_LMH_after.txt",m_options.LOG_OUTPUT_DIR.c_str(), DEBUG_STEP ) );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Saved HMAP_%05i_LMH_after.txt\n",DEBUG_STEP);
		}

		logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_AA] Msg from AA took %f ms                      ]\n", tictac.Tac()*1000 );

		MRPT_END
}


/*---------------------------------------------------------------
					LSLAM_process_message_from_TBI
  ---------------------------------------------------------------*/
void CHMTSLAM::LSLAM_process_message_from_TBI( const TMessageLSLAMfromTBI &myMsg )
{
	MRPT_START

	CTicTac    tictac;
	tictac.Tic();
	logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_TBI] Beginning of Msg from TBI processing...              [\n" );

	// In case of multiple areas involved in a TLC, we need a mapping from the old areaIDs to the new ones:
	std::map<CHMHMapNode::TNodeID,CHMHMapNode::TNodeID>  alreadyClosedLoops;

	for (map< CHMHMapNode::TNodeID, TMessageLSLAMfromTBI::TBI_info >::const_iterator candidate = myMsg.loopClosureData.begin();candidate != myMsg.loopClosureData.end();++candidate)
	{
		logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_TBI] Processing TLC of areas: %u <-> %u...  \n",(unsigned)myMsg.cur_area, (unsigned)candidate->first );

		// Check if the area has already been merged:
		CHMHMapNode::TNodeID  currentArea = myMsg.cur_area;
		if (alreadyClosedLoops.find(myMsg.cur_area)!=alreadyClosedLoops.end())
		{
			currentArea = alreadyClosedLoops[myMsg.cur_area];
			cout << "[LSLAM_proc_msg_TBI] Using " << myMsg.cur_area << " -> " <<  currentArea  << "  due to area being already merged." << endl;
		}

		// Get pose PDF according to HMAP
		// -----------------------------------------
		CPose3DPDFParticles pdfPartsHMap;
		m_map.computeCoordinatesTransformationBetweenNodes(
			currentArea,			// Pose of "candidate->first" as seen from "currentArea" (The order is critical!!)
			candidate->first,
			pdfPartsHMap,
			myMsg.hypothesisID,
			100,
			0.10f,DEG2RAD(1.0f)  // Extra noise in each "arc"
			);

		CPose3DPDFGaussian	pdfDeltaMap;
		pdfDeltaMap.copyFrom(pdfPartsHMap);

		// Increase the uncertainty to avoid too understimated covariances and make the chi-test fail:
		pdfDeltaMap.cov(0,0) += square(1.0);
		pdfDeltaMap.cov(1,1) += square(1.0);
		pdfDeltaMap.cov(2,2) += square(1.0);
		pdfDeltaMap.cov(3,3) += square(DEG2RAD(5));
		pdfDeltaMap.cov(4,4) += square(DEG2RAD(5));
		pdfDeltaMap.cov(5,5) += square(DEG2RAD(5));

		cout << "[LSLAM_proc_msg_TBI] HMap_delta=" << pdfDeltaMap.mean << " std_x=" << sqrt(pdfDeltaMap.cov(0,0)) <<" std_y=" << sqrt(pdfDeltaMap.cov(1,1)) << endl;

		// Get pose PDF according to TLC detector:
		//  It's a SOG, so we should make an ordered list with each Gaussian mode
		//   and its probability/compatibility according to the metric information:
		// -------------------------------------------------------------------------
		ASSERT_(!candidate->second.delta_new_cur.empty());
		const double chi2_thres = mrpt::math::chi2inv(0.999, CPose3DPDFGaussian::state_length );

		map<double,CPose3DPDFGaussian>  lstModesAndCompats;  // first=log(e^-0.5*maha_dist)+log(likelihood); The list only contains those chi2 compatible

		for (CPose3DPDFSOG::const_iterator itSOG=candidate->second.delta_new_cur.begin();itSOG!=candidate->second.delta_new_cur.end();++itSOG)
		{
			const CPose3DPDFGaussian	&pdfDelta = itSOG->val;

			cout << "[LSLAM_proc_msg_TBI]  TLC_delta=" << pdfDelta.mean << " std_x=" << sqrt(pdfDelta.cov(0,0)) <<" std_y=" << sqrt(pdfDelta.cov(1,1)) <<" std_phi=" << RAD2DEG(sqrt(pdfDelta.cov(3,3))) << endl;

			// Perform chi2 test (with Mahalanobis distance):
			// ------------------------------------------------
			const double mahaDist2 = square( pdfDeltaMap.mahalanobisDistanceTo(pdfDelta) );
			cout << "[LSLAM_proc_msg_TBI] maha_dist = "  << mahaDist2 << endl;

			if (mahaDist2<chi2_thres)
			{
				const double log_lik = itSOG->log_w - 0.5*mahaDist2;
				lstModesAndCompats[ log_lik ] = itSOG->val;
				cout << "[LSLAM_proc_msg_TBI] Added to list of candidates: log(overall_lik)= "  << log_lik << endl;
			}
		} // for each SOG mode

		// Any good TLC candidate?
		if (!lstModesAndCompats.empty())
		{
			const CPose3DPDFGaussian &pdfDelta = lstModesAndCompats.rbegin()->second;

			mrpt::system::pause();

			// --------------------------------------------------------
			// Two options here:
			//  1) Create a new LMH for each acceptable possibility
			//  2) Just keep the most likely one (***** CHOICE, FOR NOW!!! *****)
			// --------------------------------------------------------
			static CTicTac  tictac;
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_TBI] Accepting TLC of areas: %u <-> %u  with an overall log(lik)=%f  \n",(unsigned)currentArea, (unsigned)candidate->first,lstModesAndCompats.rbegin()->first );

			tictac.Tic();
			this->perform_TLC(
				m_LMHs[myMsg.hypothesisID],
				currentArea,   // Area in the LMH
				candidate->first, // External area
				pdfDelta );
			logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_TBI] TLC of areas %u <-> %u  - DONE in %.03f ms\n",(unsigned)currentArea, (unsigned)candidate->first,1e3*tictac.Tac() );

			// The old area "myMsg.cur_area" is now "candidate->first"
			alreadyClosedLoops[myMsg.cur_area] = candidate->first;

		} // end there is any good TLC candidate


	} // end for each candidate

	logFmt(mrpt::utils::LVL_INFO,"[LSLAM_proc_msg_TBI] Msg from TBI took %f ms                      ]\n", tictac.Tac()*1000 );

	MRPT_END
}
