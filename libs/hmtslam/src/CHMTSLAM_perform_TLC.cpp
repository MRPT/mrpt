/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/hmtslam/CRobotPosesGraph.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/utils/CFileStream.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

/*---------------------------------------------------------------

						perform_TLC

	Topological Loop Closure: Performs all the required operations
	to close a loop between two areas which have been determined 
	to be the same.

  ---------------------------------------------------------------*/
void CHMTSLAM::perform_TLC(
	CLocalMetricHypothesis					&LMH,
	const CHMHMapNode::TNodeID				Ai,  // areaInLMH,
	const CHMHMapNode::TNodeID				Ae, // areaLoopClosure,
	const mrpt::poses::CPose3DPDFGaussian	&pose_i_wrt_e
	)
{
	MRPT_START
	ASSERT_(Ai!=Ae)

	synch::CCriticalSectionLocker locker ( &LMH.m_robotPosesGraph.lock );

	logFmt(mrpt::utils::LVL_DEBUG, "[perform_TLC] TLC of areas: %u <-> %u \n",(unsigned)Ai, (unsigned)Ae );

	// * Verify a1 \in LMH & a2 \notin LMH
	// ----------------------------------------------------------------------
	ASSERT_( LMH.m_neighbors.count(Ai) == 1);
	ASSERT_( LMH.m_neighbors.count(Ae) == 0);

	// * Get the old pose reference of the area which will desapear:
	// ----------------------------------------------------------------------
	TPoseID		poseID_Ai_origin; 
	m_map.getNodeByID(Ai)->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID,poseID_Ai_origin, LMH.m_ID, true);
//	TPoseID		poseID_new_origin;
//	m_map.getNodeByID(Ae)->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID,poseID_new_origin, LMH.m_ID, true);

	// * Make a list Li of pose IDs in Ai, which need a change in coordinates:
	// ----------------------------------------------------------------------------------------------
	//TPoseIDSet  Li;
	//for (map<TPoseID,CHMHMapNode::TNodeID>::const_iterator it=LMH.m_nodeIDmemberships.begin();it!=LMH.m_nodeIDmemberships.end();++it)
	//	if(it->second==Ai)
	//		Li.insert(it->first);

	// * Change all poses in "Ai" from "p" to " 0 (-) pose1wrt2 (+) (p (-) oldRef )"
	// ----------------------------------------------------------------------------------------------
	// For all the particles
	//for ( CLocalMetricHypothesis::CParticleList::const_iterator it = LMH.m_particles.begin(); it!=LMH.m_particles.end(); ++it)
	//{	
	//	// Touch only the affected poseIDs:
	//	for (TPoseIDSet::const_iterator n=Li.begin();n!=Li.end();++n)
	//	{
	//		map<TPoseID,CPose3D>::iterator itPos = it->d->robotPoses.find(*n);
	//		ASSERT_(itPos!=it->d->robotPoses.end());
	//		itPos->second = Apose_ei + (itPos->second - oldRef);
	//	}
	//}

	// Change coords in incr. partitioning as well:
	//CSimpleMap *SFseq = LMH.m_robotPosesGraph.partitioner.getSequenceOfFrames();
	//for (std::map<uint32_t,TPoseID>::const_iterator it=LMH.m_robotPosesGraph.idx2pose.begin();it!=LMH.m_robotPosesGraph.idx2pose.end();++it)
	//{
	//	// Only if the pose has been affected:
	//	if (Li.count(it->second)==0) continue;

	//	CPose3DPDFPtr		pdf;
	//	CSensoryFramePtr	sf;
	//	SFseq->get( it->first, pdf, sf);

	//	// Copy from particles:
	//	CPose3DPDFParticlesPtr pdfParts = CPose3DPDFParticlesPtr(pdf);
	//	LMH.getPoseParticles( it->second, *pdfParts);
	//}

	// * Mark all poses in LMH \in Ai as being of "Ae":
	// ----------------------------------------------------------------------------------------------
	for (map<TPoseID,CHMHMapNode::TNodeID>::iterator it=LMH.m_nodeIDmemberships.begin();it!=LMH.m_nodeIDmemberships.end();++it)
		if(it->second==Ai) 
			it->second=Ae;

	// * Replace "Ai" by "Ae" in the list of neighbors areas:
	// ----------------------------------------------------------------------------------------------
	LMH.m_neighbors.erase(LMH.m_neighbors.find(Ai));
	LMH.m_neighbors.insert(Ae);


	// * Reflect the change of coordinates in all existing arcs from/to "Ai", which are not part of the LMH (list of neighbors)
	//     "p" to " 0 (-) pose1wrt2 (+) (p (-) oldRef )"...
	//  NOTE: Arcs to "Ai" must be replaced by arcs to "Ae"
	// ----------------------------------------------------------------------------------------------
	{
		TArcList lstArcs;
		m_map.getNodeByID(Ai)->getArcs(lstArcs,LMH.m_ID);
		for(TArcList::iterator arc=lstArcs.begin();arc!=lstArcs.end();++arc)
		{
			// TODO ...
		}
	}

	// * The node for "area Ai" must be deleted in the HMAP, and all its arcs.
	// ----------------------------------------------------------------------------------------------
	{
		TArcList lstArcs;
		m_map.getNodeByID(Ai)->getArcs(lstArcs);
		for(TArcList::iterator arc=lstArcs.begin();arc!=lstArcs.end();++arc)
			arc->clear(); // The "delete" will automatically remove the entry in "m_map". Other smrtpnts will be cleared too.

		m_map.getNodeByID(Ai).clear();	// The "delete" will automatically remove the entry in "m_map"
	}


	// * Merge Ae into the LMH: This includes the change of coordinates to those used now in "Ai" and the rest of neighbors!
	// ----------------------------------------------------------------------------------------------
	CPose3DPDFParticles pdfAiRef;
	LMH.getPoseParticles(poseID_Ai_origin,pdfAiRef);
	CPose3D AiRef;
	pdfAiRef.getMean(AiRef);
	const CPose3D &Apose_ie = pose_i_wrt_e.mean;
	//const CPose3D  Apose_ei = -Apose_ie;
	const CPose3D AeRefInLMH = AiRef + Apose_ie;

	CRobotPosesGraphPtr AePG = m_map.getNodeByID(Ae)->m_annotations.getAs<CRobotPosesGraph>(NODE_ANNOTATION_POSES_GRAPH, LMH.m_ID);

	// For each pose in Ae:
	for (CRobotPosesGraph::const_iterator it=AePG->begin();it!=AePG->end();++it)
	{
		const TPoseID   poseId = it->first;
		const TPoseInfo &pi = it->second;

		//  Insert the observations into the metric maps:
		ASSERT_(LMH.m_particles.size() == pi.pdf.size() );

		for (size_t i=0;i<pi.pdf.size();i++)
		{
			// Transport coordinates:
			const CPose3D  &p = *pi.pdf.m_particles[i].d;
			LMH.m_particles[i].d->robotPoses[poseId] = AeRefInLMH + p;
			//pi.sf.insertObservationsInto( &LMH.m_particles[i].d->metricMaps, pi.pdf.m_particles[i].d );
		}

		// Add node membership:
		LMH.m_nodeIDmemberships[poseId] = Ae;

		// Now, insert the SF
		LMH.m_SFs[poseId] = pi.sf;

		// Also add to the list of poses pending to be evaluated by the AA:
		LMH.m_posesPendingAddPartitioner.push_back(poseId);
	}

	// * The current pose of the robot should be in the new coordinate system:
	// ----------------------------------------------------------------------------------------------
	//  This is already done since the current pose is just another poseID and it's threaded like the others.


	// * Rebuild metric maps for consistency:
	// ----------------------------------------------------------------------------------------------
	LMH.rebuildMetricMaps();


	// * Update all the data in the node "Ae":
	// ----------------------------------------------------------------------------------------------
	LMH.updateAreaFromLMH(Ae);



	MRPT_END
}
