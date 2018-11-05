/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/system/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/io/CFileStream.h>
#include <mrpt/system/os.h>

#include <memory>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------

						CHMTSLAM_TBI

	Topological Bayesian Inference (TBI) process within HMT-SLAM

  ---------------------------------------------------------------*/
void CHMTSLAM::thread_TBI()
{
	CHMTSLAM* obj = this;
	CTicTac tictac;

	// Seems that must be called in each thread??
	if (obj->m_options.random_seed)
		getRandomGenerator().randomize(obj->m_options.random_seed);
	else
		getRandomGenerator().randomize();

	try
	{
		// Start thread:
		// -------------------------
		obj->logFmt(
			mrpt::system::LVL_DEBUG,
			"[thread_TBI] Thread started (ID=0x%08lX)\n",
			std::this_thread::get_id());

		// --------------------------------------------
		//    The main loop
		//  Executes until termination is signaled
		// --------------------------------------------
		while (!obj->m_terminateThreads)
		{
			std::this_thread::sleep_for(100ms);
		};  // end while execute thread

		// Finish thread:
		// -------------------------
		MRPT_TODO("Fix thread times");
		//		try { mrpt::system::getCurrentThreadTimes(
		// timCreat,timExit,timCPU); } catch(...) {};
		obj->logFmt(mrpt::system::LVL_DEBUG, "[thread_TBI] Thread finished");
		obj->m_terminationFlag_TBI = true;
	}
	catch (const std::exception& e)
	{
		obj->m_terminationFlag_TBI = true;

		// Release semaphores:

		obj->logFmt(mrpt::system::LVL_ERROR, "%s", e.what());

		// DEBUG: Terminate application:
		obj->m_terminateThreads = true;
	}
	catch (...)
	{
		obj->m_terminationFlag_TBI = true;

		obj->logFmt(
			mrpt::system::LVL_ERROR,
			"\n---------------------- EXCEPTION CAUGHT! ---------------------\n"
			" In CHierarchicalMappingFramework::thread_TBI. Unexpected runtime "
			"error!!\n");

		// Release semaphores:

		// DEBUG: Terminate application:
		obj->m_terminateThreads = true;
	}
}

/*---------------------------------------------------------------

						TBI_main_method

	Topological Bayesian Inference (TBI) process within HMT-SLAM

  ---------------------------------------------------------------*/
CHMTSLAM::TMessageLSLAMfromTBI::Ptr CHMTSLAM::TBI_main_method(
	CLocalMetricHypothesis* LMH, const CHMHMapNode::TNodeID& areaID)
{
	MRPT_START

	auto* obj = (CHMTSLAM*)LMH->m_parent.get();

	const THypothesisID LMH_ID = LMH->m_ID;

	// Lock the map:
	std::lock_guard<std::mutex>(obj->m_map_cs);

	TMessageLSLAMfromTBI::Ptr msg = std::make_shared<TMessageLSLAMfromTBI>();

	// Fill out easy data:
	msg->hypothesisID = LMH_ID;
	msg->cur_area = areaID;

	// get a pointer to the current area:
	const CHMHMapNode::Ptr currentArea = obj->m_map.getNodeByID(areaID);
	ASSERT_(currentArea);

	obj->logFmt(
		mrpt::system::LVL_DEBUG, "[TBI] Request for area id=%i\n", (int)areaID);

	// --------------------------------------------------------
	// 1) Use bounding-boxes to get a first list of candidates
	//    The candidates are saved in "msg->loopClosureData"
	// -------------------------------------------------------
	// But first: if the areas are within the LMH, then we have to update the
	// maps in the HMAP!
	if (LMH->m_neighbors.find(areaID) != LMH->m_neighbors.end())
	{
		// Update:
		LMH->updateAreaFromLMH(areaID);
	}

	for (auto a = obj->m_map.begin(); a != obj->m_map.end(); ++a)
	{
		// Only for other areas!
		if (a->first == areaID) continue;

		// Test hypothesis: LMH_ID
		if (a->second->m_hypotheses.has(LMH_ID))
		{
			// Not neighbors:
			if (a->second->isNeighbor(areaID, LMH_ID)) continue;

			// OK, check:
			// But first: if the areas are within the LMH, then we have to
			// update the maps in the HMAP!
			if (LMH->m_neighbors.find(a->first) != LMH->m_neighbors.end())
			{
				// Update:
				LMH->updateAreaFromLMH(a->first);
			}

			// Compute it:
			double match = obj->m_map.computeOverlapProbabilityBetweenNodes(
				areaID,  // From
				a->first,  // To
				LMH_ID);

			obj->logFmt(
				mrpt::system::LVL_DEBUG, "[TBI] %i-%i -> overlap prob=%f\n",
				(int)areaID, (int)a->first, match);

			if (match > 0.9)
			{
				// Initialize the new entry in "msg->loopClosureData" for the
				// areas:
				//  "areaID" <-> "a->first"
				TMessageLSLAMfromTBI::TBI_info& tbi_info =
					msg->loopClosureData[a->first];

				tbi_info.log_lik = 0;
				tbi_info.delta_new_cur.clear();
			}
		}
	}  // end for each node in the graph.

	// ----------------------------------------------------
	// 2) Use the TBI engines
	// ----------------------------------------------------
	std::set<CHMHMapNode::TNodeID> lstNodesToErase;
	{
		std::lock_guard<std::mutex> lock(obj->m_topLCdets_cs);

		for (auto it = obj->m_topLCdets.begin(); it != obj->m_topLCdets.end();
			 ++it)
		{
			for (auto& candidate : msg->loopClosureData)
			{
				// If the current log_lik of this area is reaaaally low, we
				// could skip the computation with other LC detectors...
				// ----------------------------------------------------------------------------------------------------------------
				// TODO: ...

				// Proceed:
				// ----------------------------------------------------------------------------------------------------------------
				const CHMHMapNode::Ptr refArea =
					obj->m_map.getNodeByID(candidate.first);
				double this_log_lik;

				// get the output from this LC detector:
				CPose3DPDF::Ptr pdf = (*it)->computeTopologicalObservationModel(
					LMH->m_ID, currentArea, refArea, this_log_lik);

				// Add to the output:
				candidate.second.log_lik += this_log_lik;

				// This is because not all LC detector MUST return a pose PDF
				// (i.e. image-based detectors)
				if (pdf)
				{
					ASSERT_(IS_CLASS(pdf, CPose3DPDFSOG));
					CPose3DPDFSOG::Ptr SOG =
						std::dynamic_pointer_cast<CPose3DPDFSOG>(pdf);

					// Mix (append) the modes, if any:
					if (SOG->size() > 0)
						candidate.second.delta_new_cur.appendFrom(*SOG);
					else
						lstNodesToErase.insert(candidate.first);
				}
			}  // end for each candidate area
		}  // end for each LC detector

	}  // end of m_topLCdets_cs lock

	// Delete candidates which had no PDF when they should.
	for (unsigned long it : lstNodesToErase) msg->loopClosureData.erase(it);

	obj->logFmt(
		mrpt::system::LVL_DEBUG, "[TBI_main] Done. %u candidates found.\n",
		(unsigned int)msg->loopClosureData.size());

	return msg;
	MRPT_END
}
