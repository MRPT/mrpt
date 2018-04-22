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
#include <mrpt/containers/printf_vector.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace std;

/*---------------------------------------------------------------

						areaAbstraction

	Area Abstraction (AA) process within HMT-SLAM

  ---------------------------------------------------------------*/
CHMTSLAM::TMessageLSLAMfromAA::Ptr CHMTSLAM::areaAbstraction(
	CLocalMetricHypothesis* LMH, const TPoseIDList& newPoseIDs)
{
	MRPT_START

	ASSERT_(!newPoseIDs.empty());
	ASSERT_(LMH);
	CHMTSLAM* obj = LMH->m_parent.get();
	ASSERT_(obj);

	// The output results:
	TMessageLSLAMfromAA::Ptr resMsg =
		TMessageLSLAMfromAA::Ptr(new TMessageLSLAMfromAA());

	// Process msg:
	THypothesisID LMH_ID = LMH->m_ID;
	resMsg->hypothesisID = LMH_ID;

	for (TPoseIDList::const_iterator newID = newPoseIDs.begin();
		 newID != newPoseIDs.end(); ++newID)
	{
		// Add a new node to the graph:
		obj->logFmt(
			mrpt::system::LVL_DEBUG, "[thread_AA] Processing new pose ID: %u\n",
			static_cast<unsigned>(*newID));

		// Get SF & pose pdf for the new pose.
		const CSensoryFrame* sf;
		CPose3DPDFParticles::Ptr posePDF =
			mrpt::make_aligned_shared<CPose3DPDFParticles>();

		{
			// std::lock_guard<std::mutex>	lock( LMH->m_lock ); // We are
			// already within the LMH's lock!

			// SF:
			std::map<TPoseID, CSensoryFrame>::const_iterator itSFs =
				LMH->m_SFs.find(*newID);
			ASSERT_(itSFs != LMH->m_SFs.end());
			sf = &itSFs->second;

			// Pose PDF:
			LMH->getPoseParticles(*newID, *posePDF);
		}  // end of LMH's critical section lock

		{
			std::lock_guard<std::mutex> locker(LMH->m_robotPosesGraph.lock);

			// Add to the graph partitioner:
			LMH->m_robotPosesGraph.partitioner.options =
				obj->m_options.AA_options;

			unsigned int newIdx =
				LMH->m_robotPosesGraph.partitioner.addMapFrame(
					*sf, *posePDF);
			LMH->m_robotPosesGraph.idx2pose[newIdx] = *newID;
		}  // end of critical section lock on "m_robotPosesGraph.lock"
	}  // end for each new ID

	vector<std::vector<uint32_t>> partitions;
	{
		std::lock_guard<std::mutex> locker(LMH->m_robotPosesGraph.lock);
		LMH->m_robotPosesGraph.partitioner.updatePartitions(partitions);
	}

	// Send result to LSLAM
	resMsg->partitions.resize(partitions.size());
	vector<TPoseIDList>::iterator itDest;
	vector<std::vector<uint32_t>>::const_iterator itSrc;
	for (itDest = resMsg->partitions.begin(), itSrc = partitions.begin();
		 itSrc != partitions.end(); itSrc++, itDest++)
	{
		itDest->resize(itSrc->size());
		std::vector<uint32_t>::const_iterator it1;
		TPoseIDList::iterator it2;
		for (it1 = itSrc->begin(), it2 = itDest->begin(); it1 != itSrc->end();
			 it1++, it2++)
			*it2 = LMH->m_robotPosesGraph.idx2pose[*it1];
	}

	resMsg->dumpToConsole();

	return resMsg;

	MRPT_END
}

void CHMTSLAM::TMessageLSLAMfromAA::dumpToConsole() const
{
	cout << format(
		"Hypo ID: %i has %i partitions:\n", (int)hypothesisID,
		(int)partitions.size());

	for (std::vector<TPoseIDList>::const_iterator it = partitions.begin();
		 it != partitions.end(); ++it)
	{
		mrpt::containers::printf_vector("%i", *it);
		cout << endl;
	}
}
