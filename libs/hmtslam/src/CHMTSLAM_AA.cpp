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
#include <mrpt/random.h>
#include <mrpt/utils/CFileStream.h>
#include <mrpt/utils/printf_vector.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;

/*---------------------------------------------------------------

						areaAbstraction

	Area Abstraction (AA) process within HMT-SLAM

  ---------------------------------------------------------------*/
CHMTSLAM::TMessageLSLAMfromAAPtr CHMTSLAM::areaAbstraction(
	CLocalMetricHypothesis	*LMH,
	const TPoseIDList		&newPoseIDs )
{
	MRPT_START

	ASSERT_( !newPoseIDs.empty() );
	ASSERT_(LMH);
	CHMTSLAM *obj = LMH->m_parent.get();
	ASSERT_(obj);

	// The output results:
	TMessageLSLAMfromAAPtr resMsg = TMessageLSLAMfromAAPtr(new TMessageLSLAMfromAA());


	// Process msg:
	THypothesisID			LMH_ID = LMH->m_ID;
	resMsg->hypothesisID = LMH_ID;

	for (TPoseIDList::const_iterator newID=newPoseIDs.begin();newID!=newPoseIDs.end();++newID)
	{
		// Add a new node to the graph:
		obj->logFmt(mrpt::utils::LVL_DEBUG, "[thread_AA] Processing new pose ID: %u\n", static_cast<unsigned>( *newID ) );

		// Get SF & pose pdf for the new pose.
		const CSensoryFrame			*sf;
		CPose3DPDFParticlesPtr		posePDF = CPose3DPDFParticles::Create();

		{
			// CCriticalSectionLocker	lock( & LMH->m_lock ); // We are already within the LMH's lock!

			// SF:
			std::map<TPoseID,CSensoryFrame>::const_iterator itSFs = LMH->m_SFs.find( *newID );
			ASSERT_(itSFs != LMH->m_SFs.end() );
			sf = &itSFs->second;

			// Pose PDF:
			LMH->getPoseParticles( *newID, *posePDF );
		} // end of LMH's critical section lock

		{
			synch::CCriticalSectionLocker locker ( &LMH->m_robotPosesGraph.lock );

			// Add to the graph partitioner:
			LMH->m_robotPosesGraph.partitioner.options = obj->m_options.AA_options;

			unsigned int newIdx = LMH->m_robotPosesGraph.partitioner.addMapFrame(
				CSensoryFramePtr(new CSensoryFrame(*sf)),
				posePDF );
			LMH->m_robotPosesGraph.idx2pose[ newIdx ] = *newID;
		} // end of critical section lock on "m_robotPosesGraph.lock"
	} // end for each new ID

	vector< vector_uint >   partitions;
	{
		synch::CCriticalSectionLocker locker ( &LMH->m_robotPosesGraph.lock );
		// Recompute partitions:
		LMH->m_robotPosesGraph.partitioner.markAllNodesForReconsideration(); // We will have always small local maps.
		LMH->m_robotPosesGraph.partitioner.updatePartitions( partitions );
	}

	// Send result to LSLAM
	resMsg->partitions.resize( partitions.size() );
	vector< TPoseIDList >::iterator  itDest;
	vector< vector_uint >::const_iterator itSrc;
	for (itDest = resMsg->partitions.begin(), itSrc = partitions.begin(); itSrc!=partitions.end(); itSrc++, itDest++)
	{
		itDest->resize( itSrc->size() );
		vector_uint::const_iterator   it1;
		TPoseIDList::iterator	  it2;
		for (it1=itSrc->begin(), it2=itDest->begin(); it1!=itSrc->end(); it1++,it2++)
			*it2 = LMH->m_robotPosesGraph.idx2pose[ *it1 ];
	}

	resMsg->dumpToConsole( );

	return resMsg;

	MRPT_END
}



void CHMTSLAM::TMessageLSLAMfromAA::dumpToConsole( ) const
{
	cout << format("Hypo ID: %i has %i partitions:\n",(int)hypothesisID,(int)partitions.size());

	for (std::vector< TPoseIDList >::const_iterator it=partitions.begin();it!=partitions.end();++it)
	{
		printf_vector( "%i",*it);
		cout << endl;
	}
}
