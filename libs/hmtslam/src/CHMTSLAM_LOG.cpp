/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/system/os.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------

				CHMTSLAM_LOG

	Implements a 2D local SLAM method based on scan matching
	  between near observations and an EKF. A part of HMT-SLAM

\param LMH   The local metric hypothesis which must be updated by this SLAM algorithm.
\param act   The action to process (or NULL).
\param sf    The observations to process (or NULL).

--------------------------------------------------------------- */
void CHMTSLAM::generateLogFiles(unsigned int nIteration)
{
	MRPT_START

	// Speed up the storage of images (in opengl::CTexturedPlane's):
	//CImage::DISABLE_ZIP_COMPRESSION = true;

	static CTicTac	tictac;

	tictac.Tic();

	THypothesisID	bestHypoID;
	CLocalMetricHypothesis  *bestLMH = NULL;
	{
		CCriticalSectionLocker  locker( &m_LMHs_cs );

		MRPT_LOG_INFO_STREAM << "[LOG] Number of LMHs: " <<  m_LMHs.size();

		// Generate 3D view of local areas:
		{
			string filLocalAreas = format("%s/LSLAM_3D/mostLikelyLMH_LSLAM_%05u.3Dscene", m_options.LOG_OUTPUT_DIR.c_str(), nIteration );
			COpenGLScenePtr	sceneLSLAM = COpenGLScene::Create();

			// Look for the most likely LMH:
			aligned_containers<THypothesisID, CLocalMetricHypothesis>::map_t::iterator  it;
			for ( it = m_LMHs.begin();it!=m_LMHs.end();it++)
			{
				if (!bestLMH)
				{
					bestLMH = & it->second;
				}
				else if ( it->second.m_log_w > bestLMH->m_log_w)
				{
					bestLMH = & it->second;
				}
			}
			ASSERT_(bestLMH!=NULL)

			bestHypoID = bestLMH->m_ID;

			{
				CCriticalSectionLocker  lockerLMH( &bestLMH->m_lock );

				{
					// Generate the metric maps 3D view...
					opengl::CSetOfObjectsPtr maps3D = opengl::CSetOfObjects::Create();
					maps3D->setName("metric-maps");
					bestLMH->getMostLikelyParticle()->d->metricMaps.getAs3DObject( maps3D );
					sceneLSLAM->insert( maps3D );

					// ...and the robot poses, areas, etc:
					opengl::CSetOfObjectsPtr LSLAM_3D = opengl::CSetOfObjects::Create();
					LSLAM_3D->setName("LSLAM_3D");
					bestLMH->getAs3DScene( LSLAM_3D );
					sceneLSLAM->insert( LSLAM_3D );

					sceneLSLAM->enableFollowCamera(true);

					MRPT_LOG_INFO_STREAM << "[LOG] Saving " << filLocalAreas;
					CFileGZOutputStream(filLocalAreas) << *sceneLSLAM;
				}


				// Save the SSO matrix:
#if 0
				{
					CCriticalSectionLocker  locker( &bestLMH->m_robotPosesGraph.lock );
					string filSSO = format("%s/ASSO/mostLikelyLMH_ASSO_%05u.3Dscene", m_options.LOG_OUTPUT_DIR.c_str(), nIteration );
					COpenGLScene	sceneSSO;
					opengl::CSetOfObjectsPtr sso3D = opengl::CSetOfObjects::Create();
					bestLMH->m_robotPosesGraph.partitioner.getAs3DScene( sso3D, &bestLMH->m_robotPosesGraph.idx2pose );
					sceneSSO.insert(sso3D);
					CFileGZOutputStream(filSSO) << sceneSSO;

					if (1)
					{
						CMatrix  A;
						bestLMH->m_robotPosesGraph.partitioner.getAdjacencyMatrix( A );
						if (A.getColCount()>0)
						{
							A.adjustRange();
							A.saveToTextFile( format("%s/ASSO/mostLikelyLMH_ASSO_%05u.txt", m_options.LOG_OUTPUT_DIR.c_str(), nIteration ) );
							CImage(A,true).saveToFile( format("%s/ASSO/mostLikelyLMH_ASSO_%05u.png", m_options.LOG_OUTPUT_DIR.c_str(), nIteration ) );
						}
					}
				} // end lock partitioner's CS
#endif

			} // end LMH's lock

		}

	}  // end of lock on LMHs_cs

#if 1
	{
		// Save the whole HMT-SLAM state to a dump file
		static int CNT = 0;
		if ((CNT++ % 20) == 0)
		{
			string hmtmap_file( format("%s/HMTSLAM_state/state_%05u.hmtslam", m_options.LOG_OUTPUT_DIR.c_str(), nIteration ) );
			MRPT_LOG_INFO_STREAM << "[LOG] Saving: " << hmtmap_file.c_str();
			CFileGZOutputStream(hmtmap_file) << *this;
		}
	}
#endif


#if 1
	{
		// Update the poses-graph in the HMT-map from the LMH to draw it:
		static int CNT = 0;
		if ((CNT++ % 5) == 0)
		{
			CCriticalSectionLocker  lockerLMH( &bestLMH->m_lock );

			for (TNodeIDSet::const_iterator n = bestLMH->m_neighbors.begin();n!=bestLMH->m_neighbors.end();++n)
				bestLMH->updateAreaFromLMH( *n );

			// Save global map for most likely hypothesis:
			COpenGLScene	sceneGlobalHMTMAP;
			{
				CCriticalSectionLocker  locker( &m_map_cs );
				MRPT_LOG_INFO_STREAM << "[LOG] HMT-map: "<< m_map.nodeCount() << " nodes/ "<< m_map.arcCount() << " arcs";

				m_map.getAs3DScene(
					sceneGlobalHMTMAP,					// Scene
					m_map.getFirstNode()->getID(),		// Reference node
					bestHypoID,							// Hypothesis to get
					3									// iterations
					);
			}

			string hmtmap_file( format("%s/HMAP_3D/mostLikelyHMT_MAP_%05u.3Dscene", m_options.LOG_OUTPUT_DIR.c_str(), nIteration ) );
			MRPT_LOG_INFO_STREAM << "[LOG] Saving "<< hmtmap_file;
			CFileGZOutputStream(hmtmap_file) << sceneGlobalHMTMAP;
		}
	}
#endif


	// Save the memory usage:
	unsigned long memUsage = mrpt::system::getMemoryUsage();

	FILE		*f=os::fopen( format("%s/log_MemoryUsage.txt",m_options.LOG_OUTPUT_DIR.c_str()).c_str() ,"at");
	if (f)
	{
		os::fprintf(f,"%u\t%f\n",nIteration,memUsage/(1024.0*1024.0));
		os::fclose(f);
	}

	double t_log = tictac.Tac();
	MRPT_LOG_INFO_STREAM << "[LOG] Time for logging: " << mrpt::system::formatTimeInterval(t_log);

	MRPT_END
}

