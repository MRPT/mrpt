/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/system/os.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------

				CHMTSLAM_LOG

	Implements a 2D local SLAM method based on scan matching
	  between near observations and an EKF. A part of HMT-SLAM

\param LMH   The local metric hypothesis which must be updated by this SLAM
algorithm.
\param act   The action to process (or nullptr).
\param sf    The observations to process (or nullptr).

--------------------------------------------------------------- */
void CHMTSLAM::generateLogFiles(unsigned int nIteration)
{
	MRPT_START

	// Speed up the storage of images (in opengl::CTexturedPlane's):
	// CImage::DISABLE_ZIP_COMPRESSION = true;

	static CTicTac tictac;

	tictac.Tic();

	THypothesisID bestHypoID;
	CLocalMetricHypothesis* bestLMH = nullptr;
	{
		std::lock_guard<std::mutex> lock(m_LMHs_cs);

		MRPT_LOG_INFO_STREAM("[LOG] Number of LMHs: " << m_LMHs.size());

		// Generate 3D view of local areas:
		{
			string filLocalAreas = format(
				"%s/LSLAM_3D/mostLikelyLMH_LSLAM_%05u.3Dscene",
				m_options.LOG_OUTPUT_DIR.c_str(), nIteration);
			COpenGLScene::Ptr sceneLSLAM =
				mrpt::make_aligned_shared<COpenGLScene>();

			// Look for the most likely LMH:
			for (auto& m : m_LMHs)
			{
				if (!bestLMH)
				{
					bestLMH = &m.second;
				}
				else if (m.second.m_log_w > bestLMH->m_log_w)
				{
					bestLMH = &m.second;
				}
			}
			ASSERT_(bestLMH != nullptr);

			bestHypoID = bestLMH->m_ID;

			{
				std::lock_guard<std::mutex> lockerLMH(
					bestLMH->threadLocks.m_lock);

				{
					// Generate the metric maps 3D view...
					opengl::CSetOfObjects::Ptr maps3D =
						mrpt::make_aligned_shared<opengl::CSetOfObjects>();
					maps3D->setName("metric-maps");
					bestLMH->getMostLikelyParticle()
						->d->metricMaps.getAs3DObject(maps3D);
					sceneLSLAM->insert(maps3D);

					// ...and the robot poses, areas, etc:
					opengl::CSetOfObjects::Ptr LSLAM_3D =
						mrpt::make_aligned_shared<opengl::CSetOfObjects>();
					LSLAM_3D->setName("LSLAM_3D");
					bestLMH->getAs3DScene(LSLAM_3D);
					sceneLSLAM->insert(LSLAM_3D);

					sceneLSLAM->enableFollowCamera(true);

					MRPT_LOG_INFO_STREAM("[LOG] Saving " << filLocalAreas);
					CFileGZOutputStream f(filLocalAreas);
					archiveFrom(f) << *sceneLSLAM;
				}

// Save the SSO matrix:
#if 0
				{
					std::lock_guard<std::mutex>  lock(bestLMH->m_robotPosesGraph.lock );
					string filSSO = format("%s/ASSO/mostLikelyLMH_ASSO_%05u.3Dscene", m_options.LOG_OUTPUT_DIR.c_str(), nIteration );
					COpenGLScene	sceneSSO;
					opengl::CSetOfObjects::Ptr sso3D = mrpt::make_aligned_shared<opengl::CSetOfObjects>();
					bestLMH->m_robotPosesGraph.partitioner.getAs3DScene( sso3D, &bestLMH->m_robotPosesGraph.idx2pose );
					sceneSSO.insert(sso3D);
					CFileGZOutputStream(filSSO) << sceneSSO;

					if (1)
					{
						CMatrix  A;
						bestLMH->m_robotPosesGraph.partitioner.getAdjacencyMatrix( A );
						if (A.cols()>0)
						{
							A.adjustRange();
							A.saveToTextFile( format("%s/ASSO/mostLikelyLMH_ASSO_%05u.txt", m_options.LOG_OUTPUT_DIR.c_str(), nIteration ) );
							CImage(A,true).saveToFile( format("%s/ASSO/mostLikelyLMH_ASSO_%05u.png", m_options.LOG_OUTPUT_DIR.c_str(), nIteration ) );
						}
					}
				} // end lock partitioner's CS
#endif

			}  // end LMH's lock
		}

	}  // end of lock on LMHs_cs

#if 1
	{
		// Save the whole HMT-SLAM state to a dump file
		static int CNT = 0;
		if ((CNT++ % 20) == 0)
		{
			string hmtmap_file(format(
				"%s/HMTSLAM_state/state_%05u.hmtslam",
				m_options.LOG_OUTPUT_DIR.c_str(), nIteration));
			MRPT_LOG_INFO_STREAM("[LOG] Saving: " << hmtmap_file.c_str());
			CFileGZOutputStream f(hmtmap_file);
			archiveFrom(f) << *this;
		}
	}
#endif

#if 1
	{
		// Update the poses-graph in the HMT-map from the LMH to draw it:
		static int CNT = 0;
		if ((CNT++ % 5) == 0)
		{
			std::lock_guard<std::mutex> lockerLMH(bestLMH->threadLocks.m_lock);

			for (auto n = bestLMH->m_neighbors.begin();
				 n != bestLMH->m_neighbors.end(); ++n)
				bestLMH->updateAreaFromLMH(*n);

			// Save global map for most likely hypothesis:
			COpenGLScene sceneGlobalHMTMAP;
			{
				std::lock_guard<std::mutex> lock(m_map_cs);
				MRPT_LOG_INFO_STREAM(
					"[LOG] HMT-map: " << m_map.nodeCount() << " nodes/ "
									  << m_map.arcCount() << " arcs");

				m_map.getAs3DScene(
					sceneGlobalHMTMAP,  // Scene
					m_map.getFirstNode()->getID(),  // Reference node
					bestHypoID,  // Hypothesis to get
					3  // iterations
				);
			}

			string hmtmap_file(format(
				"%s/HMAP_3D/mostLikelyHMT_MAP_%05u.3Dscene",
				m_options.LOG_OUTPUT_DIR.c_str(), nIteration));
			MRPT_LOG_INFO_STREAM("[LOG] Saving " << hmtmap_file);
			CFileGZOutputStream f(hmtmap_file);
			archiveFrom(f) << sceneGlobalHMTMAP;
		}
	}
#endif

	// Save the memory usage:
	unsigned long memUsage = mrpt::system::getMemoryUsage();

	FILE* f = os::fopen(
		format("%s/log_MemoryUsage.txt", m_options.LOG_OUTPUT_DIR.c_str())
			.c_str(),
		"at");
	if (f)
	{
		os::fprintf(f, "%u\t%f\n", nIteration, memUsage / (1024.0 * 1024.0));
		os::fclose(f);
	}

	double t_log = tictac.Tac();
	MRPT_LOG_INFO_STREAM(
		"[LOG] Time for logging: " << mrpt::system::formatTimeInterval(t_log));

	MRPT_END
}
