/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/** An implementation of Hybrid Metric Topological SLAM (HMT-SLAM).
 *
 *   The main entry points for a user are pushAction() and pushObservations().
 *Several parameters can be modified
 *   through m_options.
 *
 *  The mathematical models of this approach have been reported in:
 *		- Blanco J.L., Fernandez-Madrigal J.A., and Gonzalez J.,
 *		  "Towards a Unified Bayesian Approach to Hybrid Metric-Topological
 *SLAM",
 *		    in  IEEE Transactions on Robotics (TRO), Vol. 24, No. 2, April 2008.
 *		- ...
 *
 *  More information in the wiki page: http://www.mrpt.org/HMT-SLAM
 *
 */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/io/CFileStream.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/io/CMemoryStream.h>

#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CHMTSLAM, CSerializable, mrpt::hmtslam)

// Initialization of static members:
int64_t CHMTSLAM::m_nextAreaLabel = 0;
TPoseID CHMTSLAM::m_nextPoseID = 0;
THypothesisID CHMTSLAM::m_nextHypID = COMMON_TOPOLOG_HYP + 1;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHMTSLAM::CHMTSLAM()
{
	// Initialize data structures:
	// ----------------------------
	m_terminateThreads = false;
	m_terminationFlag_LSLAM = m_terminationFlag_TBI =
		m_terminationFlag_3D_viewer = false;

	// Create threads:
	// -----------------------
	m_hThread_LSLAM = std::thread(&CHMTSLAM::thread_LSLAM, this);
	m_hThread_TBI = std::thread(&CHMTSLAM::thread_TBI, this);
	m_hThread_3D_viewer = std::thread(&CHMTSLAM::thread_3D_viewer, this);

	// Other variables:
	m_LSLAM_method = nullptr;

	// Register default LC detectors:
	// --------------------------------
	registerLoopClosureDetector(
		"gridmaps", &CTopLCDetector_GridMatching::createNewInstance);
	registerLoopClosureDetector(
		"fabmap", &CTopLCDetector_FabMap::createNewInstance);

	// Prepare an empty map:
	initializeEmptyMap();
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CHMTSLAM::~CHMTSLAM()
{
	// Signal to threads that we are closing:
	// -------------------------------------------
	m_terminateThreads = true;

	// Wait for threads:
	// ----------------------------------
	MRPT_LOG_DEBUG("[CHMTSLAM::destructor] Waiting for threads end...\n");

	m_hThread_3D_viewer.join();
	m_hThread_LSLAM.join();
	m_hThread_TBI.join();

	MRPT_LOG_DEBUG("[CHMTSLAM::destructor] All threads finished.\n");

	// Save the resulting H.Map if logging
	// --------------------------------------
	if (!m_options.LOG_OUTPUT_DIR.empty())
	{
		try
		{
			/*			// Update the last area(s) in the HMAP:
						updateHierarchicalMapFromRBPF();

						// Save:
						os::sprintf(auxFil,1000,"%s/hierarchicalMap.hmap",m_options.logOutputDirectory.c_str());

						CFileStream		f(auxFil,fomWrite);
						f << m_knownAreas;
						*/
		}
		catch (std::exception& e)
		{
			MRPT_LOG_WARN(
				mrpt::format(
					"Ignoring exception at ~CHMTSLAM():\n%s", e.what()));
		}
		catch (...)
		{
			MRPT_LOG_WARN("Ignoring untyped exception at ~CHMTSLAM()");
		}
	}

	// Delete data structures:
	// ----------------------------------
	clearInputQueue();

	// Others:
	if (m_LSLAM_method)
	{
		delete m_LSLAM_method;
		m_LSLAM_method = nullptr;
	}

	// Delete TLC-detectors
	{
		std::lock_guard<std::mutex> lock(m_topLCdets_cs);

		// Clear old list:
		for (std::deque<CTopLCDetectorBase*>::iterator it = m_topLCdets.begin();
			 it != m_topLCdets.end(); ++it)
			delete *it;
		m_topLCdets.clear();
	}
}

/*---------------------------------------------------------------
						clearInputQueue
  ---------------------------------------------------------------*/
void CHMTSLAM::clearInputQueue()
{
	// Wait for critical section
	{
		std::lock_guard<std::mutex> lock(m_inputQueue_cs);

		while (!m_inputQueue.empty())
		{
			// delete m_inputQueue.front();
			m_inputQueue.pop();
		};
	}
}

/*---------------------------------------------------------------
						pushAction
  ---------------------------------------------------------------*/
void CHMTSLAM::pushAction(const CActionCollection::Ptr& acts)
{
	if (m_terminateThreads)
	{
		// Discard it:
		// delete acts;
		return;
	}

	{  // Wait for critical section
		std::lock_guard<std::mutex> lock(m_inputQueue_cs);
		m_inputQueue.push(acts);
	}
}

/*---------------------------------------------------------------
						pushObservations
  ---------------------------------------------------------------*/
void CHMTSLAM::pushObservations(const CSensoryFrame::Ptr& sf)
{
	if (m_terminateThreads)
	{
		// Discard it:
		// delete sf;
		return;
	}

	{  // Wait for critical section
		std::lock_guard<std::mutex> lock(m_inputQueue_cs);
		m_inputQueue.push(sf);
	}
}

/*---------------------------------------------------------------
						pushObservation
  ---------------------------------------------------------------*/
void CHMTSLAM::pushObservation(const CObservation::Ptr& obs)
{
	if (m_terminateThreads)
	{  // Discard it:
		// delete obs;
		return;
	}

	// Add a CSensoryFrame with the obs:
	CSensoryFrame::Ptr sf = mrpt::make_aligned_shared<CSensoryFrame>();
	sf->insert(
		obs);  // memory will be freed when deleting the SF in other thread

	{  // Wait for critical section
		std::lock_guard<std::mutex> lock(m_inputQueue_cs);
		m_inputQueue.push(sf);
	}
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CHMTSLAM::loadOptions(const mrpt::config::CConfigFileBase& cfg)
{
	m_options.loadFromConfigFile(cfg, "HMT-SLAM");

	m_options.defaultMapsInitializers.loadFromConfigFile(cfg, "MetricMaps");

	m_options.pf_options.loadFromConfigFile(cfg, "PARTICLE_FILTER");

	m_options.KLD_params.loadFromConfigFile(cfg, "KLD");

	m_options.AA_options.loadFromConfigFile(cfg, "GRAPH_CUT");

	// Topological Loop Closure detector options:
	m_options.TLC_grid_options.loadFromConfigFile(cfg, "TLC_GRIDMATCHING");
	m_options.TLC_fabmap_options.loadFromConfigFile(cfg, "TLC_FABMAP");

	m_options.dumpToConsole();
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CHMTSLAM::loadOptions(const std::string& configFile)
{
	ASSERT_(mrpt::system::fileExists(configFile));
	CConfigFile cfg(configFile);
	loadOptions(cfg);
}

/*---------------------------------------------------------------
						TOptions
  ---------------------------------------------------------------*/
CHMTSLAM::TOptions::TOptions()
{
	LOG_OUTPUT_DIR = "";
	LOG_FREQUENCY = 1;

	SLAM_METHOD = lsmRBPF_2DLASER;

	SLAM_MIN_DIST_BETWEEN_OBS = 1.0f;
	SLAM_MIN_HEADING_BETWEEN_OBS = DEG2RAD(25.0f);

	MIN_ODOMETRY_STD_XY = 0;
	MIN_ODOMETRY_STD_PHI = 0;

	VIEW3D_AREA_SPHERES_HEIGHT = 10.0f;
	VIEW3D_AREA_SPHERES_RADIUS = 1.0f;

	random_seed = 1234;

	TLC_detectors.clear();

	stds_Q_no_odo.resize(3);
	stds_Q_no_odo[0] = stds_Q_no_odo[1] = 0.10f;
	stds_Q_no_odo[2] = DEG2RAD(4.0f);
}

/*---------------------------------------------------------------
						loadFromConfigFile
  ---------------------------------------------------------------*/
void CHMTSLAM::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_LOAD_CONFIG_VAR(LOG_OUTPUT_DIR, string, source, section);
	MRPT_LOAD_CONFIG_VAR(LOG_FREQUENCY, int, source, section);

	MRPT_LOAD_CONFIG_VAR_CAST_NO_DEFAULT(
		SLAM_METHOD, int, TLSlamMethod, source, section);

	MRPT_LOAD_CONFIG_VAR(SLAM_MIN_DIST_BETWEEN_OBS, float, source, section);
	MRPT_LOAD_CONFIG_VAR_DEGREES(SLAM_MIN_HEADING_BETWEEN_OBS, source, section);

	MRPT_LOAD_CONFIG_VAR(MIN_ODOMETRY_STD_XY, float, source, section);
	MRPT_LOAD_CONFIG_VAR_DEGREES(MIN_ODOMETRY_STD_PHI, source, section);

	MRPT_LOAD_CONFIG_VAR(VIEW3D_AREA_SPHERES_HEIGHT, float, source, section);
	MRPT_LOAD_CONFIG_VAR(VIEW3D_AREA_SPHERES_RADIUS, float, source, section);

	MRPT_LOAD_CONFIG_VAR(random_seed, int, source, section);

	stds_Q_no_odo[2] = RAD2DEG(stds_Q_no_odo[2]);
	source.read_vector(section, "stds_Q_no_odo", stds_Q_no_odo, stds_Q_no_odo);
	ASSERT_(stds_Q_no_odo.size() == 3)

	stds_Q_no_odo[2] = DEG2RAD(stds_Q_no_odo[2]);

	std::string sTLC_detectors =
		source.read_string(section, "TLC_detectors", "", true);

	mrpt::system::tokenize(sTLC_detectors, ", ", TLC_detectors);

	std::cout << "TLC_detectors: " << TLC_detectors.size() << std::endl;

	// load other sub-classes:
	AA_options.loadFromConfigFile(source, section);
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void CHMTSLAM::TOptions::dumpToTextStreamstd::ostream& out, int* version) const
{
	if (version)
		*version = 0;
	else
	{
		// Acquire all critical sections before!
		// -------------------------------------------
		// std::map< THypothesisID, CLocalMetricHypothesis >::const_iterator it;

		// std::lock_guard<std::mutex> LMHs( & m_LMHs_cs );
		// for (it=m_LMHs.begin();it!=m_LMHs.end();it++)
		// it->second.m_lock.lock();

		std::lock_guard<std::mutex> lock_map(m_map_cs);

		// Data:
		out << m_nextAreaLabel << m_nextPoseID << m_nextHypID;

		// The HMT-MAP:
		out << m_map;

		// The LMHs:
		out << m_LMHs;

		// Save options??? Better allow changing them...

		// Release all critical sections:
		// for (it=m_LMHs.begin();it!=m_LMHs.end();it++)
		// it->second.m_lock.lock();
	}
}
