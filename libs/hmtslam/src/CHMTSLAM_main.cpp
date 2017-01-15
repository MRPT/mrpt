/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** An implementation of Hybrid Metric Topological SLAM (HMT-SLAM).
 *
 *   The main entry points for a user are pushAction() and pushObservations(). Several parameters can be modified
 *   through m_options.
 *
 *  The mathematical models of this approach have been reported in:
 *		- Blanco J.L., Fernandez-Madrigal J.A., and Gonzalez J.,
 *		  "Towards a Unified Bayesian Approach to Hybrid Metric-Topological SLAM",
 *		    in  IEEE Transactions on Robotics (TRO), Vol. 24, No. 2, April 2008.
 *		- ...
 *
 *  More information in the wiki page: http://www.mrpt.org/HMT-SLAM
 *
 */


#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/utils/CFileStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/CMemoryStream.h>

#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace std;


IMPLEMENTS_SERIALIZABLE(CHMTSLAM, CSerializable,mrpt::hmtslam)


// Initialization of static members:
int64_t			CHMTSLAM::m_nextAreaLabel = 0;
TPoseID 		CHMTSLAM::m_nextPoseID    = 0;
THypothesisID	CHMTSLAM::m_nextHypID     = COMMON_TOPOLOG_HYP + 1;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHMTSLAM::CHMTSLAM( )
 :  m_inputQueue_cs("inputQueue_cs"),
    m_map_cs("map_cs"),
    m_LMHs_cs("LMHs_cs")
//	m_semaphoreInputQueueHasData (0 /*Init state*/ ,1 /*Max*/ ),
//	m_eventNewObservationInserted(0 /*Init state*/ ,10000 /*Max*/ )
{
	// Initialize data structures:
	// ----------------------------
	m_terminateThreads				= false;
	m_terminationFlag_LSLAM			=
	m_terminationFlag_TBI			=
	m_terminationFlag_3D_viewer		= false;

	// Create threads:
	// -----------------------
	m_hThread_LSLAM 	= mrpt::system::createThreadFromObjectMethod( this, &CHMTSLAM::thread_LSLAM );
	m_hThread_TBI 		= mrpt::system::createThreadFromObjectMethod( this, &CHMTSLAM::thread_TBI );
	m_hThread_3D_viewer	= mrpt::system::createThreadFromObjectMethod( this, &CHMTSLAM::thread_3D_viewer );


	// Other variables:
	m_LSLAM_method		= NULL;


	// Register default LC detectors:
	// --------------------------------
	registerLoopClosureDetector("gridmaps", & CTopLCDetector_GridMatching::createNewInstance );
	registerLoopClosureDetector("fabmap", & CTopLCDetector_FabMap::createNewInstance );

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
	m_terminateThreads	= true;

	// Wait for threads:
	// ----------------------------------
	MRPT_LOG_DEBUG("[CHMTSLAM::destructor] Waiting for threads end...\n");

	mrpt::system::joinThread( m_hThread_3D_viewer );
	mrpt::system::joinThread( m_hThread_LSLAM );
	mrpt::system::joinThread( m_hThread_TBI );

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
		catch(std::exception &e)
		{
			MRPT_LOG_WARN(mrpt::format("Ignoring exception at ~CHMTSLAM():\n%s",e.what()));
		}
		catch(...)
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
		m_LSLAM_method = NULL;
	}

	// Delete TLC-detectors
	{
		synch::CCriticalSectionLocker	lock( &m_topLCdets_cs );

		// Clear old list:
		for (std::deque<CTopLCDetectorBase*>::iterator it=m_topLCdets.begin();it!=m_topLCdets.end();++it)
			delete *it;
		m_topLCdets.clear();
	}
}


/*---------------------------------------------------------------
						clearInputQueue
  ---------------------------------------------------------------*/
void  CHMTSLAM::clearInputQueue()
{
	// Wait for critical section
	{
		CCriticalSectionLocker  locker( &m_inputQueue_cs);

		while (!m_inputQueue.empty())
		{
			//delete m_inputQueue.front();
			m_inputQueue.pop();
		};
	}
}


/*---------------------------------------------------------------
						pushAction
  ---------------------------------------------------------------*/
void  CHMTSLAM::pushAction( const CActionCollectionPtr &acts )
{
	if (m_terminateThreads)
	{
		// Discard it:
		//delete acts;
		return;
	}

	{	// Wait for critical section
		CCriticalSectionLocker  locker( &m_inputQueue_cs);
		m_inputQueue.push( acts );
	}
}

/*---------------------------------------------------------------
						pushObservations
  ---------------------------------------------------------------*/
void  CHMTSLAM::pushObservations( const CSensoryFramePtr &sf )
{
	if (m_terminateThreads)
	{
		// Discard it:
		//delete sf;
		return;
	}

	{	// Wait for critical section
		CCriticalSectionLocker  locker( &m_inputQueue_cs);
		m_inputQueue.push( sf );
	}
}

/*---------------------------------------------------------------
						pushObservation
  ---------------------------------------------------------------*/
void  CHMTSLAM::pushObservation( const CObservationPtr &obs )
{
	if (m_terminateThreads)
	{   // Discard it:
		//delete obs;
		return;
	}

	// Add a CSensoryFrame with the obs:
	CSensoryFramePtr sf = CSensoryFrame::Create();
	sf->insert(obs);  // memory will be freed when deleting the SF in other thread

	{	// Wait for critical section
		CCriticalSectionLocker  locker( &m_inputQueue_cs);
		m_inputQueue.push( sf );
	}
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CHMTSLAM::loadOptions( const mrpt::utils::CConfigFileBase &cfg )
{
	m_options.loadFromConfigFile(cfg,"HMT-SLAM");

	m_options.defaultMapsInitializers.loadFromConfigFile(cfg,"MetricMaps");

	m_options.pf_options.loadFromConfigFile(cfg,"PARTICLE_FILTER");

	m_options.KLD_params.loadFromConfigFile(cfg,"KLD");

	m_options.AA_options.loadFromConfigFile(cfg,"GRAPH_CUT");

	// Topological Loop Closure detector options:
	m_options.TLC_grid_options.loadFromConfigFile(cfg,"TLC_GRIDMATCHING");
	m_options.TLC_fabmap_options.loadFromConfigFile(cfg,"TLC_FABMAP");

	m_options.dumpToConsole();
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CHMTSLAM::loadOptions( const std::string &configFile )
{
	ASSERT_( mrpt::system::fileExists(configFile) );
	CConfigFile		cfg(configFile);
	loadOptions(cfg);
}

/*---------------------------------------------------------------
						TOptions
  ---------------------------------------------------------------*/
CHMTSLAM::TOptions::TOptions()
{
	LOG_OUTPUT_DIR				= "";
	LOG_FREQUENCY				= 1;

	SLAM_METHOD						= lsmRBPF_2DLASER;

	SLAM_MIN_DIST_BETWEEN_OBS	  	= 1.0f;
	SLAM_MIN_HEADING_BETWEEN_OBS  	= DEG2RAD(25.0f);

	MIN_ODOMETRY_STD_XY		= 0;
	MIN_ODOMETRY_STD_PHI		= 0;

	VIEW3D_AREA_SPHERES_HEIGHT		= 10.0f;
	VIEW3D_AREA_SPHERES_RADIUS  	= 1.0f;

	random_seed						= 1234;

	TLC_detectors.clear();

	stds_Q_no_odo.resize(3);
	stds_Q_no_odo[0] =
	stds_Q_no_odo[1] = 0.10f;
	stds_Q_no_odo[2] = DEG2RAD(4.0f);
}

/*---------------------------------------------------------------
						loadFromConfigFile
  ---------------------------------------------------------------*/
void  CHMTSLAM::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string		&section)
{
	MRPT_LOAD_CONFIG_VAR( LOG_OUTPUT_DIR, string,    source, section);
	MRPT_LOAD_CONFIG_VAR( LOG_FREQUENCY,  int,     	source, section);

	MRPT_LOAD_CONFIG_VAR_CAST_NO_DEFAULT(SLAM_METHOD, int,  TLSlamMethod,  source, section);

	MRPT_LOAD_CONFIG_VAR( SLAM_MIN_DIST_BETWEEN_OBS, float, 		source, section);
	MRPT_LOAD_CONFIG_VAR_DEGREES( SLAM_MIN_HEADING_BETWEEN_OBS, 	source, section);

	MRPT_LOAD_CONFIG_VAR(MIN_ODOMETRY_STD_XY,float, source,section);
	MRPT_LOAD_CONFIG_VAR_DEGREES( MIN_ODOMETRY_STD_PHI, source,section);

	MRPT_LOAD_CONFIG_VAR( VIEW3D_AREA_SPHERES_HEIGHT, float,  	source, section);
	MRPT_LOAD_CONFIG_VAR( VIEW3D_AREA_SPHERES_RADIUS, float,  	source, section);

	MRPT_LOAD_CONFIG_VAR( random_seed, int, source,section);

	stds_Q_no_odo[2] = RAD2DEG(stds_Q_no_odo[2]);
	source.read_vector(section,"stds_Q_no_odo", stds_Q_no_odo, stds_Q_no_odo );
	ASSERT_(stds_Q_no_odo.size()==3)

	stds_Q_no_odo[2] = DEG2RAD(stds_Q_no_odo[2]);

	std::string sTLC_detectors = source.read_string(section,"TLC_detectors", "", true );

	mrpt::system::tokenize(sTLC_detectors,", ",TLC_detectors);

	std::cout << "TLC_detectors: " << TLC_detectors.size() << std::endl;

	// load other sub-classes:
	AA_options.loadFromConfigFile(source,section);
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void  CHMTSLAM::TOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CHMTSLAM::TOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR( LOG_OUTPUT_DIR,  string );
	LOADABLEOPTS_DUMP_VAR( LOG_FREQUENCY, int);

	LOADABLEOPTS_DUMP_VAR( SLAM_METHOD, int);

	LOADABLEOPTS_DUMP_VAR( SLAM_MIN_DIST_BETWEEN_OBS, float );
	LOADABLEOPTS_DUMP_VAR_DEG( SLAM_MIN_HEADING_BETWEEN_OBS );

	LOADABLEOPTS_DUMP_VAR( MIN_ODOMETRY_STD_XY, float );
	LOADABLEOPTS_DUMP_VAR_DEG( MIN_ODOMETRY_STD_PHI );

	LOADABLEOPTS_DUMP_VAR( random_seed, int );

	AA_options.dumpToTextStream(out);
	pf_options.dumpToTextStream(out);
	KLD_params.dumpToTextStream(out);
	defaultMapsInitializers.dumpToTextStream(out);
	TLC_grid_options.dumpToTextStream(out);
	TLC_fabmap_options.dumpToTextStream(out);
}

/*---------------------------------------------------------------
					isInputQueueEmpty
  ---------------------------------------------------------------*/
bool  CHMTSLAM::isInputQueueEmpty()
{
	bool	res;

	{	// Wait for critical section
		CCriticalSectionLocker  locker( &m_inputQueue_cs);
		res = m_inputQueue.empty();
	}
	return res;
}

/*---------------------------------------------------------------
					inputQueueSize
  ---------------------------------------------------------------*/
size_t CHMTSLAM::inputQueueSize()
{
	size_t  res;
	{	// Wait for critical section
		CCriticalSectionLocker  locker( &m_inputQueue_cs);
		res = m_inputQueue.size();
	}
	return res;
}

/*---------------------------------------------------------------
					getNextObjectFromInputQueue
  ---------------------------------------------------------------*/
CSerializablePtr CHMTSLAM::getNextObjectFromInputQueue()
{
	CSerializablePtr obj;

	{	// Wait for critical section
		CCriticalSectionLocker  locker( &m_inputQueue_cs);
		if (!m_inputQueue.empty())
		{
			obj = m_inputQueue.front();
			m_inputQueue.pop();
		}
	}
	return obj;
}

/*---------------------------------------------------------------
						initializeEmptyMap
  ---------------------------------------------------------------*/
void  CHMTSLAM::initializeEmptyMap()
{
	THypothesisIDSet		LMH_hyps;
	THypothesisID			newHypothID = generateHypothesisID();

	LMH_hyps.insert( COMMON_TOPOLOG_HYP );
	LMH_hyps.insert( newHypothID );

	// ------------------------------------
	// CLEAR HIERARCHICAL MAP
	// ------------------------------------
	CHMHMapNode::TNodeID	firstAreaID;
	{
		synch::CCriticalSectionLocker	locker( &m_map_cs );

		// Initialize hierarchical structures:
		// -----------------------------------------------------
		m_map.clear();

		// Create a single node for the starting area:
		CHMHMapNodePtr firstArea = CHMHMapNode::Create( &m_map );
		firstAreaID = firstArea->getID();

		firstArea->m_hypotheses = LMH_hyps;
		CMultiMetricMapPtr		emptyMap = CMultiMetricMapPtr(new CMultiMetricMap(&m_options.defaultMapsInitializers) );

		firstArea->m_nodeType.setType( "Area" );
		firstArea->m_label = generateUniqueAreaLabel();
		firstArea->m_annotations.set( NODE_ANNOTATION_METRIC_MAPS,     emptyMap, 		newHypothID );
		firstArea->m_annotations.setElemental( NODE_ANNOTATION_REF_POSEID,  POSEID_INVALID , 	newHypothID );
	} // end of lock m_map_cs

	// ------------------------------------
	// CLEAR LIST OF HYPOTHESES
	// ------------------------------------
	{
		synch::CCriticalSectionLocker	lock( &m_LMHs_cs );

		// Add to the list:
		m_LMHs.clear();
		CLocalMetricHypothesis	&newLMH = m_LMHs[newHypothID];
		newLMH.m_parent = this;

		newLMH.m_currentRobotPose = POSEID_INVALID ;  // Special case: map is empty
		newLMH.m_log_w			  = 0;
		newLMH.m_ID 			  = newHypothID;

		newLMH.m_neighbors.clear();
		newLMH.m_neighbors.insert( firstAreaID );

		newLMH.clearRobotPoses();
	} // end of cs


	// ------------------------------------------
	//   Create the local SLAM algorithm object
	// -----------------------------------------
	switch( m_options.SLAM_METHOD )
	{
		case lsmRBPF_2DLASER:
		{
			if (m_LSLAM_method) delete m_LSLAM_method;
			m_LSLAM_method = new CLSLAM_RBPF_2DLASER(this);
		}
		break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("Invalid selection for LSLAM method: %i",(int)m_options.SLAM_METHOD );
	};

	// ------------------------------------
	//  Topological LC detectors:
	// ------------------------------------
	{
		synch::CCriticalSectionLocker	lock( &m_topLCdets_cs );

		// Clear old list:
		for (std::deque<CTopLCDetectorBase*>::iterator it=m_topLCdets.begin();it!=m_topLCdets.end();++it)
			delete *it;
		m_topLCdets.clear();

		// Create new list:
		//  1: Occupancy Grid matching.
		//  2: Cummins' image matching.
		for (vector_string::const_iterator d=m_options.TLC_detectors.begin();d!=m_options.TLC_detectors.end();++d)
			m_topLCdets.push_back( loopClosureDetector_factory(*d) );
	}


	// ------------------------------------
	//  Other variables:
	// ------------------------------------
	m_LSLAM_queue.clear();

	// ------------------------------------
	// Delete log files:
	// ------------------------------------
	if (!m_options.LOG_OUTPUT_DIR.empty())
	{
	    mrpt::system::deleteFilesInDirectory( m_options.LOG_OUTPUT_DIR );
	    mrpt::system::createDirectory( m_options.LOG_OUTPUT_DIR );
	    mrpt::system::createDirectory( m_options.LOG_OUTPUT_DIR + "/HMAP_txt" );
	    mrpt::system::createDirectory( m_options.LOG_OUTPUT_DIR + "/HMAP_3D" );
	    mrpt::system::createDirectory( m_options.LOG_OUTPUT_DIR + "/LSLAM_3D" );
	    mrpt::system::createDirectory( m_options.LOG_OUTPUT_DIR + "/ASSO" );
	    mrpt::system::createDirectory( m_options.LOG_OUTPUT_DIR + "/HMTSLAM_state" );
	}

}

/*---------------------------------------------------------------
						generateUniqueAreaLabel
  ---------------------------------------------------------------*/
std::string	 CHMTSLAM::generateUniqueAreaLabel()
{
	return format( "%li", (long int)(m_nextAreaLabel++) );
}

/*---------------------------------------------------------------
						generatePoseID
  ---------------------------------------------------------------*/
TPoseID CHMTSLAM::generatePoseID()
{
	return m_nextPoseID++;
}

/*---------------------------------------------------------------
						generateHypothesisID
  ---------------------------------------------------------------*/
THypothesisID CHMTSLAM::generateHypothesisID()
{
	return m_nextHypID++;
}


/*---------------------------------------------------------------
						getAs3DScene
  ---------------------------------------------------------------*/
void  CHMTSLAM::getAs3DScene( COpenGLScene	&scene3D )
{
	MRPT_UNUSED_PARAM(scene3D);
}


/*---------------------------------------------------------------
						abortedDueToErrors
  ---------------------------------------------------------------*/
bool CHMTSLAM::abortedDueToErrors()
{
	return  m_terminationFlag_LSLAM ||
			m_terminationFlag_TBI ||
			m_terminationFlag_3D_viewer;
}

/*---------------------------------------------------------------
						registerLoopClosureDetector
  ---------------------------------------------------------------*/
void CHMTSLAM::registerLoopClosureDetector(
	const std::string   	&name,
	CTopLCDetectorBase*		(*ptrCreateObject)(CHMTSLAM*)
	)
{
	m_registeredLCDetectors[name] = ptrCreateObject;
}

/*---------------------------------------------------------------
				loopClosureDetector_factory
  ---------------------------------------------------------------*/
CTopLCDetectorBase* CHMTSLAM::loopClosureDetector_factory(const std::string  &name)
{
	MRPT_START
	std::map<std::string,TLopLCDetectorFactory>::const_iterator it=m_registeredLCDetectors.find( name );
	if (it==m_registeredLCDetectors.end())
		THROW_EXCEPTION_CUSTOM_MSG1("Invalid value for TLC_detectors: %s", name.c_str() );
	return it->second(this);
	MRPT_END
}



/*---------------------------------------------------------------
					saveState
  ---------------------------------------------------------------*/
bool CHMTSLAM::saveState( CStream &out ) const
{
	try
	{
		out << *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					loadState
  ---------------------------------------------------------------*/
bool CHMTSLAM::loadState( CStream &in )
{
	try
	{
		in >> *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CHMTSLAM::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			// Acquire all critical sections before!
			// -------------------------------------------
			//std::map< THypothesisID, CLocalMetricHypothesis >::const_iterator it;

			//CCriticalSectionLocker LMHs( & m_LMHs_cs );
			//for (it=m_LMHs.begin();it!=m_LMHs.end();it++) it->second.m_lock.enter();

			CCriticalSectionLocker lock_map( &m_map_cs );

			// Data:
			in  >> m_nextAreaLabel
				>> m_nextPoseID
				>> m_nextHypID;

			// The HMT-MAP:
			in >> m_map;

			// The LMHs:
			in >> m_LMHs;

			// Save options??? Better allow changing them...

			// Release all critical sections:
			//for (it=m_LMHs.begin();it!=m_LMHs.end();it++) it->second.m_lock.enter();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CHMTSLAM::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// Acquire all critical sections before!
		// -------------------------------------------
		//std::map< THypothesisID, CLocalMetricHypothesis >::const_iterator it;

		//CCriticalSectionLocker LMHs( & m_LMHs_cs );
		//for (it=m_LMHs.begin();it!=m_LMHs.end();it++) it->second.m_lock.enter();

		CCriticalSectionLocker lock_map( &m_map_cs );

		// Data:
		out << m_nextAreaLabel
			<< m_nextPoseID
			<< m_nextHypID;

		// The HMT-MAP:
		out << m_map;

		// The LMHs:
		out << m_LMHs;

		// Save options??? Better allow changing them...

		// Release all critical sections:
		//for (it=m_LMHs.begin();it!=m_LMHs.end();it++) it->second.m_lock.enter();


	}
}


