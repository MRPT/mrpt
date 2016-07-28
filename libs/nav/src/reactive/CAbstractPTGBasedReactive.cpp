/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CAbstractPTGBasedReactive.h>
#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h> // sum()
#include <mrpt/utils/printf_vector.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CMemoryStream.h>
#include <limits>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace std;

// ------ CAbstractPTGBasedReactive::TNavigationParamsPTG -----
std::string CAbstractPTGBasedReactive::TNavigationParamsPTG::getAsText() const
{

	std::string s = TNavigationParams::getAsText();
	s += "restrict_PTG_indices: ";
	s += mrpt::utils::sprintf_vector("%u ",this->restrict_PTG_indices);
	s += "\n";
	return s;
}

// Ctor:
CAbstractPTGBasedReactive::CAbstractPTGBasedReactive(CRobot2NavInterface &react_iterf_impl, bool enableConsoleOutput, bool enableLogFile):
	CWaypointsNavigator(react_iterf_impl),
	m_holonomicMethod            (),
	m_logFile                    (NULL),
	m_enableKeepLogRecords       (false),

	m_last_vel_cmd               (react_iterf_impl.getVelCmdLength()  , 0.0),
	m_new_vel_cmd                (react_iterf_impl.getVelCmdLength(), 0.0),

	m_enableConsoleOutput        (enableConsoleOutput),
	m_init_done                  (false),
	ptg_cache_files_directory    ("."),
	refDistance                  (4.0f),
	SPEEDFILTER_TAU              (0.0),
	secureDistanceStart          (0.05),
	secureDistanceEnd            (0.20),
	meanExecutionPeriod          (0.1f),
	m_timelogger                 (false), // default: disabled
	m_PTGsMustBeReInitialized    (true),
	meanExecutionTime            (0.1f),
	meanTotalExecutionTime       (0.1f),
	m_closing_navigator          (false),
	m_infoPerPTG_timestamp       (INVALID_TIMESTAMP)
{
	this->enableLogFile( enableLogFile );
}

void CAbstractPTGBasedReactive::preDestructor()
{
	m_closing_navigator = true;

	// Wait to end of navigation (multi-thread...)
	m_nav_cs.enter();
	m_nav_cs.leave();

	// Just in case.
	try {
		m_robot.stop();
	} catch (...) { }

	mrpt::utils::delete_safe(m_logFile);

	// Free holonomic method:
	this->deleteHolonomicObjects();
}

CAbstractPTGBasedReactive::~CAbstractPTGBasedReactive()
{
	// Just in case the user didn't call this, call it again to reduce memory leaks to those of the user:
	this->preDestructor();
}

void CAbstractPTGBasedReactive::initialize()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	m_infoPerPTG_timestamp = INVALID_TIMESTAMP;

	// Compute collision grids:
	STEP1_InitPTGs();
}

/*---------------------------------------------------------------
						enableLogFile
  ---------------------------------------------------------------*/
void CAbstractPTGBasedReactive::enableLogFile(bool enable)
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	try
	{
		// Disable:
		// -------------------------------
		if (!enable)
		{
			if (m_logFile)
			{
				printf_debug("[CAbstractPTGBasedReactive::enableLogFile] Stopping logging.\n");
				// Close file:
				delete m_logFile;
				m_logFile = NULL;
			}
			else return;	// Already disabled.
		}
		else
		{	// Enable
			// -------------------------------
			if (m_logFile) return; // Already enabled:

			// Open file, find the first free file-name.
			char	aux[100];
			unsigned int nFile = 0;
			bool    free_name = false;

			system::createDirectory("./reactivenav.logs");

			while (!free_name)
			{
				nFile++;
				sprintf(aux, "./reactivenav.logs/log_%03u.reactivenavlog", nFile );
				free_name = !system::fileExists(aux);
			}

			// Open log file:
			m_logFile = new CFileOutputStream(aux);

			printf_debug("[CAbstractPTGBasedReactive::enableLogFile] Logging to file `%s`\n",aux);

		}
	} catch (...) {
		printf_debug("[CAbstractPTGBasedReactive::enableLogFile] Exception!!\n");
	}

}

void CAbstractPTGBasedReactive::getLastLogRecord( CLogFileRecord &o )
{
	mrpt::synch::CCriticalSectionLocker lock(&m_critZoneLastLog);
	o = lastLogRecord;
}


void CAbstractPTGBasedReactive::loadHolonomicMethodConfig(
	const mrpt::utils::CConfigFileBase &ini,
	const std::string &section )
{
	THolonomicMethod holoMethod = ini.read_enum<THolonomicMethod>(section,"HOLONOMIC_METHOD",hmVIRTUAL_FORCE_FIELDS, true);
	this->setHolonomicMethod( holoMethod, ini );
}

void CAbstractPTGBasedReactive::deleteHolonomicObjects()
{
	for (size_t i=0;i<m_holonomicMethod.size();i++)
		delete m_holonomicMethod[i];
	m_holonomicMethod.clear();
}

void CAbstractPTGBasedReactive::setHolonomicMethod(
    const THolonomicMethod method,
	const mrpt::utils::CConfigFileBase &ini)
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	this->deleteHolonomicObjects();

	const size_t nPTGs = this->getPTG_count();
	ASSERT_(nPTGs!=0);
	m_holonomicMethod.resize(nPTGs);

	for (size_t i=0; i<nPTGs; i++)
	{
		switch (method)
		{
		case hmSEARCH_FOR_BEST_GAP:  m_holonomicMethod[i] = new CHolonomicND();  break;
		case hmVIRTUAL_FORCE_FIELDS: m_holonomicMethod[i] = new CHolonomicVFF(); break;
		case hmFULL_EVAL: m_holonomicMethod[i] = new CHolonomicFullEval(); break;
		default: THROW_EXCEPTION_CUSTOM_MSG1("Unknown Holonomic method: %u",static_cast<unsigned int>(method))
		};
		// Load params:
		m_holonomicMethod[i]->initialize( ini );
	}
}


// The main method: executes one time-iteration of the reactive navigation algorithm.
void CAbstractPTGBasedReactive::performNavigationStep()
{
	// Security tests:
	if (m_closing_navigator) return;  // Are we closing in the main thread?
	if (!m_init_done) THROW_EXCEPTION("Have you called loadConfigFile() before?")

	const size_t nPTGs = this->getPTG_count();

	// Whether to worry about log files:
	const bool fill_log_record = (m_logFile!=NULL || m_enableKeepLogRecords);
	CLogFileRecord newLogRec;
	newLogRec.infoPerPTG.resize(nPTGs);

	// At the beginning of each log file, add an introductory block explaining which PTGs are we using:
	{
		static mrpt::utils::CStream  *prev_logfile = NULL;
		if (m_logFile && m_logFile!=prev_logfile)  // Only the first time
		{
			prev_logfile=m_logFile;
			for (size_t i=0;i<nPTGs;i++)
			{
				// If we make a direct copy (=) we will store the entire, heavy, collision grid.
				// Let's just store the parameters of each PTG by serializing it, so paths can be reconstructed
				// by invoking initialize()
				mrpt::utils::CMemoryStream buf;
				buf << *this->getPTG(i);
				buf.Seek(0);
				newLogRec.infoPerPTG[i].ptg = mrpt::nav::CParameterizedTrajectoryGeneratorPtr ( buf.ReadObject() );
			}
		}
	}


	CTimeLoggerEntry tle1(m_timelogger,"navigationStep");

	try
	{
		totalExecutionTime.Tic(); // Start timer

		const mrpt::system::TTimeStamp tim_start_iteration = mrpt::system::now();

		// Compute target location relative to current robot pose:
		// ---------------------------------------------------------------------
		const CPose2D relTarget = CPose2D(m_navigationParams->target) - CPose2D(m_curPose);

		STEP1_InitPTGs(); // Will only recompute if "m_PTGsMustBeReInitialized==true"

		// Update kinematic state in all PTGs:
		for (size_t i=0;i<nPTGs;i++)
			getPTG(i)->updateCurrentRobotVel(m_curVelLocal);

		// STEP2: Load the obstacles and sort them in height bands.
		// -----------------------------------------------------------------------------
		if (! STEP2_SenseObstacles() )
		{
			printf_debug("Error while loading and sorting the obstacles. Robot will be stopped.\n");
			m_robot.stop();
			m_navigationState = NAV_ERROR;
			return;
		}

		// Start timer
		executionTime.Tic();

		m_infoPerPTG.resize(nPTGs);
		m_infoPerPTG_timestamp = tim_start_iteration;
		vector<THolonomicMovement> holonomicMovements(nPTGs);

		for (size_t indexPTG=0;indexPTG<nPTGs;indexPTG++)
		{
			CHolonomicLogFileRecordPtr HLFR;

			//  STEP3(a): Transform target location into TP-Space for each PTG
			// -----------------------------------------------------------------------------
			CParameterizedTrajectoryGenerator * ptg = getPTG(indexPTG);
			ASSERT_(ptg)
			TInfoPerPTG &ipf = m_infoPerPTG[indexPTG];

			// The picked movement in TP-Space (to be determined by holonomic method below)
			THolonomicMovement &holonomicMovement = holonomicMovements[indexPTG];
			holonomicMovement.PTG = ptg;

			// If the user doesn't want to use this PTG, just mark it as invalid:
			ipf.valid_TP = true;
			{
				const TNavigationParamsPTG * navp = dynamic_cast<const TNavigationParamsPTG*>(m_navigationParams);
				if (navp && !navp->restrict_PTG_indices.empty())
				{
					bool use_this_ptg = false;
					for (size_t i=0;i<navp->restrict_PTG_indices.size() && !use_this_ptg;i++) {
						if (navp->restrict_PTG_indices[i]==indexPTG)
							use_this_ptg = true;
					}
					ipf.valid_TP = use_this_ptg;
				}
			}

			double timeForTPObsTransformation=.0, timeForHolonomicMethod=.0;

			// Normal PTG validity filter: check if target falls into the PTG domain:
			if (ipf.valid_TP)
			{
				ipf.valid_TP = ptg->inverseMap_WS2TP(relTarget.x(),relTarget.y(),ipf.target_k,ipf.target_dist);
			}

			if (!ipf.valid_TP)
			{
				ipf.target_k=0;
				ipf.target_dist=0;

				{   // Invalid PTG (target out of reachable space):
					// - holonomicMovement= Leave default values
					HLFR = CLogFileRecord_VFF::Create();
				}
			}
			else
			{
				ipf.target_alpha = ptg->index2alpha(ipf.target_k);
				ipf.TP_Target.x = cos(ipf.target_alpha) * ipf.target_dist;
				ipf.TP_Target.y = sin(ipf.target_alpha) * ipf.target_dist;

				//  STEP3(b): Build TP-Obstacles
				// -----------------------------------------------------------------------------
				{
					tictac.Tic();

					// Initialize TP-Obstacles:
					const size_t Ki = ptg->getAlphaValuesCount();
					ptg->initTPObstacles(ipf.TP_Obstacles);

					// Implementation-dependent conversion:
					STEP3_WSpaceToTPSpace(indexPTG,ipf.TP_Obstacles);

					// Distances in TP-Space are normalized to [0,1]:
					const double _refD = 1.0/ptg->getRefDistance();
					for (size_t i=0;i<Ki;i++) ipf.TP_Obstacles[i] *= _refD;

					timeForTPObsTransformation= tictac.Tac();
					if (m_timelogger.isEnabled())
						m_timelogger.registerUserMeasure("navigationStep.STEP3_WSpaceToTPSpace",timeForTPObsTransformation);
				}

				//  STEP4: Holonomic navigation method
				// -----------------------------------------------------------------------------
				{
					tictac.Tic();

					ASSERT_(m_holonomicMethod[indexPTG])
					m_holonomicMethod[indexPTG]->navigate(
						ipf.TP_Target,     // Normalized [0,1]
						ipf.TP_Obstacles,  // Normalized [0,1]
						1.0, // Was: ptg->getMax_V_inTPSpace(),
						holonomicMovement.direction,
						holonomicMovement.speed,
						HLFR, 
						1.0 /* max obstacle dist*/ );

					MRPT_TODO("Honor targetIsIntermediaryWaypoint wrt approaching slow down")

					// Security: Scale down the velocity when heading towards obstacles,
					//  such that it's assured that we never go thru an obstacle!
					const int kDirection = static_cast<int>( holonomicMovement.PTG->alpha2index( holonomicMovement.direction ) );
					const double obsFreeNormalizedDistance = ipf.TP_Obstacles[kDirection];
					double velScale = 1.0;
					ASSERT_(secureDistanceEnd>secureDistanceStart);
					if (obsFreeNormalizedDistance<secureDistanceEnd)
					{
						if (obsFreeNormalizedDistance<=secureDistanceStart)
							 velScale = 0.0; // security stop
						else velScale = (obsFreeNormalizedDistance-secureDistanceStart)/(secureDistanceEnd-secureDistanceStart);
					}

					// Scale:
					holonomicMovement.speed *= velScale;

					timeForHolonomicMethod = tictac.Tac();
					if (m_timelogger.isEnabled())
						m_timelogger.registerUserMeasure("navigationStep.STEP4_HolonomicMethod",timeForHolonomicMethod);
				}

				// STEP5: Evaluate each movement to assign them a "evaluation" value.
				// ---------------------------------------------------------------------
				{
					CTimeLoggerEntry tle(m_timelogger,"navigationStep.STEP5_PTGEvaluator");

					STEP5_PTGEvaluator(
						holonomicMovement,
						ipf.TP_Obstacles,
						TPose2D(relTarget),
						ipf.TP_Target,
						newLogRec.infoPerPTG[indexPTG]);
				}


			} // end "valid_TP"

			// Logging:
			if (fill_log_record)
			{
				metaprogramming::copy_container_typecasting(ipf.TP_Obstacles, newLogRec.infoPerPTG[indexPTG].TP_Obstacles);
				CLogFileRecord::TInfoPerPTG &ipp = newLogRec.infoPerPTG[indexPTG];
				ipp.PTG_desc  = ptg->getDescription();
				ipp.TP_Target = ipf.TP_Target;
				ipp.HLFR	     = HLFR;
				ipp.desiredDirection = holonomicMovement.direction;
				ipp.desiredSpeed     = holonomicMovement.speed;
				ipp.evaluation       = holonomicMovement.evaluation;
				ipp.timeForTPObsTransformation = timeForTPObsTransformation;
				ipp.timeForHolonomicMethod     = timeForHolonomicMethod;
			}

		} // end for each PTG


		// STEP6: After all PTGs have been evaluated, pick the best scored:
		// ---------------------------------------------------------------------
		int nSelectedPTG = 0;
		for (size_t indexPTG=0;indexPTG<nPTGs;indexPTG++) {
			if ( holonomicMovements[indexPTG].evaluation > holonomicMovements[nSelectedPTG].evaluation )
				nSelectedPTG = indexPTG;
		}
		const THolonomicMovement & selectedHolonomicMovement = holonomicMovements[nSelectedPTG];


		// STEP7: Get the non-holonomic movement command.
		// ---------------------------------------------------------------------
		{
			CTimeLoggerEntry tle(m_timelogger,"navigationStep.STEP7_NonHolonomicMovement");
			STEP7_GenerateSpeedCommands( selectedHolonomicMovement );
		}

		// ---------------------------------------------------------------------
		//				SEND MOVEMENT COMMAND TO THE ROBOT
		// ---------------------------------------------------------------------
		// All equal to 0 means "stop".
		if (std::find_if(m_new_vel_cmd.begin(), m_new_vel_cmd.end(), std::bind2nd(std::not_equal_to<double>(), 0.0) ) == m_new_vel_cmd.end() )
		{
			m_robot.stop();
		}
		else
		{
			if ( !m_robot.changeSpeeds(m_new_vel_cmd) )
			{
				doEmergencyStop("\nERROR calling RobotMotionControl::changeSpeeds!! Stopping robot and finishing navigation\n");
				return;
			}
		}

		// Statistics:
		// ----------------------------------------------------
		const double executionTimeValue = executionTime.Tac();
		meanExecutionTime=  0.3 * meanExecutionTime + 0.7 * executionTimeValue;
		meanTotalExecutionTime=  0.3 * meanTotalExecutionTime + 0.7 * totalExecutionTime.Tac();
		meanExecutionPeriod = 0.3 * meanExecutionPeriod + 0.7 * min(1.0, timerForExecutionPeriod.Tac());

		timerForExecutionPeriod.Tic();

		if (m_enableConsoleOutput)
		{
			printf_debug(
				"CMD: %s \t"
				"T=%.01lfms Exec:%.01lfms|%.01lfms \t"
				"E=%.01lf PTG#%i\n",
					mrpt::utils::sprintf_vector("%.02f ",m_new_vel_cmd).c_str(),
					1000.0*meanExecutionPeriod,
					1000.0*meanExecutionTime,
					1000.0*meanTotalExecutionTime,
					(double)selectedHolonomicMovement.evaluation,
					nSelectedPTG
					);
		}

		// ---------------------------------------
		// STEP8: Generate log record
		// ---------------------------------------
		if (fill_log_record)
		{
			m_timelogger.enter("navigationStep.populate_log_info");

			this->loggingGetWSObstaclesAndShape(newLogRec);

			newLogRec.robotOdometryPose   = m_curPose;
			newLogRec.WS_target_relative  = TPoint2D(relTarget.x(), relTarget.y());
			newLogRec.cmd_vel             = m_new_vel_cmd;
			newLogRec.cmd_vel_filterings  = m_cmd_vel_filterings;
			newLogRec.nSelectedPTG        = nSelectedPTG;
			newLogRec.executionTime       = executionTimeValue;
			newLogRec.cur_vel             = m_curVel;
			newLogRec.cur_vel_local       = m_curVelLocal;
			newLogRec.estimatedExecutionPeriod = meanExecutionPeriod;
			newLogRec.timestamp = tim_start_iteration;
			newLogRec.nPTGs = nPTGs;

			m_timelogger.leave("navigationStep.populate_log_info");

			//  Save to log file:
			// --------------------------------------
			{
				mrpt::utils::CTimeLoggerEntry tle(m_timelogger,"navigationStep.write_log_file");
				if (m_logFile) (*m_logFile) << newLogRec;
			}

			// Set as last log record
			{
				mrpt::synch::CCriticalSectionLocker lock_log(&m_critZoneLastLog);    // Lock
				lastLogRecord = newLogRec; // COPY
			}
		} // if (fill_log_record)

	}
	catch (std::exception &e)
	{
		std::cout << e.what();
		std::cout << "[CAbstractPTGBasedReactive::performNavigationStep] Exceptions!!\n";
	}
	catch (...)
	{
		std::cout << "[CAbstractPTGBasedReactive::performNavigationStep] Unexpected exception!!:\n";
	}
}

void CAbstractPTGBasedReactive::STEP5_PTGEvaluator(
	THolonomicMovement         & holonomicMovement,
	const std::vector<double>        & in_TPObstacles,
	const mrpt::math::TPose2D  & WS_Target,
	const mrpt::math::TPoint2D & TP_Target,
	CLogFileRecord::TInfoPerPTG & log )
{
	const double   refDist	    = holonomicMovement.PTG->getRefDistance();
	const double   TargetDir    = (TP_Target.x!=0 || TP_Target.y!=0) ? atan2( TP_Target.y, TP_Target.x) : 0.0;
	const int      TargetSector = static_cast<int>( holonomicMovement.PTG->alpha2index( TargetDir ) );
	const double   TargetDist   = TP_Target.norm();
	// Picked movement direction:
	const int      kDirection   = static_cast<int>( holonomicMovement.PTG->alpha2index( holonomicMovement.direction ) );

	// Coordinates of the trajectory end for the given PTG and "alpha":
	const double d = min( in_TPObstacles[ kDirection ], 0.90*TargetDist);
	uint16_t nStep;
	bool pt_in_range = holonomicMovement.PTG->getPathStepForDist(kDirection, d, nStep);
	ASSERT_(pt_in_range)

	mrpt::math::TPose2D pose;
	holonomicMovement.PTG->getPathPose(kDirection, nStep,pose);

	// Factor 1: Free distance for the chosen PTG and "alpha" in the TP-Space:
	// ----------------------------------------------------------------------
	const double factor1 = in_TPObstacles[kDirection];


	// Factor 2: Distance in sectors:
	// -------------------------------------------
	double dif = std::abs(((double)( TargetSector - kDirection )));
	const double nSectors = (float)in_TPObstacles.size();
	if ( dif > (0.5*nSectors)) dif = nSectors - dif;
	const double factor2 = exp(-square( dif / (in_TPObstacles.size()/3.0f))) ;

	// Factor 3: Angle between the robot at the end of the chosen trajectory and the target
	// -------------------------------------------------------------------------------------
	double t_ang = atan2( WS_Target.y - pose.y, WS_Target.x - pose.x );
	t_ang -= pose.phi;
	mrpt::math::wrapToPiInPlace(t_ang);

	const double factor3 = exp(-square( t_ang / (float)(0.5f*M_PI)) );

	// Factor4:		Decrease in euclidean distance between (x,y) and the target:
	//  Moving away of the target is negatively valued
	// ---------------------------------------------------------------------------
	const double dist_eucl_final = std::sqrt(square(WS_Target.x- pose.x)+square(WS_Target.y- pose.y));
	const double dist_eucl_now   = std::sqrt(square(WS_Target.x)+square(WS_Target.y));

	const double factor4 = min(2.0*refDist,max(0.0,((dist_eucl_now - dist_eucl_final)+refDist)))/(2*refDist);

	// Factor5: Hysteresis:
	// -----------------------------------------------------
	std::vector<double> desired_cmd;
	holonomicMovement.PTG->directionToMotionCommand( kDirection, desired_cmd);
	ASSERT_EQUAL_(m_last_vel_cmd.size(), desired_cmd.size());

	double simil_score = 1.0;
	for (size_t i=0;i<desired_cmd.size();i++)
	{
		const double scr = exp(-std::abs(desired_cmd[i] - m_last_vel_cmd[i]) / 0.20);
		mrpt::utils::keep_min(simil_score, scr);
	}
	const double factor5 = simil_score;

	// Factor6: free space
	// -----------------------------------------------------
	float aver_obs = 0;
	for (size_t i=0; i<in_TPObstacles.size(); i++)
		aver_obs += in_TPObstacles[i];

	aver_obs = aver_obs/in_TPObstacles.size();

	const double factor6 = aver_obs; // *want_v;

	//  SAVE LOG
	log.evalFactors.resize(6);
	log.evalFactors[0] = factor1;
	log.evalFactors[1] = factor2;
	log.evalFactors[2] = factor3;
	log.evalFactors[3] = factor4;
	log.evalFactors[4] = factor5;
	log.evalFactors[5] = factor6;

	if (holonomicMovement.speed == 0)
	{
		// If no movement has been found -> the worst evaluation:
		holonomicMovement.evaluation = 0;
	}
	else
	{
		if (dif<2 /* heading almost exactly towards goal */ &&
			in_TPObstacles[kDirection]*0.95f>TargetDist // and free space towards the target
			)
		{
			//	Direct path to target:
			const double normalizedDistAlongPTG = holonomicMovement.PTG->getPathDist(kDirection, nStep) / holonomicMovement.PTG->getRefDistance();

			// Return a score >1.0 so we ensure this option is preferred over any other which does not reach the target:
			holonomicMovement.evaluation = 1.0f + (1.0 - normalizedDistAlongPTG) + factor5 * weights[4] + factor6*weights[5];
		}
		else
		{
		// General case:
		holonomicMovement.evaluation = holonomicMovement.PTG->getScorePriority() *(
			factor1 * weights[0] +
			factor2 * weights[1] +
			factor3 * weights[2] +
			factor4 * weights[3] +
			factor5 * weights[4] +
			factor6 * weights[5]
			) / ( math::sum(weights));
		}
	}
}

void CAbstractPTGBasedReactive::STEP7_GenerateSpeedCommands( const THolonomicMovement &in_movement)
{
	mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "STEP7_GenerateSpeedCommands");
	try
	{
		m_cmd_vel_filterings.clear();
		if (in_movement.speed == 0)
		{
			// The robot will stop:
			m_new_vel_cmd.assign(m_new_vel_cmd.size(), 0.0);
			m_cmd_vel_filterings.push_back( m_new_vel_cmd );
		}
		else
		{
			// Take the normalized movement command:
			in_movement.PTG->directionToMotionCommand( in_movement.PTG->alpha2index( in_movement.direction ), m_new_vel_cmd);
			m_cmd_vel_filterings.push_back(m_new_vel_cmd);

			// Scale holonomic speeds to real-world one:
			m_robot.cmdVel_scale(m_new_vel_cmd, in_movement.speed);
			m_cmd_vel_filterings.push_back(m_new_vel_cmd);

			// Honor user speed limits & "blending":
			const double beta = meanExecutionPeriod / (meanExecutionPeriod + SPEEDFILTER_TAU);
			m_robot.cmdVel_limits(m_new_vel_cmd, m_last_vel_cmd, beta);
			m_cmd_vel_filterings.push_back(m_new_vel_cmd);
		}

		m_last_vel_cmd = m_new_vel_cmd; // Save for filtering in next step
	}
	catch (std::exception &e)
	{
		printf_debug("[CReactiveNavigationSystem::STEP7_NonHolonomicMovement] Exception: %s",e.what());
	}
}

void CAbstractPTGBasedReactive::loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section_prefix)
{
	MRPT_START;
	m_PTGsMustBeReInitialized = true;

	// ========= Config file section name:
	const std::string sectRob = section_prefix + std::string("ROBOT_CONFIG");
	const std::string sectCfg = section_prefix + std::string("ReactiveParams");
	const std::string sectGlobal("GLOBAL_CONFIG");

	// ========= Load parameters of this base class:
	robotName = cfg.read_string(sectRob,"Name", "MyRobot", false );

	refDistance = cfg.read_double(sectCfg,"MAX_REFERENCE_DISTANCE", refDistance, true);
	SPEEDFILTER_TAU =  cfg.read_float(sectCfg,"SPEEDFILTER_TAU", .0);
	secureDistanceStart = cfg.read_float(sectCfg,"secureDistanceStart", secureDistanceStart);
	secureDistanceEnd = cfg.read_float(sectCfg,"secureDistanceEnd", secureDistanceEnd);

	DIST_TO_TARGET_FOR_SENDING_EVENT = cfg.read_float(sectCfg, "DIST_TO_TARGET_FOR_SENDING_EVENT", DIST_TO_TARGET_FOR_SENDING_EVENT, false);
	m_badNavAlarm_AlarmTimeout = cfg.read_double(sectCfg,"ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", m_badNavAlarm_AlarmTimeout, false);

	cfg.read_vector(sectCfg, "weights", vector<float> (0), weights, 1);
	ASSERT_(weights.size()==6);

	// =========  Show configuration parameters:
	printf_debug("-------------------------------------------------------------\n");
	printf_debug("       PTG-based Reactive Navigation parameters               \n");
	printf_debug("-------------------------------------------------------------\n");

	// ========= Load Derived class-specific params:
	this->internal_loadConfigFile(cfg, section_prefix);

	// ========= Load "Global params" (must be called after initialization of PTGs in `internal_loadConfigFile()`)
	this->m_robot.loadConfigFile(cfg,sectGlobal); // robot interface params
	this->loadWaypointsParamsConfigFile(cfg,sectCfg);
	this->loadHolonomicMethodConfig(cfg,sectGlobal); // Load holonomic method params
	ASSERT_(!m_holonomicMethod.empty())

	printf_debug(" Holonomic method   = %s\n", typeid(m_holonomicMethod[0]).name());
	printf_debug(" PTG Count          = %u\n", static_cast<unsigned int>( this->getPTG_count() ) );
	printf_debug(" Reference distance = %f\n", refDistance );

	// ========= If we reached this point without an exception, all is good.
	m_init_done = true;

	MRPT_END;
}

bool CAbstractPTGBasedReactive::impl_waypoint_is_reachable(const mrpt::math::TPoint2D &wp) const
{
	MRPT_START;

	const size_t N = this->getPTG_count();
	if (N!=m_infoPerPTG.size() ||
		m_infoPerPTG_timestamp == INVALID_TIMESTAMP ||
		mrpt::system::timeDifference(m_infoPerPTG_timestamp, mrpt::system::now() ) > 0.5
		)
		return false; // We didn't run yet or obstacle info is old

	for (size_t i=0;i<N;i++)
	{
		const CParameterizedTrajectoryGenerator* ptg = getPTG(i);

		const std::vector<double> & tp_obs = m_infoPerPTG[i].TP_Obstacles; // normalized distances
		if (tp_obs.size() != ptg->getPathCount() )
			continue; // May be this PTG has not been used so far? (Target out of domain,...)
		
		int wp_k;
		double wp_norm_d;
		bool is_into_domain = ptg->inverseMap_WS2TP(wp.x,wp.y,  wp_k, wp_norm_d);
		if (!is_into_domain) 
			continue;

		ASSERT_(wp_k<int(tp_obs.size()));

		const double collision_free_dist = tp_obs[wp_k];
		if (collision_free_dist>1.01*wp_norm_d)
			return true; // free path found to target
	}

	return false; // no way found
	MRPT_END;
}
