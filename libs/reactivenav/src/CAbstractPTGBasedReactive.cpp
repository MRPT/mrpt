/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header

#include <mrpt/reactivenav/CAbstractPTGBasedReactive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/ops_containers.h> // sum()
#include <mrpt/utils/printf_vector.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/utils/CFileOutputStream.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;
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
CAbstractPTGBasedReactive::CAbstractPTGBasedReactive(CReactiveInterfaceImplementation &react_iterf_impl, bool enableConsoleOutput, bool enableLogFile):
	CAbstractReactiveNavigationSystem(react_iterf_impl),
	m_holonomicMethod            (),
	m_logFile                    (NULL),
	m_enableKeepLogRecords       (false),
	last_cmd_v                   (0),
	last_cmd_w                   (0),
	new_cmd_v                    (0),
	new_cmd_w                    (0),
	navigationEndEventSent       (false),
	m_enableConsoleOutput        (enableConsoleOutput),
	m_init_done                  (false),
	refDistance                  (4.0f),
	colGridRes                   (0.10f),
	robotMax_V_mps               (1.0f),
	robotMax_W_degps             (50.0f),
	SPEEDFILTER_TAU              (0.0f),
	secureDistanceStart          (0.05f),
	secureDistanceEnd            (0.20f),
	DIST_TO_TARGET_FOR_SENDING_EVENT(0.4f),
	meanExecutionPeriod          (0.1f),
	m_timelogger                 (false), // default: disabled
	badNavAlarm_AlarmTimeout     (30.0f),
	m_collisionGridsMustBeUpdated(true),
	meanExecutionTime            (0.1f),
	meanTotalExecutionTime       (0.1f),
	m_closing_navigator          (false)
{
	MRPT_UNUSED_PARAM(enableLogFile);
	this->enableLogFile( m_enableConsoleOutput );
}

void CAbstractPTGBasedReactive::preDestructor()
{
	m_closing_navigator = true;

	// Wait to end of navigation (multi-thread...)
	m_critZoneNavigating.enter();
	m_critZoneNavigating.leave();

	// Just in case.
	m_robot.stop();

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
	// Compute collision grids:
	STEP1_CollisionGridsBuilder();
}

/*---------------------------------------------------------------
						enableLogFile
  ---------------------------------------------------------------*/
void CAbstractPTGBasedReactive::enableLogFile(bool enable)
{
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

			printf_debug("[CAbstractPTGBasedReactive::enableLogFile] Logging to file:");
			printf_debug(aux);
			printf_debug("\n");

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

void CAbstractPTGBasedReactive::navigate(const CAbstractReactiveNavigationSystem::TNavigationParams *params )
{
	navigationEndEventSent = false;

	// Copy data:
	mrpt::utils::delete_safe(m_navigationParams);
	m_navigationParams = params->clone();

	// Transform: relative -> absolute, if needed.
	if ( m_navigationParams->targetIsRelative )
	{
		poses::CPose2D currentPose;
		float velLineal_actual,velAngular_actual;

		if ( !m_robot.getCurrentPoseAndSpeeds(currentPose, velLineal_actual,velAngular_actual) )
		{
			doEmergencyStop("\n[CAbstractPTGBasedReactive] Error querying current robot pose to resolve relative coordinates\n");
			return;
		}

		const poses::CPose2D relTarget(m_navigationParams->target.x,m_navigationParams->target.y,m_navigationParams->targetHeading);
		poses::CPose2D absTarget;
		absTarget.composeFrom(currentPose, relTarget);

		m_navigationParams->target = mrpt::math::TPoint2D(absTarget.x(),absTarget.y());
		m_navigationParams->targetHeading = absTarget.phi();

		m_navigationParams->targetIsRelative = false; // Now it's not relative
	}

	// new state:
	m_navigationState = NAVIGATING;

	// Reset the bad navigation alarm:
	badNavAlarm_minDistTarget = 1e10f;
	badNavAlarm_lastMinDistTime = system::getCurrentTime();
}

void CAbstractPTGBasedReactive::doEmergencyStop( const char *msg )
{
	m_navigationState = NAV_ERROR;
	m_robot.stop();
	printf_debug(msg);
	printf_debug("\n");
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
	this->deleteHolonomicObjects();

	const size_t nPTGs = this->getPTG_count();
	m_holonomicMethod.resize(nPTGs);

	for (size_t i=0; i<nPTGs; i++)
	{
		switch (method)
		{
		case hmSEARCH_FOR_BEST_GAP:  m_holonomicMethod[i] = new CHolonomicND();  break;
		case hmVIRTUAL_FORCE_FIELDS: m_holonomicMethod[i] = new CHolonomicVFF(); break;
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

	// Lock
	mrpt::synch::CCriticalSectionLocker lock( &m_critZoneNavigating );

	CTimeLoggerEntry tle1(m_timelogger,"navigationStep");

	try
	{
		totalExecutionTime.Tic(); // Start timer

		const mrpt::system::TTimeStamp tim_start_iteration = mrpt::system::now();

		/* ----------------------------------------------------------------
		 	  Request current robot pose and velocities
		   ---------------------------------------------------------------- */
		poses::CPose2D curPose;
		float          curVL; // in m/s
		float          curW;  // in rad/s

		{
			CTimeLoggerEntry tle2(m_timelogger,"navigationStep.getCurrentPoseAndSpeeds");
			if ( !m_robot.getCurrentPoseAndSpeeds(curPose, curVL, curW ) )
			{
				doEmergencyStop("ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot and finishing navigation");
				return;
			}
		}

		/* ----------------------------------------------------------------
		 	  Have we reached the target location?
		   ---------------------------------------------------------------- */
		const double targetDist = curPose.distance2DTo( m_navigationParams->target.x, m_navigationParams->target.y );

		// Should "End of navigation" event be sent??
		if (!navigationEndEventSent && targetDist < DIST_TO_TARGET_FOR_SENDING_EVENT)
		{
			navigationEndEventSent = true;
			m_robot.sendNavigationEndEvent();
		}

		// Have we really reached the target?
		if ( targetDist < m_navigationParams->targetAllowedDistance )
		{
			m_robot.stop();
			m_navigationState = IDLE;
			if (m_enableConsoleOutput) printf_debug("Navigation target (%.03f,%.03f) was reached\n", m_navigationParams->target.x,m_navigationParams->target.y);

			if (!navigationEndEventSent)
			{
				navigationEndEventSent = true;
				m_robot.sendNavigationEndEvent();
			}
			return;
		}

		// Check the "no approaching the target"-alarm:
		// -----------------------------------------------------------
		if (targetDist < badNavAlarm_minDistTarget )
		{
			badNavAlarm_minDistTarget = targetDist;
			badNavAlarm_lastMinDistTime =  system::getCurrentTime();
		}
		else
		{
			// Too much time have passed?
			if ( system::timeDifference( badNavAlarm_lastMinDistTime, system::getCurrentTime() ) > badNavAlarm_AlarmTimeout)
			{
				std::cout << "\n--------------------------------------------\nWARNING: Timeout for approaching toward the target expired!! Aborting navigation!! \n---------------------------------\n";

				m_navigationState = NAV_ERROR;
				return;
			}
		}


		// Compute target location relative to current robot pose:
		// ---------------------------------------------------------------------
		const CPose2D relTarget = CPose2D(m_navigationParams->target.x,m_navigationParams->target.y,m_navigationParams->targetHeading) - curPose;

		// STEP1: Collision Grids Builder.
		// -----------------------------------------------------------------------------
		STEP1_CollisionGridsBuilder(); // Will only recompute if "m_collisionGridsMustBeUpdated==true"

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

			// Normal PTG validity filter: check if target falls into the PTG domain:
			ipf.valid_TP = ipf.valid_TP && ptg->PTG_IsIntoDomain( relTarget.x(),relTarget.y() );

			if (ipf.valid_TP)
			{
				ptg->inverseMap_WS2TP(relTarget.x(),relTarget.y(),ipf.target_k,ipf.target_dist);

				ipf.target_alpha = ptg->index2alpha(ipf.target_k);
				ipf.TP_Target.x = cos(ipf.target_alpha) * ipf.target_dist;
				ipf.TP_Target.y = sin(ipf.target_alpha) * ipf.target_dist;

				//  STEP3(b): Build TP-Obstacles
				// -----------------------------------------------------------------------------
				{
					CTimeLoggerEntry tle(m_timelogger,"navigationStep.STEP3_WSpaceToTPSpace");

					// Initialize TP-Obstacles:
					const size_t Ki = ptg->getAlfaValuesCount();
					ipf.TP_Obstacles.resize( Ki );
					for (size_t k=0;k<Ki;k++)
					{
						ipf.TP_Obstacles[k] = ptg->refDistance;
						// if the robot ends the trajectory due to a >180deg turn, set that as the max. distance, not D_{max}:
						float phi = ptg->GetCPathPoint_phi(k,ptg->getPointsCountInCPath_k(k)-1);
						if (fabs(phi) >= M_PI* 0.95f )
							ipf.TP_Obstacles[k]= ptg->GetCPathPoint_d(k,ptg->getPointsCountInCPath_k(k)-1);
					}

					// Implementation-dependent conversion:
					STEP3_WSpaceToTPSpace(indexPTG,ipf.TP_Obstacles);

					// Distances in TP-Space are normalized to [0,1]:
					const double _refD = 1.0/ptg->refDistance;
					for (size_t i=0;i<Ki;i++) ipf.TP_Obstacles[i] *= _refD;
				}

				//  STEP4: Holonomic navigation method
				// -----------------------------------------------------------------------------
				{
					CTimeLoggerEntry tle(m_timelogger,"navigationStep.STEP4_HolonomicMethod");

					ASSERT_(m_holonomicMethod[indexPTG])
					m_holonomicMethod[indexPTG]->navigate(
						ipf.TP_Target,
						ipf.TP_Obstacles,
						ptg->getMax_V_inTPSpace(),
						holonomicMovement.direction,
						holonomicMovement.speed,
						HLFR);

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
			else
			{   // Invalid PTG (target out of reachable space):
				// - holonomicMovement= Leave default values
				HLFR = CLogFileRecord_VFF::Create();
			}

			// Logging:
			if (fill_log_record)
			{
				metaprogramming::copy_container_typecasting(ipf.TP_Obstacles, newLogRec.infoPerPTG[indexPTG].TP_Obstacles);
				newLogRec.infoPerPTG[indexPTG].PTG_desc  = ptg->getDescription();
				newLogRec.infoPerPTG[indexPTG].TP_Target = ipf.TP_Target;
				newLogRec.infoPerPTG[indexPTG].HLFR	     = HLFR;
				newLogRec.infoPerPTG[indexPTG].desiredDirection = holonomicMovement.direction;
				newLogRec.infoPerPTG[indexPTG].desiredSpeed     = holonomicMovement.speed;
				newLogRec.infoPerPTG[indexPTG].evaluation       = holonomicMovement.evaluation;
				newLogRec.infoPerPTG[indexPTG].timeForTPObsTransformation = 0;  // XXX
				newLogRec.infoPerPTG[indexPTG].timeForHolonomicMethod     = 0; // XXX
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
		if ( new_cmd_v == 0.0 && new_cmd_w == 0.0 )
		{
			m_robot.stop();
		}
		else
		{
			if ( !m_robot.changeSpeeds( new_cmd_v, new_cmd_w ) )
			{
				doEmergencyStop("\nERROR calling RobotMotionControl::changeSpeeds!! Stopping robot and finishing navigation\n");
				return;
			}
		}

		// Statistics:
		// ----------------------------------------------------
		float	executionTimeValue = (float) executionTime.Tac();
		meanExecutionTime=  0.3f * meanExecutionTime +
		                    0.7f * executionTimeValue;
		meanTotalExecutionTime=  0.3f * meanTotalExecutionTime +
		                         0.7f * ((float)totalExecutionTime.Tac() );
		meanExecutionPeriod = 0.3f * meanExecutionPeriod +
		                      0.7f * min(1.0f, (float)timerForExecutionPeriod.Tac());


		timerForExecutionPeriod.Tic();

        if (m_enableConsoleOutput)
        {
            printf_debug(
				"CMD:%.02fm/s,%.02fd/s \t"
				"T=%.01lfms Exec:%.01lfms|%.01lfms \t"
				"E=%.01lf PTG#%i\n",
		           (double)new_cmd_v,
		           (double)RAD2DEG( new_cmd_w ),
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

			newLogRec.robotOdometryPose			= curPose;
			newLogRec.WS_target_relative		= TPoint2D(relTarget.x(), relTarget.y());
			newLogRec.v							= new_cmd_v;
			newLogRec.w							= new_cmd_w;
			newLogRec.nSelectedPTG				= nSelectedPTG;
			newLogRec.executionTime				= executionTimeValue;
			newLogRec.actual_v					= curVL;
			newLogRec.actual_w					= curW;
			newLogRec.estimatedExecutionPeriod	= meanExecutionPeriod;
			newLogRec.timestamp					= tim_start_iteration;
			newLogRec.nPTGs						= nPTGs;
			newLogRec.navigatorBehavior			= nSelectedPTG;

			m_timelogger.leave("navigationStep.populate_log_info");

			//  Save to log file:
			// --------------------------------------
			m_timelogger.enter("navigationStep.write_log_file");
			if (m_logFile) (*m_logFile) << newLogRec;
			m_timelogger.leave("navigationStep.write_log_file");

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
	const std::vector<float>        & in_TPObstacles,
	const mrpt::math::TPose2D  & WS_Target,
	const mrpt::math::TPoint2D & TP_Target,
	CLogFileRecord::TInfoPerPTG & log )
{
	const double   refDist	    = holonomicMovement.PTG->refDistance;
	const double   TargetDir    = (TP_Target.x!=0 || TP_Target.y!=0) ? atan2( TP_Target.y, TP_Target.x) : 0.0;
	const int      TargetSector = static_cast<int>( holonomicMovement.PTG->alpha2index( TargetDir ) );
	const float    TargetDist   = TP_Target.norm();
	// Picked movement direction:
	const int      kDirection   = static_cast<int>( holonomicMovement.PTG->alpha2index( holonomicMovement.direction ) );

	// Coordinates of the trajectory end for the given PTG and "alpha":
	float	x,y,phi,t,d;
	d = min( in_TPObstacles[ kDirection ], 0.90f*TargetDist);
	holonomicMovement.PTG->getCPointWhen_d_Is( d, kDirection,x,y,phi,t );

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
	double t_ang = atan2( WS_Target.y - y, WS_Target.x - x );
	t_ang -= phi;
	mrpt::math::wrapToPiInPlace(t_ang);

	const double factor3 = exp(-square( t_ang / (float)(0.5f*M_PI)) );

	// Factor4:		Decrease in euclidean distance between (x,y) and the target:
	//  Moving away of the target is negatively valued
	// ---------------------------------------------------------------------------
	const double dist_eucl_final = std::sqrt(square(WS_Target.x-x)+square(WS_Target.y-y));
	const double dist_eucl_now   = std::sqrt(square(WS_Target.x)+square(WS_Target.y));

	const double factor4 = min(2.0*refDist,max(0.0,((dist_eucl_now - dist_eucl_final)+refDist)))/(2*refDist);

	// ---------
	//	float decrementDistanc = dist_eucl_now - dist_eucl_final;
	//	if (dist_eucl_now>0)
	//			factor4 = min(1.0,min(refDist*2,max(0,decrementDistanc + refDist)) / dist_eucl_now);
	//	else	factor4 = 0;
	// ---------
	//	factor4 = min(2*refDist2,max(0,decrementDistanc + refDist2)) / (2*refDist2);
	//  factor4=  (refDist2 - min( refDist2, dist_eucl ) ) / refDist2;

	// Factor5: Hysteresis:
	// -----------------------------------------------------
	float	want_v,want_w;
	holonomicMovement.PTG->directionToMotionCommand( kDirection, want_v,want_w);

	float	likely_v = exp( -fabs(want_v-last_cmd_v)/0.10f );
	float	likely_w = exp( -fabs(want_w-last_cmd_w)/0.40f );

	const double factor5 = min( likely_v,likely_w );


	// Factor6: Fast when free space
	// -----------------------------------------------------
	float aver_obs = 0;
	for (size_t i=0; i<in_TPObstacles.size(); i++)
		aver_obs += in_TPObstacles[i];

	aver_obs = aver_obs/in_TPObstacles.size();

	const double factor6 = aver_obs*want_v;

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
		// Sum: two cases: *************************************I'm not sure about this...
		if (dif<2	&&											// Heading the target
			    in_TPObstacles[kDirection]*0.95f>TargetDist 	// and free space towards the target
			)
		{
			//	Direct path to target:
			//	holonomicMovement.evaluation = 1.0f + (1 - TargetDist) + factor5 * weight5 + factor6*weight6;
			holonomicMovement.evaluation = 1.0f + (1 - t/15.0f) + factor5 * weights[4] + factor6*weights[5];
		}
		else
		{
		// General case:
		holonomicMovement.evaluation = (
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
	try
	{

		last_cmd_v = new_cmd_v;
		last_cmd_w = new_cmd_w;

		if (in_movement.speed == 0)
		{
			// The robot will stop:
			new_cmd_v = new_cmd_w = 0;
		}
		else
		{
			// Take the normalized movement command:
			in_movement.PTG->directionToMotionCommand(
			    in_movement.PTG->alpha2index( in_movement.direction ),
			    new_cmd_v,
			    new_cmd_w );

			// Scale holonomic speeds to real-world one:
			//const double reduction = min(1.0, in_movement.speed / sqrt( square(new_cmd_v) + square(r*new_cmd_w)));
			double reduction = in_movement.speed;
			if (reduction < 0.5)
				reduction = 0.5;

			// To scale:
			new_cmd_v*=reduction;
			new_cmd_w*=reduction;

			// Assure maximum speeds:
			if (fabs(new_cmd_v) > robotMax_V_mps)
			{
				// Scale:
				float F = fabs(robotMax_V_mps / new_cmd_v);
				new_cmd_v *= F;
				new_cmd_w *= F;
			}

			if (fabs(new_cmd_w) > DEG2RAD(robotMax_W_degps))
			{
				// Scale:
				float F = fabs((float)DEG2RAD(robotMax_W_degps) / new_cmd_w);
				new_cmd_v *= F;
				new_cmd_w *= F;
			}

			//First order low-pass filter
			float alfa = meanExecutionPeriod/( meanExecutionPeriod + SPEEDFILTER_TAU);
			new_cmd_v = alfa*new_cmd_v + (1-alfa)*last_cmd_v;
			new_cmd_w = alfa*new_cmd_w + (1-alfa)*last_cmd_w;

		}
		m_timelogger.leave("navigationStep.STEP7_NonHolonomicMovement");
	}
	catch (std::exception &e)
	{
		m_timelogger.leave("navigationStep.STEP7_NonHolonomicMovement");
		printf_debug("[CReactiveNavigationSystem::STEP7_NonHolonomicMovement] Exception:");
		printf_debug((char*)(e.what()));
	}
	catch (...)
	{
		m_timelogger.leave("navigationStep.STEP7_NonHolonomicMovement");
		printf_debug("[CReactiveNavigationSystem::STEP7_NonHolonomicMovement] Unexpected exception!\n");
	}
}
