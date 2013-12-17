/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/reactivenav.h>  // Precomp header

#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;
using namespace std;

CAbstractPTGBasedReactive::CAbstractPTGBasedReactive(CReactiveInterfaceImplementation &react_iterf_impl, bool enableConsoleOutput, bool enableLogFile):
	CAbstractReactiveNavigationSystem(react_iterf_impl),
	last_cmd_v                   (0),
	last_cmd_w                   (0),
	new_cmd_v                    (0),
	new_cmd_w                    (0),
	navigationEndEventSent       (false),
	m_holonomicMethod            (),
	logFile                      (NULL),
	m_enableConsoleOutput        (enableConsoleOutput),
	m_init_done                  (false),
	meanExecutionPeriod          (0.1f),
	m_timelogger                 (false), // default: disabled
	m_collisionGridsMustBeUpdated(true),
	meanExecutionTime            (0.1f),
	meanTotalExecutionTime       (0.1f),
	m_closing_navigator          (false)
{
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

	mrpt::utils::delete_safe(logFile);

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
			if (logFile)
			{
				printf_debug("[CAbstractPTGBasedReactive::enableLogFile] Stopping logging.\n");
				// Close file:
				delete logFile;
				logFile = NULL;
			}
			else return;	// Already disabled.
		}
		else
		{	// Enable
			// -------------------------------
			if (logFile) return; // Already enabled:

			// Open file, find the first free file-name.
			char	aux[100];
			int     nFile = 0;
			bool    free_name = false;

			system::createDirectory("./reactivenav.logs");

			while (!free_name)
			{
				nFile++;
				sprintf(aux, "./reactivenav.logs/log_%03u.reactivenavlog", nFile );
				free_name = !system::fileExists(aux);
			}

			// Open log file:
			logFile = new CFileOutputStream(aux);

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

void CAbstractPTGBasedReactive::navigate(const CReactiveNavigationSystem3D::TNavigationParams &params )
{
	navigationEndEventSent = false;

	// Copy data:
	m_navigationParams = params;

	// Transform: relative -> absolute, if needed.
	if ( m_navigationParams.targetIsRelative )
	{
		poses::CPose2D currentPose;
		float velLineal_actual,velAngular_actual;

		if ( !m_robot.getCurrentPoseAndSpeeds(currentPose, velLineal_actual,velAngular_actual) )
		{
			doEmergencyStop("\n[CAbstractPTGBasedReactive] Error querying current robot pose to resolve relative coordinates\n");
			return;
		}

		const poses::CPose2D relTarget(m_navigationParams.target.x,m_navigationParams.target.y,m_navigationParams.targetHeading);
		poses::CPose2D absTarget;
		absTarget.composeFrom(currentPose, relTarget);

		m_navigationParams.target = mrpt::math::TPoint2D(absTarget.x(),absTarget.y());
		m_navigationParams.targetHeading = absTarget.phi();

		m_navigationParams.targetIsRelative = false; // Now it's not relative
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
	THolonomicMethod holoMethod = ini.read_enum<THolonomicMethod>("GLOBAL_CONFIG","HOLONOMIC_METHOD",hmVIRTUAL_FORCE_FIELDS, true);
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



void CAbstractPTGBasedReactive::performNavigationStep()
{
	// Security tests:
	if (m_closing_navigator) return;  // Are we closing in the main thread?
	if (!m_init_done) THROW_EXCEPTION("Have you called loadConfigFile() before?")

	const size_t nPTGs = this->getPTG_count();

	std::vector<CHolonomicLogFileRecordPtr> HLFRs(nPTGs);
	//int											nSelectedPTG;
	//THolonomicMovement							selectedHolonomicMovement;

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
		const double targetDist = curPose.distance2DTo( m_navigationParams.target.x, m_navigationParams.target.y );

		// Should "End of navigation" event be sent??
		if (!navigationEndEventSent && targetDist < DIST_TO_TARGET_FOR_SENDING_EVENT)
		{
			navigationEndEventSent = true;
			m_robot.sendNavigationEndEvent();
		}

		// Have we really reached the target?
		if ( targetDist < m_navigationParams.targetAllowedDistance )
		{
			m_robot.stop();
			m_navigationState = IDLE;
			if (m_enableConsoleOutput) printf_debug("Navigation target (%.03f,%.03f) was reached\n", m_navigationParams.target.x,m_navigationParams.target.y);

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
		const CPose2D relTarget = CPose2D(m_navigationParams.target.x,m_navigationParams.target.y,m_navigationParams.targetHeading) - curPose;

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


		// Clip obstacles out of the reactive method range:
		//CPoint2D    dumm(0,0);
		//WS_Obstacles.clipOutOfRange( dumm, refDistance+1.5f );
		XXX

		//  STEP3: Build TP-Obstacles and transform target location into TP-Space
		// -----------------------------------------------------------------------------
		STEP3_WSpaceToTPSpace(relTarget);


		//  STEP4: Holonomic navigation method
		// -----------------------------------------------------------------------------
		STEP4_HolonomicMethod(HLFRs);


		// STEP5: Evaluate each movement to assign them a "evaluation" value.
		// ---------------------------------------------------------------------
		// For each PTG:
		for (unsigned int i=0; i<m_ptgmultilevel.size(); i++)
		{
			m_ptgmultilevel[i].holonomicmov.PTG = m_ptgmultilevel[i].PTGs[0];
            CPoint2D TP_Target_i = CPoint2D(m_ptgmultilevel[i].TP_Target);
			STEP5_PTGEvaluator(	m_ptgmultilevel[i].holonomicmov,
								m_ptgmultilevel[i].TPObstacles,
								relTarget,
								TP_Target_i,
								newLogRec.infoPerPTG[i]);
		}



		// STEP6: Selects the best movement
		// ---------------------------------------------------------------------
		STEP6_Selector( selectedHolonomicMovement, nSelectedPTG );
		

		// STEP7: Get the non-holonomic movement command.
		// ---------------------------------------------------------------------
		STEP7_GenerateSpeedCommands( selectedHolonomicMovement );

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
		m_timelogger.enter("navigationStep.populate_log_info");

		CSimplePointsMap auxpointmap;

		//Include the points of all levels (this could be improved depending on STEP2)
		for (unsigned int i=0; i < WS_Obstacles_inlevels.size(); i++)
			auxpointmap += *WS_Obstacles_inlevels[i].getAsSimplePointsMap();

		newLogRec.WS_Obstacles				= auxpointmap;
		newLogRec.robotOdometryPose			= curPose;
		newLogRec.WS_target_relative		= relTarget;
		newLogRec.v							= new_cmd_v;
		newLogRec.w							= new_cmd_w;
		newLogRec.nSelectedPTG				= nSelectedPTG;
		newLogRec.executionTime				= executionTimeValue;
		newLogRec.actual_v					= curVL;
		newLogRec.actual_w					= curW;
		newLogRec.estimatedExecutionPeriod	= meanExecutionPeriod;
		newLogRec.timestamp					= tim_start_iteration;
		newLogRec.nPTGs						= m_ptgmultilevel.size();
		newLogRec.navigatorBehavior			= holonomicMethodSel;


		//Polygons of each height level are drawn (but they are all shown connected...)
		if (newLogRec.robotShape_x.size() == 0)
		{
			size_t nVerts = 0;
			TPoint2D paux;
			size_t cuenta = 0;
			for (unsigned int i=0; i < m_robotShape.heights.size(); i++)
				nVerts += m_robotShape.polygons[i].size() + 1;
			if (size_t(newLogRec.robotShape_x.size()) != nVerts)
			{
				newLogRec.robotShape_x.resize(nVerts);
				newLogRec.robotShape_y.resize(nVerts);
			}
			for (unsigned int i=0; i<m_robotShape.heights.size(); i++)
			{
				for (unsigned int j=0; j<m_robotShape.polygons[i].size(); j++)
				{
					paux = m_robotShape.polygons[i][j];
					newLogRec.robotShape_x[cuenta]= paux.x;
					newLogRec.robotShape_y[cuenta]= paux.y;
					cuenta++;
				}
				paux = m_robotShape.polygons[i][0];
				newLogRec.robotShape_x[cuenta]= paux.x;
				newLogRec.robotShape_y[cuenta]= paux.y;
				cuenta++;
			}
		}


		// For each PTG:
		for (size_t i = 0; i< m_ptgmultilevel.size(); i++)
		{
			//sizeobs = m_ptgmultilevel[i].TPObstacles.size();
			metaprogramming::copy_container_typecasting(m_ptgmultilevel[i].TPObstacles, newLogRec.infoPerPTG[i].TP_Obstacles);
			newLogRec.infoPerPTG[i].PTG_desc					= m_ptgmultilevel[i].PTGs[0]->getDescription();
			newLogRec.infoPerPTG[i].TP_Target					= m_ptgmultilevel[i].TP_Target;
			newLogRec.infoPerPTG[i].timeForTPObsTransformation	= 0;
			newLogRec.infoPerPTG[i].timeForHolonomicMethod		= 0;
			newLogRec.infoPerPTG[i].HLFR						= HLFRs[i];
			newLogRec.infoPerPTG[i].desiredDirection			= m_ptgmultilevel[i].holonomicmov.direction;
			newLogRec.infoPerPTG[i].desiredSpeed				= m_ptgmultilevel[i].holonomicmov.speed;
			newLogRec.infoPerPTG[i].evaluation					= m_ptgmultilevel[i].holonomicmov.evaluation;
		}

		m_timelogger.leave("navigationStep.populate_log_info");


		// --------------------------------------
		//  Save to log file:
		// --------------------------------------
		m_timelogger.enter("navigationStep.write_log_file");
		if (logFile) (*logFile) << newLogRec;
		m_timelogger.leave("navigationStep.write_log_file");
		
		// Set as last log record
		{   // Lock
			mrpt::synch::CCriticalSectionLocker lock_log(&m_critZoneLastLog);
			// COPY
			lastLogRecord = newLogRec;
		}
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
