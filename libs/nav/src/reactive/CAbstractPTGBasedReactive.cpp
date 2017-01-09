/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
#include <iomanip>

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

const double ESTIM_LOWPASSFILTER_ALPHA = 0.7;

// Ctor:
CAbstractPTGBasedReactive::CAbstractPTGBasedReactive(CRobot2NavInterface &react_iterf_impl, bool enableConsoleOutput, bool enableLogFile):
	CWaypointsNavigator(react_iterf_impl),
	m_holonomicMethod            (),
	m_logFile                    (NULL),
	m_enableKeepLogRecords       (false),

	m_enableConsoleOutput        (enableConsoleOutput),
	m_init_done                  (false),
	ptg_cache_files_directory    ("."),
	refDistance                  (4.0f),
	SPEEDFILTER_TAU              (0.0),
	secureDistanceStart          (0.05),
	secureDistanceEnd            (0.20),
	USE_DELAYS_MODEL             (false),
	MAX_DISTANCE_PREDICTED_ACTUAL_PATH(0.05),
	m_timelogger                 (false), // default: disabled
	m_PTGsMustBeReInitialized    (true),
	meanExecutionTime            (ESTIM_LOWPASSFILTER_ALPHA, 0.1),
	meanTotalExecutionTime       (ESTIM_LOWPASSFILTER_ALPHA, 0.1),
	meanExecutionPeriod          (ESTIM_LOWPASSFILTER_ALPHA, 0.1),
	tim_changeSpeed_avr          (ESTIM_LOWPASSFILTER_ALPHA),
	timoff_obstacles_avr         (ESTIM_LOWPASSFILTER_ALPHA),
	timoff_curPoseAndSpeed_avr   (ESTIM_LOWPASSFILTER_ALPHA),
	timoff_sendVelCmd_avr        (ESTIM_LOWPASSFILTER_ALPHA),
	m_closing_navigator          (false),
	m_WS_Obstacles_timestamp     (INVALID_TIMESTAMP),
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
		this->stop(false /*not emergency*/);
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
				MRPT_LOG_DEBUG("[CAbstractPTGBasedReactive::enableLogFile] Stopping logging.");
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

			MRPT_LOG_DEBUG(mrpt::format("[CAbstractPTGBasedReactive::enableLogFile] Logging to file `%s`\n",aux));

		}
	} catch (...) {
		MRPT_LOG_ERROR("[CAbstractPTGBasedReactive::enableLogFile] Exception!!");
	}

}

void CAbstractPTGBasedReactive::getLastLogRecord( CLogFileRecord &o )
{
	mrpt::synch::CCriticalSectionLocker lock(&m_critZoneLastLog);
	o = lastLogRecord;
}

static std::string holoMethodEnum2ClassName(const THolonomicMethod method)
{
	std::string className;
	switch (method)
	{
	case hmSEARCH_FOR_BEST_GAP:  className = "CHolonomicND";  break;
	case hmVIRTUAL_FORCE_FIELDS: className = "CHolonomicVFF"; break;
	case hmFULL_EVAL: className = "CHolonomicFullEval"; break;
	default: THROW_EXCEPTION_CUSTOM_MSG1("Unknown Holonomic method: %u", static_cast<unsigned int>(method))
	};
	return className;
}

void CAbstractPTGBasedReactive::loadHolonomicMethodConfig(
	const mrpt::utils::CConfigFileBase &ini,
	const std::string &section )
{
	std::string  holoMethodName = ini.read_string(section, "HOLONOMIC_METHOD", "", true);

	// Backwards compatible with numeric entries:
	if (!holoMethodName.empty() && isdigit(holoMethodName[0])) {
		holoMethodName = holoMethodEnum2ClassName(static_cast<THolonomicMethod>(atoi(&holoMethodName[0])));
	}
	// Backwards compatible with enum type name:
	try {
		holoMethodName = holoMethodEnum2ClassName(mrpt::utils::TEnumType<THolonomicMethod>::name2value(holoMethodName));
	}
	catch (...) {
		// Ignore exception if string is not an enum.
	}

	this->setHolonomicMethod(holoMethodName, ini );
}

void CAbstractPTGBasedReactive::deleteHolonomicObjects()
{
	for (size_t i=0;i<m_holonomicMethod.size();i++)
		delete m_holonomicMethod[i];
	m_holonomicMethod.clear();
}

void CAbstractPTGBasedReactive::setHolonomicMethod(const std::string & method, const mrpt::utils::CConfigFileBase & ini)
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	this->deleteHolonomicObjects();
	const size_t nPTGs = this->getPTG_count();
	ASSERT_(nPTGs != 0);
	m_holonomicMethod.resize(nPTGs);

	for (size_t i = 0; i<nPTGs; i++)
	{
		m_holonomicMethod[i] = CAbstractHolonomicReactiveMethod::Create(method);
		if (!m_holonomicMethod[i])
			THROW_EXCEPTION_CUSTOM_MSG1("Non-registered holonomic method className=`%s`", method.c_str());

		m_holonomicMethod[i]->setAssociatedPTG(this->getPTG(i));
		m_holonomicMethod[i]->initialize(ini); // load params
	}
}

void CAbstractPTGBasedReactive::setHolonomicMethod(const THolonomicMethod method, const mrpt::utils::CConfigFileBase &ini)
{
	setHolonomicMethod(holoMethodEnum2ClassName(method), ini);
}

// The main method: executes one time-iteration of the reactive navigation algorithm.
void CAbstractPTGBasedReactive::performNavigationStep()
{
	// Security tests:
	if (m_closing_navigator) return;  // Are we closing in the main thread?
	if (!m_init_done) THROW_EXCEPTION("Have you called loadConfigFile() before?")
	ASSERT_(m_navigationParams)

	const size_t nPTGs = this->getPTG_count();

	// Whether to worry about log files:
	const bool fill_log_record = (m_logFile!=NULL || m_enableKeepLogRecords);
	CLogFileRecord newLogRec;
	newLogRec.infoPerPTG.resize(nPTGs+1); /* +1: [N] is the "NOP cmdvel" option; not to be present in all log entries. */

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
		const TPose2D relTarget = TPose2D(CPose2D(m_navigationParams->target) - CPose2D(m_curPoseVel.pose));

		// Detect changes in target since last iteration (for NOP):
		const bool target_changed_since_last_iteration = m_navigationParams->target != m_lastTarget;
		m_lastTarget = m_navigationParams->target;

		STEP1_InitPTGs(); // Will only recompute if "m_PTGsMustBeReInitialized==true"

		// Update kinematic state in all PTGs:
		for (size_t i=0;i<nPTGs;i++)
			getPTG(i)->updateCurrentRobotVel(m_curPoseVel.velLocal);

		// STEP2: Load the obstacles and sort them in height bands.
		// -----------------------------------------------------------------------------
		if (! STEP2_SenseObstacles() )
		{
			doEmergencyStop("Error while loading and sorting the obstacles. Robot will be stopped.");
			if (fill_log_record)
			{
				CPose2D rel_cur_pose_wrt_last_vel_cmd_NOP, rel_pose_PTG_origin_wrt_sense_NOP;
				STEP8_GenerateLogRecord(newLogRec,
					relTarget,
					-1, // nSelectedPTG,
					m_robot.getEmergencyStopCmd(),
					nPTGs,
					false, //best_is_NOP_cmdvel,
					rel_cur_pose_wrt_last_vel_cmd_NOP,
					rel_pose_PTG_origin_wrt_sense_NOP,
					0, //executionTimeValue,
					0, //tim_changeSpeed,
					tim_start_iteration);
			}
			return;
		}

		// ------- start of motion decision zone ---------
		executionTime.Tic();

		// Round #1: As usual, pure reactive, evaluate all PTGs and all directions from scratch.
		// =========

		CPose2D rel_pose_PTG_origin_wrt_sense,relPoseSense, relPoseVelCmd;
		if (USE_DELAYS_MODEL)
		{
			/*
			*                                          Delays model
			*
			* Event:          OBSTACLES_SENSED         RNAV_ITERATION_STARTS       GET_ROBOT_POSE_VEL         VEL_CMD_SENT_TO_ROBOT
			* Timestamp:  (m_WS_Obstacles_timestamp)   (tim_start_iteration)     (m_curPoseVelTimestamp)     ("tim_send_cmd_vel")
			* Delay                       | <---+--------------->|<--------------+-------->|                         |
			* estimator:                        |                |               |                                   |
			*                timoff_obstacles <-+                |               +--> timoff_curPoseVelAge           |
			*                                                    |<---------------------------------+--------------->|
			*                                                                                       +--> timoff_sendVelCmd_avr (estimation)
			*                                                                                                |<-------------------->|
			*                                                                                                   tim_changeSpeed_avr (estim)
			*
			*                             |<-----------------------------------------------|-------------------------->|
			*  Relative poses:                              relPoseSense                           relPoseVelCmd
			*  Time offsets (signed):                       timoff_pose2sense                     timoff_pose2VelCmd
			*/
			const double timoff_obstacles = mrpt::system::timeDifference(tim_start_iteration, m_WS_Obstacles_timestamp);
			timoff_obstacles_avr.filter(timoff_obstacles);
			newLogRec.values["timoff_obstacles"] = timoff_obstacles;
			newLogRec.values["timoff_obstacles_avr"] = timoff_obstacles_avr.getLastOutput();
			newLogRec.timestamps["obstacles"] = m_WS_Obstacles_timestamp;

			const double timoff_curPoseVelAge = mrpt::system::timeDifference(tim_start_iteration, m_curPoseVel.timestamp);
			timoff_curPoseAndSpeed_avr.filter(timoff_curPoseVelAge);
			newLogRec.values["timoff_curPoseVelAge"] = timoff_curPoseVelAge;
			newLogRec.values["timoff_curPoseVelAge_avr"] = timoff_curPoseAndSpeed_avr.getLastOutput();

			// time offset estimations:
			const double timoff_pose2sense = timoff_obstacles - timoff_curPoseVelAge;
			const double timoff_pose2VelCmd = timoff_sendVelCmd_avr.getLastOutput() + 0.5*tim_changeSpeed_avr.getLastOutput() - timoff_curPoseVelAge;
			newLogRec.values["timoff_pose2sense"] = timoff_pose2sense;
			newLogRec.values["timoff_pose2VelCmd"] = timoff_pose2VelCmd;

			if (std::abs(timoff_pose2sense) > 0.5) MRPT_LOG_WARN_FMT("timoff_pose2sense=%e is too large! Path extrapolation may be not accurate.", timoff_pose2sense);
			if (std::abs(timoff_pose2VelCmd) > 0.5) MRPT_LOG_WARN_FMT("timoff_pose2VelCmd=%e is too large! Path extrapolation may be not accurate.", timoff_pose2VelCmd);

			// Path extrapolation: robot relative poses along current path estimation:
			robotPoseExtrapolateIncrement(m_curPoseVel.velLocal, timoff_pose2sense, relPoseSense);
			robotPoseExtrapolateIncrement(m_curPoseVel.velLocal, timoff_pose2VelCmd, relPoseVelCmd);
			// relative pose for PTGs:
			rel_pose_PTG_origin_wrt_sense = relPoseVelCmd - relPoseSense;

			// logging:
			newLogRec.relPoseSense = relPoseSense;
			newLogRec.relPoseVelCmd = relPoseVelCmd;
		}
		else {
			// No delays model: poses to their default values.
		}

		m_infoPerPTG.resize(nPTGs);
		m_infoPerPTG_timestamp = tim_start_iteration;
		vector<THolonomicMovement> holonomicMovements(nPTGs+1); // the last extra one is for the evaluation of "NOP motion command" choice.

		for (size_t indexPTG=0;indexPTG<nPTGs;indexPTG++)
		{
			CParameterizedTrajectoryGenerator * ptg = getPTG(indexPTG);
			TInfoPerPTG &ipf = m_infoPerPTG[indexPTG];

			// The picked movement in TP-Space (to be determined by holonomic method below)
			THolonomicMovement &holonomicMovement = holonomicMovements[indexPTG];

			ASSERT_(m_navigationParams);
			ptg_eval_target_build_obstacles(
				ptg, indexPTG,
				relTarget, rel_pose_PTG_origin_wrt_sense,
				ipf, holonomicMovement,
				newLogRec, false /* this is a regular PTG reactive case */,
				m_holonomicMethod[indexPTG],
				*m_navigationParams);
		} // end for each PTG

		// Round #2: Evaluate dont sending any new velocity command ("NOP" motion)
		// =========
		// This approach is only possible if:
		const bool can_do_nop_motion = (m_lastSentVelCmd.isValid() &&
			!target_changed_since_last_iteration &&
			getPTG(m_lastSentVelCmd.ptg_index)->supportVelCmdNOP()) &&
			mrpt::system::timeDifference(m_lastSentVelCmd.tim_send_cmd_vel, tim_start_iteration) < getPTG(m_lastSentVelCmd.ptg_index)->maxTimeInVelCmdNOP(m_lastSentVelCmd.ptg_alpha_index);

		CPose2D rel_cur_pose_wrt_last_vel_cmd_NOP, rel_pose_PTG_origin_wrt_sense_NOP;

		if (can_do_nop_motion)
		{
			CPose3D robot_pose3d_at_send_cmd;
			bool valid_pose;
			m_latestPoses.interpolate(m_lastSentVelCmd.tim_send_cmd_vel, robot_pose3d_at_send_cmd, valid_pose);
			if (valid_pose)
			{
				const CPose2D robot_pose_at_send_cmd = CPose2D(robot_pose3d_at_send_cmd);

				CParameterizedTrajectoryGenerator * ptg = getPTG(m_lastSentVelCmd.ptg_index);
				ptg->updateCurrentRobotVel(m_lastSentVelCmd.curRobotVelLocal);

				TInfoPerPTG ipf_NOP;
				const TPose2D relTarget_NOP = TPose2D(CPose2D(m_navigationParams->target) - robot_pose_at_send_cmd);
				rel_pose_PTG_origin_wrt_sense_NOP = robot_pose_at_send_cmd - (CPose2D(m_curPoseVel.pose) + relPoseSense);
				rel_cur_pose_wrt_last_vel_cmd_NOP = CPose2D(m_curPoseVel.pose) - robot_pose_at_send_cmd;

				if (fill_log_record)
				{
					newLogRec.additional_debug_msgs["rel_cur_pose_wrt_last_vel_cmd_NOP"] = rel_cur_pose_wrt_last_vel_cmd_NOP.asString();
					newLogRec.additional_debug_msgs["robot_pose_at_send_cmd"] = robot_pose_at_send_cmd.asString();
				}

				ASSERT_(m_navigationParams);
				ptg_eval_target_build_obstacles(
					ptg, m_lastSentVelCmd.ptg_index,
					relTarget_NOP, rel_pose_PTG_origin_wrt_sense_NOP,
					ipf_NOP, holonomicMovements[nPTGs],
					newLogRec, true /* this is the PTG continuation (NOP) choice */,
					m_holonomicMethod[m_lastSentVelCmd.ptg_index],
					*m_navigationParams,
					rel_cur_pose_wrt_last_vel_cmd_NOP);

			} // end valid interpolated origin pose
		} //end can_do_NOP_motion

		// STEP6: After all PTGs have been evaluated, pick the best scored:
		// ---------------------------------------------------------------------
		int nSelectedPTG = 0;
		double best_PTG_eval = .0;
		for (size_t indexPTG=0;indexPTG<=nPTGs;indexPTG++) {
			if (holonomicMovements[indexPTG].evaluation > best_PTG_eval) {
				nSelectedPTG = indexPTG;
				best_PTG_eval = holonomicMovements[nSelectedPTG].evaluation;
			}
		}
		const THolonomicMovement & selectedHolonomicMovement = holonomicMovements[nSelectedPTG];

		// If the selected PTG is (N+1), it means the NOP cmd. vel is selected as the best alternative, i.e. do NOT send any new motion command.
		const bool best_is_NOP_cmdvel =  (nSelectedPTG==int(nPTGs) && best_PTG_eval>0);

		// ---------------------------------------------------------------------
		//				SEND MOVEMENT COMMAND TO THE ROBOT
		// ---------------------------------------------------------------------
		mrpt::kinematics::CVehicleVelCmdPtr new_vel_cmd;
		if (best_is_NOP_cmdvel)
		{
			// Notify the robot that we want it to keep executing the last cmdvel:
			if (!this->changeSpeedsNOP())
			{
				doEmergencyStop("\nERROR calling changeSpeedsNOP()!! Stopping robot and finishing navigation\n");
				if(fill_log_record)
				{
					STEP8_GenerateLogRecord(newLogRec,
						relTarget,
						nSelectedPTG,
						m_robot.getEmergencyStopCmd(),
						nPTGs,
						best_is_NOP_cmdvel,
						rel_cur_pose_wrt_last_vel_cmd_NOP,
						rel_pose_PTG_origin_wrt_sense_NOP,
						0, //executionTimeValue,
						0, //tim_changeSpeed,
						tim_start_iteration);
				}
				return;
			}
		}
		else
		{
			// STEP7: Get the non-holonomic movement command.
			// ---------------------------------------------------------------------
			if (best_PTG_eval>0)
			{
				CTimeLoggerEntry tle(m_timelogger, "navigationStep.STEP7_NonHolonomicMovement");
				STEP7_GenerateSpeedCommands(selectedHolonomicMovement, new_vel_cmd);
				ASSERT_(new_vel_cmd);
			}

			if (!new_vel_cmd /* which means best_PTG_eval==.0*/ || new_vel_cmd->isStopCmd()) {
				MRPT_LOG_DEBUG("Best velocity command is STOP (no way found), calling robot.stop()");
				this->stop(true /* emergency */);  // don't call doEmergencyStop() here since that will stop navigation completely
				new_vel_cmd = m_robot.getEmergencyStopCmd();
				m_lastSentVelCmd.reset();
			}
			else
			{
				mrpt::system::TTimeStamp tim_send_cmd_vel;
				{
					mrpt::utils::CTimeLoggerEntry tle(m_timlog_delays, "changeSpeeds()");
					tim_send_cmd_vel = mrpt::system::now();
					newLogRec.timestamps["tim_send_cmd_vel"] = tim_send_cmd_vel;
					if (!this->changeSpeeds(*new_vel_cmd))
					{
						doEmergencyStop("\nERROR calling changeSpeeds()!! Stopping robot and finishing navigation\n");
						if (fill_log_record)
						{
							new_vel_cmd = m_robot.getEmergencyStopCmd();
							STEP8_GenerateLogRecord(newLogRec,
								relTarget,
								nSelectedPTG,
								new_vel_cmd,
								nPTGs,
								best_is_NOP_cmdvel,
								rel_cur_pose_wrt_last_vel_cmd_NOP,
								rel_pose_PTG_origin_wrt_sense_NOP,
								0, //executionTimeValue,
								0, //tim_changeSpeed,
								tim_start_iteration);
						}
						return;
					}
				}
				// Save last sent cmd:
				m_lastSentVelCmd.ptg_index = nSelectedPTG;
				m_lastSentVelCmd.ptg_alpha_index = selectedHolonomicMovement.PTG->alpha2index(selectedHolonomicMovement.direction);
				m_lastSentVelCmd.tim_poseVel = m_curPoseVel.timestamp;
				m_lastSentVelCmd.tim_send_cmd_vel = tim_send_cmd_vel;
				m_lastSentVelCmd.curRobotVelLocal = m_curPoseVel.velLocal;
				m_lastSentVelCmd.curRobotPose = m_curPoseVel.pose;


				// Update delay model:
				const double timoff_sendVelCmd = mrpt::system::timeDifference(tim_start_iteration, tim_send_cmd_vel);
				timoff_sendVelCmd_avr.filter(timoff_sendVelCmd);
				newLogRec.values["timoff_sendVelCmd"] = timoff_sendVelCmd;
				newLogRec.values["timoff_sendVelCmd_avr"] = timoff_sendVelCmd_avr.getLastOutput();
			}
		}

		// ------- end of motion decision zone ---------


		// Statistics:
		// ----------------------------------------------------
		const double executionTimeValue = executionTime.Tac();
		meanExecutionTime.filter(executionTimeValue);
		meanTotalExecutionTime.filter(totalExecutionTime.Tac());

		const double tim_changeSpeed = m_timlog_delays.getLastTime("changeSpeeds()");
		tim_changeSpeed_avr.filter(tim_changeSpeed);

		// Running period estim:
		meanExecutionPeriod.filter( timerForExecutionPeriod.Tac());
		timerForExecutionPeriod.Tic();

		if (m_enableConsoleOutput)
		{
			MRPT_LOG_DEBUG(mrpt::format(
				"CMD: %s "
				"speedScale=%.04f "
				"T=%.01lfms Exec:%.01lfms|%.01lfms "
				"E=%.01lf PTG#%i\n",
					new_vel_cmd ? new_vel_cmd->asString().c_str() : "NOP",
					selectedHolonomicMovement.speed,
					1000.0*meanExecutionPeriod.getLastOutput(),
					1000.0*meanExecutionTime.getLastOutput(),
					1000.0*meanTotalExecutionTime.getLastOutput(),
					best_PTG_eval,
					nSelectedPTG
					) );
		}
		if (fill_log_record)
		{
			STEP8_GenerateLogRecord(newLogRec,
				relTarget,
				nSelectedPTG,
				new_vel_cmd,
				nPTGs,
				best_is_NOP_cmdvel,
				rel_cur_pose_wrt_last_vel_cmd_NOP,
				rel_pose_PTG_origin_wrt_sense_NOP,
				executionTimeValue,
				tim_changeSpeed,
				tim_start_iteration);
		}
	}
	catch (std::exception &e) {
		doEmergencyStop(std::string("[CAbstractPTGBasedReactive::performNavigationStep] Stopping robot and finishing navigation due to exception:\n") + std::string(e.what()));
	}
	catch (...) {
		doEmergencyStop("[CAbstractPTGBasedReactive::performNavigationStep] Stopping robot and finishing navigation due to untyped exception." );
	}
}


void CAbstractPTGBasedReactive::STEP8_GenerateLogRecord(CLogFileRecord &newLogRec,const TPose2D& relTarget,int nSelectedPTG, const mrpt::kinematics::CVehicleVelCmdPtr &new_vel_cmd, const int nPTGs, const bool best_is_NOP_cmdvel, const mrpt::poses::CPose2D &rel_cur_pose_wrt_last_vel_cmd_NOP, const mrpt::poses::CPose2D &rel_pose_PTG_origin_wrt_sense_NOP, const double executionTimeValue, const double tim_changeSpeed, const mrpt::system::TTimeStamp &tim_start_iteration)
{
	// ---------------------------------------
	// STEP8: Generate log record
	// ---------------------------------------
	m_timelogger.enter("navigationStep.populate_log_info");

	this->loggingGetWSObstaclesAndShape(newLogRec);

	newLogRec.robotOdometryPose   = m_curPoseVel.pose;
	newLogRec.WS_target_relative  = TPoint2D(relTarget);
	newLogRec.nSelectedPTG        = nSelectedPTG;
	newLogRec.cur_vel             = m_curPoseVel.velGlobal;
	newLogRec.cur_vel_local       = m_curPoseVel.velLocal;
	newLogRec.cmd_vel = new_vel_cmd;
	newLogRec.values["estimatedExecutionPeriod"] = meanExecutionPeriod.getLastOutput();
	newLogRec.values["executionTime"] = executionTimeValue;
	newLogRec.values["executionTime_avr"] = meanExecutionTime.getLastOutput();
	newLogRec.values["time_changeSpeeds()"] = tim_changeSpeed;
	newLogRec.values["time_changeSpeeds()_avr"] = tim_changeSpeed_avr.getLastOutput();
	newLogRec.timestamps["tim_start_iteration"] = tim_start_iteration;
	newLogRec.timestamps["curPoseAndVel"] = m_curPoseVel.timestamp;
	newLogRec.nPTGs = nPTGs;

	// NOP mode  stuff:
	newLogRec.rel_cur_pose_wrt_last_vel_cmd_NOP = rel_cur_pose_wrt_last_vel_cmd_NOP;
	newLogRec.rel_pose_PTG_origin_wrt_sense_NOP = rel_pose_PTG_origin_wrt_sense_NOP;
	newLogRec.ptg_index_NOP = best_is_NOP_cmdvel ? m_lastSentVelCmd.ptg_index : -1;
	newLogRec.ptg_last_k_NOP = m_lastSentVelCmd.ptg_alpha_index;
	newLogRec.ptg_last_curRobotVelLocal = m_lastSentVelCmd.curRobotVelLocal;

	// Last entry in info-per-PTG:
	{
		CLogFileRecord::TInfoPerPTG &ipp = *newLogRec.infoPerPTG.rbegin();
		if (!ipp.HLFR) ipp.HLFR = CLogFileRecord_VFF::Create();
	}

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
}

void CAbstractPTGBasedReactive::STEP5_PTGEvaluator(
	THolonomicMovement         & holonomicMovement,
	const std::vector<double>        & in_TPObstacles,
	const mrpt::nav::ClearanceDiagram & in_clearance,
	const mrpt::math::TPose2D  & WS_Target,
	const mrpt::math::TPoint2D & TP_Target,
	CLogFileRecord::TInfoPerPTG & log,
	CLogFileRecord & newLogRec,
	const bool this_is_PTG_continuation,
	const mrpt::poses::CPose2D & rel_cur_pose_wrt_last_vel_cmd_NOP,
	const unsigned int ptg_idx4weights)
{
	MRPT_START;

	const double   refDist	    = holonomicMovement.PTG->getRefDistance();
	const double   TargetDir    = (TP_Target.x!=0 || TP_Target.y!=0) ? atan2( TP_Target.y, TP_Target.x) : 0.0;
	const int      TargetSector = static_cast<int>( holonomicMovement.PTG->alpha2index( TargetDir ) );
	const double   TargetDist   = TP_Target.norm();
	// Picked movement direction:
	const int      kDirection   = static_cast<int>( holonomicMovement.PTG->alpha2index( holonomicMovement.direction ) );

	// Coordinates of the trajectory end for the given PTG and "alpha":
	const double d = min( in_TPObstacles[ kDirection ], 0.99*TargetDist);
	uint16_t nStep;
	bool pt_in_range = holonomicMovement.PTG->getPathStepForDist(kDirection, d, nStep);
	ASSERT_(pt_in_range)

	mrpt::math::TPose2D pose;
	holonomicMovement.PTG->getPathPose(kDirection, nStep,pose);

	std::vector<double> eval_factors(6);
	// Factor 1: Free distance for the chosen PTG and "alpha" in the TP-Space:
	// ----------------------------------------------------------------------
	eval_factors[0] = in_TPObstacles[kDirection];

	// Special case for NOP motion cmd:
	// consider only the empty space *after* the current robot pose, which is not at the origin.
	if (this_is_PTG_continuation &&
		( rel_cur_pose_wrt_last_vel_cmd_NOP.x()!=0 || rel_cur_pose_wrt_last_vel_cmd_NOP.y()!=0) // edge case: if the rel pose is (0,0), the evaluation is exactly as in an no-NOP case.
		)
	{
		int cur_k=0;
		double cur_norm_d=.0;
		bool is_exact = holonomicMovement.PTG->inverseMap_WS2TP(rel_cur_pose_wrt_last_vel_cmd_NOP.x(), rel_cur_pose_wrt_last_vel_cmd_NOP.y(), cur_k, cur_norm_d);

		if (!is_exact)
		{
			// Don't trust this step: we are not 100% sure of the robot pose in TP-Space for this "PTG continuation" step:
			holonomicMovement.evaluation = .0;
			newLogRec.additional_debug_msgs["PTGEvaluator"] = "PTG-continuation not allowed, cur. pose out of PTG domain.";
			return;
		}
		{
			const double cur_a = holonomicMovement.PTG->index2alpha(cur_k);
			log.TP_Robot.x = cos(cur_a)*cur_norm_d;
			log.TP_Robot.y = sin(cur_a)*cur_norm_d;
		}
		{
			uint16_t cur_ptg_step;
			bool ok1 = holonomicMovement.PTG->getPathStepForDist(m_lastSentVelCmd.ptg_alpha_index, cur_norm_d * holonomicMovement.PTG->getRefDistance(), cur_ptg_step);
			if (ok1) {
				mrpt::math::TPose2D predicted_rel_pose;
				holonomicMovement.PTG->getPathPose(m_lastSentVelCmd.ptg_alpha_index, cur_ptg_step, predicted_rel_pose);
				const CPose2D predicted_pose_global = CPose2D(m_lastSentVelCmd.curRobotPose) + CPose2D(predicted_rel_pose);
				const double predicted2real_dist = predicted_pose_global.distance2DTo(m_curPoseVel.pose.x, m_curPoseVel.pose.y);
				newLogRec.additional_debug_msgs["PTGEvaluator.PTGcont"] = mrpt::format("last_cmdvel_pose=%s mismatchDistance=%.03f cm", m_lastSentVelCmd.curRobotPose.asString().c_str(), 1e2*predicted2real_dist );

				if (predicted2real_dist > MAX_DISTANCE_PREDICTED_ACTUAL_PATH)
				{
					holonomicMovement.evaluation = .0;
					newLogRec.additional_debug_msgs["PTGEvaluator"] = "PTG-continuation not allowed, mismatchDistance above threshold.";
					return;
				}
			}
			else {
				holonomicMovement.evaluation = .0;
				newLogRec.additional_debug_msgs["PTGEvaluator"] = "PTG-continuation not allowed, couldn't get PTG step for cur. robot pose.";
				return;
			}
		}

		eval_factors[0] -= cur_norm_d;
		if (eval_factors[0] < 0.50) {
			// Don't trust this step: we are reaching too close to obstacles:
			newLogRec.additional_debug_msgs["PTGEvaluator"] = "PTG-continuation not allowed, too close to obstacles.";
			holonomicMovement.evaluation = .0;
			return;
		}
	}

	// Factor 2: Distance in sectors:
	// -------------------------------------------
	int dif = std::abs(TargetSector - kDirection);
	const size_t nSectors = in_TPObstacles.size();
	if ( dif > int(nSectors/2)) dif = nSectors - dif;
	eval_factors[1] = exp(-square( dif / (nSectors/3.0))) ;

	// Factor 3: Angle between the robot at the end of the chosen trajectory and the target
	// -------------------------------------------------------------------------------------
	double t_ang = atan2( WS_Target.y - pose.y, WS_Target.x - pose.x );
	t_ang -= pose.phi;
	mrpt::math::wrapToPiInPlace(t_ang);

	eval_factors[2] = exp(-square( t_ang / (0.5*M_PI)) );

	// Factor4:		Decrease in euclidean distance between (x,y) and the target:
	//  Moving away of the target is negatively valued
	// ---------------------------------------------------------------------------
	const double dist_eucl_final = std::sqrt(square(WS_Target.x- pose.x)+square(WS_Target.y- pose.y));
	eval_factors[3] = (refDist - dist_eucl_final) / refDist;
	mrpt::utils::saturate(eval_factors[3], 0.0, 1.0);

	// Factor5: Hysteresis:
	// -----------------------------------------------------
	eval_factors[4] = .0;

	if (this_is_PTG_continuation)
		eval_factors[4] = 1.0;
	else if (m_last_vel_cmd)
	{
		mrpt::kinematics::CVehicleVelCmdPtr desired_cmd;
		desired_cmd = holonomicMovement.PTG->directionToMotionCommand(kDirection);
		const mrpt::kinematics::CVehicleVelCmd *ptr1 = m_last_vel_cmd.pointer();
		const mrpt::kinematics::CVehicleVelCmd *ptr2 = desired_cmd.pointer();
		if(typeid(*ptr1) == typeid(*ptr2)){
			ASSERT_EQUAL_(m_last_vel_cmd->getVelCmdLength(), desired_cmd->getVelCmdLength());

			double simil_score = 0.5;
			for (size_t i = 0; i < desired_cmd->getVelCmdLength(); i++)
			{
				const double scr = exp(-std::abs(desired_cmd->getVelCmdElement(i) - m_last_vel_cmd->getVelCmdElement(i)) / 0.20);
				mrpt::utils::keep_min(simil_score, scr);
			}
			eval_factors[4] = simil_score;
		}
	}

	// Factor6: clearance
	// -----------------------------------------------------
	eval_factors[5] = in_clearance.getClearance(kDirection, TargetDist*1.01 );

	//  SAVE LOG
	log.evalFactors.resize(eval_factors.size());
	mrpt::utils::metaprogramming::copy_container_typecasting(eval_factors, log.evalFactors);

	if (holonomicMovement.speed == 0)
	{
		// If no movement has been found -> the worst evaluation:
		holonomicMovement.evaluation = 0;
	}
	else
	{
		// General case:
		double global_eval = .0;

		// Select set of weights:
		ASSERT_(!weights.empty() || weights4ptg.size()>ptg_idx4weights);
		const std::vector<float> & w = this->weights.empty() ? this->weights4ptg[ptg_idx4weights] : this->weights;
		ASSERT_EQUAL_(w.size(),eval_factors.size());
		
		// Sum:
		for (size_t i = 0; i < eval_factors.size(); i++)
			global_eval += w[i] * eval_factors[i];
		global_eval /= math::sum(w);

		// Don't reduce the priority of "PTG continuation" "NOPs".
		if (!this_is_PTG_continuation)
			global_eval *= holonomicMovement.PTG->getScorePriority();

		holonomicMovement.evaluation = global_eval;
	}

	MRPT_END;
}

void CAbstractPTGBasedReactive::STEP7_GenerateSpeedCommands( const THolonomicMovement &in_movement, mrpt::kinematics::CVehicleVelCmdPtr &new_vel_cmd )
{
	mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "STEP7_GenerateSpeedCommands");
	try
	{
		if (in_movement.speed == 0)
		{
			// The robot will stop:
			new_vel_cmd = in_movement.PTG->getSupportedKinematicVelocityCommand();
			new_vel_cmd->setToStop();
		}
		else
		{
			// Take the normalized movement command:
			new_vel_cmd = in_movement.PTG->directionToMotionCommand( in_movement.PTG->alpha2index( in_movement.direction ) );

			// Scale holonomic speeds to real-world one:
			new_vel_cmd->cmdVel_scale(in_movement.speed);

			if (!m_last_vel_cmd) // first iteration? Use default values:
				m_last_vel_cmd = in_movement.PTG->getSupportedKinematicVelocityCommand();

			// Honor user speed limits & "blending":
			const double beta = meanExecutionPeriod.getLastOutput() / (meanExecutionPeriod.getLastOutput() + SPEEDFILTER_TAU);
			new_vel_cmd->cmdVel_limits(*m_last_vel_cmd, beta, this->robotVelocityParams);
		}

		m_last_vel_cmd = new_vel_cmd; // Save for filtering in next step
	}
	catch (std::exception &e)
	{
		MRPT_LOG_ERROR_STREAM << "[CAbstractPTGBasedReactive::STEP7_GenerateSpeedCommands] Exception: " << e.what();
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

	USE_DELAYS_MODEL = cfg.read_bool(sectCfg, "USE_DELAYS_MODEL", USE_DELAYS_MODEL);
	MAX_DISTANCE_PREDICTED_ACTUAL_PATH = cfg.read_double(sectCfg, "MAX_DISTANCE_PREDICTED_ACTUAL_PATH", MAX_DISTANCE_PREDICTED_ACTUAL_PATH);

	DIST_TO_TARGET_FOR_SENDING_EVENT = cfg.read_float(sectCfg, "DIST_TO_TARGET_FOR_SENDING_EVENT", DIST_TO_TARGET_FOR_SENDING_EVENT, false);
	m_badNavAlarm_AlarmTimeout = cfg.read_double(sectCfg,"ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", m_badNavAlarm_AlarmTimeout, false);

	// =========  Show configuration parameters:
	MRPT_LOG_INFO("-------------------------------------------------------------\n");
	MRPT_LOG_INFO("       PTG-based Reactive Navigation parameters               \n");
	MRPT_LOG_INFO("-------------------------------------------------------------\n");

	// ========= Load Derived class-specific params:
	this->internal_loadConfigFile(cfg, section_prefix);

	// weights: read global or PTG specific weights:
	cfg.read_vector(sectCfg, "weights", vector<float>(0), weights, false /* don't fail if not found */);
	ASSERT_(weights.size() == 6 || weights.empty() );
	if (weights.empty())
	{
		weights4ptg.resize(getPTG_count());
		for (unsigned int i = 0; i < getPTG_count(); i++)
		{
			std::vector<float> w;
			const std::string sKey = mrpt::format("PTG%u_weights", i);
			cfg.read_vector(sectCfg, sKey, vector<float>(0), w, false /* don't fail if not found */);
			ASSERTMSG_(w.size()==6, mrpt::format("Found neither `weights` nor `%s`. Please, specify one of them.", sKey.c_str()) );
			weights4ptg[i] = w;
		}
	}

	// ========= Load "Global params" (must be called after initialization of PTGs in `internal_loadConfigFile()`)
	this->robotVelocityParams.loadConfigFile(cfg,sectGlobal); // robot interface params
	this->loadWaypointsParamsConfigFile(cfg,sectCfg);
	this->loadHolonomicMethodConfig(cfg,sectGlobal); // Load holonomic method params
	ASSERT_(!m_holonomicMethod.empty())

	MRPT_LOG_INFO_FMT(" Holonomic method   = %s\n", m_holonomicMethod[0]->GetRuntimeClass()->className );
	MRPT_LOG_INFO_FMT(" PTG Count          = %u\n", static_cast<unsigned int>( this->getPTG_count() ) );
	MRPT_LOG_INFO_FMT(" Reference distance = %f\n", refDistance );

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

bool CAbstractPTGBasedReactive::STEP2_SenseObstacles()
{
	return implementSenseObstacles(m_WS_Obstacles_timestamp);
}

void CAbstractPTGBasedReactive::robotPoseExtrapolateIncrement(const mrpt::math::TTwist2D & globalVel, const double time_offset, mrpt::poses::CPose2D & out_pose)
{
	out_pose.x(globalVel.vx * time_offset);
	out_pose.y(globalVel.vy * time_offset);
	out_pose.phi( globalVel.omega * time_offset );
}

void CAbstractPTGBasedReactive::onStartNewNavigation()
{
	m_lastSentVelCmd.reset();
}

CAbstractPTGBasedReactive::TSentVelCmd::TSentVelCmd()
{
	reset();
}
void CAbstractPTGBasedReactive::TSentVelCmd::reset()
{
	ptg_index = -1;
	ptg_alpha_index = -1;
	tim_send_cmd_vel = INVALID_TIMESTAMP;
	tim_poseVel = INVALID_TIMESTAMP;
	curRobotVelLocal.vx = .0;
	curRobotVelLocal.vy = .0;
	curRobotVelLocal.omega = .0;
}
bool CAbstractPTGBasedReactive::TSentVelCmd::isValid() const
{
	return tim_poseVel != INVALID_TIMESTAMP;
}


void CAbstractPTGBasedReactive::ptg_eval_target_build_obstacles(
	CParameterizedTrajectoryGenerator * ptg,
	const size_t indexPTG,
	const TPose2D &relTarget,
	const CPose2D &rel_pose_PTG_origin_wrt_sense,
	TInfoPerPTG &ipf,
	THolonomicMovement &holonomicMovement,
	CLogFileRecord &newLogRec,
	const bool this_is_PTG_continuation,
	mrpt::nav::CAbstractHolonomicReactiveMethod *holoMethod,
	const TNavigationParams &navp,
	const mrpt::poses::CPose2D &rel_cur_pose_wrt_last_vel_cmd_NOP
	)
{
	ASSERT_(ptg);

	const size_t idx_in_log_infoPerPTGs = this_is_PTG_continuation ? getPTG_count() : indexPTG;

	CHolonomicLogFileRecordPtr HLFR;
	holonomicMovement.PTG = ptg;

	// If the user doesn't want to use this PTG, just mark it as invalid:
	ipf.valid_TP = true;
	{
		const TNavigationParamsPTG * navpPTG = dynamic_cast<const TNavigationParamsPTG*>(&navp);
		if (navpPTG && !navpPTG->restrict_PTG_indices.empty())
		{
			bool use_this_ptg = false;
			for (size_t i = 0; i < navpPTG->restrict_PTG_indices.size() && !use_this_ptg; i++) {
				if (navpPTG->restrict_PTG_indices[i] == indexPTG)
					use_this_ptg = true;
			}
			ipf.valid_TP = use_this_ptg;
		}
	}

	double timeForTPObsTransformation = .0, timeForHolonomicMethod = .0;

	// Normal PTG validity filter: check if target falls into the PTG domain:
	if (ipf.valid_TP)
	{
		ipf.valid_TP = ptg->inverseMap_WS2TP(relTarget.x, relTarget.y, ipf.target_k, ipf.target_dist);
	}

	if (!ipf.valid_TP)
	{
		ipf.target_k = 0;
		ipf.target_dist = 0;

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
			STEP3_WSpaceToTPSpace(indexPTG, ipf.TP_Obstacles, ipf.clearance, -rel_pose_PTG_origin_wrt_sense);
			ptg->updateClearancePost(ipf.clearance, ipf.TP_Obstacles);

			// Distances in TP-Space are normalized to [0,1]:
			const double _refD = 1.0 / ptg->getRefDistance();
			for (size_t i = 0; i < Ki; i++) ipf.TP_Obstacles[i] *= _refD;

			timeForTPObsTransformation = tictac.Tac();
			if (m_timelogger.isEnabled())
				m_timelogger.registerUserMeasure("navigationStep.STEP3_WSpaceToTPSpace", timeForTPObsTransformation);
		}

		//  STEP4: Holonomic navigation method
		// -----------------------------------------------------------------------------
		if (!this_is_PTG_continuation)
		{
			tictac.Tic();

			ASSERT_(holoMethod);
			holoMethod->enableApproachTargetSlowDown( !navp.targetIsIntermediaryWaypoint );

			holoMethod->navigate(
					ipf.TP_Target,     // Normalized [0,1]
					ipf.TP_Obstacles,  // Normalized [0,1]
					1.0, // Was: ptg->getMax_V_inTPSpace(),
					holonomicMovement.direction,
					holonomicMovement.speed,
					HLFR,
					1.0 /* max obstacle dist*/,
					&ipf.clearance);

			// Security: Scale down the velocity when heading towards obstacles,
			//  such that it's assured that we never go thru an obstacle!
			const int kDirection = static_cast<int>(holonomicMovement.PTG->alpha2index(holonomicMovement.direction));
			double obsFreeNormalizedDistance = ipf.TP_Obstacles[kDirection];

			// Take into account the future robot pose after NOP motion iterations to slow down accordingly *now*
			if (ptg->supportVelCmdNOP()) {
				const double v = ::hypot(m_curPoseVel.velLocal.vx, m_curPoseVel.velLocal.vy);
				const double d = v * ptg->maxTimeInVelCmdNOP(kDirection);
				obsFreeNormalizedDistance = std::min(obsFreeNormalizedDistance, std::max(0.90, obsFreeNormalizedDistance - d) );
			}

			double velScale = 1.0;
			ASSERT_(secureDistanceEnd > secureDistanceStart);
			if (obsFreeNormalizedDistance < secureDistanceEnd)
			{
				if (obsFreeNormalizedDistance <= secureDistanceStart)
					velScale = 0.0; // security stop
				else velScale = (obsFreeNormalizedDistance - secureDistanceStart) / (secureDistanceEnd - secureDistanceStart);
			}

			// Scale:
			holonomicMovement.speed *= velScale;

			timeForHolonomicMethod = tictac.Tac();
			if (m_timelogger.isEnabled())
				m_timelogger.registerUserMeasure("navigationStep.STEP4_HolonomicMethod", timeForHolonomicMethod);
		}
		else
		{
			// "NOP cmdvel" case: don't need to re-run holo algorithm, just keep the last selection:
			holonomicMovement.direction = ptg->index2alpha( m_lastSentVelCmd.ptg_alpha_index );
			holonomicMovement.speed = 1.0; // Not used.
		}

		// STEP5: Evaluate each movement to assign them a "evaluation" value.
		// ---------------------------------------------------------------------
		{
			CTimeLoggerEntry tle(m_timelogger, "navigationStep.STEP5_PTGEvaluator");

			STEP5_PTGEvaluator(
				holonomicMovement,
				ipf.TP_Obstacles,
				ipf.clearance,
				relTarget,
				ipf.TP_Target,
				newLogRec.infoPerPTG[idx_in_log_infoPerPTGs], newLogRec,
				this_is_PTG_continuation, rel_cur_pose_wrt_last_vel_cmd_NOP,
				indexPTG);
		}


	} // end "valid_TP"

	// Logging:
	const bool fill_log_record = (m_logFile != NULL || m_enableKeepLogRecords);
	if (fill_log_record)
	{
		CLogFileRecord::TInfoPerPTG &ipp = newLogRec.infoPerPTG[idx_in_log_infoPerPTGs];
		if (!this_is_PTG_continuation)
		     ipp.PTG_desc = ptg->getDescription();
		else ipp.PTG_desc = mrpt::format("NOP cmdvel (prev PTG idx=%u)", static_cast<unsigned int>(m_lastSentVelCmd.ptg_index) );

		metaprogramming::copy_container_typecasting(ipf.TP_Obstacles, ipp.TP_Obstacles);
		ipp.clearance = ipf.clearance;
		ipp.TP_Target = ipf.TP_Target;
		ipp.HLFR = HLFR;
		ipp.desiredDirection = holonomicMovement.direction;
		ipp.desiredSpeed = holonomicMovement.speed;
		ipp.evaluation = holonomicMovement.evaluation;
		ipp.timeForTPObsTransformation = timeForTPObsTransformation;
		ipp.timeForHolonomicMethod = timeForHolonomicMethod;
	}
}
