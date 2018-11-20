/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/reactive/CAbstractPTGBasedReactive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>  // sum()
#include <mrpt/containers/printf_vector.h>
#include <mrpt/containers/copy_container_typecasting.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CPointCloudFilterByDistance.h>
#include <mrpt/serialization/CArchive.h>
#include <limits>
#include <iomanip>
#include <array>

using namespace mrpt;
using namespace mrpt::io;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::nav;
using namespace mrpt::serialization;
using namespace std;

// ------ CAbstractPTGBasedReactive::TNavigationParamsPTG -----
std::string CAbstractPTGBasedReactive::TNavigationParamsPTG::getAsText() const
{
	std::string s =
		CWaypointsNavigator::TNavigationParamsWaypoints::getAsText();
	s += "restrict_PTG_indices: ";
	s += mrpt::containers::sprintf_vector("%u ", this->restrict_PTG_indices);
	s += "\n";
	return s;
}

bool CAbstractPTGBasedReactive::TNavigationParamsPTG::isEqual(
	const CAbstractNavigator::TNavigationParamsBase& rhs) const
{
	auto o =
		dynamic_cast<const CAbstractPTGBasedReactive::TNavigationParamsPTG*>(
			&rhs);
	return o != nullptr &&
		   CWaypointsNavigator::TNavigationParamsWaypoints::isEqual(rhs) &&
		   restrict_PTG_indices == o->restrict_PTG_indices;
}

const double ESTIM_LOWPASSFILTER_ALPHA = 0.7;

// Ctor:
CAbstractPTGBasedReactive::CAbstractPTGBasedReactive(
	CRobot2NavInterface& react_iterf_impl, bool enableConsoleOutput,
	bool enableLogFile, const std::string& sLogDir)
	: CWaypointsNavigator(react_iterf_impl),
	  m_holonomicMethod(),
	  m_prev_logfile(nullptr),
	  m_enableKeepLogRecords(false),
	  m_enableConsoleOutput(enableConsoleOutput),
	  m_init_done(false),
	  m_timelogger(false),  // default: disabled
	  m_PTGsMustBeReInitialized(true),
	  meanExecutionTime(ESTIM_LOWPASSFILTER_ALPHA, 0.1),
	  meanTotalExecutionTime(ESTIM_LOWPASSFILTER_ALPHA, 0.1),
	  meanExecutionPeriod(ESTIM_LOWPASSFILTER_ALPHA, 0.1),
	  tim_changeSpeed_avr(ESTIM_LOWPASSFILTER_ALPHA),
	  timoff_obstacles_avr(ESTIM_LOWPASSFILTER_ALPHA),
	  timoff_curPoseAndSpeed_avr(ESTIM_LOWPASSFILTER_ALPHA),
	  timoff_sendVelCmd_avr(ESTIM_LOWPASSFILTER_ALPHA),
	  m_closing_navigator(false),
	  m_WS_Obstacles_timestamp(INVALID_TIMESTAMP),
	  m_infoPerPTG_timestamp(INVALID_TIMESTAMP),
	  m_navlogfiles_dir(sLogDir)
{
	this->enableLogFile(enableLogFile);
}

void CAbstractPTGBasedReactive::preDestructor()
{
	m_closing_navigator = true;

	// Wait to end of navigation (multi-thread...)
	m_nav_cs.lock();
	m_nav_cs.unlock();

	// Just in case.
	try
	{
		// Call base class method, NOT the generic, virtual one,
		// to avoid problems if we are in the dtor, while the vtable
		// is being destroyed.
		CAbstractNavigator::stop(false /*not emergency*/);
	}
	catch (...)
	{
	}

	m_logFile.reset();

	// Free holonomic method:
	this->deleteHolonomicObjects();
}

CAbstractPTGBasedReactive::~CAbstractPTGBasedReactive()
{
	this->preDestructor();  // ensure the robot is stopped; free dynamic objects
}

/** \callergraph */
void CAbstractPTGBasedReactive::initialize()
{
	std::lock_guard<std::recursive_mutex> csl(m_nav_cs);

	m_infoPerPTG_timestamp = INVALID_TIMESTAMP;

	ASSERT_(m_multiobjopt);
	m_multiobjopt->clear();

	// Compute collision grids:
	STEP1_InitPTGs();
}

/*---------------------------------------------------------------
						enableLogFile
  ---------------------------------------------------------------*/
void CAbstractPTGBasedReactive::enableLogFile(bool enable)
{
	std::lock_guard<std::recursive_mutex> csl(m_nav_cs);

	try
	{
		// Disable:
		// -------------------------------
		if (!enable)
		{
			if (m_logFile)
			{
				MRPT_LOG_DEBUG(
					"[CAbstractPTGBasedReactive::enableLogFile] Stopping "
					"logging.");
				m_logFile.reset();  // Close file:
			}
			else
				return;  // Already disabled.
		}
		else
		{  // Enable
			// -------------------------------
			if (m_logFile) return;  // Already enabled:

			// Open file, find the first free file-name.
			MRPT_LOG_DEBUG_FMT(
				"[CAbstractPTGBasedReactive::enableLogFile] Creating rnav log "
				"directory: %s",
				m_navlogfiles_dir.c_str());
			mrpt::system::createDirectory(m_navlogfiles_dir);
			if (!mrpt::system::directoryExists(m_navlogfiles_dir))
			{
				THROW_EXCEPTION_FMT(
					"Could not create directory for navigation logs: `%s`",
					m_navlogfiles_dir.c_str());
			}

			std::string filToOpen;
			for (unsigned int nFile = 0;; nFile++)
			{
				filToOpen = mrpt::format(
					"%s/log_%03u.reactivenavlog", m_navlogfiles_dir.c_str(),
					nFile);
				if (!system::fileExists(filToOpen)) break;
			}

			// Open log file:
			{
				std::unique_ptr<CFileGZOutputStream> fil(
					new CFileGZOutputStream);
				bool ok = fil->open(filToOpen, 1 /* compress level */);
				if (!ok)
				{
					THROW_EXCEPTION_FMT(
						"Error opening log file: `%s`", filToOpen.c_str());
				}
				else
				{
					m_logFile = std::move(fil);
				}
			}

			MRPT_LOG_DEBUG(mrpt::format(
				"[CAbstractPTGBasedReactive::enableLogFile] Logging to "
				"file `%s`",
				filToOpen.c_str()));
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_FMT(
			"[CAbstractPTGBasedReactive::enableLogFile] Exception: %s",
			e.what());
	}
}

void CAbstractPTGBasedReactive::getLastLogRecord(CLogFileRecord& o)
{
	std::lock_guard<std::recursive_mutex> lock(m_critZoneLastLog);
	o = lastLogRecord;
}

void CAbstractPTGBasedReactive::deleteHolonomicObjects()
{
	m_holonomicMethod.clear();
}

void CAbstractPTGBasedReactive::setHolonomicMethod(
	const std::string& method, const mrpt::config::CConfigFileBase& ini)
{
	std::lock_guard<std::recursive_mutex> csl(m_nav_cs);

	this->deleteHolonomicObjects();
	const size_t nPTGs = this->getPTG_count();
	ASSERT_(nPTGs != 0);
	m_holonomicMethod.resize(nPTGs);

	for (size_t i = 0; i < nPTGs; i++)
	{
		m_holonomicMethod[i] =
			CAbstractHolonomicReactiveMethod::Factory(method);
		if (!m_holonomicMethod[i])
			THROW_EXCEPTION_FMT(
				"Non-registered holonomic method className=`%s`",
				method.c_str());

		m_holonomicMethod[i]->setAssociatedPTG(this->getPTG(i));
		m_holonomicMethod[i]->initialize(ini);  // load params
	}
}

/** The main method: executes one time-iteration of the reactive navigation
 * algorithm.
 * \callergraph */
void CAbstractPTGBasedReactive::performNavigationStep()
{
	// Security tests:
	if (m_closing_navigator) return;  // Are we closing in the main thread?
	if (!m_init_done)
		THROW_EXCEPTION("Have you called loadConfigFile() before?");
	ASSERT_(m_navigationParams);
	const size_t nPTGs = this->getPTG_count();

	// Whether to worry about log files:
	const bool fill_log_record = (m_logFile || m_enableKeepLogRecords);
	CLogFileRecord newLogRec;
	newLogRec.infoPerPTG.resize(nPTGs + 1); /* +1: [N] is the "NOP cmdvel"
											   option; not to be present in all
											   log entries. */

	// At the beginning of each log file, add an introductory block explaining
	// which PTGs are we using:
	{
		if (m_logFile &&
			m_logFile.get() != m_prev_logfile)  // Only the first time
		{
			m_prev_logfile = m_logFile.get();
			for (size_t i = 0; i < nPTGs; i++)
			{
				// If we make a direct copy (=) we will store the entire, heavy,
				// collision grid.
				// Let's just store the parameters of each PTG by serializing
				// it, so paths can be reconstructed
				// by invoking initialize()
				mrpt::io::CMemoryStream buf;
				auto arch = archiveFrom(buf);
				arch << *this->getPTG(i);
				buf.Seek(0);
				newLogRec.infoPerPTG[i].ptg = std::dynamic_pointer_cast<
					mrpt::nav::CParameterizedTrajectoryGenerator>(
					arch.ReadObject());
			}
		}
	}

	CTimeLoggerEntry tle1(m_timelogger, "navigationStep");

	try
	{
		totalExecutionTime.Tic();  // Start timer

		const mrpt::system::TTimeStamp tim_start_iteration =
			mrpt::system::now();

		// Compute target location relative to current robot pose:
		// ---------------------------------------------------------------------
		// Detect changes in target since last iteration (for NOP):
		const bool target_changed_since_last_iteration =
			(!m_copy_prev_navParams) ||
			!(*m_copy_prev_navParams == *m_navigationParams);
		if (target_changed_since_last_iteration)
			m_copy_prev_navParams = m_navigationParams->clone();

		// Load the list of target(s) from the navigationParam user command.
		// Semantic is: any of the target is good. If several targets are
		// reachable, head to latest one.
		std::vector<CAbstractNavigator::TargetInfo> targets;
		{
			auto p = dynamic_cast<
				const CWaypointsNavigator::TNavigationParamsWaypoints*>(
				m_navigationParams.get());
			if (p && !p->multiple_targets.empty())
			{
				targets = p->multiple_targets;
			}
			else
			{
				targets.push_back(m_navigationParams->target);
			}
		}
		const size_t nTargets = targets.size();  // Normally = 1, will be >1 if
		// we want the robot local
		// planner to be "smarter" in
		// skipping dynamic obstacles.
		ASSERT_(nTargets >= 1);

		STEP1_InitPTGs();  // Will only recompute if
		// "m_PTGsMustBeReInitialized==true"

		// STEP2: Load the obstacles and sort them in height bands.
		// -----------------------------------------------------------------------------
		if (!STEP2_SenseObstacles())
		{
			doEmergencyStop(
				"Error while loading and sorting the obstacles. Robot will be "
				"stopped.");
			if (fill_log_record)
			{
				CPose2D rel_cur_pose_wrt_last_vel_cmd_NOP,
					rel_pose_PTG_origin_wrt_sense_NOP;
				STEP8_GenerateLogRecord(
					newLogRec,
					std::vector<mrpt::math::TPose2D>() /*no targets*/,
					-1,  // best_ptg_idx,
					m_robot.getEmergencyStopCmd(), nPTGs,
					false,  // best_is_NOP_cmdvel,
					rel_cur_pose_wrt_last_vel_cmd_NOP.asTPose(),
					rel_pose_PTG_origin_wrt_sense_NOP.asTPose(),
					0,  // executionTimeValue,
					0,  // tim_changeSpeed,
					tim_start_iteration);
			}
			return;
		}

		// ------- start of motion decision zone ---------
		executionTime.Tic();

		// Round #1: As usual, pure reactive, evaluate all PTGs and all
		// directions from scratch.
		// =========

		mrpt::math::TPose2D rel_pose_PTG_origin_wrt_sense(0, 0, 0),
			relPoseSense(0, 0, 0), relPoseVelCmd(0, 0, 0);
		if (params_abstract_ptg_navigator.use_delays_model)
		{
			/*
			 *                                          Delays model
			 *
			 * Event:          OBSTACLES_SENSED         RNAV_ITERATION_STARTS
			 * GET_ROBOT_POSE_VEL         VEL_CMD_SENT_TO_ROBOT
			 * Timestamp:  (m_WS_Obstacles_timestamp)   (tim_start_iteration)
			 * (m_curPoseVelTimestamp)     ("tim_send_cmd_vel")
			 * Delay                       |
			 * <---+--------------->|<--------------+-------->| |
			 * estimator:                        |                |
			 * |                                   |
			 *                timoff_obstacles <-+                |
			 * +--> timoff_curPoseVelAge           |
			 *                                                    |<---------------------------------+--------------->|
			 *                                                                                       +-->
			 * timoff_sendVelCmd_avr (estimation)
			 *
			 *                                                                                                |<-------------------->|
			 *                                                                                                   tim_changeSpeed_avr
			 * (estim)
			 *
			 *                             |<-----------------------------------------------|-------------------------->|
			 *  Relative poses:                              relPoseSense
			 * relPoseVelCmd
			 *  Time offsets (signed):                       timoff_pose2sense
			 * timoff_pose2VelCmd
			 */
			const double timoff_obstacles = mrpt::system::timeDifference(
				tim_start_iteration, m_WS_Obstacles_timestamp);
			timoff_obstacles_avr.filter(timoff_obstacles);
			newLogRec.values["timoff_obstacles"] = timoff_obstacles;
			newLogRec.values["timoff_obstacles_avr"] =
				timoff_obstacles_avr.getLastOutput();
			newLogRec.timestamps["obstacles"] = m_WS_Obstacles_timestamp;

			const double timoff_curPoseVelAge = mrpt::system::timeDifference(
				tim_start_iteration, m_curPoseVel.timestamp);
			timoff_curPoseAndSpeed_avr.filter(timoff_curPoseVelAge);
			newLogRec.values["timoff_curPoseVelAge"] = timoff_curPoseVelAge;
			newLogRec.values["timoff_curPoseVelAge_avr"] =
				timoff_curPoseAndSpeed_avr.getLastOutput();

			// time offset estimations:
			const double timoff_pose2sense =
				timoff_obstacles - timoff_curPoseVelAge;

			double timoff_pose2VelCmd;
			timoff_pose2VelCmd = timoff_sendVelCmd_avr.getLastOutput() +
								 0.5 * tim_changeSpeed_avr.getLastOutput() -
								 timoff_curPoseVelAge;
			newLogRec.values["timoff_pose2sense"] = timoff_pose2sense;
			newLogRec.values["timoff_pose2VelCmd"] = timoff_pose2VelCmd;

			if (std::abs(timoff_pose2sense) > 1.25)
				MRPT_LOG_WARN_FMT(
					"timoff_pose2sense=%e is too large! Path extrapolation may "
					"be not accurate.",
					timoff_pose2sense);
			if (std::abs(timoff_pose2VelCmd) > 1.25)
				MRPT_LOG_WARN_FMT(
					"timoff_pose2VelCmd=%e is too large! Path extrapolation "
					"may be not accurate.",
					timoff_pose2VelCmd);

			// Path extrapolation: robot relative poses along current path
			// estimation:
			relPoseSense = m_curPoseVel.velLocal * timoff_pose2sense;
			relPoseVelCmd = m_curPoseVel.velLocal * timoff_pose2VelCmd;

			// relative pose for PTGs:
			rel_pose_PTG_origin_wrt_sense = relPoseVelCmd - relPoseSense;

			// logging:
			newLogRec.relPoseSense = relPoseSense;
			newLogRec.relPoseVelCmd = relPoseVelCmd;
		}
		else
		{
			// No delays model: poses to their default values.
		}

		for (auto& t : targets)
		{
			if (t.target_frame_id != m_curPoseVel.pose_frame_id)
			{
				auto frame_tf = m_frame_tf.lock();
				if (!frame_tf)
				{
					THROW_EXCEPTION_FMT(
						"Different frame_id's but no frame_tf object "
						"attached (or it expired)!: target_frame_id=`%s` != "
						"pose_frame_id=`%s`",
						t.target_frame_id.c_str(),
						m_curPoseVel.pose_frame_id.c_str());
				}
				mrpt::math::TPose2D robot_frame2map_frame;
				frame_tf->lookupTransform(
					t.target_frame_id, m_curPoseVel.pose_frame_id,
					robot_frame2map_frame);

				MRPT_LOG_DEBUG_FMT(
					"frame_tf: target_frame_id=`%s` as seen from "
					"pose_frame_id=`%s` = %s",
					t.target_frame_id.c_str(),
					m_curPoseVel.pose_frame_id.c_str(),
					robot_frame2map_frame.asString().c_str());

				t.target_coords = robot_frame2map_frame + t.target_coords;
				t.target_frame_id =
					m_curPoseVel.pose_frame_id;  // Now the coordinates are in
				// the same frame than robot
				// pose
			}
		}

		std::vector<TPose2D> relTargets;
		const auto curPoseExtrapol = (m_curPoseVel.pose + relPoseVelCmd);
		std::transform(
			targets.begin(), targets.end(),  // in
			std::back_inserter(relTargets),  // out
			[curPoseExtrapol](const CAbstractNavigator::TargetInfo& e) {
				return e.target_coords - curPoseExtrapol;
			});
		ASSERT_EQUAL_(relTargets.size(), targets.size());

		// Use the distance to the first target as reference:
		const double relTargetDist = relTargets.begin()->norm();

		// PTG dynamic state
		/** Allow PTGs to be responsive to target location, dynamics, etc. */
		CParameterizedTrajectoryGenerator::TNavDynamicState ptg_dynState;

		ptg_dynState.curVelLocal = m_curPoseVel.velLocal;
		ptg_dynState.relTarget = relTargets[0];
		ptg_dynState.targetRelSpeed =
			m_navigationParams->target.targetDesiredRelSpeed;

		newLogRec.navDynState = ptg_dynState;

		for (size_t i = 0; i < nPTGs; i++)
		{
			getPTG(i)->updateNavDynamicState(ptg_dynState);
		}

		m_infoPerPTG.assign(nPTGs + 1, TInfoPerPTG());  // reset contents
		m_infoPerPTG_timestamp = tim_start_iteration;
		vector<TCandidateMovementPTG> candidate_movs(
			nPTGs + 1);  // the last extra one is for the evaluation of "NOP
		// motion command" choice.

		for (size_t indexPTG = 0; indexPTG < nPTGs; indexPTG++)
		{
			CParameterizedTrajectoryGenerator* ptg = getPTG(indexPTG);
			TInfoPerPTG& ipf = m_infoPerPTG[indexPTG];

			// Ensure the method knows about its associated PTG:
			auto holoMethod = this->getHoloMethod(indexPTG);
			ASSERT_(holoMethod);
			holoMethod->setAssociatedPTG(this->getPTG(indexPTG));

			// The picked movement in TP-Space (to be determined by holonomic
			// method below)
			TCandidateMovementPTG& cm = candidate_movs[indexPTG];

			ASSERT_(m_navigationParams);
			build_movement_candidate(
				ptg, indexPTG, relTargets, rel_pose_PTG_origin_wrt_sense, ipf,
				cm, newLogRec, false /* this is a regular PTG reactive case */,
				*holoMethod, tim_start_iteration, *m_navigationParams);
		}  // end for each PTG

		// check for collision, which is reflected by ALL TP-Obstacles being
		// zero:
		bool is_all_ptg_collision = true;
		for (size_t indexPTG = 0; indexPTG < nPTGs; indexPTG++)
		{
			bool is_collision = true;
			const auto& obs = m_infoPerPTG[indexPTG].TP_Obstacles;
			for (const auto o : obs)
			{
				if (o != 0)
				{
					is_collision = false;
					break;
				}
			}
			if (!is_collision)
			{
				is_all_ptg_collision = false;
				break;
			}
		}
		if (is_all_ptg_collision)
		{
			m_pending_events.emplace_back(std::bind(
				&CRobot2NavInterface::sendApparentCollisionEvent,
				std::ref(m_robot)));
		}

		// Round #2: Evaluate dont sending any new velocity command ("NOP"
		// motion)
		// =========
		bool NOP_not_too_old = true;
		bool NOP_not_too_close_and_have_to_slowdown = true;
		double NOP_max_time = -1.0, NOP_At = -1.0;
		double slowdowndist = .0;
		CParameterizedTrajectoryGenerator* last_sent_ptg =
			(m_lastSentVelCmd.isValid() && m_lastSentVelCmd.ptg_index >= 0)
				? getPTG(m_lastSentVelCmd.ptg_index)
				: nullptr;
		if (last_sent_ptg)
		{
			// So supportSpeedAtTarget() below is evaluated in the correct
			// context:
			last_sent_ptg->updateNavDynamicState(m_lastSentVelCmd.ptg_dynState);
		}

		// This approach is only possible if:
		const bool can_do_nop_motion =
			(m_lastSentVelCmd.isValid() &&
			 !target_changed_since_last_iteration && last_sent_ptg &&
			 last_sent_ptg->supportVelCmdNOP()) &&
			(NOP_not_too_old =
				 (NOP_At = mrpt::system::timeDifference(
					  m_lastSentVelCmd.tim_send_cmd_vel, tim_start_iteration)) <
				 (NOP_max_time =
					  last_sent_ptg->maxTimeInVelCmdNOP(
						  m_lastSentVelCmd.ptg_alpha_index) /
					  std::max(0.1, m_lastSentVelCmd.speed_scale))) &&
			(NOP_not_too_close_and_have_to_slowdown =
				 (last_sent_ptg->supportSpeedAtTarget() ||
				  (relTargetDist >
				   (slowdowndist = m_holonomicMethod[m_lastSentVelCmd.ptg_index]
									   ->getTargetApproachSlowDownDistance())
				   // slowdowndist is assigned here, inside the if()
				   // to be sure the index in m_lastSentVelCmd is valid!
				   )));

		if (!NOP_not_too_old)
		{
			newLogRec.additional_debug_msgs["PTG_cont"] = mrpt::format(
				"PTG-continuation not allowed: previous command timed-out "
				"(At=%.03f > Max_At=%.03f)",
				NOP_At, NOP_max_time);
		}
		if (!NOP_not_too_close_and_have_to_slowdown)
		{
			newLogRec.additional_debug_msgs["PTG_cont_trgdst"] = mrpt::format(
				"PTG-continuation not allowed: target too close and must start "
				"slow-down (trgDist=%.03f < SlowDownDist=%.03f)",
				relTargetDist, slowdowndist);
		}

		mrpt::math::TPose2D rel_cur_pose_wrt_last_vel_cmd_NOP(0, 0, 0),
			rel_pose_PTG_origin_wrt_sense_NOP(0, 0, 0);

		if (can_do_nop_motion)
		{
			// Add the estimation of how long it takes to run the changeSpeeds()
			// callback (usually a tiny period):
			const mrpt::system::TTimeStamp tim_send_cmd_vel_corrected =
				mrpt::system::timestampAdd(
					m_lastSentVelCmd.tim_send_cmd_vel,
					tim_changeSpeed_avr.getLastOutput());

			// Note: we use (uncorrected) raw odometry as basis to the following
			// calculation since it's normally
			// smoother than particle filter-based localization data, more
			// accurate in the middle/long term,
			// but not in the short term:
			mrpt::math::TPose2D robot_pose_at_send_cmd, robot_odom_at_send_cmd;
			bool valid_odom, valid_pose;

			m_latestOdomPoses.interpolate(
				tim_send_cmd_vel_corrected, robot_odom_at_send_cmd, valid_odom);
			m_latestPoses.interpolate(
				tim_send_cmd_vel_corrected, robot_pose_at_send_cmd, valid_pose);

			if (valid_odom && valid_pose)
			{
				ASSERT_(last_sent_ptg != nullptr);

				std::vector<TPose2D> relTargets_NOPs;
				std::transform(
					targets.begin(), targets.end(),  // in
					std::back_inserter(relTargets_NOPs),  // out
					[robot_pose_at_send_cmd](
						const CAbstractNavigator::TargetInfo& e) {
						return e.target_coords - robot_pose_at_send_cmd;
					});
				ASSERT_EQUAL_(relTargets_NOPs.size(), targets.size());

				rel_pose_PTG_origin_wrt_sense_NOP =
					robot_odom_at_send_cmd -
					(m_curPoseVel.rawOdometry + relPoseSense);
				rel_cur_pose_wrt_last_vel_cmd_NOP =
					m_curPoseVel.rawOdometry - robot_odom_at_send_cmd;

				// Update PTG response to dynamic params:
				last_sent_ptg->updateNavDynamicState(
					m_lastSentVelCmd.ptg_dynState);

				if (fill_log_record)
				{
					newLogRec.additional_debug_msgs
						["rel_cur_pose_wrt_last_vel_cmd_NOP(interp)"] =
						rel_cur_pose_wrt_last_vel_cmd_NOP.asString();
					newLogRec.additional_debug_msgs
						["robot_odom_at_send_cmd(interp)"] =
						robot_odom_at_send_cmd.asString();
				}

				// No need to call setAssociatedPTG(), already correctly
				// associated above.

				ASSERT_(m_navigationParams);
				build_movement_candidate(
					last_sent_ptg, m_lastSentVelCmd.ptg_index, relTargets_NOPs,
					rel_pose_PTG_origin_wrt_sense_NOP, m_infoPerPTG[nPTGs],
					candidate_movs[nPTGs], newLogRec,
					true /* this is the PTG continuation (NOP) choice */,
					*getHoloMethod(m_lastSentVelCmd.ptg_index),
					tim_start_iteration, *m_navigationParams,
					rel_cur_pose_wrt_last_vel_cmd_NOP);

			}  // end valid interpolated origin pose
			else
			{
				// Can't interpolate pose, hence can't evaluate NOP:
				candidate_movs[nPTGs].speed =
					-0.01;  // <0 means inviable movement
			}
		}  // end can_do_NOP_motion

		// Evaluate all the candidates and pick the "best" one, using
		// the user-defined multiobjective optimizer
		// --------------------------------------------------------------
		CMultiObjectiveMotionOptimizerBase::TResultInfo mo_info;
		ASSERT_(m_multiobjopt);
		int best_ptg_idx = m_multiobjopt->decide(candidate_movs, mo_info);

		if (fill_log_record)
		{
			if (mo_info.final_evaluation.size() == newLogRec.infoPerPTG.size())
			{
				for (unsigned int i = 0; i < newLogRec.infoPerPTG.size(); i++)
				{
					newLogRec.infoPerPTG[i].evaluation =
						mo_info.final_evaluation[i];
				}
			}
			int idx = 0;
			for (const auto& le : mo_info.log_entries)
			{
				newLogRec.additional_debug_msgs[mrpt::format(
					"MultiObjMotOptmzr_msg%03i", idx++)] = le;
			}
		}

		// Pick best movement (or null if none is good)
		const TCandidateMovementPTG* selectedHolonomicMovement = nullptr;
		if (best_ptg_idx >= 0)
		{
			selectedHolonomicMovement = &candidate_movs[best_ptg_idx];
		}

		// If the selected PTG is (N+1), it means the NOP cmd. vel is selected
		// as the best alternative, i.e. do NOT send any new motion command.
		const bool best_is_NOP_cmdvel = (best_ptg_idx == int(nPTGs));

		// ---------------------------------------------------------------------
		//				SEND MOVEMENT COMMAND TO THE ROBOT
		// ---------------------------------------------------------------------
		mrpt::kinematics::CVehicleVelCmd::Ptr new_vel_cmd;
		if (best_is_NOP_cmdvel)
		{
			// Notify the robot that we want it to keep executing the last
			// cmdvel:
			if (!this->changeSpeedsNOP())
			{
				doEmergencyStop(
					"\nERROR calling changeSpeedsNOP()!! Stopping robot and "
					"finishing navigation\n");
				if (fill_log_record)
				{
					STEP8_GenerateLogRecord(
						newLogRec, relTargets, best_ptg_idx,
						m_robot.getEmergencyStopCmd(), nPTGs,
						best_is_NOP_cmdvel, rel_cur_pose_wrt_last_vel_cmd_NOP,
						rel_pose_PTG_origin_wrt_sense_NOP,
						0,  // executionTimeValue,
						0,  // tim_changeSpeed,
						tim_start_iteration);
				}
				return;
			}
		}
		else
		{
			// Make sure the dynamic state of a NOP cmd has not overwritten the
			// state for a "regular" PTG choice:
			for (size_t i = 0; i < nPTGs; i++)
			{
				getPTG(i)->updateNavDynamicState(ptg_dynState);
			}

			// STEP7: Get the non-holonomic movement command.
			// ---------------------------------------------------------------------
			double cmd_vel_speed_ratio = 1.0;
			if (selectedHolonomicMovement)
			{
				CTimeLoggerEntry tle(
					m_timelogger, "navigationStep.selectedHolonomicMovement");
				cmd_vel_speed_ratio =
					generate_vel_cmd(*selectedHolonomicMovement, new_vel_cmd);
				ASSERT_(new_vel_cmd);
			}

			if (!new_vel_cmd /* which means best_PTG_eval==.0*/ ||
				new_vel_cmd->isStopCmd())
			{
				MRPT_LOG_DEBUG(
					"Best velocity command is STOP (no way found), calling "
					"robot.stop()");
				this->stop(true /* emergency */);  // don't call
				// doEmergencyStop() here
				// since that will stop
				// navigation completely
				new_vel_cmd = m_robot.getEmergencyStopCmd();
				m_lastSentVelCmd.reset();
			}
			else
			{
				mrpt::system::TTimeStamp tim_send_cmd_vel;
				{
					mrpt::system::CTimeLoggerEntry tle(
						m_timlog_delays, "changeSpeeds()");
					tim_send_cmd_vel = mrpt::system::now();
					newLogRec.timestamps["tim_send_cmd_vel"] = tim_send_cmd_vel;
					if (!this->changeSpeeds(*new_vel_cmd))
					{
						doEmergencyStop(
							"\nERROR calling changeSpeeds()!! Stopping robot "
							"and finishing navigation\n");
						if (fill_log_record)
						{
							new_vel_cmd = m_robot.getEmergencyStopCmd();
							STEP8_GenerateLogRecord(
								newLogRec, relTargets, best_ptg_idx,
								new_vel_cmd, nPTGs, best_is_NOP_cmdvel,
								rel_cur_pose_wrt_last_vel_cmd_NOP,
								rel_pose_PTG_origin_wrt_sense_NOP,
								0,  // executionTimeValue,
								0,  // tim_changeSpeed,
								tim_start_iteration);
						}
						return;
					}
				}
				// Save last sent cmd:
				m_lastSentVelCmd.speed_scale = cmd_vel_speed_ratio;
				m_lastSentVelCmd.ptg_index = best_ptg_idx;
				m_lastSentVelCmd.ptg_alpha_index =
					selectedHolonomicMovement
						? selectedHolonomicMovement->PTG->alpha2index(
							  selectedHolonomicMovement->direction)
						: 0;
				m_lastSentVelCmd.original_holo_eval =
					selectedHolonomicMovement->props["holo_stage_eval"];

				m_lastSentVelCmd.colfreedist_move_k =
					best_ptg_idx >= 0
						? m_infoPerPTG[best_ptg_idx]
							  .TP_Obstacles[m_lastSentVelCmd.ptg_alpha_index]
						: .0;
				m_lastSentVelCmd.was_slowdown =
					(selectedHolonomicMovement->props["is_slowdown"] != 0.0);

				m_lastSentVelCmd.poseVel = m_curPoseVel;
				m_lastSentVelCmd.tim_send_cmd_vel = tim_send_cmd_vel;
				m_lastSentVelCmd.ptg_dynState = ptg_dynState;

				// Update delay model:
				const double timoff_sendVelCmd = mrpt::system::timeDifference(
					tim_start_iteration, tim_send_cmd_vel);
				timoff_sendVelCmd_avr.filter(timoff_sendVelCmd);
				newLogRec.values["timoff_sendVelCmd"] = timoff_sendVelCmd;
				newLogRec.values["timoff_sendVelCmd_avr"] =
					timoff_sendVelCmd_avr.getLastOutput();
			}
		}

		// ------- end of motion decision zone ---------

		// Statistics:
		// ----------------------------------------------------
		const double executionTimeValue = executionTime.Tac();
		meanExecutionTime.filter(executionTimeValue);
		meanTotalExecutionTime.filter(totalExecutionTime.Tac());

		const double tim_changeSpeed =
			m_timlog_delays.getLastTime("changeSpeeds()");
		tim_changeSpeed_avr.filter(tim_changeSpeed);

		// Running period estim:
		const double period_tim = timerForExecutionPeriod.Tac();
		if (period_tim > 1.5 * meanExecutionPeriod.getLastOutput())
		{
			MRPT_LOG_WARN_FMT(
				"Timing warning: Suspicious executionPeriod=%.03f ms is far "
				"above the average of %.03f ms",
				1e3 * period_tim, meanExecutionPeriod.getLastOutput() * 1e3);
		}
		meanExecutionPeriod.filter(period_tim);
		timerForExecutionPeriod.Tic();

		if (m_enableConsoleOutput)
		{
			MRPT_LOG_DEBUG(mrpt::format(
				"CMD: %s "
				"speedScale=%.04f "
				"T=%.01lfms Exec:%.01lfms|%.01lfms "
				"PTG#%i\n",
				new_vel_cmd ? new_vel_cmd->asString().c_str() : "NOP",
				selectedHolonomicMovement ? selectedHolonomicMovement->speed
										  : .0,
				1000.0 * meanExecutionPeriod.getLastOutput(),
				1000.0 * meanExecutionTime.getLastOutput(),
				1000.0 * meanTotalExecutionTime.getLastOutput(), best_ptg_idx));
		}
		if (fill_log_record)
		{
			STEP8_GenerateLogRecord(
				newLogRec, relTargets, best_ptg_idx, new_vel_cmd, nPTGs,
				best_is_NOP_cmdvel, rel_cur_pose_wrt_last_vel_cmd_NOP,
				rel_pose_PTG_origin_wrt_sense_NOP, executionTimeValue,
				tim_changeSpeed, tim_start_iteration);
		}
	}
	catch (const std::exception& e)
	{
		doEmergencyStop(
			std::string("[CAbstractPTGBasedReactive::performNavigationStep] "
						"Stopping robot and finishing navigation due to "
						"exception:\n") +
			std::string(e.what()));
	}
	catch (...)
	{
		doEmergencyStop(
			"[CAbstractPTGBasedReactive::performNavigationStep] Stopping robot "
			"and finishing navigation due to untyped exception.");
	}
}

/** \callergraph */
void CAbstractPTGBasedReactive::STEP8_GenerateLogRecord(
	CLogFileRecord& newLogRec, const std::vector<TPose2D>& relTargets,
	int nSelectedPTG, const mrpt::kinematics::CVehicleVelCmd::Ptr& new_vel_cmd,
	const int nPTGs, const bool best_is_NOP_cmdvel,
	const mrpt::math::TPose2D& rel_cur_pose_wrt_last_vel_cmd_NOP,
	const mrpt::math::TPose2D& rel_pose_PTG_origin_wrt_sense_NOP,
	const double executionTimeValue, const double tim_changeSpeed,
	const mrpt::system::TTimeStamp& tim_start_iteration)
{
	// ---------------------------------------
	// STEP8: Generate log record
	// ---------------------------------------
	m_timelogger.enter("navigationStep.populate_log_info");

	this->loggingGetWSObstaclesAndShape(newLogRec);

	newLogRec.robotPoseLocalization = m_curPoseVel.pose;
	newLogRec.robotPoseOdometry = m_curPoseVel.rawOdometry;
	newLogRec.WS_targets_relative = relTargets;
	newLogRec.nSelectedPTG = nSelectedPTG;
	newLogRec.cur_vel = m_curPoseVel.velGlobal;
	newLogRec.cur_vel_local = m_curPoseVel.velLocal;
	newLogRec.cmd_vel = new_vel_cmd;
	newLogRec.values["estimatedExecutionPeriod"] =
		meanExecutionPeriod.getLastOutput();
	newLogRec.values["executionTime"] = executionTimeValue;
	newLogRec.values["executionTime_avr"] = meanExecutionTime.getLastOutput();
	newLogRec.values["time_changeSpeeds()"] = tim_changeSpeed;
	newLogRec.values["time_changeSpeeds()_avr"] =
		tim_changeSpeed_avr.getLastOutput();
	newLogRec.values["CWaypointsNavigator::navigationStep()"] =
		m_timlog_delays.getLastTime("CWaypointsNavigator::navigationStep()");
	newLogRec.values["CAbstractNavigator::navigationStep()"] =
		m_timlog_delays.getLastTime("CAbstractNavigator::navigationStep()");
	newLogRec.timestamps["tim_start_iteration"] = tim_start_iteration;
	newLogRec.timestamps["curPoseAndVel"] = m_curPoseVel.timestamp;
	newLogRec.nPTGs = nPTGs;

	// NOP mode  stuff:
	newLogRec.rel_cur_pose_wrt_last_vel_cmd_NOP =
		rel_cur_pose_wrt_last_vel_cmd_NOP;
	newLogRec.rel_pose_PTG_origin_wrt_sense_NOP =
		rel_pose_PTG_origin_wrt_sense_NOP;
	newLogRec.ptg_index_NOP =
		best_is_NOP_cmdvel ? m_lastSentVelCmd.ptg_index : -1;
	newLogRec.ptg_last_k_NOP = m_lastSentVelCmd.ptg_alpha_index;
	newLogRec.ptg_last_navDynState = m_lastSentVelCmd.ptg_dynState;

	m_timelogger.leave("navigationStep.populate_log_info");

	//  Save to log file:
	// --------------------------------------
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timelogger, "navigationStep.write_log_file");
		if (m_logFile) archiveFrom(*m_logFile) << newLogRec;
	}
	// Set as last log record
	{
		std::lock_guard<std::recursive_mutex> lock_log(
			m_critZoneLastLog);  // Lock
		lastLogRecord = newLogRec;  // COPY
	}
}

/** \callergraph */
void CAbstractPTGBasedReactive::calc_move_candidate_scores(
	TCandidateMovementPTG& cm, const std::vector<double>& in_TPObstacles,
	const mrpt::nav::ClearanceDiagram& in_clearance,
	const std::vector<mrpt::math::TPose2D>& WS_Targets,
	const std::vector<CAbstractPTGBasedReactive::PTGTarget>& TP_Targets,
	CLogFileRecord::TInfoPerPTG& log, CLogFileRecord& newLogRec,
	const bool this_is_PTG_continuation,
	const mrpt::math::TPose2D& rel_cur_pose_wrt_last_vel_cmd_NOP,
	const unsigned int ptg_idx4weights,
	const mrpt::system::TTimeStamp tim_start_iteration,
	const mrpt::nav::CHolonomicLogFileRecord::Ptr& hlfr)
{
	MRPT_START;

	const double ref_dist = cm.PTG->getRefDistance();

	// Replaced by: TP_Targets[i].*
	// const double   target_dir    = (TP_Target.x!=0 || TP_Target.y!=0) ?
	// atan2( TP_Target.y, TP_Target.x) : 0.0;
	// const int      target_k = static_cast<int>( cm.PTG->alpha2index(
	// target_dir ) );
	// const double   target_d_norm   = TP_Target.norm();

	// We need to evaluate the movement wrt to ONE target of the possibly many
	// input ones.
	// Policy: use the target whose TP-Space direction is closer to this
	// candidate direction:
	size_t selected_trg_idx = 0;
	{
		double best_trg_angdist = std::numeric_limits<double>::max();
		for (size_t i = 0; i < TP_Targets.size(); i++)
		{
			const double angdist = std::abs(mrpt::math::angDistance(
				TP_Targets[i].target_alpha, cm.direction));
			if (angdist < best_trg_angdist)
			{
				best_trg_angdist = angdist;
				selected_trg_idx = i;
			}
		}
	}
	ASSERT_(selected_trg_idx < WS_Targets.size());
	const auto WS_Target = WS_Targets[selected_trg_idx];
	const auto TP_Target = TP_Targets[selected_trg_idx];

	const double target_d_norm = TP_Target.target_dist;

	// Picked movement direction:
	const int move_k = static_cast<int>(cm.PTG->alpha2index(cm.direction));
	const double target_WS_d = WS_Target.norm();

	// Coordinates of the trajectory end for the given PTG and "alpha":
	const double d = std::min(in_TPObstacles[move_k], 0.99 * target_d_norm);
	uint32_t nStep;
	bool pt_in_range = cm.PTG->getPathStepForDist(move_k, d, nStep);
	ASSERT_(pt_in_range);
	mrpt::math::TPose2D pose;
	cm.PTG->getPathPose(move_k, nStep, pose);

	// Make sure that the target slow-down is honored, as seen in real-world
	// Euclidean space
	// (as opposed to TP-Space, where the holo methods are evaluated)
	if (m_navigationParams &&
		m_navigationParams->target.targetDesiredRelSpeed < 1.0 &&
		!m_holonomicMethod.empty() && getHoloMethod(0) != nullptr &&
		!cm.PTG->supportSpeedAtTarget()  // If the PTG is able to handle the
		// slow-down on its own, dont change
		// speed here
	)
	{
		const double TARGET_SLOW_APPROACHING_DISTANCE =
			getHoloMethod(0)->getTargetApproachSlowDownDistance();

		const double Vf =
			m_navigationParams->target.targetDesiredRelSpeed;  // [0,1]

		const double f = std::min(
			1.0,
			Vf + target_WS_d * (1.0 - Vf) / TARGET_SLOW_APPROACHING_DISTANCE);
		if (f < cm.speed)
		{
			newLogRec.additional_debug_msgs["PTG_eval.speed"] = mrpt::format(
				"Relative speed reduced %.03f->%.03f based on Euclidean "
				"nearness to target.",
				cm.speed, f);
			cm.speed = f;
		}
	}

	// Start storing params in the candidate move structure:
	cm.props["ptg_idx"] = ptg_idx4weights;
	cm.props["ref_dist"] = ref_dist;
	cm.props["target_dir"] = TP_Target.target_alpha;
	cm.props["target_k"] = TP_Target.target_k;
	cm.props["target_d_norm"] = target_d_norm;
	cm.props["move_k"] = move_k;
	double& move_cur_d = cm.props["move_cur_d"] =
		0;  // current robot path normalized distance over path (0 unless in a
	// NOP cmd)
	cm.props["is_PTG_cont"] = this_is_PTG_continuation ? 1 : 0;
	cm.props["num_paths"] = in_TPObstacles.size();
	cm.props["WS_target_x"] = WS_Target.x;
	cm.props["WS_target_y"] = WS_Target.y;
	cm.props["robpose_x"] = pose.x;
	cm.props["robpose_y"] = pose.y;
	cm.props["robpose_phi"] = pose.phi;
	cm.props["ptg_priority"] =
		cm.PTG->getScorePriority() *
		cm.PTG->evalPathRelativePriority(TP_Target.target_k, target_d_norm);
	const bool is_slowdown =
		this_is_PTG_continuation
			? m_lastSentVelCmd.was_slowdown
			: (cm.PTG->supportSpeedAtTarget() && TP_Target.target_k == move_k);
	cm.props["is_slowdown"] = is_slowdown ? 1 : 0;
	cm.props["holo_stage_eval"] =
		this_is_PTG_continuation
			? m_lastSentVelCmd.original_holo_eval
			: (hlfr && !hlfr->dirs_eval.empty() &&
			   hlfr->dirs_eval.rbegin()->size() == in_TPObstacles.size())
				  ? hlfr->dirs_eval.rbegin()->at(move_k)
				  : .0;
	// Factor 1: Free distance for the chosen PTG and "alpha" in the TP-Space:
	// ----------------------------------------------------------------------
	double& colfree = cm.props["collision_free_distance"];

	// distance to collision:
	colfree = in_TPObstacles[move_k];  // we'll next substract here the
	// already-traveled distance, for NOP
	// motion candidates.

	// Special case for NOP motion cmd:
	// consider only the empty space *after* the current robot pose, which is
	// not at the origin.

	if (this_is_PTG_continuation)
	{
		int cur_k = 0;
		double cur_norm_d = .0;
		bool is_exact, is_time_based = false;
		uint32_t cur_ptg_step = 0;

		// Use: time-based prediction for shorter distances, PTG inverse
		// mapping-based for longer ranges:
		const double maxD = params_abstract_ptg_navigator
								.max_dist_for_timebased_path_prediction;
		if (std::abs(rel_cur_pose_wrt_last_vel_cmd_NOP.x) > maxD ||
			std::abs(rel_cur_pose_wrt_last_vel_cmd_NOP.y) > maxD)
		{
			is_exact = cm.PTG->inverseMap_WS2TP(
				rel_cur_pose_wrt_last_vel_cmd_NOP.x,
				rel_cur_pose_wrt_last_vel_cmd_NOP.y, cur_k, cur_norm_d);
		}
		else
		{
			// Use time:
			is_time_based = true;
			is_exact = true;  // well, sort of...
			const double NOP_At =
				m_lastSentVelCmd.speed_scale *
				mrpt::system::timeDifference(
					m_lastSentVelCmd.tim_send_cmd_vel, tim_start_iteration);
			newLogRec.additional_debug_msgs["PTG_eval.NOP_At"] =
				mrpt::format("%.06f s", NOP_At);
			cur_k = move_k;
			cur_ptg_step = mrpt::round(NOP_At / cm.PTG->getPathStepDuration());
			cur_norm_d = cm.PTG->getPathDist(cur_k, cur_ptg_step) /
						 cm.PTG->getRefDistance();
			{
				const double cur_a = cm.PTG->index2alpha(cur_k);
				log.TP_Robot.x = cos(cur_a) * cur_norm_d;
				log.TP_Robot.y = sin(cur_a) * cur_norm_d;
				cm.starting_robot_dir = cur_a;
				cm.starting_robot_dist = cur_norm_d;
			}
		}

		if (!is_exact)
		{
			// Don't trust this step: we are not 100% sure of the robot pose in
			// TP-Space for this "PTG continuation" step:
			cm.speed = -0.01;  // this enforces a 0 global evaluation score
			newLogRec.additional_debug_msgs["PTG_eval"] =
				"PTG-continuation not allowed, cur. pose out of PTG domain.";
			return;
		}
		bool WS_point_is_unique = true;
		if (!is_time_based)
		{
			bool ok1 = cm.PTG->getPathStepForDist(
				m_lastSentVelCmd.ptg_alpha_index,
				cur_norm_d * cm.PTG->getRefDistance(), cur_ptg_step);
			if (ok1)
			{
				// Check bijective:
				WS_point_is_unique = cm.PTG->isBijectiveAt(cur_k, cur_ptg_step);
				const uint32_t predicted_step =
					mrpt::system::timeDifference(
						m_lastSentVelCmd.tim_send_cmd_vel,
						mrpt::system::now()) /
					cm.PTG->getPathStepDuration();
				WS_point_is_unique =
					WS_point_is_unique &&
					cm.PTG->isBijectiveAt(move_k, predicted_step);
				newLogRec.additional_debug_msgs["PTG_eval.bijective"] =
					mrpt::format(
						"isBijectiveAt(): k=%i step=%i -> %s", (int)cur_k,
						(int)cur_ptg_step, WS_point_is_unique ? "yes" : "no");

				if (!WS_point_is_unique)
				{
					// Don't trust direction:
					cur_k = move_k;
					cur_ptg_step = predicted_step;
					cur_norm_d = cm.PTG->getPathDist(cur_k, cur_ptg_step);
				}
				{
					const double cur_a = cm.PTG->index2alpha(cur_k);
					log.TP_Robot.x = cos(cur_a) * cur_norm_d;
					log.TP_Robot.y = sin(cur_a) * cur_norm_d;
					cm.starting_robot_dir = cur_a;
					cm.starting_robot_dist = cur_norm_d;
				}

				mrpt::math::TPose2D predicted_rel_pose;
				cm.PTG->getPathPose(
					m_lastSentVelCmd.ptg_alpha_index, cur_ptg_step,
					predicted_rel_pose);
				const auto predicted_pose_global =
					m_lastSentVelCmd.poseVel.rawOdometry + predicted_rel_pose;
				const double predicted2real_dist = mrpt::hypot_fast(
					predicted_pose_global.x - m_curPoseVel.rawOdometry.x,
					predicted_pose_global.y - m_curPoseVel.rawOdometry.y);
				newLogRec.additional_debug_msgs["PTG_eval.lastCmdPose(raw)"] =
					m_lastSentVelCmd.poseVel.pose.asString();
				newLogRec.additional_debug_msgs["PTG_eval.PTGcont"] =
					mrpt::format(
						"mismatchDistance=%.03f cm", 1e2 * predicted2real_dist);

				if (predicted2real_dist >
						params_abstract_ptg_navigator
							.max_distance_predicted_actual_path &&
					(!is_slowdown ||
					 (target_d_norm - cur_norm_d) * ref_dist > 2.0 /*meters*/))
				{
					cm.speed =
						-0.01;  // this enforces a 0 global evaluation score
					newLogRec.additional_debug_msgs["PTG_eval"] =
						"PTG-continuation not allowed, mismatchDistance above "
						"threshold.";
					return;
				}
			}
			else
			{
				cm.speed = -0.01;  // this enforces a 0 global evaluation score
				newLogRec.additional_debug_msgs["PTG_eval"] =
					"PTG-continuation not allowed, couldn't get PTG step for "
					"cur. robot pose.";
				return;
			}
		}

		// Path following isn't perfect: we can't be 100% sure of whether the
		// robot followed exactly
		// the intended path (`kDirection`), or if it's actually a bit shifted,
		// as reported in `cur_k`.
		// Take the least favorable case.
		// [Update] Do this only when the PTG gave us a unique-mapped WS<->TPS
		// point:

		colfree = WS_point_is_unique
					  ? std::min(in_TPObstacles[move_k], in_TPObstacles[cur_k])
					  : in_TPObstacles[move_k];

		// Only discount free space if there was a real obstacle, not the "end
		// of path" due to limited refDistance.
		if (colfree < 0.99)
		{
			colfree -= cur_norm_d;
		}

		// Save estimated robot pose over path as a parameter for scores:
		move_cur_d = cur_norm_d;
	}

	// Factor4: Decrease in euclidean distance between (x,y) and the target:
	//  Moving away of the target is negatively valued.
	cm.props["dist_eucl_final"] =
		mrpt::hypot_fast(WS_Target.x - pose.x, WS_Target.y - pose.y);

	// dist_eucl_min: Use PTG clearance methods to evaluate the real-world
	// (WorkSpace) minimum distance to target:
	{
		using map_d2d_t = std::map<double, double>;
		map_d2d_t pathDists;
		const double D = cm.PTG->getRefDistance();
		const int num_steps = ceil(D * 2.0);
		for (int i = 0; i < num_steps; i++)
		{
			pathDists[i / double(num_steps)] =
				100.0;  // default normalized distance to target (a huge value)
		}

		cm.PTG->evalClearanceSingleObstacle(
			WS_Target.x, WS_Target.y, move_k, pathDists,
			false /*treat point as target, not obstacle*/);

		const auto it = std::min_element(
			pathDists.begin(), pathDists.end(),
			[colfree](map_d2d_t::value_type& l, map_d2d_t::value_type& r)
				-> bool { return (l.second < r.second) && l.first < colfree; });
		cm.props["dist_eucl_min"] = (it != pathDists.end())
										? it->second * cm.PTG->getRefDistance()
										: 100.0;
	}

	// Factor5: Hysteresis:
	// -----------------------------------------------------
	double& hysteresis = cm.props["hysteresis"];
	hysteresis = .0;

	if (cm.PTG->supportVelCmdNOP())
	{
		hysteresis = this_is_PTG_continuation ? 1.0 : 0.;
	}
	else if (m_last_vel_cmd)
	{
		mrpt::kinematics::CVehicleVelCmd::Ptr desired_cmd;
		desired_cmd = cm.PTG->directionToMotionCommand(move_k);
		const mrpt::kinematics::CVehicleVelCmd* ptr1 = m_last_vel_cmd.get();
		const mrpt::kinematics::CVehicleVelCmd* ptr2 = desired_cmd.get();
		if (typeid(*ptr1) == typeid(*ptr2))
		{
			ASSERT_EQUAL_(
				m_last_vel_cmd->getVelCmdLength(),
				desired_cmd->getVelCmdLength());

			double simil_score = 0.5;
			for (size_t i = 0; i < desired_cmd->getVelCmdLength(); i++)
			{
				const double scr =
					exp(-std::abs(
							desired_cmd->getVelCmdElement(i) -
							m_last_vel_cmd->getVelCmdElement(i)) /
						0.20);
				mrpt::keep_min(simil_score, scr);
			}
			hysteresis = simil_score;
		}
	}

	// Factor6: clearance
	// -----------------------------------------------------
	// clearance indicators that may be useful in deciding the best motion:
	double& clearance = cm.props["clearance"];
	clearance = in_clearance.getClearance(
		move_k, target_d_norm * 1.01, false /* spot, dont interpolate */);
	cm.props["clearance_50p"] = in_clearance.getClearance(
		move_k, target_d_norm * 0.5, false /* spot, dont interpolate */);
	cm.props["clearance_path"] = in_clearance.getClearance(
		move_k, target_d_norm * 0.9, true /* average */);
	cm.props["clearance_path_50p"] = in_clearance.getClearance(
		move_k, target_d_norm * 0.5, true /* average */);

	// Factor: ETA (Estimated Time of Arrival to target or to closest obstacle,
	// whatever it's first)
	// -----------------------------------------------------
	double& eta = cm.props["eta"];
	eta = .0;
	if (cm.PTG && cm.speed > .0)  // for valid cases only
	{
		// OK, we have a direct path to target without collisions.
		const double path_len_meters = d * ref_dist;

		// Calculate their ETA
		uint32_t target_step;
		bool valid_step =
			cm.PTG->getPathStepForDist(move_k, path_len_meters, target_step);
		if (valid_step)
		{
			eta = cm.PTG->getPathStepDuration() *
				  target_step /* PTG original time to get to target point */
				  * cm.speed /* times the speed scale factor*/;

			double discount_time = .0;
			if (this_is_PTG_continuation)
			{
				// Heuristic: discount the time already executed.
				// Note that hm.speed above scales the overall path time using
				// the current speed scale, not the exact
				// integration over the past timesteps. It's an approximation,
				// probably good enough...
				discount_time = mrpt::system::timeDifference(
					m_lastSentVelCmd.tim_send_cmd_vel, tim_start_iteration);
			}
			eta -= discount_time;  // This could even become negative if the
			// approximation is poor...
		}
	}

	MRPT_END;
}

double CAbstractPTGBasedReactive::generate_vel_cmd(
	const TCandidateMovementPTG& in_movement,
	mrpt::kinematics::CVehicleVelCmd::Ptr& new_vel_cmd)
{
	mrpt::system::CTimeLoggerEntry tle(m_timelogger, "generate_vel_cmd");
	double cmdvel_speed_scale = 1.0;
	try
	{
		if (in_movement.speed == 0)
		{
			// The robot will stop:
			new_vel_cmd =
				in_movement.PTG->getSupportedKinematicVelocityCommand();
			new_vel_cmd->setToStop();
		}
		else
		{
			// Take the normalized movement command:
			new_vel_cmd = in_movement.PTG->directionToMotionCommand(
				in_movement.PTG->alpha2index(in_movement.direction));

			// Scale holonomic speeds to real-world one:
			new_vel_cmd->cmdVel_scale(in_movement.speed);
			cmdvel_speed_scale *= in_movement.speed;

			if (!m_last_vel_cmd)  // first iteration? Use default values:
				m_last_vel_cmd =
					in_movement.PTG->getSupportedKinematicVelocityCommand();

			// Honor user speed limits & "blending":
			const double beta = meanExecutionPeriod.getLastOutput() /
								(meanExecutionPeriod.getLastOutput() +
								 params_abstract_ptg_navigator.speedfilter_tau);
			cmdvel_speed_scale *= new_vel_cmd->cmdVel_limits(
				*m_last_vel_cmd, beta,
				params_abstract_ptg_navigator.robot_absolute_speed_limits);
		}

		m_last_vel_cmd = new_vel_cmd;  // Save for filtering in next step
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"[CAbstractPTGBasedReactive::generate_vel_cmd] Exception: "
			<< e.what());
	}
	return cmdvel_speed_scale;
}

/** \callergraph */
bool CAbstractPTGBasedReactive::impl_waypoint_is_reachable(
	const mrpt::math::TPoint2D& wp) const
{
	MRPT_START;

	const size_t N = this->getPTG_count();
	if (m_infoPerPTG.size() < N ||
		m_infoPerPTG_timestamp == INVALID_TIMESTAMP ||
		mrpt::system::timeDifference(
			m_infoPerPTG_timestamp, mrpt::system::now()) > 0.5)
		return false;  // We didn't run yet or obstacle info is old

	for (size_t i = 0; i < N; i++)
	{
		const CParameterizedTrajectoryGenerator* ptg = getPTG(i);

		const std::vector<double>& tp_obs =
			m_infoPerPTG[i].TP_Obstacles;  // normalized distances
		if (tp_obs.size() != ptg->getPathCount())
			continue;  // May be this PTG has not been used so far? (Target out
		// of domain,...)

		int wp_k;
		double wp_norm_d;
		bool is_into_domain =
			ptg->inverseMap_WS2TP(wp.x, wp.y, wp_k, wp_norm_d);
		if (!is_into_domain) continue;

		ASSERT_(wp_k < int(tp_obs.size()));

		const double collision_free_dist = tp_obs[wp_k];
		if (collision_free_dist > 1.01 * wp_norm_d)
			return true;  // free path found to target
	}

	return false;  // no way found
	MRPT_END;
}

/** \callergraph */
bool CAbstractPTGBasedReactive::STEP2_SenseObstacles()
{
	return implementSenseObstacles(m_WS_Obstacles_timestamp);
}

/** \callergraph */
void CAbstractPTGBasedReactive::onStartNewNavigation()
{
	m_last_curPoseVelUpdate_robot_time = -1e9;
	m_lastSentVelCmd.reset();

	CWaypointsNavigator::onStartNewNavigation();  // Call base method we
	// override
}

CAbstractPTGBasedReactive::TSentVelCmd::TSentVelCmd() { reset(); }
void CAbstractPTGBasedReactive::TSentVelCmd::reset()
{
	ptg_index = -1;
	ptg_alpha_index = -1;
	tim_send_cmd_vel = INVALID_TIMESTAMP;
	poseVel = TRobotPoseVel();
	colfreedist_move_k = .0;
	was_slowdown = false;
	speed_scale = 1.0;
	original_holo_eval = .0;
	ptg_dynState = CParameterizedTrajectoryGenerator::TNavDynamicState();
}
bool CAbstractPTGBasedReactive::TSentVelCmd::isValid() const
{
	return this->poseVel.timestamp != INVALID_TIMESTAMP;
}

/** \callergraph */
void CAbstractPTGBasedReactive::build_movement_candidate(
	CParameterizedTrajectoryGenerator* ptg, const size_t indexPTG,
	const std::vector<mrpt::math::TPose2D>& relTargets,
	const mrpt::math::TPose2D& rel_pose_PTG_origin_wrt_sense, TInfoPerPTG& ipf,
	TCandidateMovementPTG& cm, CLogFileRecord& newLogRec,
	const bool this_is_PTG_continuation,
	mrpt::nav::CAbstractHolonomicReactiveMethod& holoMethod,
	const mrpt::system::TTimeStamp tim_start_iteration,
	const TNavigationParams& navp,
	const mrpt::math::TPose2D& rel_cur_pose_wrt_last_vel_cmd_NOP)
{
	ASSERT_(ptg);

	const size_t idx_in_log_infoPerPTGs =
		this_is_PTG_continuation ? getPTG_count() : indexPTG;

	CHolonomicLogFileRecord::Ptr HLFR;
	cm.PTG = ptg;

	// If the user doesn't want to use this PTG, just mark it as invalid:
	ipf.targets.clear();
	bool use_this_ptg = true;
	{
		const auto* navpPTG = dynamic_cast<const TNavigationParamsPTG*>(&navp);
		if (navpPTG && !navpPTG->restrict_PTG_indices.empty())
		{
			use_this_ptg = false;
			for (size_t i = 0;
				 i < navpPTG->restrict_PTG_indices.size() && !use_this_ptg; i++)
			{
				if (navpPTG->restrict_PTG_indices[i] == indexPTG)
					use_this_ptg = true;
			}
		}
	}

	double timeForTPObsTransformation = .0, timeForHolonomicMethod = .0;

	// Normal PTG validity filter: check if target falls into the PTG domain:
	bool any_TPTarget_is_valid = false;
	if (use_this_ptg)
	{
		for (const auto& trg : relTargets)
		{
			PTGTarget ptg_target;

			ptg_target.valid_TP = ptg->inverseMap_WS2TP(
				trg.x, trg.y, ptg_target.target_k, ptg_target.target_dist);
			if (!ptg_target.valid_TP) continue;

			any_TPTarget_is_valid = true;
			ptg_target.target_alpha = ptg->index2alpha(ptg_target.target_k);
			ptg_target.TP_Target.x =
				cos(ptg_target.target_alpha) * ptg_target.target_dist;
			ptg_target.TP_Target.y =
				sin(ptg_target.target_alpha) * ptg_target.target_dist;

			ipf.targets.emplace_back(ptg_target);
		}
	}

	if (!any_TPTarget_is_valid)
	{
		newLogRec.additional_debug_msgs[mrpt::format(
			"mov_candidate_%u", static_cast<unsigned int>(indexPTG))] =
			"PTG discarded since target(s) is(are) out of domain.";
	}
	else
	{
		//  STEP3(b): Build TP-Obstacles
		// -----------------------------------------------------------------------------
		{
			tictac.Tic();

			// Initialize TP-Obstacles:
			const size_t Ki = ptg->getAlphaValuesCount();
			ptg->initTPObstacles(ipf.TP_Obstacles);
			if (params_abstract_ptg_navigator.evaluate_clearance)
			{
				ptg->initClearanceDiagram(ipf.clearance);
			}

			// Implementation-dependent conversion:
			STEP3_WSpaceToTPSpace(
				indexPTG, ipf.TP_Obstacles, ipf.clearance,
				mrpt::math::TPose2D(0, 0, 0) - rel_pose_PTG_origin_wrt_sense,
				params_abstract_ptg_navigator.evaluate_clearance);

			if (params_abstract_ptg_navigator.evaluate_clearance)
			{
				ptg->updateClearancePost(ipf.clearance, ipf.TP_Obstacles);
			}

			// Distances in TP-Space are normalized to [0,1]:
			const double _refD = 1.0 / ptg->getRefDistance();
			for (size_t i = 0; i < Ki; i++) ipf.TP_Obstacles[i] *= _refD;

			timeForTPObsTransformation = tictac.Tac();
			if (m_timelogger.isEnabled())
				m_timelogger.registerUserMeasure(
					"navigationStep.STEP3_WSpaceToTPSpace",
					timeForTPObsTransformation);
		}

		//  STEP4: Holonomic navigation method
		// -----------------------------------------------------------------------------
		if (!this_is_PTG_continuation)
		{
			tictac.Tic();

			// Slow down if we are approaching the final target, etc.
			holoMethod.enableApproachTargetSlowDown(
				navp.target.targetDesiredRelSpeed < .11);

			// Prepare holonomic algorithm call:
			CAbstractHolonomicReactiveMethod::NavInput ni;
			ni.clearance = &ipf.clearance;
			ni.maxObstacleDist = 1.0;
			ni.maxRobotSpeed = 1.0;  // So, we use a normalized max speed here.
			ni.obstacles = ipf.TP_Obstacles;  // Normalized [0,1]

			ni.targets.clear();  // Normalized [0,1]
			for (const auto& t : ipf.targets)
			{
				ni.targets.push_back(t.TP_Target);
			}

			CAbstractHolonomicReactiveMethod::NavOutput no;

			holoMethod.navigate(ni, no);

			// Extract resuls:
			cm.direction = no.desiredDirection;
			cm.speed = no.desiredSpeed;
			HLFR = no.logRecord;

			// Security: Scale down the velocity when heading towards obstacles,
			//  such that it's assured that we never go thru an obstacle!
			const int kDirection =
				static_cast<int>(cm.PTG->alpha2index(cm.direction));
			double obsFreeNormalizedDistance = ipf.TP_Obstacles[kDirection];

			// Take into account the future robot pose after NOP motion
			// iterations to slow down accordingly *now*
			if (ptg->supportVelCmdNOP())
			{
				const double v = mrpt::hypot_fast(
					m_curPoseVel.velLocal.vx, m_curPoseVel.velLocal.vy);
				const double d = v * ptg->maxTimeInVelCmdNOP(kDirection);
				obsFreeNormalizedDistance = std::min(
					obsFreeNormalizedDistance,
					std::max(0.90, obsFreeNormalizedDistance - d));
			}

			double velScale = 1.0;
			ASSERT_(
				params_abstract_ptg_navigator.secure_distance_end >
				params_abstract_ptg_navigator.secure_distance_start);
			if (obsFreeNormalizedDistance <
				params_abstract_ptg_navigator.secure_distance_end)
			{
				if (obsFreeNormalizedDistance <=
					params_abstract_ptg_navigator.secure_distance_start)
					velScale = 0.0;  // security stop
				else
					velScale =
						(obsFreeNormalizedDistance -
						 params_abstract_ptg_navigator.secure_distance_start) /
						(params_abstract_ptg_navigator.secure_distance_end -
						 params_abstract_ptg_navigator.secure_distance_start);
			}

			// Scale:
			cm.speed *= velScale;

			timeForHolonomicMethod = tictac.Tac();
			if (m_timelogger.isEnabled())
				m_timelogger.registerUserMeasure(
					"navigationStep.STEP4_HolonomicMethod",
					timeForHolonomicMethod);
		}
		else
		{
			// "NOP cmdvel" case: don't need to re-run holo algorithm, just keep
			// the last selection:
			cm.direction = ptg->index2alpha(m_lastSentVelCmd.ptg_alpha_index);
			cm.speed = 1.0;  // Not used.
		}

		// STEP5: Evaluate each movement to assign them a "evaluation" value.
		// ---------------------------------------------------------------------
		{
			CTimeLoggerEntry tle(
				m_timelogger, "navigationStep.calc_move_candidate_scores");

			calc_move_candidate_scores(
				cm, ipf.TP_Obstacles, ipf.clearance, relTargets, ipf.targets,
				newLogRec.infoPerPTG[idx_in_log_infoPerPTGs], newLogRec,
				this_is_PTG_continuation, rel_cur_pose_wrt_last_vel_cmd_NOP,
				indexPTG, tim_start_iteration, HLFR);

			// Store NOP related extra vars:
			cm.props["original_col_free_dist"] =
				this_is_PTG_continuation ? m_lastSentVelCmd.colfreedist_move_k
										 : .0;

			//  SAVE LOG
			newLogRec.infoPerPTG[idx_in_log_infoPerPTGs].evalFactors = cm.props;
		}

	}  // end "valid_TP"

	// Logging:
	const bool fill_log_record =
		(m_logFile != nullptr || m_enableKeepLogRecords);
	if (fill_log_record)
	{
		CLogFileRecord::TInfoPerPTG& ipp =
			newLogRec.infoPerPTG[idx_in_log_infoPerPTGs];
		if (!this_is_PTG_continuation)
			ipp.PTG_desc = ptg->getDescription();
		else
			ipp.PTG_desc = mrpt::format(
				"NOP cmdvel (prev PTG idx=%u)",
				static_cast<unsigned int>(m_lastSentVelCmd.ptg_index));

		mrpt::containers::copy_container_typecasting(
			ipf.TP_Obstacles, ipp.TP_Obstacles);
		ipp.clearance = ipf.clearance;
		ipp.TP_Targets.clear();
		for (const auto& t : ipf.targets)
		{
			ipp.TP_Targets.push_back(t.TP_Target);
		}
		ipp.HLFR = HLFR;
		ipp.desiredDirection = cm.direction;
		ipp.desiredSpeed = cm.speed;
		ipp.timeForTPObsTransformation = timeForTPObsTransformation;
		ipp.timeForHolonomicMethod = timeForHolonomicMethod;
	}
}

void CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	MRPT_START;

	robot_absolute_speed_limits.loadConfigFile(c, s);

	MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(holonomic_method, string);
	MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(motion_decider_method, string);
	MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(ref_distance, double);
	MRPT_LOAD_CONFIG_VAR_CS(speedfilter_tau, double);
	MRPT_LOAD_CONFIG_VAR_CS(secure_distance_start, double);
	MRPT_LOAD_CONFIG_VAR_CS(secure_distance_end, double);
	MRPT_LOAD_CONFIG_VAR_CS(use_delays_model, bool);
	MRPT_LOAD_CONFIG_VAR_CS(max_distance_predicted_actual_path, double);
	MRPT_LOAD_CONFIG_VAR_CS(
		min_normalized_free_space_for_ptg_continuation, double);
	MRPT_LOAD_CONFIG_VAR_CS(enable_obstacle_filtering, bool);
	MRPT_LOAD_CONFIG_VAR_CS(evaluate_clearance, bool);
	MRPT_LOAD_CONFIG_VAR_CS(max_dist_for_timebased_path_prediction, double);

	MRPT_END;
}

void CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	robot_absolute_speed_limits.saveToConfigFile(c, s);

	// Build list of known holo methods:
	string lstHoloStr = "# List of known classes:\n";
	{
		const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(
			CLASS_ID(CAbstractHolonomicReactiveMethod));
		for (const auto& cl : lst)
			lstHoloStr +=
				string("# - `") + string(cl->className) + string("`\n");
	}
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		holonomic_method,
		string("C++ class name of the holonomic navigation method to run in "
			   "the transformed TP-Space.\n") +
			lstHoloStr);

	// Build list of known decider methods:
	string lstDecidersStr = "# List of known classes:\n";
	{
		const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(
			CLASS_ID(CMultiObjectiveMotionOptimizerBase));
		for (const auto& cl : lst)
			lstDecidersStr +=
				string("# - `") + string(cl->className) + string("`\n");
	}
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		motion_decider_method,
		string("C++ class name of the motion decider method.\n") +
			lstDecidersStr);

	MRPT_SAVE_CONFIG_VAR_COMMENT(
		ref_distance,
		"Maximum distance up to obstacles will be considered (D_{max} in "
		"papers).");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		speedfilter_tau,
		"Time constant (in seconds) for the low-pass filter applied to "
		"kinematic velocity commands (default=0: no filtering)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		secure_distance_start,
		"In normalized distance [0,1], start/end of a ramp function that "
		"scales the holonomic navigator output velocity.");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		secure_distance_end,
		"In normalized distance [0,1], start/end of a ramp function that "
		"scales the holonomic navigator output velocity.");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		use_delays_model,
		"Whether to use robot pose inter/extrapolation to improve accuracy "
		"(Default:false)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		max_distance_predicted_actual_path,
		"Max distance [meters] to discard current PTG and issue a new vel cmd "
		"(default= 0.05)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		min_normalized_free_space_for_ptg_continuation,
		"Min normalized dist [0,1] after current pose in a PTG continuation to "
		"allow it.");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		enable_obstacle_filtering,
		"Enabled obstacle filtering (params in its own section)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		evaluate_clearance,
		"Enable exact computation of clearance (default=false)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		max_dist_for_timebased_path_prediction,
		"Max dist [meters] to use time-based path prediction for NOP "
		"evaluation");
}

CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::
	TAbstractPTGNavigatorParams()
	: holonomic_method(),
	  ptg_cache_files_directory("."),

	  robot_absolute_speed_limits()

{
}

void CAbstractPTGBasedReactive::loadConfigFile(
	const mrpt::config::CConfigFileBase& c)
{
	MRPT_START;
	m_PTGsMustBeReInitialized = true;

	// At this point, we have been called from the derived class, who must be
	// already loaded all its specific params, including PTGs.

	// Load my params:
	params_abstract_ptg_navigator.loadFromConfigFile(
		c, "CAbstractPTGBasedReactive");

	// Filtering:
	if (params_abstract_ptg_navigator.enable_obstacle_filtering)
	{
		auto* filter = new mrpt::maps::CPointCloudFilterByDistance;
		m_WS_filter = mrpt::maps::CPointCloudFilterBase::Ptr(filter);
		filter->options.loadFromConfigFile(c, "CPointCloudFilterByDistance");
	}
	else
	{
		m_WS_filter.reset();
	}

	// Movement chooser:
	m_multiobjopt = CMultiObjectiveMotionOptimizerBase::Factory(
		params_abstract_ptg_navigator.motion_decider_method);
	if (!m_multiobjopt)
		THROW_EXCEPTION_FMT(
			"Non-registered CMultiObjectiveMotionOptimizerBase className=`%s`",
			params_abstract_ptg_navigator.motion_decider_method.c_str());

	m_multiobjopt->loadConfigFile(c);

	// Holo method:
	this->setHolonomicMethod(params_abstract_ptg_navigator.holonomic_method, c);
	ASSERT_(!m_holonomicMethod.empty());
	CWaypointsNavigator::loadConfigFile(c);  // Load parent params

	m_init_done =
		true;  // If we reached this point without an exception, all is good.
	MRPT_END;
}

void CAbstractPTGBasedReactive::saveConfigFile(
	mrpt::config::CConfigFileBase& c) const
{
	CWaypointsNavigator::saveConfigFile(c);
	params_abstract_ptg_navigator.saveToConfigFile(
		c, "CAbstractPTGBasedReactive");

	// Filtering:
	{
		mrpt::maps::CPointCloudFilterByDistance filter;
		filter.options.saveToConfigFile(c, "CPointCloudFilterByDistance");
	}

	// Holo method:
	if (!m_holonomicMethod.empty() && m_holonomicMethod[0])
	{
		// Save my current settings:
		m_holonomicMethod[0]->saveConfigFile(c);
	}
	else
	{
		// save options of ALL known methods:
		const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(
			CLASS_ID(CAbstractHolonomicReactiveMethod));
		for (const auto& cl : lst)
		{
			mrpt::rtti::CObject::Ptr obj =
				mrpt::rtti::CObject::Ptr(cl->createObject());
			auto* holo =
				dynamic_cast<CAbstractHolonomicReactiveMethod*>(obj.get());
			if (holo)
			{
				holo->saveConfigFile(c);
			}
		}
	}

	// Decider method:
	if (m_multiobjopt)
	{
		// Save my current settings:
		m_multiobjopt->saveConfigFile(c);
	}
	else
	{
		// save options of ALL known methods:
		const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(
			CLASS_ID(CMultiObjectiveMotionOptimizerBase));
		for (const auto& cl : lst)
		{
			mrpt::rtti::CObject::Ptr obj =
				mrpt::rtti::CObject::Ptr(cl->createObject());
			auto* momo =
				dynamic_cast<CMultiObjectiveMotionOptimizerBase*>(obj.get());
			if (momo)
			{
				momo->saveConfigFile(c);
			}
		}
	}
}

void CAbstractPTGBasedReactive::setTargetApproachSlowDownDistance(
	const double dist)
{
	for (auto& o : m_holonomicMethod)
	{
		o->setTargetApproachSlowDownDistance(dist);
	}
}

double CAbstractPTGBasedReactive::getTargetApproachSlowDownDistance() const
{
	ASSERT_(!m_holonomicMethod.empty());
	return m_holonomicMethod[0]->getTargetApproachSlowDownDistance();
}

CAbstractHolonomicReactiveMethod* CAbstractPTGBasedReactive::getHoloMethod(
	int idx)
{
	return m_holonomicMethod[idx].get();
}
