/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/serialization/CArchive.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CSetOfLines.h>

using namespace mrpt::nav;

static std::string OUTPUT_DEBUG_PATH_PREFIX = "./reactivenav.logs";
static PTG_collision_behavior_t COLLISION_BEHAVIOR =
	mrpt::nav::COLL_BEH_BACK_AWAY;

std::string& CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX()
{
	return ::OUTPUT_DEBUG_PATH_PREFIX;
}

PTG_collision_behavior_t&
	CParameterizedTrajectoryGenerator::COLLISION_BEHAVIOR()
{
	return ::COLLISION_BEHAVIOR;
}

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CParameterizedTrajectoryGenerator, CSerializable, mrpt::nav)

CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator()
	: m_nav_dyn_state(), m_nav_dyn_state_target_k(INVALID_PTG_PATH_INDEX)

{
}

void CParameterizedTrajectoryGenerator::loadDefaultParams()
{
	m_alphaValuesCount = 121;
	refDistance = 6.0;
	m_score_priority = 1.0;
	m_clearance_num_points = 5;
	m_clearance_decimated_paths = 15;
}

bool CParameterizedTrajectoryGenerator::supportVelCmdNOP() const
{
	return false;
}
double CParameterizedTrajectoryGenerator::maxTimeInVelCmdNOP(int path_k) const
{
	return .0;
}

void CParameterizedTrajectoryGenerator::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(
		num_paths, uint64_t, m_alphaValuesCount, cfg, sSection);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(refDistance, double, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(
		score_priority, double, m_score_priority, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(
		clearance_num_points, double, m_clearance_num_points, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(
		clearance_decimated_paths, double, m_clearance_decimated_paths, cfg,
		sSection);

	// Ensure a minimum of resolution:
	mrpt::keep_max(
		m_clearance_num_points,
		static_cast<decltype(m_clearance_num_points)>(refDistance / 1.0));

	// Optional params, for debugging only
	MRPT_LOAD_HERE_CONFIG_VAR(
		vxi, double, m_nav_dyn_state.curVelLocal.vx, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(
		vyi, double, m_nav_dyn_state.curVelLocal.vy, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES(
		wi, double, m_nav_dyn_state.curVelLocal.omega, cfg, sSection);

	MRPT_LOAD_HERE_CONFIG_VAR(
		reltrg_x, double, m_nav_dyn_state.relTarget.x, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(
		reltrg_y, double, m_nav_dyn_state.relTarget.y, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES(
		reltrg_phi, double, m_nav_dyn_state.relTarget.phi, cfg, sSection);

	MRPT_LOAD_HERE_CONFIG_VAR(
		target_rel_speed, double, m_nav_dyn_state.targetRelSpeed, cfg,
		sSection);
}
void CParameterizedTrajectoryGenerator::saveToConfigFile(
	mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;

	cfg.write(
		sSection, "num_paths", m_alphaValuesCount, WN, WV,
		"Number of discrete paths (`resolution`) in the PTG");
	cfg.write(
		sSection, "refDistance", refDistance, WN, WV,
		"Maximum distance (meters) for building trajectories (visibility "
		"range)");
	cfg.write(
		sSection, "score_priority", m_score_priority, WN, WV,
		"When used in path planning, a multiplying factor (default=1.0) for "
		"the scores for this PTG. Assign values <1 to PTGs with low priority.");
	cfg.write(
		sSection, "clearance_num_points", m_clearance_num_points, WN, WV,
		"Number of steps for the piecewise-constant approximation of clearance "
		"(Default=5).");
	cfg.write(
		sSection, "clearance_decimated_paths", m_clearance_decimated_paths, WN,
		WV,
		"Number of decimated paths for estimation of clearance (Default=15).");

	// Optional params, for debugging only
	cfg.write(
		sSection, "vxi", m_nav_dyn_state.curVelLocal.vx, WN, WV,
		"(Only for debugging) Current robot velocity vx [m/s].");
	cfg.write(
		sSection, "vyi", m_nav_dyn_state.curVelLocal.vy, WN, WV,
		"(Only for debugging) Current robot velocity vy [m/s].");
	cfg.write(
		sSection, "wi", mrpt::RAD2DEG(m_nav_dyn_state.curVelLocal.omega), WN,
		WV, "(Only for debugging) Current robot velocity omega [deg/s].");

	cfg.write(
		sSection, "reltrg_x", m_nav_dyn_state.relTarget.x, WN, WV,
		"(Only for debugging) Relative target x [m].");
	cfg.write(
		sSection, "reltrg_y", m_nav_dyn_state.relTarget.y, WN, WV,
		"(Only for debugging) Relative target y [m].");
	cfg.write(
		sSection, "reltrg_phi", mrpt::RAD2DEG(m_nav_dyn_state.relTarget.phi),
		WN, WV, "(Only for debugging) Relative target phi [deg].");

	cfg.write(
		sSection, "target_rel_speed", m_nav_dyn_state.targetRelSpeed, WN, WV,
		"(Only for debugging) Desired relative speed at target [0,1]");

	MRPT_END
}

void CParameterizedTrajectoryGenerator::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	this->deinitialize();

	uint8_t version;
	in >> version;
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			in >> refDistance >> m_alphaValuesCount >> m_score_priority;
			if (version >= 1) in >> m_clearance_num_points;
			if (version == 2)
			{
				bool old_use_approx_clearance;
				in >> old_use_approx_clearance;  // ignored in v>=3
			}
			if (version >= 4)
			{
				in >> m_clearance_decimated_paths;
			}
			else
			{
				m_clearance_decimated_paths = m_alphaValuesCount;
			}
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CParameterizedTrajectoryGenerator::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const uint8_t version = 4;
	out << version;

	out << refDistance << m_alphaValuesCount << m_score_priority
		<< m_clearance_num_points /* v1 */;
	out << m_clearance_decimated_paths /* v4*/;
}

double CParameterizedTrajectoryGenerator::index2alpha(
	uint16_t k, const unsigned int num_paths)
{
	ASSERT_BELOW_(k, num_paths);
	return M_PI * (-1.0 + 2.0 * (k + 0.5) / num_paths);
}

double CParameterizedTrajectoryGenerator::index2alpha(uint16_t k) const
{
	return index2alpha(k, m_alphaValuesCount);
}

uint16_t CParameterizedTrajectoryGenerator::alpha2index(
	double alpha, const unsigned int num_paths)
{
	mrpt::math::wrapToPi(alpha);
	int k = mrpt::round(0.5 * (num_paths * (1.0 + alpha / M_PI) - 1.0));
	if (k < 0) k = 0;
	if (k >= static_cast<int>(num_paths)) k = num_paths - 1;
	return (uint16_t)k;
}

uint16_t CParameterizedTrajectoryGenerator::alpha2index(double alpha) const
{
	return alpha2index(alpha, m_alphaValuesCount);
}

void CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(
	const uint16_t k, mrpt::opengl::CSetOfLines& gl_obj,
	const double decimate_distance, const double max_path_distance) const
{
	const size_t nPointsInPath = getPathStepCount(k);

	bool first = true;
	// Decimate trajectories: we don't need centimeter resolution!
	double last_added_dist = 0.0;
	for (size_t n = 0; n < nPointsInPath; n++)
	{
		const double d = this->getPathDist(
			k, n);  // distance thru path "k" until timestep "n"

		// Draw the TP only until we reach the target of the "motion" segment:
		if (max_path_distance >= 0.0 && d >= max_path_distance) break;

		if (d < last_added_dist + decimate_distance && n != 0)
			continue;  // skip: decimation

		last_added_dist = d;

		mrpt::math::TPose2D p;
		this->getPathPose(k, n, p);

		if (first)
		{
			first = false;
			gl_obj.appendLine(0, 0, 0, p.x, p.y, 0);
		}
		else
			gl_obj.appendLineStrip(p.x, p.y, 0);
	}
}

void CParameterizedTrajectoryGenerator::initTPObstacles(
	std::vector<double>& TP_Obstacles) const
{
	TP_Obstacles.resize(m_alphaValuesCount);
	for (size_t k = 0; k < m_alphaValuesCount; k++)
		initTPObstacleSingle(k, TP_Obstacles[k]);
}
void CParameterizedTrajectoryGenerator::initTPObstacleSingle(
	uint16_t k, double& TP_Obstacle_k) const
{
	TP_Obstacle_k = std::min(
		refDistance, m_nav_dyn_state_target_k != INVALID_PTG_PATH_INDEX
						 ? refDistance
						 : this->getPathDist(k, this->getPathStepCount(k) - 1));
}

bool CParameterizedTrajectoryGenerator::debugDumpInFiles(
	const std::string& ptg_name) const
{
	using namespace mrpt::system;
	using namespace std;

	const char* sPath =
		CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX().c_str();

	mrpt::system::createDirectory(sPath);
	mrpt::system::createDirectory(mrpt::format("%s/PTGs", sPath));

	const string sFilTxt_x =
		mrpt::format("%s/PTGs/PTG%s_x.txt", sPath, ptg_name.c_str());
	const string sFilTxt_y =
		mrpt::format("%s/PTGs/PTG%s_y.txt", sPath, ptg_name.c_str());
	const string sFilTxt_phi =
		mrpt::format("%s/PTGs/PTG%s_phi.txt", sPath, ptg_name.c_str());
	const string sFilTxt_t =
		mrpt::format("%s/PTGs/PTG%s_t.txt", sPath, ptg_name.c_str());
	const string sFilTxt_d =
		mrpt::format("%s/PTGs/PTG%s_d.txt", sPath, ptg_name.c_str());

	ofstream fx(sFilTxt_x.c_str());
	if (!fx.is_open()) return false;
	ofstream fy(sFilTxt_y.c_str());
	if (!fy.is_open()) return false;
	ofstream fp(sFilTxt_phi.c_str());
	if (!fp.is_open()) return false;
	ofstream fd(sFilTxt_d.c_str());
	if (!fd.is_open()) return false;

	const size_t nPaths = getAlphaValuesCount();

	// Text version:
	fx << "% PTG data file for 'x'. Each row is the trajectory for a different "
		  "'alpha' parameter value."
	   << endl;
	fy << "% PTG data file for 'y'. Each row is the trajectory for a different "
		  "'alpha' parameter value."
	   << endl;
	fp << "% PTG data file for 'phi'. Each row is the trajectory for a "
		  "different 'alpha' parameter value."
	   << endl;
	fd << "% PTG data file for 'd'. Each row is the trajectory for a different "
		  "'alpha' parameter value."
	   << endl;

	vector<size_t> path_length(nPaths);
	for (size_t k = 0; k < nPaths; k++) path_length[k] = getPathStepCount(k);

	size_t maxPoints = 0;
	for (size_t k = 0; k < nPaths; k++)
		maxPoints = max(maxPoints, path_length[k]);

	for (size_t k = 0; k < nPaths; k++)
	{
		for (size_t n = 0; n < maxPoints; n++)
		{
			const size_t nn = std::min(n, path_length[k] - 1);
			mrpt::math::TPose2D p;
			this->getPathPose(k, nn, p);
			fx << p.x << " ";
			fy << p.y << " ";
			fp << p.phi << " ";
			fd << this->getPathDist(k, nn) << " ";
		}
		fx << endl;
		fy << endl;
		fp << endl;
		fd << endl;
	}

	return true;
}

bool CParameterizedTrajectoryGenerator::isInitialized() const
{
	return m_is_initialized;
}

void CParameterizedTrajectoryGenerator::updateNavDynamicState(
	const CParameterizedTrajectoryGenerator::TNavDynamicState& newState,
	const bool force_update)
{
	// Make sure there is a real difference: notifying a PTG that a condition
	// changed
	// may imply a significant computational cost if paths need to be
	// re-evaluated on the fly, etc.
	// so the cost of the comparison here is totally worth:
	if (force_update || m_nav_dyn_state != newState)
	{
		ASSERT_(
			newState.targetRelSpeed >= .0 &&
			newState.targetRelSpeed <= 1.0);  // sanity check
		m_nav_dyn_state = newState;

		// 1st) Build PTG paths without counting for target slow-down:
		m_nav_dyn_state_target_k = INVALID_PTG_PATH_INDEX;

		this->onNewNavDynamicState();

		// 2nd) Save the special path for slow-down:
		if (this->supportSpeedAtTarget())
		{
			int target_k = -1;
			double target_norm_d;
			// bool is_exact = // JLB removed this constraint for being too
			// restrictive.
			this->inverseMap_WS2TP(
				m_nav_dyn_state.relTarget.x, m_nav_dyn_state.relTarget.y,
				target_k, target_norm_d, 1.0 /*large tolerance*/);
			if (target_norm_d > 0.01 && target_norm_d < 0.99 && target_k >= 0 &&
				target_k < m_alphaValuesCount)
			{
				m_nav_dyn_state_target_k = target_k;
				this->onNewNavDynamicState();  // Recalc
			}
		}
	}
}

void CParameterizedTrajectoryGenerator::initialize(
	const std::string& cacheFilename, const bool verbose)
{
	if (m_is_initialized) return;

	const std::string sCache =
		!cacheFilename.empty()
			? cacheFilename
			: std::string("cache_") +
				  mrpt::system::fileNameStripInvalidChars(getDescription()) +
				  std::string(".bin.gz");

	this->internal_initialize(sCache, verbose);
	m_is_initialized = true;
}
void CParameterizedTrajectoryGenerator::deinitialize()
{
	if (!m_is_initialized) return;
	this->internal_deinitialize();
	m_is_initialized = false;
}

void CParameterizedTrajectoryGenerator::internal_TPObsDistancePostprocess(
	const double ox, const double oy, const double new_tp_obs_dist,
	double& inout_tp_obs) const
{
	const bool is_obs_inside_robot_shape = isPointInsideRobotShape(ox, oy);
	if (!is_obs_inside_robot_shape)
	{
		mrpt::keep_min(inout_tp_obs, new_tp_obs_dist);
		return;
	}

	// Handle the special case of obstacles *inside* the robot at the begining
	// of the PTG path:
	switch (COLLISION_BEHAVIOR())
	{
		case COLL_BEH_STOP:
			inout_tp_obs = .0;
			break;

		case COLL_BEH_BACK_AWAY:
		{
			if (new_tp_obs_dist < getMaxRobotRadius())
			{
				// This means that we are getting apart of the obstacle:
				// ignore it to allow the robot to get off the near-collision:
				// Don't change inout_tp_obs.
				return;
			}
			else
			{
				// This means we are already in collision and trying to get even
				// closer
				// to the obstacle: totally disprove this action:
				inout_tp_obs = .0;
			}
		}
		break;

		default:
			THROW_EXCEPTION("Obstacle postprocessing enum not implemented!");
	}
}

void mrpt::nav::CParameterizedTrajectoryGenerator::initClearanceDiagram(
	ClearanceDiagram& cd) const
{
	cd.resize(m_alphaValuesCount, m_clearance_decimated_paths);
	for (unsigned int decim_k = 0; decim_k < m_clearance_decimated_paths;
		 decim_k++)
	{
		const auto real_k = cd.decimated_k_to_real_k(decim_k);
		const size_t numPathSteps = getPathStepCount(real_k);
		const double numStepsPerIncr =
			(numPathSteps - 1.0) / double(m_clearance_num_points);

		auto& cl_path = cd.get_path_clearance_decimated(decim_k);
		for (double step_pointer_dbl = 0.0; step_pointer_dbl < numPathSteps;
			 step_pointer_dbl += numStepsPerIncr)
		{
			const size_t step = mrpt::round(step_pointer_dbl);
			const double dist_over_path = this->getPathDist(real_k, step);
			cl_path[dist_over_path] = 1.0;  // create entry in map<>
		}
	}
}

void CParameterizedTrajectoryGenerator::updateClearance(
	const double ox, const double oy, ClearanceDiagram& cd) const
{
	// Initialize CD on first call:
	ASSERT_(cd.get_actual_num_paths() == m_alphaValuesCount);
	ASSERT_(m_clearance_num_points > 0 && m_clearance_num_points < 10000);

	// evaluate in derived-class: this function also keeps the minimum
	// automatically.
	for (uint16_t decim_k = 0; decim_k < cd.get_decimated_num_paths();
		 decim_k++)
	{
		const auto real_k = cd.decimated_k_to_real_k(decim_k);
		this->evalClearanceSingleObstacle(
			ox, oy, real_k, cd.get_path_clearance_decimated(decim_k));
	}
}

void CParameterizedTrajectoryGenerator::updateClearancePost(
	ClearanceDiagram& cd, const std::vector<double>& TP_obstacles) const
{
	// Used only when in approx mode (Removed 30/01/2017)
}

void CParameterizedTrajectoryGenerator::evalClearanceSingleObstacle(
	const double ox, const double oy, const uint16_t k,
	ClearanceDiagram::dist2clearance_t& inout_realdist2clearance,
	bool treat_as_obstacle) const
{
	bool had_collision = false;

	const size_t numPathSteps = getPathStepCount(k);
	ASSERT_(numPathSteps > inout_realdist2clearance.size());

	const double numStepsPerIncr =
		(numPathSteps - 1.0) / (inout_realdist2clearance.size());

	double step_pointer_dbl = 0.0;
	const mrpt::math::TPoint2D og(ox, oy);  // obstacle in "global" frame
	mrpt::math::TPoint2D ol;  // obstacle in robot frame

	for (auto& e : inout_realdist2clearance)
	{
		step_pointer_dbl += numStepsPerIncr;
		const size_t step = mrpt::round(step_pointer_dbl);
		// const double dist_over_path = e.first;
		double& inout_clearance = e.second;

		if (had_collision)
		{
			// We found a collision in a previous step along this "k" path, so
			// it does not make sense to evaluate the clearance of a pose which
			// is not reachable:
			inout_clearance = .0;
			continue;
		}

		mrpt::math::TPose2D pose;
		this->getPathPose(k, step, pose);

		// obstacle to robot clearance:
		ol = pose.inverseComposePoint(og);
		const double this_clearance =
			treat_as_obstacle ? this->evalClearanceToRobotShape(ol.x, ol.y)
							  : ol.norm();
		if (this_clearance <= .0 && treat_as_obstacle)
		{
			// Collision:
			had_collision = true;
			inout_clearance = .0;
		}
		else
		{
			// The obstacle is not a direct collision.
			const double this_clearance_norm =
				this_clearance / this->refDistance;

			// Update minimum in output structure
			mrpt::keep_min(inout_clearance, this_clearance_norm);
		}
	}
}

CParameterizedTrajectoryGenerator::TNavDynamicState::TNavDynamicState()
	: curVelLocal(0, 0, 0), relTarget(20.0, 0, 0)

{
}

bool CParameterizedTrajectoryGenerator::TNavDynamicState::operator==(
	const TNavDynamicState& o) const
{
	return (curVelLocal == o.curVelLocal) && (relTarget == o.relTarget) &&
		   (targetRelSpeed == o.targetRelSpeed);
}

void mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState::
	writeToStream(mrpt::serialization::CArchive& out) const
{
	const uint8_t version = 0;
	out << version;
	// Data:
	out << curVelLocal << relTarget << targetRelSpeed;
}

void mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState::
	readFromStream(mrpt::serialization::CArchive& in)
{
	uint8_t version;
	in >> version;
	switch (version)
	{
		case 0:
			in >> curVelLocal >> relTarget >> targetRelSpeed;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
