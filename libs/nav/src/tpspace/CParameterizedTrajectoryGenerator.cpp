/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/utils/CStream.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CSetOfLines.h>

using namespace mrpt::nav;

std::string CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX = "./reactivenav.logs";
PTG_collision_behavior_t CParameterizedTrajectoryGenerator::COLLISION_BEHAVIOR = mrpt::nav::COLL_BEH_BACK_AWAY;


IMPLEMENTS_VIRTUAL_SERIALIZABLE(CParameterizedTrajectoryGenerator, CSerializable, mrpt::nav)

CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator() :
	refDistance(.0),
	m_alphaValuesCount(0),
	m_score_priority(1.0),
	m_clearance_num_points(5),
	m_use_approx_clearance(true),
	m_is_initialized(false)
{ }

void CParameterizedTrajectoryGenerator::loadDefaultParams()
{
	m_alphaValuesCount = 121;
	refDistance = 6.0;
	m_score_priority = 1.0;
	m_clearance_num_points = 5;
	m_use_approx_clearance = true;
}

bool CParameterizedTrajectoryGenerator::supportVelCmdNOP() const
{
	return false;
}
double CParameterizedTrajectoryGenerator::maxTimeInVelCmdNOP(int path_k) const
{
	return .0;
}


void CParameterizedTrajectoryGenerator::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(num_paths   , uint64_t, m_alphaValuesCount, cfg,sSection);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT     (refDistance , double,  cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(score_priority , double, m_score_priority, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(clearance_num_points, double, m_clearance_num_points, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(use_approx_clearance, bool, m_use_approx_clearance, cfg, sSection);
}
void CParameterizedTrajectoryGenerator::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;

	cfg.write(sSection,"num_paths",m_alphaValuesCount,   WN,WV, "Number of discrete paths (`resolution`) in the PTG");
	cfg.write(sSection,"refDistance",refDistance,   WN,WV, "Maximum distance (meters) for building trajectories (visibility range)");
	cfg.write(sSection,"score_priority",m_score_priority,   WN,WV, "When used in path planning, a multiplying factor (default=1.0) for the scores for this PTG. Assign values <1 to PTGs with low priority.");
	cfg.write(sSection, "clearance_num_points", m_clearance_num_points, WN, WV, "Number of steps for the piecewise-constant approximation of clearance (Default=5).");
	cfg.write(sSection,"use_approx_clearance", m_use_approx_clearance,   WN,WV, "Use approximate (faster) clearance calculation. (Default=true).");
	
	MRPT_END
}


void CParameterizedTrajectoryGenerator::internal_readFromStream(mrpt::utils::CStream &in)
{
	this->deinitialize();

	uint8_t version;
	in >> version;
	switch (version)
	{
	case 0:
	case 1:
	case 2:
		in >> refDistance >> m_alphaValuesCount >> m_score_priority;
		if (version >= 1) in >> m_clearance_num_points;
		if (version >= 2) in >> m_use_approx_clearance;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CParameterizedTrajectoryGenerator::internal_writeToStream(mrpt::utils::CStream &out) const
{
	const uint8_t version = 2;
	out << version;

	out << refDistance << m_alphaValuesCount << m_score_priority << m_clearance_num_points /* v1 */ << m_use_approx_clearance /*v2*/;
}

double CParameterizedTrajectoryGenerator::index2alpha(uint16_t k, const unsigned int num_paths)
{
	if (k >= num_paths) throw std::runtime_error("PTG: alpha value out of range!");
	return M_PI * (-1.0 + 2.0 * (k + 0.5) / num_paths);
}

double CParameterizedTrajectoryGenerator::index2alpha(uint16_t k) const
{
	return index2alpha(k, m_alphaValuesCount);
}

uint16_t CParameterizedTrajectoryGenerator::alpha2index(double alpha, const unsigned int num_paths)
{
	mrpt::math::wrapToPi(alpha);
	int k = mrpt::utils::round(0.5*(num_paths*(1.0 + alpha / M_PI) - 1.0));
	if (k<0) k = 0;
	if (k >= static_cast<int>(num_paths) ) k = num_paths - 1;
	return (uint16_t)k;
}

uint16_t CParameterizedTrajectoryGenerator::alpha2index( double alpha ) const 
{
	return alpha2index(alpha, m_alphaValuesCount);
}

void CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(
	const uint16_t k, 
	mrpt::opengl::CSetOfLines &gl_obj, 
	const double decimate_distance, 
	const double max_path_distance) const
{
	const size_t nPointsInPath = getPathStepCount(k);

	bool first=true;
	// Decimate trajectories: we don't need centimeter resolution!
	double last_added_dist = 0.0;
	for (size_t n=0;n<nPointsInPath;n++)
	{
		const double d = this->getPathDist(k, n); // distance thru path "k" until timestep "n"

		// Draw the TP only until we reach the target of the "motion" segment:
		if (max_path_distance>=0.0 && d>=max_path_distance) break;

		if (d<last_added_dist+decimate_distance && n!=0)
			continue; // skip: decimation

		last_added_dist = d;

		mrpt::math::TPose2D p;
		this->getPathPose(k, n, p);

		if (first) {
			first=false;
			gl_obj.appendLine(0,0,0, p.x, p.y,0);
		}
		else gl_obj.appendLineStrip(p.x, p.y,0);
	}
}


void CParameterizedTrajectoryGenerator::initTPObstacles(std::vector<double> &TP_Obstacles) const
{
	TP_Obstacles.resize(m_alphaValuesCount);
	for (size_t k = 0; k < m_alphaValuesCount; k++)
		initTPObstacleSingle(k, TP_Obstacles[k]);
}
void CParameterizedTrajectoryGenerator::initTPObstacleSingle(uint16_t k, double &TP_Obstacle_k) const
{
	TP_Obstacle_k = std::min(
		refDistance,
		this->getPathDist(k, this->getPathStepCount(k) - 1));
}


bool CParameterizedTrajectoryGenerator::debugDumpInFiles( const std::string &ptg_name ) const
{
	using namespace mrpt::system;
	using namespace std;

	const char *sPath = CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX.c_str();
	
	mrpt::system::createDirectory( sPath );
	mrpt::system::createDirectory( mrpt::format("%s/PTGs",sPath) );

	const string sFilTxt_x   = mrpt::format("%s/PTGs/PTG%s_x.txt",sPath,ptg_name.c_str() );
	const string sFilTxt_y   = mrpt::format("%s/PTGs/PTG%s_y.txt",sPath,ptg_name.c_str() );
	const string sFilTxt_phi = mrpt::format("%s/PTGs/PTG%s_phi.txt",sPath,ptg_name.c_str() );
	const string sFilTxt_t   = mrpt::format("%s/PTGs/PTG%s_t.txt",sPath,ptg_name.c_str() );
	const string sFilTxt_d   = mrpt::format("%s/PTGs/PTG%s_d.txt",sPath,ptg_name.c_str() );

	ofstream fx(sFilTxt_x.c_str());  if (!fx.is_open()) return false;
	ofstream fy(sFilTxt_y.c_str());  if (!fy.is_open()) return false;
	ofstream fp(sFilTxt_phi.c_str());if (!fp.is_open()) return false;
	ofstream fd(sFilTxt_d.c_str());  if (!fd.is_open()) return false;

	const size_t nPaths = getAlphaValuesCount();

	// Text version:
	fx << "% PTG data file for 'x'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	fy << "% PTG data file for 'y'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	fp << "% PTG data file for 'phi'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	fd << "% PTG data file for 'd'. Each row is the trajectory for a different 'alpha' parameter value." << endl;

	vector<size_t> path_length(nPaths);
	for (size_t k=0;k<nPaths;k++)
		path_length[k] = getPathStepCount(k);

	size_t maxPoints=0;
	for (size_t k=0;k<nPaths;k++)
		maxPoints = max( maxPoints, path_length[k] );

	for (size_t k=0;k<nPaths;k++)
	{
		for (size_t n=0;n< maxPoints;n++)
		{
			const size_t nn = std::min( n, path_length[k]-1 );
			mrpt::math::TPose2D p;
			this->getPathPose(k,nn, p);
			fx << p.x << " ";
			fy << p.y << " ";
			fp << p.phi << " ";
			fd << this->getPathDist(k,nn) << " ";
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

void CParameterizedTrajectoryGenerator::initialize(const std::string & cacheFilename, const bool verbose)
{
	if (m_is_initialized) return;
	
	const std::string sCache = !cacheFilename.empty() ?
		cacheFilename
		:
		std::string("cache_")+mrpt::system::fileNameStripInvalidChars(getDescription())+std::string(".bin.gz");

	this->internal_initialize(sCache,verbose);
	m_is_initialized = true;
}
void CParameterizedTrajectoryGenerator::deinitialize()
{
	if (!m_is_initialized) return;
	this->internal_deinitialize();
	m_is_initialized = false;
}

void CParameterizedTrajectoryGenerator::internal_TPObsDistancePostprocess(const double ox, const double oy, const double new_tp_obs_dist, double &inout_tp_obs) const
{
	const bool is_obs_inside_robot_shape = isPointInsideRobotShape(ox,oy);
	if (!is_obs_inside_robot_shape)
	{
		mrpt::utils::keep_min(inout_tp_obs, new_tp_obs_dist);
		return;
	}

	// Handle the special case of obstacles *inside* the robot at the begining of the PTG path:
	switch (COLLISION_BEHAVIOR)
	{
	case COLL_BEH_STOP:
		inout_tp_obs = .0;
		break;

	case COLL_BEH_BACK_AWAY:
		{
			if (new_tp_obs_dist < getApproxRobotRadius() ) {
				// This means that we are getting apart of the obstacle: 
				// ignore it to allow the robot to get off the near-collision:
				// Don't change inout_tp_obs.
				return;
			}
			else {
				// This means we are already in collision and trying to get even closer 
				// to the obstacle: totally disprove this action:
				inout_tp_obs = .0;
			}
		}
		break;

	default:
		THROW_EXCEPTION("Obstacle postprocessing enum not implemented!");
	}
}

void CParameterizedTrajectoryGenerator::updateClearance(const double ox, const double oy, ClearanceDiagram & cd) const
{
	// Initialize CD on first call:
	ASSERT_(cd.raw_clearances.size() == m_alphaValuesCount || cd.raw_clearances.empty());
	ASSERT_(m_clearance_num_points>0 && m_clearance_num_points<10000);

	if (cd.raw_clearances.empty())
	{
		const double dd = 1.0 / m_clearance_num_points;
		cd.raw_clearances.resize(m_alphaValuesCount);
		for (unsigned int k = 0; k < m_alphaValuesCount; k++) {
			for (double d = 0.0; d < 1.0; d += dd) {
				cd.raw_clearances[k][d] = 1.0;
			}
		}
	}

	if (m_use_approx_clearance)
		return; // will be actually computed in updateClearancePost();

	// evaluate in derived-class: this function also keeps the minimum automatically.
	for (uint16_t k=0;k<m_alphaValuesCount;k++)
		this->evalClearanceSingleObstacle(ox,oy,k, cd.raw_clearances[k]);
}

void CParameterizedTrajectoryGenerator::updateClearancePost(ClearanceDiagram & cd, const std::vector<double> &TP_obstacles) const
{
	// Use only when in approx mode:
	if (!m_use_approx_clearance)
		return;

	ASSERT_(cd.raw_clearances.size() == m_alphaValuesCount);

	std::vector<double> k2dir(m_alphaValuesCount);
	for (uint16_t k = 0; k < m_alphaValuesCount; k++)
		k2dir[k] = M_PI*(-1 + 2 * (0.5 + k) / m_alphaValuesCount);

	for (uint16_t k = 0; k < m_alphaValuesCount; k++)
	{
		// For each segment (path):
		const double ak = k2dir[k];

		for (auto &e : cd.raw_clearances[k])
		{
			const double d = e.first;
			const double x = d*cos(ak), y = d*sin(ak);

			for (uint16_t i = 0; i < m_alphaValuesCount; i++)
			{
				const double ai = k2dir[i];
				double di = TP_obstacles[i] / refDistance; // std::min(TP_obstacles[k], 0.95*target_dist); ??
				if (di >= 0.99) di = 2.0;

				const double this_clearance_norm = ::hypot(di*cos(ai) - x, di*sin(ai) - y);
				// Update minimum in output structure
				mrpt::utils::keep_min(e.second, this_clearance_norm);
			}
		}
	}
}

void CParameterizedTrajectoryGenerator::evalClearanceSingleObstacle(const double ox, const double oy, const uint16_t k, std::map<double, double> & inout_realdist2clearance) const
{
	bool had_collision = false;

	const size_t numPathSteps = getPathStepCount(k);
	ASSERT_(numPathSteps >  inout_realdist2clearance.size());

	const double numStepsPerIncr = (numPathSteps - 1.0) / (inout_realdist2clearance.size());

	double step_pointer_dbl = 0.0;

	for (auto &e : inout_realdist2clearance)
	{
		step_pointer_dbl += numStepsPerIncr;
		const size_t step = mrpt::utils::round(step_pointer_dbl);

		const double dist_over_path = e.first;
		double & inout_clearance = e.second;

		if (dist_over_path == .0) {
			// Special case: don't eval clearance at init pose, to 
			// 1) avoid biasing the rest of the path for near obstacles, and
			// 2) let the obstacle_behavior to work when in a "collision state":
			const double fake_clearance =  this->getApproxRobotRadius() / refDistance;
			mrpt::utils::keep_min(inout_clearance, fake_clearance);
			continue;
		}

		if (had_collision) {
			// We found a collision in a previous step along this "k" path, so 
			// it does not make sense to evaluate the clearance of a pose which is not reachable:
			inout_clearance = .0;
			continue;
		}

		mrpt::math::TPose2D pose;
		this->getPathPose(k, step, pose);

		// obstacle to robot clearance:
		const double this_clearance = this->evalClearanceToRobotShape(ox - pose.x, oy - pose.y);
		if (this_clearance <= .0) {
			// Collision:
			had_collision = true;
			inout_clearance = .0;
		}
		else {
			const double this_clearance_norm = this_clearance / this->refDistance;

			// Update minimum in output structure
			mrpt::utils::keep_min(inout_clearance, this_clearance_norm);
		}
	}
}




