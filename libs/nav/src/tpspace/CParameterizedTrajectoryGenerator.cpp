/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/utils/CStream.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt::nav;

std::string CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX = "./reactivenav.logs";

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CParameterizedTrajectoryGenerator, CSerializable, mrpt::nav)

extern mrpt::utils::CStartUpClassesRegister  mrpt_reactivenav_class_reg;
static const int dumm = mrpt_reactivenav_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator() :
	refDistance(.0),
	m_alphaValuesCount(0),
	m_score_priority(1.0),
	m_is_initialized(false)
{ }

void CParameterizedTrajectoryGenerator::loadDefaultParams()
{
	m_alphaValuesCount = 121;
	refDistance = 6.0;
	m_score_priority = 1.0;
}

void CParameterizedTrajectoryGenerator::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(num_paths   , uint64_t, m_alphaValuesCount, cfg,sSection);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT     (refDistance , double,  cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(score_priority , double, m_score_priority, cfg,sSection);
}
void CParameterizedTrajectoryGenerator::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(sSection,"num_paths",m_alphaValuesCount,   WN,WV, "Number of discrete paths (`resolution`) in the PTG");
	cfg.write(sSection,"refDistance",refDistance,   WN,WV, "Maximum distance (meters) for building trajectories (visibility range)");
	cfg.write(sSection,"score_priority",m_score_priority,   WN,WV, "When used in path planning, a multiplying factor (default=1.0) for the scores for this PTG. Assign values <1 to PTGs with low priority.");

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
		in >> refDistance >> m_alphaValuesCount >> m_score_priority;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CParameterizedTrajectoryGenerator::internal_writeToStream(mrpt::utils::CStream &out) const
{
	const uint8_t version = 0;
	out << version;

	out << refDistance << m_alphaValuesCount << m_score_priority;
}

uint16_t CParameterizedTrajectoryGenerator::alpha2index( double alpha ) const 
{
	mrpt::math::wrapToPi(alpha);
	int k = mrpt::utils::round(0.5*(m_alphaValuesCount*(1.0+alpha/M_PI) - 1.0));
	if (k<0) k=0;
	if (k>=m_alphaValuesCount) k=m_alphaValuesCount-1;
	return (uint16_t)k;
}

void CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(
	const uint16_t k, 
	mrpt::opengl::CSetOfLines &gl_obj, 
	const float decimate_distance, 
	const float max_path_distance) const
{
	const size_t nPointsInPath = getPathStepCount(k);

	bool first=true;
	// Decimate trajectories: we don't need centimeter resolution!
	float last_added_dist = 0.0f;
	for (size_t n=0;n<nPointsInPath;n++)
	{
		const float d = this->getPathDist(k, n); // distance thru path "k" until timestep "n"

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

		// Draw the TP only until we reach the target of the "motion" segment:
		if (max_path_distance!=0.0f && d>=max_path_distance) break;
	}
}


void CParameterizedTrajectoryGenerator::initTPObstacles(std::vector<double> &TP_Obstacles) const
{
	TP_Obstacles.resize(m_alphaValuesCount);
	for (size_t k=0;k<m_alphaValuesCount;k++)
	{
		TP_Obstacles[k] = std::min(
			refDistance, 
			this->getPathDist(k, this->getPathStepCount(k)-1) );
	}
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

void CPTG_RobotShape_Polygonal::loadDefaultParams()
{
	m_robotShape.clear();
	m_robotShape.AddVertex(-0.15, 0.15);
	m_robotShape.AddVertex( 0.2, 0.1);
	m_robotShape.AddVertex( 0.2,-0.1);
	m_robotShape.AddVertex(-0.15,-0.15);
}

void CPTG_RobotShape_Polygonal::loadShapeFromConfigFile(const mrpt::utils::CConfigFileBase & cfg,const std::string & sSection)
{
	bool any_pt= false;
	const double BADNUM = std::numeric_limits<double>::max();

	for (unsigned int nPt = 0; ; ++nPt)
	{
		const std::string sPtx = mrpt::format("shape_x%u", nPt);
		const std::string sPty = mrpt::format("shape_y%u", nPt);

		const double ptx = cfg.read_double(sSection, sPtx,BADNUM, false);
		const double pty = cfg.read_double(sSection, sPty,BADNUM, false);
		if (ptx==BADNUM && pty==BADNUM) break;
		ASSERTMSG_( (ptx!=BADNUM && pty!=BADNUM), "Error: mismatch between number of pts in {x,y} defining robot shape");

		if (!any_pt) {
			m_robotShape.clear();
			any_pt=true;
		}

		m_robotShape.AddVertex(ptx,pty);
	}

	if (any_pt)
		internal_processNewRobotShape();
}

void CPTG_RobotShape_Polygonal::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	const int WN = 40, WV = 20;

	for (unsigned int i=0;i<m_robotShape.size();i++)
	{
		const std::string sPtx = mrpt::format("shape_x%u", i);
		const std::string sPty = mrpt::format("shape_y%u", i);
	
		cfg.write(sSection,sPtx,m_robotShape[i].x,   WN,WV, "Robot polygonal shape, `x` [m].");
		cfg.write(sSection,sPty,m_robotShape[i].y,   WN,WV, "Robot polygonal shape, `y` [m].");
	}
}


void CPTG_RobotShape_Circular::loadDefaultParams()
{
	m_robotRadius = 0.2;
}

void CPTG_RobotShape_Circular::loadShapeFromConfigFile(const mrpt::utils::CConfigFileBase & cfg,const std::string & sSection)
{
	const double old_R = m_robotRadius;
	MRPT_LOAD_HERE_CONFIG_VAR(robot_radius, double, m_robotRadius, cfg,sSection);
	
	if (m_robotRadius!=old_R)
		internal_processNewRobotShape();
}
void CPTG_RobotShape_Circular::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	const int WN = 40, WV = 20;

	cfg.write(sSection,"robot_radius",m_robotRadius,   WN,WV, "Robot radius [m].");
}

bool CParameterizedTrajectoryGenerator::isInitialized() const 
{ 
	return m_is_initialized; 
}

void CParameterizedTrajectoryGenerator::initialize(const std::string & cacheFilename, const bool verbose)
{
	if (m_is_initialized) return;
	this->internal_initialize(cacheFilename,verbose);
	m_is_initialized = true;
}
void CParameterizedTrajectoryGenerator::deinitialize()
{
	if (!m_is_initialized) return;
	this->internal_deinitialize();
	m_is_initialized = false;
}



void CPTG_RobotShape_Circular::add_robotShape_to_setOfLines(
	mrpt::opengl::CSetOfLines &gl_shape,
	const mrpt::poses::CPose2D &origin) const 
{
	const double R = m_robotRadius;
	const int N = 21;
	// Transform coordinates:
	mrpt::math::CVectorDouble shap_x(N), shap_y(N),shap_z(N);
	for (int i=0;i<N;i++) {
		origin.composePoint(
			R*cos(i*2*M_PI/(N-1)),R*sin(i*2*M_PI/(N-1)), 0,
			shap_x[i],  shap_y[i],  shap_z[i]);
	}
	// Draw a "radius" to identify the "forward" orientation (phi=0)
	gl_shape.appendLine( origin.x(), origin.y(), .0, shap_x[0],shap_y[0],shap_z[0] );
	for (int i=1;i<=shap_x.size();i++) {
		const int idx = i % shap_x.size();
		gl_shape.appendLineStrip( shap_x[idx],shap_y[idx], shap_z[idx]);
	}
}


void CPTG_RobotShape_Circular::internal_shape_loadFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;
	
	switch (version)
	{
	case 0:
		in >> m_robotRadius;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CPTG_RobotShape_Circular::internal_shape_saveToStream(mrpt::utils::CStream &out) const
{
	uint8_t version = 0;
	out << version;

	out << m_robotRadius;
}


void CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(
	mrpt::opengl::CSetOfLines &gl_shape,
	const mrpt::poses::CPose2D &origin  ) const 
{
	const int N = m_robotShape.size();
	if (N>=2)
	{
		// Transform coordinates:
		mrpt::math::CVectorDouble shap_x(N), shap_y(N),shap_z(N);
		for (int i=0;i<N;i++) {
			origin.composePoint(
				m_robotShape[i].x, m_robotShape[i].y, 0,
				shap_x[i],  shap_y[i],  shap_z[i]);
		}

		gl_shape.appendLine( shap_x[0], shap_y[0], shap_z[0], shap_x[1],shap_y[1],shap_z[1] );
		for (int i=0;i<=shap_x.size();i++) {
			const int idx = i % shap_x.size();
			gl_shape.appendLineStrip( shap_x[idx],shap_y[idx], shap_z[idx]);
		}
	}
}

void CPTG_RobotShape_Polygonal::internal_shape_loadFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;
	
	switch (version)
	{
	case 0:
		in >> m_robotShape;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CPTG_RobotShape_Polygonal::internal_shape_saveToStream(mrpt::utils::CStream &out) const
{
	uint8_t version = 0;
	out << version;

	out << m_robotShape;
}

