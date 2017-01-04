/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/math/geometry.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>

using namespace mrpt::nav;

/** Constructor: possible values in "params":
 *   - ref_distance: The maximum distance in PTGs
 *   - resolution: The cell size
 *   - v_max, w_max: Maximum robot speeds.
 */
CPTG_DiffDrive_CollisionGridBased::CPTG_DiffDrive_CollisionGridBased() :
	V_MAX(.0), W_MAX(.0),
	turningRadiusReference(.10),
	m_resolution(0.05),
	m_stepTimeDuration(0.01),
	m_collisionGrid(-1,1,-1,1,0.5,this)
{
}

void CPTG_DiffDrive_CollisionGridBased::loadDefaultParams()
{
	CParameterizedTrajectoryGenerator::loadDefaultParams();
	CPTG_RobotShape_Polygonal::loadDefaultParams();

	m_resolution = 0.10;
	V_MAX = 1.0;
	W_MAX = mrpt::utils::DEG2RAD(120);

}

void CPTG_DiffDrive_CollisionGridBased::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	CParameterizedTrajectoryGenerator::loadFromConfigFile(cfg,sSection);
	CPTG_RobotShape_Polygonal::loadShapeFromConfigFile(cfg,sSection);

	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(resolution  ,double, m_resolution, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(v_max_mps  ,double, V_MAX, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(w_max_dps  ,double, W_MAX, cfg,sSection);
	MRPT_LOAD_CONFIG_VAR(turningRadiusReference  ,double, cfg,sSection);
}
void CPTG_DiffDrive_CollisionGridBased::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;

	CParameterizedTrajectoryGenerator::saveToConfigFile(cfg,sSection);

	cfg.write(sSection,"resolution",m_resolution,   WN,WV, "Resolution of the collision-check look-up-table [m].");
	cfg.write(sSection,"v_max_mps",V_MAX,   WN,WV, "Maximum linear velocity for trajectories [m/s].");
	cfg.write(sSection,"w_max_dps", mrpt::utils::RAD2DEG(W_MAX),   WN,WV, "Maximum angular velocity for trajectories [deg/s].");
	cfg.write(sSection,"turningRadiusReference",turningRadiusReference,   WN,WV, "An approximate dimension of the robot (not a critical parameter) [m].");

	CPTG_RobotShape_Polygonal::saveToConfigFile(cfg,sSection);

	MRPT_END
}


mrpt::utils::CStream & mrpt::nav::operator << (mrpt::utils::CStream& o, const mrpt::nav::TCPoint & p)
{
	o << p.x<< p.y<< p.phi<< p.t<< p.dist<< p.v<< p.w;
	return o;
}
mrpt::utils::CStream & mrpt::nav::operator >> (mrpt::utils::CStream& i, mrpt::nav::TCPoint & p)
{
	i >> p.x>> p.y>> p.phi>> p.t>> p.dist>> p.v>> p.w;
	return i;
}



/*---------------------------------------------------------------
					simulateTrajectories
	Solve trajectories and fill cells.
  ---------------------------------------------------------------*/
void CPTG_DiffDrive_CollisionGridBased::simulateTrajectories(
		float			max_time,
		float			max_dist,
		unsigned int	max_n,
		float			diferencial_t,
		float			min_dist,
		float			*out_max_acc_v,
		float			*out_max_acc_w)
{
	using mrpt::utils::square;

	internal_deinitialize(); // Free previous paths

	m_stepTimeDuration = diferencial_t;

	// Reserve the size in the buffers:
	m_trajectory.resize( m_alphaValuesCount );

	const float  radio_max_robot=1.0f; // Arbitrary "robot radius", only to determine the spacing of points under pure rotation

	// Aux buffer:
	TCPointVector	points;

	float          ult_dist, ult_dist1, ult_dist2;

	// For the grid:
	float		   x_min = 1e3f, x_max = -1e3;
	float		   y_min = 1e3f, y_max = -1e3;

	// Para averiguar las maximas ACELERACIONES lineales y angulares:
	float			max_acc_lin, max_acc_ang;

	max_acc_lin = max_acc_ang = 0;

	try
	{
		for (unsigned int k=0;k<m_alphaValuesCount;k++)
		{
			// Simulate / evaluate the trajectory selected by this "alpha":
			// ------------------------------------------------------------
			const float alpha = index2alpha( k );

			points.clear();
			float t = .0f, dist = .0f, girado = .0f;
			float x = .0f, y = .0f, phi = .0f, v = .0f, w = .0f, _x = .0f, _y = .0f, _phi = .0f;

			// Sliding window with latest movement commands (for the optional low-pass filtering):
			float  last_vs[2] = {.0f,.0f}, last_ws[2] = {.0f,.0f};

			// Add the first, initial point:
			points.push_back( TCPoint(	x,y,phi, t,dist, v,w ) );

			// Simulate until...
			while ( t < max_time && dist < max_dist && points.size() < max_n && fabs(girado) < 1.95 * M_PI )
			{
				// Max. aceleraciones:
				if (t>1)
				{
					float acc_lin = fabs( (last_vs[0]-last_vs[1])/diferencial_t);
					float acc_ang = fabs( (last_ws[0]-last_ws[1])/diferencial_t);
					mrpt::utils::keep_max(max_acc_lin, acc_lin);
					mrpt::utils::keep_max(max_acc_ang, acc_ang);
				}

				// Compute new movement command (v,w):
				ptgDiffDriveSteeringFunction( alpha,t, x, y, phi, v,w );

				// History of v/w ----------------------------------
				last_vs[1]=last_vs[0];
				last_ws[1]=last_ws[0];
				last_vs[0] = v;
				last_ws[0] = w;
				// -------------------------------------------

				// Finite difference equation:
				x += cos(phi)* v * diferencial_t;
				y += sin(phi)* v * diferencial_t;
				phi+= w * diferencial_t;

				// Counters:
				girado += w * diferencial_t;

				float v_inTPSpace = sqrt( square(v)+square(w*turningRadiusReference) );

				dist += v_inTPSpace  * diferencial_t;

				t += diferencial_t;

				// Save sample if we moved far enough:
				ult_dist1 = sqrt( square( _x - x )+square( _y - y  ) );
				ult_dist2 = fabs( radio_max_robot* ( _phi - phi ) );
				ult_dist = std::max( ult_dist1, ult_dist2 );

				if (ult_dist > min_dist)
				{
					// Set the (v,w) to the last record:
					points.back().v = v;
					points.back().w = w;

					// And add the new record:
					points.push_back( TCPoint(	x,y,phi,t,dist,v,w) );

					// For the next iter:
					_x = x;
					_y = y;
					_phi = phi;
				}

				// for the grid:
				x_min = std::min(x_min,x); x_max = std::max(x_max,x);
				y_min = std::min(y_min,y); y_max = std::max(y_max,y);
			}

			// Add the final point:
			points.back().v = v;
			points.back().w = w;
			points.push_back( TCPoint(	x,y,phi,t,dist,v,w) );

			// Save data to C-Space path structure:
			m_trajectory[k] = points;

		} // end for "k"

		// Save accelerations
		if (out_max_acc_v) *out_max_acc_v = max_acc_lin;
		if (out_max_acc_w) *out_max_acc_w = max_acc_ang;

		// --------------------------------------------------------
		// Build the speeding-up grid for lambda function:
		// --------------------------------------------------------
		const TCellForLambdaFunction defaultCell;
		m_lambdaFunctionOptimizer.setSize(
			x_min-0.5f,x_max+0.5f,
			y_min-0.5f,y_max+0.5f,  0.25f,
			&defaultCell);

		for (uint16_t k=0;k<m_alphaValuesCount;k++)
		{
			const uint32_t M = static_cast<uint32_t>(m_trajectory[k].size());
			for (uint32_t n=0;n<M;n++)
			{
				TCellForLambdaFunction	*cell = m_lambdaFunctionOptimizer.cellByPos(m_trajectory[k][n].x,m_trajectory[k][n].y);
				ASSERT_(cell)
				// Keep limits:
				mrpt::utils::keep_min(cell->k_min, k );
				mrpt::utils::keep_max(cell->k_max, k );
				mrpt::utils::keep_min(cell->n_min, n );
				mrpt::utils::keep_max(cell->n_max, n );
			}
		}
	}
	catch(...)
	{
		std::cout << format("[CPTG_DiffDrive_CollisionGridBased::simulateTrajectories] Simulation aborted: unexpected exception!\n");
	}

}

/** In this class, `out_action_cmd` contains: [0]: linear velocity (m/s),  [1]: angular velocity (rad/s) */
mrpt::kinematics::CVehicleVelCmdPtr CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(uint16_t k) const
{
	float v,w;
	ptgDiffDriveSteeringFunction( index2alpha(k),0, 0, 0, 0, v, w );

	mrpt::kinematics::CVehicleVelCmd_DiffDriven * cmd = new mrpt::kinematics::CVehicleVelCmd_DiffDriven();
	cmd->lin_vel = v;
	cmd->ang_vel = w;
	return mrpt::kinematics::CVehicleVelCmdPtr(cmd);
}

/*---------------------------------------------------------------
					getTPObstacle
  ---------------------------------------------------------------*/
const CPTG_DiffDrive_CollisionGridBased::TCollisionCell & CPTG_DiffDrive_CollisionGridBased::CColisionGrid::getTPObstacle(
	const float obsX, const float obsY) const
{
	static const TCollisionCell  emptyCell;
	const TCollisionCell *cell = cellByPos(obsX,obsY);
	return cell!=NULL ? *cell : emptyCell;
}

/*---------------------------------------------------------------
	Updates the info into a cell: It updates the cell only
	  if the distance d for the path k is lower than the previous value:
  ---------------------------------------------------------------*/
void CPTG_DiffDrive_CollisionGridBased::CColisionGrid::updateCellInfo(
	const unsigned int icx,
	const unsigned int icy,
	const uint16_t k,
	const float dist )
{
	TCollisionCell *cell = cellByIndex(icx,icy);
	if (!cell) return;

	// For such a small number of elements, brute-force search is not such a bad idea:
	TCollisionCell::iterator itK = cell->end();
	for (TCollisionCell::iterator it=cell->begin();it!=cell->end();++it)
		if (it->first==k)
		{
			itK = it;
			break;
		}

	if (itK==cell->end())
	{	// New entry:
		cell->push_back(std::make_pair(k,dist) );
	}
	else
	{	// Only update that "k" if the distance is shorter now:
		if (dist<itK->second)
			itK->second = dist;
	}
}


/*---------------------------------------------------------------
					Save to file
  ---------------------------------------------------------------*/
bool CPTG_DiffDrive_CollisionGridBased::saveColGridsToFile( const std::string &filename, const mrpt::math::CPolygon & computed_robotShape ) const
{
	try
	{
		mrpt::utils::CFileGZOutputStream   fo(filename);
		if (!fo.fileOpenCorrectly()) return false;

		const uint32_t n = 1; // for backwards compatibility...
		fo << n;
		return m_collisionGrid.saveToFile(&fo, computed_robotShape);
	}
	catch (...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					Load from file
  ---------------------------------------------------------------*/
bool CPTG_DiffDrive_CollisionGridBased::loadColGridsFromFile( const std::string &filename, const mrpt::math::CPolygon & current_robotShape  )
{
	try
	{
		mrpt::utils::CFileGZInputStream   fi(filename);
		if (!fi.fileOpenCorrectly()) return false;

		uint32_t n;
		fi >> n;
		if (n!=1) return false; // Incompatible (old) format, just discard and recompute.

		return m_collisionGrid.loadFromFile(&fi,current_robotShape);
	}
	catch(...)
	{
		return false;
	}
}

const uint32_t COLGRID_FILE_MAGIC     = 0xC0C0C0C3;

/*---------------------------------------------------------------
					Save to file
  ---------------------------------------------------------------*/
bool CPTG_DiffDrive_CollisionGridBased::CColisionGrid::saveToFile( mrpt::utils::CStream *f, const mrpt::math::CPolygon & computed_robotShape ) const
{
	try
	{
		if (!f) return false;

		const uint8_t serialize_version = 2; // v1: As of jun 2012, v2: As of dec-2013

		// Save magic signature && serialization version:
		*f << COLGRID_FILE_MAGIC << serialize_version;

		// Robot shape:
		*f << computed_robotShape;

		// and standard PTG data:
		*f << m_parent->getDescription()
			<< m_parent->getAlphaValuesCount()
			<< static_cast<float>(m_parent->getMax_V())
			<< static_cast<float>(m_parent->getMax_W());

		*f << m_x_min << m_x_max << m_y_min << m_y_max;
		*f << m_resolution;

		//v1 was:  *f << m_map;
		uint32_t N = m_map.size();
		*f << N;
		for (uint32_t i=0;i<N;i++)
		{
			uint32_t M = m_map[i].size();
			*f << M;
			for (uint32_t k=0;k<M;k++)
				*f << m_map[i][k].first << m_map[i][k].second;
		}

		return true;
	}
	catch(...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
						loadFromFile
  ---------------------------------------------------------------*/
bool CPTG_DiffDrive_CollisionGridBased::CColisionGrid::loadFromFile( mrpt::utils::CStream *f, const mrpt::math::CPolygon & current_robotShape  )
{
	try
	{
		if (!f) return false;

		// Return false if the file contents doesn't match what we expected:
		uint32_t file_magic;
		*f >> file_magic;

		// It doesn't seem to be a valid file or was in an old format, just recompute the grid:
		if (COLGRID_FILE_MAGIC!=file_magic)
			return false;

		uint8_t serialized_version;
		*f >> serialized_version;

		switch (serialized_version)
		{
		case 2:
			{
				mrpt::math::CPolygon stored_shape;
				*f >> stored_shape;

				const bool shapes_match =
					( stored_shape.size()==current_robotShape.size() &&
					  std::equal(stored_shape.begin(),stored_shape.end(), current_robotShape.begin() ) );

				if (!shapes_match) return false; // Must recompute if the robot shape changed.
			}
			break;

		case 1:
		default:
			// Unknown version: Maybe we are loading a file from a more recent version of MRPT? Whatever, we can't read it: It's safer just to re-generate the PTG data
			return false;
		};

		// Standard PTG data:
		const std::string expected_desc = m_parent->getDescription();
		std::string desc;
		*f >> desc;
		if (desc!=expected_desc) return false;

		// and standard PTG data:
#define READ_UINT16_CHECK_IT_MATCHES_STORED(_VAR) { uint16_t ff; *f >> ff; if (ff!=_VAR) return false; }
#define READ_FLOAT_CHECK_IT_MATCHES_STORED(_VAR) { float ff; *f >> ff; if (std::abs(ff-_VAR)>1e-4f) return false; }
#define READ_DOUBLE_CHECK_IT_MATCHES_STORED(_VAR) { double ff; *f >> ff; if (std::abs(ff-_VAR)>1e-6) return false; }

		READ_UINT16_CHECK_IT_MATCHES_STORED(m_parent->getAlphaValuesCount())
		READ_FLOAT_CHECK_IT_MATCHES_STORED(m_parent->getMax_V())
		READ_FLOAT_CHECK_IT_MATCHES_STORED(m_parent->getMax_W())

		// Cell dimensions:
		READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_x_min)
		READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_x_max)
		READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_y_min)
		READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_y_max)
		READ_DOUBLE_CHECK_IT_MATCHES_STORED(m_resolution)

		// OK, all parameters seem to be exactly the same than when we precomputed the table: load it.
		//v1 was:  *f >> m_map;
		uint32_t N;
		*f >> N;
		 m_map.resize(N);
		for (uint32_t i=0;i<N;i++)
		{
			uint32_t M;
			*f >> M;
			 m_map[i].resize(M);
			for (uint32_t k=0;k<M;k++)
				*f >> m_map[i][k].first >> m_map[i][k].second;
		}

		return true;
	}
	catch(std::exception &e)
	{
		std::cerr << "[CColisionGrid::loadFromFile] " << e.what();
		return false;
	}
	catch(...)
	{
		return false;
	}
}

bool CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist) const
{
	using mrpt::utils::square;

	ASSERTMSG_(m_alphaValuesCount>0, "Have you called simulateTrajectories() first?")

	// -------------------------------------------------------------------
	// Optimization: (24-JAN-2007 @ Jose Luis Blanco):
	//  Use a "grid" to determine the range of [k,d] values to check!!
	//  If the point (x,y) is not found in the grid, then directly skip
	//  to the next step.
	// -------------------------------------------------------------------
	uint16_t k_min = 0, k_max = m_alphaValuesCount-1;
	uint32_t n_min = 0, n_max = 0;
	bool at_least_one = false;

	// Cell indexes:
	int		cx0 = m_lambdaFunctionOptimizer.x2idx(x);
	int		cy0 = m_lambdaFunctionOptimizer.y2idx(y);

	// (cx,cy)
	for (int cx=cx0-1;cx<=cx0+1;cx++)
	{
		for (int cy=cy0-1;cy<=cy0+1;cy++)
		{
			const TCellForLambdaFunction	*cell = m_lambdaFunctionOptimizer.cellByIndex(cx,cy);
			if (cell && !cell->isEmpty())
			{
				if (!at_least_one)
				{
					k_min = cell->k_min;	k_max = cell->k_max;
					n_min = cell->n_min;	n_max = cell->n_max;
					at_least_one = true;
				}
				else
				{
					mrpt::utils::keep_min(k_min, cell->k_min);
					mrpt::utils::keep_max(k_max, cell->k_max);

					mrpt::utils::keep_min(n_min, cell->n_min);
					mrpt::utils::keep_max(n_max, cell->n_max);
				}
			}
		}
	}

	// Try to find a closest point to the paths:
	// ----------------------------------------------
	int     selected_k = -1;
	float	selected_d= 0;
	float   selected_dist = std::numeric_limits<float>::max();

	if (at_least_one) // Otherwise, don't even lose time checking...
	{
		ASSERT_BELOW_(k_max, m_trajectory.size())
		for (int k=k_min;k<=k_max;k++)
		{
			const size_t n_real = m_trajectory[k].size();
			const uint32_t n_max_this = std::min( static_cast<uint32_t>(n_real ? n_real-1 : 0), n_max);

			for (uint32_t n = n_min;n<=n_max_this; n++)
			{
				const float dist_a_punto= square( m_trajectory[k][n].x - x ) + square( m_trajectory[k][n].y - y );
				if (dist_a_punto<selected_dist)
				{
					selected_dist = dist_a_punto;
					selected_k = k;
					selected_d = m_trajectory[k][n].dist;
				}
			}
		}
	}

	if (selected_k!=-1)
	{
		out_k = selected_k;
		out_d = selected_d / refDistance;
		return (selected_dist <= square(tolerance_dist));
	}

	// If not found, compute an extrapolation:

	// ------------------------------------------------------------------------------------
	// Given a point (x,y), compute the "k_closest" whose extrapolation
	//  is closest to the point, and the associated "d_closest" distance,
	//  which can be normalized by "1/refDistance" to get TP-Space distances.
	// ------------------------------------------------------------------------------------
	selected_dist = std::numeric_limits<float>::max();
	for (uint16_t k=0;k<m_alphaValuesCount;k++)
	{
		const int n = int (m_trajectory[k].size()) -1;
		const float dist_a_punto = square( m_trajectory[k][n].dist ) + square( m_trajectory[k][n].x - x ) + square( m_trajectory[k][n].y - y );

		if (dist_a_punto<selected_dist)
		{
			selected_dist = dist_a_punto;
			selected_k = k;
			selected_d = dist_a_punto;
		}
	}

	selected_d = std::sqrt(selected_d);

	out_k = selected_k;
	out_d = selected_d / refDistance;

	// If the target dist. > refDistance, then it's normal that we had to extrapolate.
	// Otherwise, it may actually mean that the target is not reachable by this set of paths:
	const float target_dist = std::sqrt( x*x+y*y );
	return (target_dist>target_dist);
}

void CPTG_DiffDrive_CollisionGridBased::setRefDistance(const double refDist)
{
	ASSERTMSG_(m_trajectory.empty(), "Changing reference distance not allowed in this class after initialization!");
	this->refDistance = refDist;
}

void CPTG_DiffDrive_CollisionGridBased::internal_processNewRobotShape()
{
	ASSERTMSG_(m_trajectory.empty(), "Changing robot shape not allowed in this class after initialization!");
}

void CPTG_DiffDrive_CollisionGridBased::internal_deinitialize()
{
	m_trajectory.clear(); // Free trajectories
}

void CPTG_DiffDrive_CollisionGridBased::internal_initialize(const std::string & cacheFilename, const bool verbose)
{
	using namespace std;

	MRPT_START

	if (verbose)
		cout << endl << "[CPTG_DiffDrive_CollisionGridBased::initialize] Starting... *** THIS MAY TAKE A WHILE, BUT MUST BE COMPUTED ONLY ONCE!! **" << endl;

	// Sanity checks:
	ASSERTMSG_(!m_robotShape.empty(),"Robot shape was not defined");
	ASSERTMSG_(m_robotShape.size()>=3,"Robot shape must have 3 or more vertices");
	ASSERT_(refDistance>0);
	ASSERT_(V_MAX>0);
	ASSERT_(W_MAX>0);
	ASSERT_(m_resolution>0);

	mrpt::utils::CTicTac tictac;
	tictac.Tic();

	if (verbose) cout << "Initilizing PTG '" << cacheFilename << "'...";

	// Simulate paths:
	const float min_dist = 0.015f;
	simulateTrajectories(
		100,						// max.tim,
		refDistance,			// max.dist,
		10*refDistance/min_dist,	// max.n,
		0.0005f,				// diferencial_t
		min_dist					// min_dist
		);

	// Just for debugging, etc.
	//debugDumpInFiles(n);


	// Check for collisions between the robot shape and the grid cells:
	// ----------------------------------------------------------------------------
	m_collisionGrid.setSize( -refDistance,refDistance,-refDistance,refDistance, m_resolution );

	const size_t Ki = getAlphaValuesCount();
	ASSERTMSG_(Ki>0, "The PTG seems to be not initialized!");

	// Load the cached version, if possible
	if ( loadColGridsFromFile( cacheFilename, m_robotShape ) )
	{
		if (verbose)
			cout << "loaded from file OK" << endl;
	}
	else
	{
		// BUGFIX: In case we start reading the file and in the end detected an error,
		//         we must make sure that there's space enough for the grid:
		m_collisionGrid.setSize( -refDistance,refDistance,-refDistance,refDistance,m_collisionGrid.getResolution());

		const int grid_cx_max = m_collisionGrid.getSizeX()-1;
		const int grid_cy_max = m_collisionGrid.getSizeY()-1;
		const double half_cell = m_collisionGrid.getResolution()*0.5;

		const size_t nVerts = m_robotShape.verticesCount();
		std::vector<mrpt::math::TPoint2D> transf_shape(nVerts); // The robot shape at each location

		// RECOMPUTE THE COLLISION GRIDS:
		// ---------------------------------------
		for (size_t k=0;k<Ki;k++)
		{
			const size_t nPoints = getPathStepCount(k);
			ASSERT_(nPoints>1)

			for (size_t n=0;n<(nPoints-1);n++)
			{
				// Translate and rotate the robot shape at this C-Space pose:
				mrpt::math::TPose2D p;
				getPathPose(k, n, p);

				mrpt::math::TPoint2D bb_min(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
				mrpt::math::TPoint2D bb_max(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

				for (size_t m = 0;m<nVerts;m++)
				{
					transf_shape[m].x = p.x + cos(p.phi)*m_robotShape.GetVertex_x(m)-sin(p.phi)*m_robotShape.GetVertex_y(m);
					transf_shape[m].y = p.y + sin(p.phi)*m_robotShape.GetVertex_x(m)+cos(p.phi)*m_robotShape.GetVertex_y(m);
					mrpt::utils::keep_max( bb_max.x, transf_shape[m].x); mrpt::utils::keep_max( bb_max.y, transf_shape[m].y);
					mrpt::utils::keep_min( bb_min.x, transf_shape[m].x); mrpt::utils::keep_min( bb_min.y, transf_shape[m].y);
				}

				// Robot shape polygon:
				const mrpt::math::TPolygon2D poly(transf_shape);

				// Get the range of cells that may collide with this shape:
				const int ix_min = std::max(0,m_collisionGrid.x2idx(bb_min.x)-1);
				const int iy_min = std::max(0,m_collisionGrid.y2idx(bb_min.y)-1);
				const int ix_max = std::min(m_collisionGrid.x2idx(bb_max.x)+1,grid_cx_max);
				const int iy_max = std::min(m_collisionGrid.y2idx(bb_max.y)+1,grid_cy_max);

				for (int ix=ix_min;ix<ix_max;ix++)
				{
					const double cx = m_collisionGrid.idx2x(ix) - half_cell;

					for (int iy=iy_min;iy<iy_max;iy++)
					{
						const double cy = m_collisionGrid.idx2y(iy) - half_cell;

						if ( poly.contains( mrpt::math::TPoint2D(cx,cy) ) )
						{
							// Colision!! Update cell info:
							const float d = this->getPathDist(k, n);
							m_collisionGrid.updateCellInfo(ix  ,iy  ,  k,d);
							m_collisionGrid.updateCellInfo(ix-1,iy  ,  k,d);
							m_collisionGrid.updateCellInfo(ix  ,iy-1,  k,d);
							m_collisionGrid.updateCellInfo(ix-1,iy-1,  k,d);
						}
					}	// for iy
				}	// for ix

			} // n

			if (verbose)
				cout << k << "/" << Ki << ",";
		} // k

		if (verbose)
			cout << format("Done! [%.03f sec]\n",tictac.Tac() );

		// save it to the cache file for the next run:
		saveColGridsToFile( cacheFilename, m_robotShape );

	}	// "else" recompute all PTG

	MRPT_END
}

size_t CPTG_DiffDrive_CollisionGridBased::getPathStepCount(uint16_t k) const
{
	ASSERT_(k<m_trajectory.size());

	return m_trajectory[k].size();
}

void CPTG_DiffDrive_CollisionGridBased::getPathPose(uint16_t k, uint16_t step, mrpt::math::TPose2D &p) const
{
	ASSERT_(k<m_trajectory.size());
	ASSERT_(step<m_trajectory[k].size());

	p.x = m_trajectory[k][step].x;
	p.y = m_trajectory[k][step].y;
	p.phi = m_trajectory[k][step].phi;
}

double CPTG_DiffDrive_CollisionGridBased::getPathDist(uint16_t k, uint16_t step) const
{
	ASSERT_(k<m_trajectory.size());
	ASSERT_(step<m_trajectory[k].size());

	return m_trajectory[k][step].dist;
}

bool CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(uint16_t k, double dist, uint16_t &out_step) const
{
	ASSERT_(k<m_trajectory.size());
	const size_t numPoints = m_trajectory[k].size();

	ASSERT_(numPoints > 0);

	for (size_t n=0; n < numPoints-1; n++)
	{
		if (m_trajectory[k][n + 1].dist >= dist)
		{
			out_step = n;
			return true;
		}
	}

	out_step = numPoints-1;
	return false;
}

void CPTG_DiffDrive_CollisionGridBased::updateTPObstacle(
	double ox, double oy,
	std::vector<double> &tp_obstacles) const
{
	ASSERTMSG_(!m_trajectory.empty(), "PTG has not been initialized!");
	const TCollisionCell & cell = m_collisionGrid.getTPObstacle(ox, oy);
	// Keep the minimum distance:
	for (TCollisionCell::const_iterator i = cell.begin(); i != cell.end(); ++i) {
		const double dist = i->second;
		internal_TPObsDistancePostprocess(ox,oy,dist, tp_obstacles[i->first]);
	}
}

void CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(double ox, double oy, uint16_t k, double &tp_obstacle_k) const
{
	ASSERTMSG_(!m_trajectory.empty(), "PTG has not been initialized!");
	const TCollisionCell & cell = m_collisionGrid.getTPObstacle(ox, oy);
	// Keep the minimum distance:
	for (TCollisionCell::const_iterator i = cell.begin(); i != cell.end(); ++i)
		if (i->first == k) {
			const double dist = i->second;
			internal_TPObsDistancePostprocess(ox,oy,dist, tp_obstacle_k);
		}
}

void CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(mrpt::utils::CStream &in)
{
	CParameterizedTrajectoryGenerator::internal_readFromStream(in);
	CPTG_RobotShape_Polygonal::internal_shape_loadFromStream(in);

	uint8_t version;
	in >> version;
	switch (version)
	{
	case 0:
		internal_deinitialize();
		in >>V_MAX >> W_MAX
			>> turningRadiusReference
			>> m_robotShape
			>> m_resolution
			>> m_trajectory;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(mrpt::utils::CStream &out) const
{
	CParameterizedTrajectoryGenerator::internal_writeToStream(out);
	CPTG_RobotShape_Polygonal::internal_shape_saveToStream(out);

	const uint8_t version = 0;
	out << version;

	out << V_MAX << W_MAX
		<< turningRadiusReference
		<< m_robotShape
		<< m_resolution
		<< m_trajectory;
}

mrpt::kinematics::CVehicleVelCmdPtr CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand() const
{
	return mrpt::kinematics::CVehicleVelCmdPtr( new mrpt::kinematics::CVehicleVelCmd_DiffDriven() );
}

double CPTG_DiffDrive_CollisionGridBased::getPathStepDuration() const
{
	return m_stepTimeDuration;
}

