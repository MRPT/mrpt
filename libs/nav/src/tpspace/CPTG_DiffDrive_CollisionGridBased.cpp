/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/utils/stl_serialization.h>

using namespace mrpt::nav;

/** Constructor: possible values in "params":
 *   - ref_distance: The maximum distance in PTGs
 *   - resolution: The cell size
 *   - v_max, w_max: Maximum robot speeds.
 */
CPTG_DiffDrive_CollisionGridBased::CPTG_DiffDrive_CollisionGridBased(const mrpt::utils::TParameters<double> &params) :
	CParameterizedTrajectoryGenerator(params),
	m_collisionGrid(-1,1,-1,1,0.5,this),
	V_MAX(.0), W_MAX(.0),
	turningRadiusReference(.0)
{
	this->V_MAX = params["v_max"];
	this->W_MAX = params["w_max"];

	turningRadiusReference = 0.10f;

	MRPT_TODO("Move to initialize()?")
	initializeCollisionsGrid( refDistance, params["resolution"] );
}

void CPTG_DiffDrive_CollisionGridBased::initializeCollisionsGrid(float refDistance,float resolution)
{
	m_collisionGrid.setSize( -refDistance,refDistance,-refDistance,refDistance,resolution );
}

void CPTG_DiffDrive_CollisionGridBased::freeMemory()
{
	m_trajectory.clear(); // Free trajectories
	m_alphaValuesCount = 0; // This means the PTG is not initialized
}

/** Saves the simulated trajectories and other parameters to a target stream */
void CPTG_DiffDrive_CollisionGridBased::saveTrajectories( mrpt::utils::CStream &out ) const
{
	const uint8_t serial_version = 1;
	out << serial_version;
	out << this->getDescription();
	out << m_alphaValuesCount << V_MAX << W_MAX << turningRadiusReference << refDistance;
	out << m_trajectory;
}

/** Loads the simulated trajectories and other parameters from a target stream */
std::string CPTG_DiffDrive_CollisionGridBased::loadTrajectories( mrpt::utils::CStream &in )
{
	freeMemory(); // Free previous paths
	uint8_t serial_version;
	in >> serial_version;

	std::string desc;
	in >> desc;

	switch (serial_version)
	{
	case 1:
		in >> m_alphaValuesCount >>  V_MAX >> W_MAX >> turningRadiusReference >> refDistance;
		in >> m_trajectory;
		break;

	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(serial_version);
	};

	return desc;
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
		uint16_t	    alphaValuesCount,
		float			max_time,
		float			max_dist,
		unsigned int	max_n,
		float			diferencial_t,
		float			min_dist,
		float			*out_max_acc_v,
		float			*out_max_acc_w)
{
	using mrpt::utils::square;

	freeMemory(); // Free previous paths

	// The number of discreet values for ALPHA:
	this->m_alphaValuesCount = alphaValuesCount;

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

	maxV_inTPSpace = 0;
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

				if (t==0)
					mrpt::utils::keep_max(maxV_inTPSpace, (float)( std::sqrt( square(v) + square(w*turningRadiusReference) ) ) );

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
void CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand( uint16_t k, std::vector<double> &out_action_cmd ) const
{
	float v,w;
	ptgDiffDriveSteeringFunction( index2alpha(k),0, 0, 0, 0, v, w );

	out_action_cmd.resize(2);
	out_action_cmd[0] = v;
	out_action_cmd[1] = w;
}

/*---------------------------------------------------------------
					getCPointWhen_d_Is
  ---------------------------------------------------------------*/
void CPTG_DiffDrive_CollisionGridBased::getCPointWhen_d_Is (
	float d, uint16_t k,
	float &x, float &y, float &phi, float &t,
	float *v, float *w)
{
	if (k>=m_alphaValuesCount) {
		x=y=phi=0;
		return;  // Just in case...
	}

	unsigned int n=0;
	const unsigned int numPoints = m_trajectory[k].size();
	for ( ; (n+1) < numPoints ; n++)
	{
		if (m_trajectory[k][n+1].dist>=d)
			break;
	}

	x=m_trajectory[k][n].x;
	y=m_trajectory[k][n].y;
	phi=m_trajectory[k][n].phi;
	t=m_trajectory[k][n].t;
	if (v) *v =m_trajectory[k][n].v;
	if (w) *w =m_trajectory[k][n].w;
}

bool CPTG_DiffDrive_CollisionGridBased::debugDumpInFiles( const std::string &ptg_name ) const 
{
	using namespace mrpt::system;
	using namespace std;

	const char *sPath = CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX.c_str();
	
	mrpt::system::createDirectory( sPath );
	mrpt::system::createDirectory( mrpt::format("%s/PTGs",sPath) );

	const std::string sFilBin = mrpt::format("%s/PTGs/PTG%s.dat",sPath,ptg_name.c_str() );

	const std::string sFilTxt_x   = mrpt::format("%s/PTGs/PTG%s_x.txt",sPath,ptg_name.c_str() );
	const std::string sFilTxt_y   = mrpt::format("%s/PTGs/PTG%s_y.txt",sPath,ptg_name.c_str() );
	const std::string sFilTxt_phi = mrpt::format("%s/PTGs/PTG%s_phi.txt",sPath,ptg_name.c_str() );
	const std::string sFilTxt_t   = mrpt::format("%s/PTGs/PTG%s_t.txt",sPath,ptg_name.c_str() );
	const std::string sFilTxt_d   = mrpt::format("%s/PTGs/PTG%s_d.txt",sPath,ptg_name.c_str() );

	std::ofstream fx(sFilTxt_x.c_str());  if (!fx.is_open()) return false;
	std::ofstream fy(sFilTxt_y.c_str());  if (!fy.is_open()) return false;
	std::ofstream fp(sFilTxt_phi.c_str());if (!fp.is_open()) return false;
	std::ofstream ft(sFilTxt_t.c_str());  if (!ft.is_open()) return false;
	std::ofstream fd(sFilTxt_d.c_str());  if (!fd.is_open()) return false;

	FILE* fbin = os::fopen(sFilBin.c_str(),"wb");
	if (!fbin) return false;

	const size_t nPaths = getAlfaValuesCount();

	// Text version:
	fx << "% PTG data file for 'x'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	fy << "% PTG data file for 'y'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	fp << "% PTG data file for 'phi'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	ft << "% PTG data file for 't'. Each row is the trajectory for a different 'alpha' parameter value." << endl;
	fd << "% PTG data file for 'd'. Each row is the trajectory for a different 'alpha' parameter value." << endl;

	size_t maxPoints=0;
	for (size_t k=0;k<nPaths;k++)
		maxPoints = std::max( maxPoints, getPointsCountInCPath_k(k) );

	for (size_t k=0;k<nPaths;k++)
	{
		for (size_t n=0;n< maxPoints;n++)
		{
				const size_t nn = std::min( n, getPointsCountInCPath_k(k)-1 );
				fx << GetCPathPoint_x(k,nn) << " ";
				fy << GetCPathPoint_y(k,nn) << " ";
				fp << GetCPathPoint_phi(k,nn) << " ";
				ft << GetCPathPoint_t(k,nn) << " ";
				fd << GetCPathPoint_d(k,nn) << " ";
		}
		fx << endl;
		fy << endl;
		fp << endl;
		ft << endl;
		fd << endl;
	}

	// Binary dump:
	for (size_t k=0;k<nPaths;k++)
	{
		const size_t nPoints = getPointsCountInCPath_k(k);
		if (!fwrite( &nPoints ,sizeof(int),1 , fbin ))
			return false;

		float fls[5];
		for (size_t n=0;n<nPoints;n++)
		{
			fls[0] = GetCPathPoint_x(k,n);
			fls[1] = GetCPathPoint_y(k,n);
			fls[2] = GetCPathPoint_phi(k,n);
			fls[3] = GetCPathPoint_t(k,n);
			fls[4] = GetCPathPoint_d(k,n);

			if (!fwrite(&fls[0],sizeof(float),5,fbin)) return false;
		}
	}

	os::fclose(fbin);

	return true;
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
			<< m_parent->getAlfaValuesCount()
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

		READ_UINT16_CHECK_IT_MATCHES_STORED(m_parent->getAlfaValuesCount())
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
	return false;
}


void CPTG_DiffDrive_CollisionGridBased::renderPathAsSimpleLine(
	const uint16_t k, 
	mrpt::opengl::CSetOfLines &gl_obj, 
	const float decimate_distance, 
	const float max_path_distance) const
{
	const size_t nPointsInPath = getPointsCountInCPath_k(k);

	// Decimate trajectories: we don't need centimeter resolution!
	float last_added_dist = 0.0f;
	for (size_t n=0;n<nPointsInPath;n++)
	{
		const float d = GetCPathPoint_d(k, n); // distance thru path "k" until timestep "n"

		if (d<last_added_dist+decimate_distance && n!=0)
			continue; // skip: decimation

		last_added_dist = d;

		const float x = GetCPathPoint_x(k, n);
		const float y = GetCPathPoint_y(k, n);

		if (gl_obj.empty())
		     gl_obj.appendLine(0,0,0, x,y,0);
		else gl_obj.appendLineStrip(x,y,0);

		// Draw the TP only until we reach the target of the "motion" segment:
		if (max_path_distance!=0.0f && d>=max_path_distance) break;
	}
}

