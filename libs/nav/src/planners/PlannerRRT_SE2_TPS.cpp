/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>

using namespace mrpt::nav;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

PlannerRRT_SE2_TPS::PlannerRRT_SE2_TPS() :
	m_initialized(false)
{
}

/** Load all params from a config file source */
void PlannerRRT_SE2_TPS::loadConfig(const mrpt::utils::CConfigFileBase &ini, const std::string &sSect)
{
	// Robot shape:
	// ==========================
	{
		// Robot shape is a bit special to load:
		const std::string sShape = ini.read_string(sSect,"robot_shape","",true);
		CMatrixDouble mShape;
		if (!mShape.fromMatlabStringFormat(sShape))
			THROW_EXCEPTION_CUSTOM_MSG1("Error parsing robot_shape matrix: '%s'",sShape.c_str());
		ASSERT_(size(mShape,1)==2)
		ASSERT_(size(mShape,2)>=3)

		params.robot_shape.clear();
		for (size_t i=0;i<size(mShape,2);i++)
			params.robot_shape.push_back(TPoint2D(mShape(0,i),mShape(1,i)));
	}

	// Load PTG tables:
	// ==========================
	m_PTGs.clear();

	const size_t PTG_COUNT = ini.read_int(sSect,"PTG_COUNT",0, true );  //load the number of PTGs
	const float refDistance = ini.read_float(sSect,"MAX_REFERENCE_DISTANCE",5 );  //attempt to read a parameter from the file, otherwise it return the default value
	const float colGridRes = ini.read_float(sSect,"GRID_RESOLUTION",0.02f );

	for ( unsigned int n=0;n<PTG_COUNT;n++ )
	{
		// load ptg_parameters of this PTG:

		TParameters<double> ptg_parameters;
		ptg_parameters["ref_distance"] = refDistance;
		ptg_parameters["resolution"]   = colGridRes;
		ptg_parameters["PTG_type"]	= ini.read_int(sSect,format("PTG%u_Type", n ),1, true );
		ptg_parameters["v_max"]		= ini.read_float(sSect,format("PTG%u_v_max_mps", n ), 5, true);
		ptg_parameters["w_max"]		= DEG2RAD(ini.read_float(sSect,format("PTG%u_w_max_gps", n ), 0, true));
		ptg_parameters["K"]			= ini.read_int(sSect,format("PTG%u_K", n ), 1, false);
		ptg_parameters["cte_a0v"]	= DEG2RAD( ini.read_float(sSect,format("PTG%u_cte_a0v_deg", n ), 0, false) );
		ptg_parameters["cte_a0w"]	= DEG2RAD( ini.read_float(sSect,format("PTG%u_cte_a0w_deg", n ), 0, false) );
		const int nAlfas = ini.read_int(sSect,format("PTG%u_nAlfas", n ),100, true );
#if 0
		cout << "PTRRT_Navigator::initializePTG - message: READ : ref_distance   " << ptg_parameters["ref_distance"] << " [m] " << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : resolution     " << ptg_parameters["resolution"] << " [m] " << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : PTG_type       " << ptg_parameters["PTG_type"] << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : v_max          " << ptg_parameters["v_max"] << " [m/s] " << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : w_max          " << ptg_parameters["w_max"] << " [rad/s] "  << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : K              " << ptg_parameters["K"] << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : cte_a0v        " << ptg_parameters["cte_a0v"] << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : cte_a0w        " << ptg_parameters["cte_a0w"] << endl;
		cout << "PTRRT_Navigator::initializePTG - message: READ : nAlfas         " << nAlfas << "\n\n" << endl;
#endif

		// Generate it:
		m_PTGs.push_back( CParameterizedTrajectoryGeneratorPtr( CParameterizedTrajectoryGenerator::CreatePTG(ptg_parameters) ) );

		const float min_dist = 0.015f;
		const float max_time = 75.0;

		{
			mrpt::utils::CTimeLoggerEntry tle(m_timelogger,"PTG.simulateTrajectories");

			m_PTGs[n]->simulateTrajectories(
				nAlfas,       // alphas,
				max_time,     // max.tim,
				refDistance,  // max.dist,
				10*refDistance/min_dist, // max.n,
				0.5e-3,       // diferencial_t
				15e-3         // min_dist
				);
		}
	}

}

/** Must be called after setting all params (see `loadConfig()`) and before calling `solve()` */
void PlannerRRT_SE2_TPS::initialize()
{
	ASSERT_ABOVEEQ_(params.robot_shape.size(),3);
	ASSERTMSG_(!m_PTGs.empty(),"No PTG was defined! At least one must be especified.");

	// Convert to CPolygon for API requisites:
	mrpt::math::CPolygon poly_robot_shape;
	{
		vector<double> xm,ym;
		params.robot_shape.getPlotData(xm,ym);
		poly_robot_shape.setAllVertices(xm,ym);
	}

	for (size_t i=0;i<m_PTGs.size();i++)
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "build_PTG_collision_grids");

		mrpt::nav::build_PTG_collision_grids(
			m_PTGs[i].pointer(),
			poly_robot_shape,
			mrpt::format("TPRRT_PTG_%03u.dat.gz",i)
			);
	}

	m_initialized = true;
}

/** The main API entry point: tries to find a planned path from 'goal' to 'target' */
void PlannerRRT_SE2_TPS::solve( 
	const PlannerRRT_SE2_TPS::TPlannerInput &pi, 
	PlannerRRT_SE2_TPS::TPlannerResult & result )
{
	mrpt::utils::CTimeLoggerEntry tle(m_timelogger,"PT_RRT::solve->generate_random_state");

	// Sanity checks:
	ASSERTMSG_(m_initialized, "initialize() must be called before!");

	// [Algo `tp_space_rrt`: Line 1]: Init tree adding the initial pose
	if (result.move_tree.getAllNodes().empty())
	{
		result.move_tree.root = 0;
		result.move_tree.insertNode( result.move_tree.root, TNodeSE2( pi.start_pose ) );
	}

	mrpt::utils::CTicTac working_time;
	working_time.Tic();

	// [Algo `tp_space_rrt`: Line 2]: Iterate
	// ------------------------------------------
	for (;;)
	{
		// Check end coditions: 
		if (end_criteria.maxComputationTime>0 && working_time.Tac()>end_criteria.maxComputationTime) 
		{
			break;
		}

		// [Algo `tp_space_rrt`: Line 3]: sample random state (with goal biasing)
		// -----------------------------------------
		node_pose_t x_rand;
		if (mrpt::random::randomGenerator.drawUniform(0.0,1.0) < params.goalBias) {
			x_rand = pi.goal_pose;
		}
		else {
			// Sample uniform:
			for (int i=0;i<node_pose_t::static_size;i++)
				x_rand[i] = mrpt::random::randomGenerator.drawUniform( pi.world_bbox_min[i], pi.world_bbox_max[i]);
		}

		// [Algo `tp_space_rrt`: Line 4]: For each PTG
		// -----------------------------------------
		const size_t nPTGs = m_PTGs.size();
		for (size_t idxPTG=0;idxPTG<nPTGs;++idxPTG)
		{
			// [Algo `tp_space_rrt`: Line 5]: Search nearest neig. to x_rand
			// -----------------------------------------------
			PoseDistanceMetric<TNodeSE2> distance_evaluator;
			MRPT_TODO("Use PTGs to calc distance!");

			const TNodeSE2 query_node(x_rand);
			mrpt::utils::TNodeID x_nearest_id = result.move_tree.getNearestNode(query_node, distance_evaluator );
			const TNodeSE2 &     x_nearest_node = result.move_tree.getAllNodes().find(x_nearest_id)->second;

			// [Algo `tp_space_rrt`: Line 6]: Relative target
			// -----------------------------------------------
			const CPose2D x_rand_rel = CPose2D(x_rand) - CPose2D( x_nearest_node.state );

			// [Algo `tp_space_rrt`: Line 7]: Relative target in TP-Space
			// ------------------------------------------------------------
			const float D_max = m_PTGs[idxPTG]->refDistance;

			float d_rand; // Coordinates in TP-space
			int   k_rand; // k_rand is the index of target_alpha in PTGs corresponding to a specific d_rand
			bool tp_point_is_exact = m_PTGs[idxPTG]->inverseMap_WS2TP(
				x_rand_rel.x(), x_rand_rel.y(),
				k_rand, d_rand );

			d_rand *= D_max; // distance to target, in "real meters"
			
			m_timelogger.enter("PT_RRT::solve->SpaceTransformer");
			//SpaceTransformer ( local_obs_, (*rrt_PTGs)[indexPTG], rrt_TP_Obstacles[indexPTG] );
			m_timelogger.leave("PT_RRT::solve->SpaceTransformer");


		} // end for idxPTG


	} // end forever loop


}  // end solve()

