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
#include <mrpt/nav/tpspace/motion_planning_utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>

// For 3D log files
#include <mrpt/opengl/COpenGLScene.h> 
#include <mrpt/opengl/CGridPlaneXY.h> 
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>

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
			mrpt::format("TPRRT_PTG_%03u.dat.gz",static_cast<unsigned int>(i)),
			params.ptg_verbose
			);
	}

	m_initialized = true;
}

/** The main API entry point: tries to find a planned path from 'goal' to 'target' */
void PlannerRRT_SE2_TPS::solve( 
	const PlannerRRT_SE2_TPS::TPlannerInput &pi, 
	PlannerRRT_SE2_TPS::TPlannerResult & result )
{
	mrpt::utils::CTimeLoggerEntry tle(m_timelogger,"PT_RRT::solve");

	// Sanity checks:
	ASSERTMSG_(m_initialized, "initialize() must be called before!");

	// [Algo `tp_space_rrt`: Line 1]: Init tree adding the initial pose
	if (result.move_tree.getAllNodes().empty())
	{
		result.move_tree.root = 0;
		result.move_tree.insertNode( result.move_tree.root, TNodeSE2_TP( pi.start_pose ) );
	}

	mrpt::utils::CTicTac working_time;
	working_time.Tic();
	size_t rrt_iter_counter=0;

	size_t SAVE_3D_TREE_LOG_DECIMATION_CNT=0;
	static size_t SAVE_LOG_SOLVE_COUNT=0;
	SAVE_LOG_SOLVE_COUNT++;

	// Keep track of the best solution so far:
	// By reusing the contents of "result" we make the algorithm re-callable ("any-time" algorithm) to refine results

	// [Algo `tp_space_rrt`: Line 2]: Iterate
	// ------------------------------------------
	for (;;)
	{
		// Check end conditions:
		const double elap_tim = working_time.Tac();
		if (
			(end_criteria.maxComputationTime>0 && elap_tim>end_criteria.maxComputationTime) // Max comp time
			|| (result.goal_distance<end_criteria.acceptedDistToTarget && elap_tim>=end_criteria.minComputationTime) // Reach closer than this to target
			)
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
			rrt_iter_counter++;

			// [Algo `tp_space_rrt`: Line 5]: Search nearest neig. to x_rand
			// -----------------------------------------------
			const PoseDistanceMetric<TNodeSE2_TP> distance_evaluator(*m_PTGs[idxPTG]);

			const TNodeSE2_TP query_node(x_rand);
			
			m_timelogger.enter("TMoveTree::getNearestNode");
			mrpt::utils::TNodeID x_nearest_id = result.move_tree.getNearestNode(query_node, distance_evaluator );
			m_timelogger.leave("TMoveTree::getNearestNode");

			if (x_nearest_id==INVALID_NODEID)
				continue; // We can't find any close node, at least with this PTG's paths

			const TNodeSE2_TP &     x_nearest_node = result.move_tree.getAllNodes().find(x_nearest_id)->second;

			// [Algo `tp_space_rrt`: Line 6]: Relative target
			// -----------------------------------------------
			const CPose2D x_nearest_pose( x_nearest_node.state );
			const CPose2D x_rand_pose(x_rand);
			const CPose2D x_rand_rel = x_rand_pose - x_nearest_pose;

			// [Algo `tp_space_rrt`: Line 7]: Relative target in TP-Space
			// ------------------------------------------------------------
			const float D_max = std::min(static_cast<float>(params.maxLength), m_PTGs[idxPTG]->refDistance );

			float d_rand; // Coordinates in TP-space
			int   k_rand; // k_rand is the index of target_alpha in PTGs corresponding to a specific d_rand
			//bool tp_point_is_exact =
			m_PTGs[idxPTG]->inverseMap_WS2TP(
				x_rand_rel.x(), x_rand_rel.y(),
				k_rand, d_rand );
			d_rand *= m_PTGs[idxPTG]->refDistance; // distance to target, in "real meters"

			// [Algo `tp_space_rrt`: Line 8]: TP-Obstacles
			// ------------------------------------------------------------
			// Transform obstacles as seen from x_nearest_node -> TP_obstacles
			vector<float> TP_Obstacles;
			{
				CTimeLoggerEntry tle(m_timelogger,"PT_RRT::solve.changeCoordinatesReference");
				const double MAX_DIST = 1.5*params.obsMaxDistance; // Maximum radius for considering obstacles around the current robot pose
				transformPointcloudWithSquareClipping(pi.obstacles_points,m_local_obs,CPose2D(x_nearest_node.state),MAX_DIST);
			}
			{
				CTimeLoggerEntry tle(m_timelogger,"PT_RRT::solve.SpaceTransformer");
				spaceTransformer(m_local_obs, m_PTGs[idxPTG].pointer(), TP_Obstacles );
			}

			// directions k_rand in TP_obstacles[k_rand] = d_free
			// this is the collision free distance to the TP_target
			const float d_free = TP_Obstacles[k_rand];

			// [Algo `tp_space_rrt`: Line 10]: d_new
			// ------------------------------------------------------------
			double d_new = std::min(D_max, d_rand);   //distance of the new candidate state in TP-space

			// [Algo `tp_space_rrt`: Line 11]: Do we have free space?
			// ------------------------------------------------------------
			if ( d_free>=d_new )
			{
				// [Algo `tp_space_rrt`: Line 12]: PTG function
				// ------------------------------------------------------------
				float x,y,phi,t;//, v, w;
				float v,w;

				//given d_rand and k_rand provides x,y,phi of the point in c-space
				m_PTGs[idxPTG]->getCPointWhen_d_Is(d_new, k_rand,  x,y,phi,t,&v,&w);
				phi=mrpt::math::wrapToPi(phi); // wrap to [-pi,pi] -->avoid out of bounds errors

				// [Algo `tp_space_rrt`: Line 13]: pose composition
				// ------------------------------------------------------------
				const mrpt::poses::CPose2D new_state_rel(x, y, phi);
				mrpt::poses::CPose2D new_state = x_nearest_pose+new_state_rel; //compose the new_motion as the last nmotion and the new state
				const TNodeSE2_TP new_state_node(new_state);

				// Check whether there's already a too-close node around:
				// --------------------------------------------------------
				bool accept_this_node = true;
				{
					double new_nearest_dist;

					m_timelogger.enter("TMoveTree::getNearestNode");
					mrpt::utils::TNodeID new_nearest_id = result.move_tree.getNearestNode(new_state_node, distance_evaluator,&new_nearest_dist, &result.acceptable_goal_node_ids );
					m_timelogger.leave("TMoveTree::getNearestNode");

					if (new_nearest_id!=INVALID_NODEID)
						accept_this_node = (new_nearest_dist>=params.minDistanceBetweenNewNodes);
				}

				if (!accept_this_node)
				{
					continue; // Too close node, skip!
				}

				// [Algo `tp_space_rrt`: Line 13]: Accept this motion: create new edge in the tree
				// ------------------------------------------------------------
				// Create "movement" (tree edge) object:
				TMoveEdgeSE2_TP new_edge(x_nearest_id, mrpt::math::TPose2D(new_state));
				
				new_edge.cost     = d_new;
				new_edge.ptg_index= idxPTG;
				new_edge.ptg_K    = k_rand;
				new_edge.ptg_dist = d_new;

				// Insert into the tree:
				const mrpt::utils::TNodeID new_child_id = result.move_tree.getNextFreeNodeID();
				result.move_tree.insertNodeAndEdge(x_nearest_id /*parent id*/, new_child_id, new_state_node, new_edge);

				// Distance to goal:
				const double goal_dist = new_state.distance2DTo(pi.goal_pose.x,pi.goal_pose.y);
				const bool is_acceptable_goal = (goal_dist<end_criteria.acceptedDistToTarget);

				if (is_acceptable_goal)
					result.acceptable_goal_node_ids.insert(new_child_id);

				// Total path length:
				double this_path_cost = std::numeric_limits<double>::max();
				if (is_acceptable_goal)  // Don't waste time computing path length if it doesn't matter anyway
				{
					TMoveTreeSE2_TP::path_t candidate_solution_path;
					result.move_tree.backtrackPath(new_child_id,candidate_solution_path);
					this_path_cost=0;
					for (TMoveTreeSE2_TP::path_t::const_iterator it=candidate_solution_path.begin();it!=candidate_solution_path.end();++it)
						if (it->edge_to_parent)
							this_path_cost+=it->edge_to_parent->cost;
				}

				// Check if this should be the new optimal path:
				bool is_new_best_solution = false; // Just for logging purposes
				if (is_acceptable_goal && this_path_cost<result.path_cost)
				{
#if 0
					cout << "== New best solution: \n";
					cout << "==  new_child_id: " << new_child_id << endl;
					cout << "==  goal_dist: " << goal_dist << endl;
					cout << "==  path_cost: " << this_path_cost << endl;
#endif
					result.goal_distance  = goal_dist;
					result.path_cost = this_path_cost;

					result.best_goal_node_id = new_child_id;
					is_new_best_solution=true;
				}

				//  Graphical logging, if enabled:
				// ------------------------------------------------------
				if (params.save_3d_log_freq>0 && (++SAVE_3D_TREE_LOG_DECIMATION_CNT >= params.save_3d_log_freq || is_new_best_solution))
				{
					CTimeLoggerEntry tle(m_timelogger,"PT_RRT::solve.generate_log_files");
					SAVE_3D_TREE_LOG_DECIMATION_CNT=0; // Reset decimation counter

					// Render & save to file:
					mrpt::opengl::COpenGLScene scene;
					renderMoveTree(scene, pi,result,result.best_goal_node_id, &x_rand_pose,&x_nearest_pose, &m_local_obs,&new_state );
					mrpt::system::createDirectory("./rrt_log_trees");
					scene.saveToFile( mrpt::format("./rrt_log_trees/rrt_log_%03u_%06u.3Dscene",static_cast<unsigned int>(SAVE_LOG_SOLVE_COUNT),static_cast<unsigned int>(rrt_iter_counter) ) );
				}

			} // end if the path is obstacle free
		} // end for idxPTG

	} // end loop until end conditions


	// [Algo `tp_space_rrt`: Line 17]: Tree back trace
	// ------------------------------------------------------------
	result.success = (result.goal_distance<end_criteria.acceptedDistToTarget);
	result.computation_time = working_time.Tac();

}  // end solve()

// Auxiliary function:
void PlannerRRT_SE2_TPS::transformPointcloudWithSquareClipping(
	const mrpt::maps::CPointsMap & in_map,
	mrpt::maps::CPointsMap       & out_map,
	const mrpt::poses::CPose2D   & asSeenFrom,
	const double MAX_DIST_XY
	)
{
	size_t nObs;
	const float *obs_xs, *obs_ys, *obs_zs;
	in_map.getPointsBuffer(nObs, obs_xs,obs_ys, obs_zs);

	out_map.clear();
	out_map.reserve(nObs); // Prealloc mem for speed-up

	const CPose2D invPose = -asSeenFrom;
	// We can safely discard the rest of obstacles, since they cannot be converted into TP-Obstacles anyway!

	for (size_t obs=0;obs<nObs;obs++)
	{
		const double gx = obs_xs[obs], gy = obs_ys[obs];

		if (std::abs(gx-asSeenFrom.x())>MAX_DIST_XY || std::abs(gy-asSeenFrom.y())>MAX_DIST_XY)
			continue;   // ignore this obstacle: anyway, I don't know how to map it to TP-Obs!

		double ox,oy;
		invPose.composePoint(gx,gy,ox,oy);

		out_map.insertPointFast(ox,oy,0);
	}
}

/*---------------------------------------------------------------
SpaceTransformer
---------------------------------------------------------------*/
void PlannerRRT_SE2_TPS::spaceTransformer( 
	const mrpt::maps::CSimplePointsMap &in_obstacles,
	const mrpt::nav::CParameterizedTrajectoryGenerator *in_PTG,
	std::vector<float> &out_TPObstacles )
{
	using namespace mrpt::nav;
	try
	{
		const size_t Ki = in_PTG->getAlfaValuesCount();  //getAlfaValuesCount is 0!

		// check space already reserved to TP-Obstacles:
		if ( size_t(out_TPObstacles.size()) != Ki )
			out_TPObstacles.resize( Ki );

		// Take "k_rand"s and "distances" such that the collision hits the obstacles
		// in the "grid" of the given PT
		// --------------------------------------------------------------------
		size_t nObs;
		const float *obs_xs, *obs_ys, *obs_zs;
		// = in_obstacles.getPointsCount();
		in_obstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

		for (size_t k_rand=0;k_rand<Ki;k_rand++)
		{
			// Initiate at the max distance or up to abs(phi)=pi
			out_TPObstacles[k_rand] = in_PTG->refDistance;
			// If you just turned 180deg, end there:
			float phi = in_PTG->GetCPathPoint_phi(k_rand,in_PTG->getPointsCountInCPath_k(k_rand)-1);

			if (fabs(phi) >= M_PI* 0.95 )
				out_TPObstacles[k_rand]= in_PTG->GetCPathPoint_d(k_rand,in_PTG->getPointsCountInCPath_k(k_rand)-1);
		}

		const double MAX_DIST = 1.5*params.obsMaxDistance; // Maximum radius for considering obstacles around the current robot pose
		for (size_t obs=0;obs<nObs;obs++)
		{
			const float ox = obs_xs[obs];
			const float oy = obs_ys[obs];

			if (std::abs(ox)>MAX_DIST || std::abs(oy)>MAX_DIST)
				continue;   // ignore this obstacle: anyway, I don't know how to map it to TP-Obs!

			const CParameterizedTrajectoryGenerator::TCollisionCell & cell = in_PTG->m_collisionGrid.getTPObstacle(ox,oy);

			// Keep the minimum distance:
			for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();i++)
				if ( i->second < out_TPObstacles[ i->first ] )
					out_TPObstacles[i->first] = i->second;
		}

		// Leave distances in out_TPObstacles un-normalized ([0,1]), so they just represent real distances in meters.
	}
	catch (std::exception &e)
	{
		cerr << "[PT_RRT::SpaceTransformer] Exception:" << endl;
		cerr << e.what() << endl;
	}
	catch (...)
	{
		cerr << "\n[PT_RRT::SpaceTransformer] Unexpected exception!:\n";
		cerr << format("*in_PTG = %p\n", (void*)in_PTG );
		if (in_PTG)
			cerr << format("PTG = %s\n",in_PTG->getDescription().c_str());
		cerr << endl;
	}
}

void PlannerRRT_SE2_TPS::renderMoveTree(
	mrpt::opengl::COpenGLScene &scene,
	const PlannerRRT_SE2_TPS::TPlannerInput  &pi,
	const PlannerRRT_SE2_TPS::TPlannerResult &result,
	const mrpt::utils::TNodeID highlight_path_to_node_id,
	const CPose2D *x_rand_pose,
	const CPose2D *x_nearest_pose,
	const mrpt::maps::CPointsMap * local_obs_from_nearest_pose,
	const CPose2D *new_state	
	)
{
	MRPT_TODO("Draw actual vehicle shape on each node along the optimal solution")
	MRPT_TODO("Autoscale XYZcorners ~ vehicle size (from shape)")

	// "ground"
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create( pi.world_bbox_min.x,pi.world_bbox_max.x, pi.world_bbox_min.y,pi.world_bbox_max.y, 0, 10 );
		obj->setColor_u8( mrpt::utils::TColor(0xFF,0xFF,0xFF) );
		scene.insert( obj );
	}

	// Origin:
	{
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZ(2.0);
		obj->setName("ORIGIN");
		obj->enableShowName();
		obj->setColor_u8( mrpt::utils::TColor(0x00,0x00,0x00) );
		obj->setPose(pi.start_pose);
		scene.insert(obj);
	}

	// Target:
	{
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZ(3.0);
		string m_name =	"GOAL";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setColor_u8( mrpt::utils::TColor(0x00,0x00,0x00) );
		obj->setPose(pi.goal_pose);
		scene.insert(obj);
	}

	// Original randomly-pick pose:
	if (x_rand_pose)
	{
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZ(3.2);
		string m_name =	"X_rand";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setPose(*x_rand_pose);
		scene.insert(obj);
	}

	// Nearest state pose:
	if (x_nearest_pose)
	{
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZ(3.2);
		string m_name =	"X_near";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setPose(*x_nearest_pose);
		scene.insert(obj);
	}

	// Determine the up-to-now best solution, so we can highlight the best path so far:
	TMoveTreeSE2_TP::path_t  best_path;
	if (highlight_path_to_node_id!=INVALID_NODEID)
		result.move_tree.backtrackPath( highlight_path_to_node_id, best_path);

	// make list of nodes in the way of the best path:
	std::set<const TMoveEdgeSE2_TP*> edges_best_path;
	for (TMoveTreeSE2_TP::path_t::const_iterator it=best_path.begin();it!=best_path.end();++it)
		if (it->edge_to_parent) 
			edges_best_path.insert(it->edge_to_parent);

	//cout << result.move_tree.getAsTextDescription() << endl;

	// Existing nodes & edges between them:
	{
		const TMoveTreeSE2_TP::node_map_t & lstNodes = result.move_tree.getAllNodes();

		for (TMoveTreeSE2_TP::node_map_t::const_iterator itNode=lstNodes.begin();itNode!=lstNodes.end();++itNode)
		{
			const TMoveTreeSE2_TP::NODE_TYPE & node = itNode->second;
							
			mrpt::poses::CPose2D parent_state; 
			if (node.parent_id!=INVALID_NODEID) 
			{
				parent_state=  lstNodes.find(node.parent_id)->second.state;
			}
			const mrpt::poses::CPose2D trg_state = node.state;

			const bool is_new_one = (itNode==(lstNodes.end()-1));
			const bool is_best_path = edges_best_path.count(node.edge_to_parent)!=0;

			// Draw children nodes:
			{
				const float corner_scale = is_new_one ? 1.5f : 1.0f;

				mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZSimple(corner_scale);
				obj->setPose(trg_state);
				scene.insert(obj);
			}

			// Draw line parent -> children nodes.
			if (node.parent_id!=INVALID_NODEID) 
			{
				// Draw actual PT path between parent and children nodes:
				ASSERT_(node.edge_to_parent)
				const mrpt::nav::CParameterizedTrajectoryGenerator * ptg = m_PTGs[node.edge_to_parent->ptg_index].pointer();

				// Create the path shape, in relative coords to the parent node:
				mrpt::opengl::CSetOfLinesPtr obj = mrpt::opengl::CSetOfLines::Create();
				obj->setPose(parent_state); // Points are relative to this pose: let OpenGL to deal with the coords. composition

				ptg->renderPathAsSimpleLine(node.edge_to_parent->ptg_K,*obj,0.25f /*decimation*/, node.edge_to_parent->ptg_dist /*max path length*/);

				if (is_new_one)
				{
					// Last edge format:
					obj->setColor_u8( mrpt::utils::TColor(0xff,0xff,0x00));
					obj->setLineWidth(3);
				}
				else
				{
					// Normal format:
					obj->setColor_u8( mrpt::utils::TColor(0x22,0x22,0x22));
				}
				if (is_best_path)
				{
					obj->setColor_u8( mrpt::utils::TColor(0x00,0x00,0x00));//(0xff,0xff,0xff));
					obj->setLineWidth(4);
				}

				scene.insert(obj);
			}
		}
	}

	// The new node:
	if (new_state)
	{
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZ(2.5);
		string m_name =	"X_new";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setPose(*new_state);
		scene.insert(obj);
	}

	// Obstacles:
	{
		mrpt::opengl::CPointCloudPtr obj = mrpt::opengl::CPointCloud::Create();

		obj->loadFromPointsMap(&pi.obstacles_points);
		obj->setPose(mrpt::poses::CPose2D(0.0, 0.0, 0.0)); // Points are relative to the origin

		obj->setPointSize(5);
		obj->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff) );
		scene.insert(obj);
	}

	// The current set of local obstacles:
	// Draw this AFTER the global map so it's visible:
	if (local_obs_from_nearest_pose && x_nearest_pose)
	{
		mrpt::opengl::CPointCloudPtr obj = mrpt::opengl::CPointCloud::Create();

		obj->loadFromPointsMap(local_obs_from_nearest_pose);
		obj->setPose(*x_nearest_pose); // Points are relative to this pose: let OpenGL to deal with the coords. composition
		obj->setPointSize(3);
		obj->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );
		scene.insert(obj);
	}

} // end renderMoveTree()




