/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::nav;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace std;

MRPT_TODO("Optimize getNearestNode() with KD-tree!")

PlannerRRT_SE2_TPS::PlannerRRT_SE2_TPS() = default;
/** Load all params from a config file source */
void PlannerRRT_SE2_TPS::loadConfig(
	const mrpt::config::CConfigFileBase& ini, const std::string& sSect)
{
	PlannerTPS_VirtualBase::internal_loadConfig_PTG(ini, sSect);
}

/** Must be called after setting all params (see `loadConfig()`) and before
 * calling `solve()` */
void PlannerRRT_SE2_TPS::initialize()
{
	PlannerTPS_VirtualBase::internal_initialize_PTG();

	m_initialized = true;
}

/** The main API entry point: tries to find a planned path from 'goal' to
 * 'target' */
void PlannerRRT_SE2_TPS::solve(
	const PlannerRRT_SE2_TPS::TPlannerInput& pi,
	PlannerRRT_SE2_TPS::TPlannerResult& result)
{
	mrpt::system::CTimeLoggerEntry tleg(m_timelogger, "PT_RRT::solve");

	// Sanity checks:
	ASSERTMSG_(m_initialized, "initialize() must be called before!");

	// Calc maximum vehicle shape radius:
	double max_veh_radius = 0.;
	for (const auto& ptg : m_PTGs)
		mrpt::keep_max(max_veh_radius, ptg->getMaxRobotRadius());

	// [Algo `tp_space_rrt`: Line 1]: Init tree adding the initial pose
	if (result.move_tree.getAllNodes().empty())
	{
		result.move_tree.root = 0;
		result.move_tree.insertNode(
			result.move_tree.root, TNodeSE2_TP(pi.start_pose));
	}

	mrpt::system::CTicTac working_time;
	working_time.Tic();
	size_t rrt_iter_counter = 0;

	size_t SAVE_3D_TREE_LOG_DECIMATION_CNT = 0;
	static size_t SAVE_LOG_SOLVE_COUNT = 0;
	SAVE_LOG_SOLVE_COUNT++;

	// Keep track of the best solution so far:
	// By reusing the contents of "result" we make the algorithm re-callable
	// ("any-time" algorithm) to refine results

	// [Algo `tp_space_rrt`: Line 2]: Iterate
	// ------------------------------------------
	for (;;)
	{
		// Check end conditions:
		const double elap_tim = working_time.Tac();
		if ((end_criteria.maxComputationTime > 0 &&
			 elap_tim > end_criteria.maxComputationTime)  // Max comp time
			||
			(result.goal_distance < end_criteria.acceptedDistToTarget &&
			 elap_tim >= end_criteria.minComputationTime)  // Reach closer than
			// this to target
		)
		{
			break;
		}

		// [Algo `tp_space_rrt`: Line 3]: sample random state (with goal
		// biasing)
		// -----------------------------------------
		node_pose_t x_rand;
		// bool rand_is_target=false;
		if (mrpt::random::getRandomGenerator().drawUniform(0.0, 1.0) <
			params.goalBias)
		{
			x_rand = pi.goal_pose;
			// rand_is_target=true;
		}
		else
		{
			// Sample uniform:
			for (int i = 0; i < node_pose_t::static_size; i++)
				x_rand[i] = mrpt::random::getRandomGenerator().drawUniform(
					pi.world_bbox_min[i], pi.world_bbox_max[i]);
		}
		const CPose2D x_rand_pose(x_rand);

		// [Algo `tp_space_rrt`: Line 4]: Init empty solution set
		// -----------------------------------------
		using sorted_solution_list_t = std::map<double, TMoveEdgeSE2_TP>;
		sorted_solution_list_t candidate_new_nodes;  // Map: cost -> info. Pick
		// begin() to select the
		// lowest-cose one.

		const PoseDistanceMetric<TNodeSE2>
			distance_evaluator_se2;  // Plain distances in SE(2), not along PTGs
		bool is_new_best_solution = false;  // Just for logging purposes

		//#define DO_LOG_TXTS
		std::string sLogTxt;

		// [Algo `tp_space_rrt`: Line 5]: For each PTG
		// -----------------------------------------
		const size_t nPTGs = m_PTGs.size();
		for (size_t idxPTG = 0; idxPTG < nPTGs; ++idxPTG)
		{
			rrt_iter_counter++;

			// [Algo `tp_space_rrt`: Line 5]: Search nearest neig. to x_rand
			// -----------------------------------------------
			const PoseDistanceMetric<TNodeSE2_TP> distance_evaluator(
				*m_PTGs[idxPTG]);

			const TNodeSE2_TP query_node(x_rand);

			m_timelogger.enter("TMoveTree::getNearestNode");
			mrpt::graphs::TNodeID x_nearest_id =
				result.move_tree.getNearestNode(query_node, distance_evaluator);
			m_timelogger.leave("TMoveTree::getNearestNode");

			if (x_nearest_id == INVALID_NODEID)
			{
				// We can't find any close node, at least with this PTG's paths:
				// skip

				// Before that, save log:
				if (params.save_3d_log_freq > 0 &&
					(++SAVE_3D_TREE_LOG_DECIMATION_CNT >=
					 params.save_3d_log_freq))
				{
					SAVE_3D_TREE_LOG_DECIMATION_CNT =
						0;  // Reset decimation counter
					TRenderPlannedPathOptions render_options;
					render_options.highlight_path_to_node_id =
						result.best_goal_node_id;
					render_options.highlight_last_added_edge = false;
					render_options.x_rand_pose = &x_rand_pose;
					render_options.log_msg = "SKIP: Can't find any close node";
					render_options.log_msg_position = mrpt::math::TPoint3D(
						pi.world_bbox_min.x, pi.world_bbox_min.y, 0);
					render_options.ground_xy_grid_frequency = 1.0;

					mrpt::opengl::COpenGLScene scene;
					renderMoveTree(scene, pi, result, render_options);
					mrpt::system::createDirectory("./rrt_log_trees");
					scene.saveToFile(mrpt::format(
						"./rrt_log_trees/rrt_log_%03u_%06u.3Dscene",
						static_cast<unsigned int>(SAVE_LOG_SOLVE_COUNT),
						static_cast<unsigned int>(rrt_iter_counter)));
				}

				continue;  // Skip
			}

			const TNodeSE2_TP& x_nearest_node =
				result.move_tree.getAllNodes().find(x_nearest_id)->second;

			// [Algo `tp_space_rrt`: Line 6]: Relative target
			// -----------------------------------------------
			const CPose2D x_nearest_pose(x_nearest_node.state);
			const CPose2D x_rand_rel = x_rand_pose - x_nearest_pose;

			// [Algo `tp_space_rrt`: Line 7]: Relative target in TP-Space
			// ------------------------------------------------------------
			const double D_max =
				std::min(params.maxLength, m_PTGs[idxPTG]->getRefDistance());

			double d_rand;  // Coordinates in TP-space
			int k_rand;  // k_rand is the index of target_alpha in PTGs
			// corresponding to a specific d_rand
			// bool tp_point_is_exact =
			m_PTGs[idxPTG]->inverseMap_WS2TP(
				x_rand_rel.x(), x_rand_rel.y(), k_rand, d_rand);
			d_rand *=
				m_PTGs[idxPTG]
					->getRefDistance();  // distance to target, in "real meters"

			float d_free;
			// bool local_obs_ok = false; // Just for 3D log files: indicates
			// whether obstacle points have been recomputed

			// [Algo `tp_space_rrt`: Line 8]: TP-Obstacles
			// ------------------------------------------------------------
			// Transform obstacles as seen from x_nearest_node -> TP_obstacles
			double TP_Obstacles_k_rand = .0;  // vector<double> TP_Obstacles;
			const double MAX_DIST_FOR_OBSTACLES =
				1.5 * m_PTGs[idxPTG]->getRefDistance();  // Maximum Euclidean
			// distance (radius)
			// for considering
			// obstacles around the
			// current robot pose

			ASSERT_ABOVE_(
				m_PTGs[idxPTG]->getRefDistance(),
				1.1 * max_veh_radius);  // Make sure the PTG covers at least a
			// bit more than the vehicle shape!!
			// (should be much, much higher)

			{
				CTimeLoggerEntry tle(
					m_timelogger, "PT_RRT::solve.changeCoordinatesReference");
				transformPointcloudWithSquareClipping(
					pi.obstacles_points, m_local_obs,
					CPose2D(x_nearest_node.state), MAX_DIST_FOR_OBSTACLES);
				// local_obs_ok=true;
			}
			{
				CTimeLoggerEntry tle(
					m_timelogger, "PT_RRT::solve.SpaceTransformer");
				spaceTransformerOneDirectionOnly(
					k_rand, m_local_obs, m_PTGs[idxPTG].get(),
					MAX_DIST_FOR_OBSTACLES, TP_Obstacles_k_rand);
			}

			// directions k_rand in TP_obstacles[k_rand] = d_free
			// this is the collision free distance to the TP_target
			d_free = TP_Obstacles_k_rand;  // TP_Obstacles[k_rand];

			// [Algo `tp_space_rrt`: Line 10]: d_new
			// ------------------------------------------------------------
			double d_new = std::min(
				D_max,
				d_rand);  // distance of the new candidate state in TP-space

			// mrpt::poses::CPose2D *log_new_state_ptr=NULL; // For graphical
			// logs only

#ifdef DO_LOG_TXTS
			sLogTxt += mrpt::format(
				"tp_idx=%u tp_exact=%c\n d_free: %f d_rand=%f d_new=%f\n",
				static_cast<unsigned int>(idxPTG),
				tp_point_is_exact ? 'Y' : 'N', d_free, d_rand, d_new);
			sLogTxt += mrpt::format(
				" nearest:%s\n", x_nearest_pose.asString().c_str());
#endif

			// [Algo `tp_space_rrt`: Line 13]: Do we have free space?
			// ------------------------------------------------------------
			if (d_free >= d_new)
			{
				// [Algo `tp_space_rrt`: Line 14]: PTG function
				// ------------------------------------------------------------
				// given d_rand and k_rand provides x,y,phi of the point in
				// c-space
				uint32_t nStep;
				m_PTGs[idxPTG]->getPathStepForDist(k_rand, d_new, nStep);

				mrpt::math::TPose2D rel_pose;
				m_PTGs[idxPTG]->getPathPose(k_rand, nStep, rel_pose);

				mrpt::math::wrapToPiInPlace(rel_pose.phi);  // wrap to [-pi,pi]
				// -->avoid out of
				// bounds errors

				// [Algo `tp_space_rrt`: Line 15]: pose composition
				// ------------------------------------------------------------
				const mrpt::poses::CPose2D new_state_rel(rel_pose);
				mrpt::poses::CPose2D new_state =
					x_nearest_pose + new_state_rel;  // compose the new_motion
				// as the last nmotion and
				// the new state
				// log_new_state_ptr = &new_state;

				// Check whether there's already a too-close node around:
				// --------------------------------------------------------
				bool accept_this_node = true;

				// Is this a potential solution
				const double goal_dist =
					new_state.distance2DTo(pi.goal_pose.x, pi.goal_pose.y);
				const double goal_ang = std::abs(
					mrpt::math::angDistance(new_state.phi(), pi.goal_pose.phi));
				const bool is_acceptable_goal =
					(goal_dist < end_criteria.acceptedDistToTarget) &&
					(goal_ang < end_criteria.acceptedAngToTarget);

				auto new_nearest_id = INVALID_NODEID;
				if (!is_acceptable_goal)  // Only check for nearby nodes if this
				// is not a solution!
				{
					double new_nearest_dist;
					const TNodeSE2 new_state_node(new_state.asTPose());

					m_timelogger.enter("TMoveTree::getNearestNode");
					new_nearest_id = result.move_tree.getNearestNode(
						new_state_node, distance_evaluator_se2,
						&new_nearest_dist, &result.acceptable_goal_node_ids);
					m_timelogger.leave("TMoveTree::getNearestNode");

					if (new_nearest_id != INVALID_NODEID)
					{
						// Also check angular distance:
						const double new_nearest_ang =
							std::abs(mrpt::math::angDistance(
								new_state.phi(), result.move_tree.getAllNodes()
													 .find(new_nearest_id)
													 ->second.state.phi));
						accept_this_node =
							(new_nearest_dist >=
								 params.minDistanceBetweenNewNodes ||
							 new_nearest_ang >= params.minAngBetweenNewNodes);
					}
				}

				if (!accept_this_node)
				{
#ifdef DO_LOG_TXTS
					if (new_nearest_id != INVALID_NODEID)
					{
						sLogTxt += mrpt::format(
							" -> new node NOT accepted for closeness to: %s\n",
							result.move_tree.getAllNodes()
								.find(new_nearest_id)
								->second.state.asString()
								.c_str());
					}
#endif
					continue;  // Too close node, skip!
				}

				// [Algo `tp_space_rrt`: Line 16]: Add to candidate solution set
				// ------------------------------------------------------------
				// Create "movement" (tree edge) object:
				TMoveEdgeSE2_TP new_edge(x_nearest_id, new_state.asTPose());

				new_edge.cost = d_new;
				new_edge.ptg_index = idxPTG;
				new_edge.ptg_K = k_rand;
				new_edge.ptg_dist = d_new;

				candidate_new_nodes[new_edge.cost] = new_edge;

			}  // end if the path is obstacle free
			else
			{
#ifdef DO_LOG_TXTS
				sLogTxt += mrpt::format(" -> d_free NOT < d_rand\n");
#endif
			}

		}  // end for idxPTG

		// [Algo `tp_space_rrt`: Line 19]: Any solution found?
		// ------------------------------------------------------------
		if (!candidate_new_nodes.empty())
		{
			const TMoveEdgeSE2_TP& best_edge =
				candidate_new_nodes.begin()->second;
			const TNodeSE2_TP new_state_node(best_edge.end_state);

			// Insert into the tree:
			const mrpt::graphs::TNodeID new_child_id =
				result.move_tree.getNextFreeNodeID();
			result.move_tree.insertNodeAndEdge(
				best_edge.parent_id, new_child_id, new_state_node, best_edge);

			// Distance to goal:
			const double goal_dist =
				mrpt::poses::CPose2D(best_edge.end_state)
					.distance2DTo(pi.goal_pose.x, pi.goal_pose.y);
			const double goal_ang = std::abs(mrpt::math::angDistance(
				best_edge.end_state.phi, pi.goal_pose.phi));

			const bool is_acceptable_goal =
				(goal_dist < end_criteria.acceptedDistToTarget) &&
				(goal_ang < end_criteria.acceptedAngToTarget);

			if (is_acceptable_goal)
				result.acceptable_goal_node_ids.insert(new_child_id);

			// Total path length:
			double this_path_cost = std::numeric_limits<double>::max();
			if (is_acceptable_goal)  // Don't waste time computing path length
			// if it doesn't matter anyway
			{
				TMoveTreeSE2_TP::path_t candidate_solution_path;
				result.move_tree.backtrackPath(
					new_child_id, candidate_solution_path);
				this_path_cost = 0;
				for (auto it = candidate_solution_path.begin();
					 it != candidate_solution_path.end(); ++it)
					if (it->edge_to_parent)
						this_path_cost += it->edge_to_parent->cost;
			}

			// Check if this should be the new optimal path:
			if (is_acceptable_goal && this_path_cost < result.path_cost)
			{
				result.goal_distance = goal_dist;
				result.path_cost = this_path_cost;

				result.best_goal_node_id = new_child_id;
				is_new_best_solution = true;
			}
		}  // end if any candidate found

		//  Graphical logging, if enabled:
		// ------------------------------------------------------
		if (params.save_3d_log_freq > 0 &&
			(++SAVE_3D_TREE_LOG_DECIMATION_CNT >= params.save_3d_log_freq ||
			 is_new_best_solution))
		{
			CTimeLoggerEntry tle(
				m_timelogger, "PT_RRT::solve.generate_log_files");
			SAVE_3D_TREE_LOG_DECIMATION_CNT = 0;  // Reset decimation counter

			// Render & save to file:
			TRenderPlannedPathOptions render_options;
			render_options.highlight_path_to_node_id = result.best_goal_node_id;
			render_options.x_rand_pose = &x_rand_pose;
			// render_options.x_nearest_pose = &x_nearest_pose;
			// if (local_obs_ok) render_options.local_obs_from_nearest_pose =
			// &m_local_obs;
			// render_options.new_state = log_new_state_ptr;
			render_options.highlight_last_added_edge = true;
			render_options.ground_xy_grid_frequency = 1.0;

			render_options.log_msg = sLogTxt;
			render_options.log_msg_position = mrpt::math::TPoint3D(
				pi.world_bbox_min.x, pi.world_bbox_min.y, 0);

			mrpt::opengl::COpenGLScene scene;
			renderMoveTree(scene, pi, result, render_options);

			mrpt::system::createDirectory("./rrt_log_trees");
			scene.saveToFile(mrpt::format(
				"./rrt_log_trees/rrt_log_%03u_%06u.3Dscene",
				static_cast<unsigned int>(SAVE_LOG_SOLVE_COUNT),
				static_cast<unsigned int>(rrt_iter_counter)));
		}

	}  // end loop until end conditions

	// [Algo `tp_space_rrt`: Line 17]: Tree back trace
	// ------------------------------------------------------------
	result.success = (result.goal_distance < end_criteria.acceptedDistToTarget);
	result.computation_time = working_time.Tac();

}  // end solve()
