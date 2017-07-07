/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/link_pragmas.h>
#include <string>
#include <cstdlib> // size_t

namespace mrpt
{
	namespace nav
	{
		/** \addtogroup nav_planners Path planning
		* \ingroup mrpt_nav_grp
		* @{ */

		template <typename node_pose_t, typename world_limits_t>
		struct TPlannerInputTempl
		{
			node_pose_t   start_pose, goal_pose;
			/** Bounding box of the world, used to draw uniform random pose samples */
			world_limits_t  world_bbox_min, world_bbox_max; 
			/** World obstacles, as a point cloud */
			mrpt::maps::CSimplePointsMap obstacles_points; 
		};

		template <typename tree_t>
		struct TPlannerResultTempl
		{
			/** Whether the target was reached or not */
			bool success;               
			/** Time spent (in secs) */
			double computation_time;    
			/** Distance from best found path to goal */
			double goal_distance;       
			/** Total cost of the best found path (cost ~~ Euclidean distance) */
			double path_cost;           
			/** The ID of the best target node in the tree */
			mrpt::utils::TNodeID best_goal_node_id; 
			/** The set of target nodes within an acceptable distance to target (including `best_goal_node_id` and others) */
			std::set<mrpt::utils::TNodeID>  acceptable_goal_node_ids; 
			/** The generated motion tree that explores free space starting at "start" */
			tree_t move_tree;  

			TPlannerResultTempl() :
				success(false),
				computation_time(0),
				goal_distance(std::numeric_limits<double>::max()),
				path_cost(std::numeric_limits<double>::max()),
				best_goal_node_id(INVALID_NODEID)
			{
			}
		};

		struct NAV_IMPEXP RRTEndCriteria
		{
			/** Maximum distance from a pose to target to accept it as a valid solution (meters). (Both acceptedDistToTarget & acceptedAngToTarget must be satisfied) */
			double acceptedDistToTarget;  
			/** Maximum angle from a pose to target to accept it as a valid solution (rad).  (Both acceptedDistToTarget & acceptedAngToTarget must be satisfied) */
			double acceptedAngToTarget;   

			/** In seconds. 0 means no limit until a solution is found. */
			double maxComputationTime;    
			/** In seconds. 0 means the first valid path will be returned. Otherwise, the algorithm will try to refine and find a better one. */
			double minComputationTime;    

			RRTEndCriteria() :
				acceptedDistToTarget(0.1),
				acceptedAngToTarget(mrpt::utils::DEG2RAD(180)),
				maxComputationTime(0.0),
				minComputationTime(0.0)
			{
			}
		};

		struct NAV_IMPEXP RRTAlgorithmParams
		{
			/** The robot shape used when computing collisions; it's loaded from the
			*  config file/text as a single 2xN matrix in MATLAB format, first row are Xs, second are Ys, e.g:
			*  \code
			*   robot_shape = [-0.2 0.2 0.2 -0.2; -0.1 -0.1 0.1 0.1]
			*  \endcode
			* \note PTGs will use only one of either `robot_shape` or `robot_shape_circular_radius`
			*/
			mrpt::math::TPolygon2D robot_shape;
			/** The radius of a circular-shape-model of the robot shape.
			* \note PTGs will use only one of either `robot_shape` or `robot_shape_circular_radius`
			*/
			double robot_shape_circular_radius;

			/** (Default: ".") */
			std::string ptg_cache_files_directory; 

			/** Probabily of picking the goal as random target (in [0,1], default=0.05) */
			double goalBias;  
			/** (Very sensitive parameter!) Max length of each edge path (in meters, default=1.0) */
			double maxLength; 
			/** Minimum distance [meters] to nearest node to accept creating a new one (default=0.10). (Any of minDistanceBetweenNewNodes and minAngBetweenNewNodes must be satisfied) */
			double minDistanceBetweenNewNodes; 
			/** Minimum angle [rad] to nearest node to accept creating a new one (default=15 deg) (Any of minDistanceBetweenNewNodes and minAngBetweenNewNodes must be satisfied) */
			double minAngBetweenNewNodes; 
			/** Display PTG construction info (default=true) */
			bool   ptg_verbose; 

			/** Frequency (in iters) of saving tree state to debug log files viewable in SceneViewer3D (default=0, disabled) */
			size_t save_3d_log_freq; 

			RRTAlgorithmParams();
		};

		/** Virtual base class for TP-Space-based path planners */
		class NAV_IMPEXP PlannerTPS_VirtualBase
		{
		public:
			RRTEndCriteria end_criteria;
			/** Parameters specific to this path solver algorithm */
			RRTAlgorithmParams params; 

			/** ctor */
			PlannerTPS_VirtualBase(); 

			mrpt::utils::CTimeLogger & getProfiler() { return m_timelogger; }
			const mrpt::nav::TListPTGPtr & getPTGs() const { return m_PTGs; }

			/** Options for renderMoveTree()  */
			struct NAV_IMPEXP TRenderPlannedPathOptions
			{
				/** Highlight the path from root towards this node (usually, the target) */
				mrpt::utils::TNodeID highlight_path_to_node_id; 
				/** (Default=1) Draw one out of N vehicle shapes along the highlighted path */
				size_t draw_shape_decimation; 

				const mrpt::poses::CPose2D *x_rand_pose;
				const mrpt::poses::CPose2D *x_nearest_pose;
				const mrpt::maps::CPointsMap * local_obs_from_nearest_pose;
				const mrpt::poses::CPose2D *new_state;

				/** A scale factor to all XYZ corners (default=0, means auto determien from vehicle shape) */
				double xyzcorners_scale; 
				/** (Default=false) */
				bool   highlight_last_added_edge; 
				/** (Default=10 meters) Set to 0 to disable */
				double ground_xy_grid_frequency;  

				/** Robot color  */
				mrpt::utils::TColor color_vehicle;			
				/** obstacles color  */
				mrpt::utils::TColor color_obstacles;		
				/** local obstacles color  */
				mrpt::utils::TColor color_local_obstacles;  
				/** START indication color  */
				mrpt::utils::TColor color_start;            
				/** END indication color  */
				mrpt::utils::TColor color_goal;             
				mrpt::utils::TColor color_ground_xy_grid;
				mrpt::utils::TColor color_normal_edge;
				mrpt::utils::TColor color_last_edge;
				mrpt::utils::TColor color_optimal_edge;
				float width_last_edge;
				float width_normal_edge;
				float width_optimal_edge;
				int point_size_obstacles;
				int point_size_local_obstacles;

				/** (Default=0.01) Height (Z coordinate) for the vehicle shapes. Helps making it in the "first plane" */
				double vehicle_shape_z; 
				/** Robot line width for visualization - default 2.0 */
				double vehicle_line_width;  
				/** (Default=true) */
				bool   draw_obstacles;  

				std::string           log_msg;
				mrpt::math::TPoint3D  log_msg_position;
				double                log_msg_scale;

				TRenderPlannedPathOptions() :
					highlight_path_to_node_id(INVALID_NODEID),
					draw_shape_decimation(1),
					x_rand_pose(NULL),
					x_nearest_pose(NULL),
					local_obs_from_nearest_pose(NULL),
					new_state(NULL),
					xyzcorners_scale(0),
					highlight_last_added_edge(false),
					ground_xy_grid_frequency(10.0),
					color_vehicle(0xFF, 0x00, 0x00, 0xFF),
					color_obstacles(0x00, 0x00, 0xFF, 0x40),
					color_local_obstacles(0x00, 0x00, 0xFF),
					color_start(0x00, 0x00, 0x00, 0x00),
					color_goal(0x00, 0x00, 0x00, 0x00),
					color_ground_xy_grid(0xFF, 0xFF, 0xFF),
					color_normal_edge(0x22, 0x22, 0x22, 0x40),
					color_last_edge(0xff, 0xff, 0x00),
					color_optimal_edge(0x00, 0x00, 0x00),
					width_last_edge(3.f),
					width_normal_edge(1.f),
					width_optimal_edge(4.f),
					point_size_obstacles(5),
					point_size_local_obstacles(5),
					vehicle_shape_z(0.01),
					vehicle_line_width(2.0),
					draw_obstacles(true),
					log_msg_position(0, 0, 0),
					log_msg_scale(0.2)
				{
				}

				virtual ~TRenderPlannedPathOptions() {}
			};

			template <typename node_pose_t, typename world_limits_t, typename tree_t>
			void renderMoveTree(
				mrpt::opengl::COpenGLScene &scene,
				const TPlannerInputTempl<node_pose_t, world_limits_t>  &pi,
				const TPlannerResultTempl<tree_t> &result,
				const TRenderPlannedPathOptions &options
			);

		protected:
			mrpt::utils::CTimeLogger m_timelogger;
			bool  m_initialized_PTG;
			mrpt::nav::TListPTGPtr m_PTGs;
			mrpt::maps::CSimplePointsMap m_local_obs; // Temporary map. Defined as a member to save realloc time between calls

			/** Load all PTG params from a config file source */
			void internal_loadConfig_PTG(const mrpt::utils::CConfigFileBase &cfgSource, const std::string &sSectionName = std::string("PTG_CONFIG"));

			/** Must be called after setting all params (see `internal_loadConfig_PTG()`) and before calling `solve()` */
			void internal_initialize_PTG();

			static void transformPointcloudWithSquareClipping(
				const mrpt::maps::CPointsMap & in_map,
				mrpt::maps::CPointsMap       & out_map,
				const mrpt::poses::CPose2D   & asSeenFrom,
				const double MAX_DIST_XY
			);

			void spaceTransformer(
				const mrpt::maps::CSimplePointsMap &in_obstacles,
				const mrpt::nav::CParameterizedTrajectoryGenerator *in_PTG,
				const double MAX_DIST,
				std::vector<double> &out_TPObstacles
			);

			void spaceTransformerOneDirectionOnly(
				const int tp_space_k_direction,
				const mrpt::maps::CSimplePointsMap &in_obstacles,
				const mrpt::nav::CParameterizedTrajectoryGenerator *in_PTG,
				const double MAX_DIST,
				double &out_TPObstacle_k
			);

		}; // end class PlannerTPS_VirtualBase
		/** @} */
	}
}

// Impl of renderMoveTree<>
#include "impl_renderMoveTree.h"

