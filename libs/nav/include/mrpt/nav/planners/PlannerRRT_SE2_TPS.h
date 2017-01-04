/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/TMoveTree.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/utils/CTimeLogger.h>
#include <numeric>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
	/** \addtogroup nav_planners Path planning
	  * \ingroup mrpt_nav_grp
	  * @{ */
	  
		/** TP Space-based RRT path planning for SE(2) (planar) robots.
		* 
		*  This planner algorithm is described in the paper:
		*   - M. Bellone, J.L. Blanco, A. Gimenez, "TP-Space RRT: Kinematic path planning of non-holonomic any-shape vehicles", International Journal of Advanced Robotic Systems, 2015.
		*
		*  Typical usage:
		* \code
		* mrpt::nav::PlannerRRT_SE2_TPS  planner;
		* 
		* // Set or load planner parameters:
		* //planner.loadConfig( mrpt::utils::CConfigFile("config_file.cfg") );
		* //planner.params.... // See TAlgorithmParams
		* 
		* // Set RRT end criteria (when to stop searching for a solution)
		* //planner.end_criteria.... // See TEndCriteria
		* 
		* planner.initialize();  // Initialize after setting the algorithm parameters
		*  
		* // Set up planning problem:
		* PlannerRRT_SE2_TPS::TPlannerResult planner_result;
		* PlannerRRT_SE2_TPS::TPlannerInput planner_input;
		* // Start & goal:
		* planner_input.start_pose = mrpt::math::TPose2D(XXX,XXX,XXX);
		* planner_input.goal_pose  = mrpt::math::TPose2D(XXX,XXX,XXX);
		* // Set obtacles: (...)
		* // planner_input.obstacles_points ...
		* // Set workspace bounding box for picking random poses in the RRT algorithm:
		* planner_input.world_bbox_min = mrpt::math::TPoint2D(XX,YY);
		* planner_input.world_bbox_max = mrpt::math::TPoint2D(XX,YY);
		* // Do path planning:
		* planner.solve( planner_input, planner_result);
		* // Analyze contents of planner_result...
		* \endcode
		*
		*  - Changes history:
		*    - 06/MAR/2014: Creation (MB)
		*    - 06/JAN/2015: Refactoring (JLBC)
		*
		*  \todo Factorize into more generic path planner classes!  //template <class  POSE, class MOTIONS>...
		*/
		class NAV_IMPEXP PlannerRRT_SE2_TPS
		{
		public:
			typedef  mrpt::math::TPose2D  node_pose_t; //!< The type of poses at nodes

			struct NAV_IMPEXP TEndCriteria
			{
				double acceptedDistToTarget;  //!< Maximum distance from a pose to target to accept it as a valid solution (meters). (Both acceptedDistToTarget & acceptedAngToTarget must be satisfied)
				double acceptedAngToTarget;   //!< Maximum angle from a pose to target to accept it as a valid solution (rad).  (Both acceptedDistToTarget & acceptedAngToTarget must be satisfied)

				double maxComputationTime;    //!< In seconds. 0 means no limit until a solution is found.
				double minComputationTime;    //!< In seconds. 0 means the first valid path will be returned. Otherwise, the algorithm will try to refine and find a better one.

				TEndCriteria() : 
					acceptedDistToTarget ( 0.1 ),
					acceptedAngToTarget  ( mrpt::utils::DEG2RAD(180) ),
					maxComputationTime   ( 0.0 ),
					minComputationTime   ( 0.0 )
				{
				}
			};
			TEndCriteria end_criteria;

			struct NAV_IMPEXP TAlgorithmParams
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
				double                 robot_shape_circular_radius;

				std::string ptg_cache_files_directory; //!< (Default: ".")

				double goalBias;  //!< Probabily of picking the goal as random target (in [0,1], default=0.05)
				double maxLength; //!< (Very sensitive parameter!) Max length of each edge path (in meters, default=1.0)
				double minDistanceBetweenNewNodes; //!< Minimum distance [meters] to nearest node to accept creating a new one (default=0.10). (Any of minDistanceBetweenNewNodes and minAngBetweenNewNodes must be satisfied)
				double minAngBetweenNewNodes; //!< Minimum angle [rad] to nearest node to accept creating a new one (default=15 deg) (Any of minDistanceBetweenNewNodes and minAngBetweenNewNodes must be satisfied)
				bool   ptg_verbose; //!< Display PTG construction info (default=true)

				size_t save_3d_log_freq; //!< Frequency (in iters) of saving tree state to debug log files viewable in SceneViewer3D (default=0, disabled)

				TAlgorithmParams();
			};
			TAlgorithmParams params; //!< Parameters specific to this path solver algorithm

			struct NAV_IMPEXP TPlannerInput
			{
				mrpt::math::TPose2D  start_pose;
				mrpt::math::TPose2D  goal_pose;

				mrpt::math::TPose2D  world_bbox_min,world_bbox_max; //!< Bounding box of the world, used to draw uniform random pose samples
				
				mrpt::maps::CSimplePointsMap obstacles_points; //!< World obstacles, as a point cloud

				TPlannerInput() : 
					start_pose(0,0,0),
					goal_pose(0,0,0),
					world_bbox_min(-10.,-10.0,-M_PI),
					world_bbox_max( 10., 10.0, M_PI)
				{
				}
			};

			struct NAV_IMPEXP TPlannerResult
			{
				bool success;               //!< Whether the target was reached or not
				double computation_time;    //!< Time spent (in secs)
				double goal_distance;       //!< Distance from best found path to goal
				double path_cost;           //!< Total cost of the best found path (cost ~~ Euclidean distance)
				mrpt::utils::TNodeID best_goal_node_id; //!< The ID of the best target node in the tree
				std::set<mrpt::utils::TNodeID>  acceptable_goal_node_ids; //!< The set of target nodes within an acceptable distance to target (including `best_goal_node_id` and others)
				TMoveTreeSE2_TP move_tree;  //!< The generated motion tree that explores free space starting at "start"

				TPlannerResult() :
					success(false),
					computation_time(0),
					goal_distance( std::numeric_limits<double>::max() ),
					path_cost( std::numeric_limits<double>::max() ),
					best_goal_node_id(INVALID_NODEID)
				{
				}
			};

			/** Constructor */
			PlannerRRT_SE2_TPS();

			/** Load all params from a config file source */
			void loadConfig(const mrpt::utils::CConfigFileBase &cfgSource, const std::string &sSectionName = std::string("PTG_CONFIG"));

			/** Must be called after setting all params (see `loadConfig()`) and before calling `solve()` */
			void initialize();

			/** The main API entry point: tries to find a planned path from 'goal' to 'target' */
			void solve( const TPlannerInput &pi, TPlannerResult & result );

			/** Options for renderMoveTree()  */
			struct NAV_IMPEXP TRenderPlannedPathOptions
			{
				mrpt::utils::TNodeID highlight_path_to_node_id; //!< Highlight the path from root towards this node (usually, the target)
				size_t draw_shape_decimation; //!< (Default=1) Draw one out of N vehicle shapes along the highlighted path

				const mrpt::poses::CPose2D *x_rand_pose;
				const mrpt::poses::CPose2D *x_nearest_pose;
				const mrpt::maps::CPointsMap * local_obs_from_nearest_pose;
				const mrpt::poses::CPose2D *new_state;

				double xyzcorners_scale; //!< A scale factor to all XYZ corners (default=0, means auto determien from vehicle shape)
				bool   highlight_last_added_edge; //!< (Default=false)
				double ground_xy_grid_frequency;  //!< (Default=10 meters) Set to 0 to disable

				mrpt::utils::TColor color_vehicle;			//!< Robot color 
				mrpt::utils::TColor color_obstacles;		//!< obstacles color 
				mrpt::utils::TColor color_local_obstacles;  //!< local obstacles color 
				mrpt::utils::TColor color_start;            //!< START indication color 
				mrpt::utils::TColor color_goal;             //!< END indication color 
				mrpt::utils::TColor color_ground_xy_grid;
				mrpt::utils::TColor color_normal_edge;
				mrpt::utils::TColor color_last_edge;
				mrpt::utils::TColor color_optimal_edge;
				float width_last_edge;
				float width_normal_edge;
				float width_optimal_edge;
				int point_size_obstacles;
				int point_size_local_obstacles;

				double vehicle_shape_z; //!< (Default=0.01) Height (Z coordinate) for the vehicle shapes. Helps making it in the "first plane"
				double vehicle_line_width;  //!< Robot line width for visualization - default 2.0
				bool   draw_obstacles;  //!< (Default=true)

				std::string           log_msg;
				mrpt::math::TPoint3D  log_msg_position;
				double                log_msg_scale;

				TRenderPlannedPathOptions() :
					highlight_path_to_node_id( INVALID_NODEID ),
					draw_shape_decimation(1),
					x_rand_pose( NULL ),
					x_nearest_pose( NULL ),
					local_obs_from_nearest_pose( NULL ),
					new_state( NULL ),
					xyzcorners_scale(0),
					highlight_last_added_edge(false),
					ground_xy_grid_frequency(10.0),
					color_vehicle(0xFF,0x00,0x00,0xFF),
					color_obstacles(0x00,0x00,0xFF,0x40),
					color_local_obstacles(0x00,0x00,0xFF),
					color_start(0x00, 0x00, 0x00, 0x00),
					color_goal(0x00, 0x00, 0x00, 0x00),
					color_ground_xy_grid(0xFF,0xFF,0xFF),
					color_normal_edge(0x22,0x22,0x22,0x40),
					color_last_edge(0xff,0xff,0x00),
					color_optimal_edge(0x00,0x00,0x00),
					width_last_edge(3.f),
					width_normal_edge(1.f),
					width_optimal_edge(4.f),
					point_size_obstacles(5),
					point_size_local_obstacles(5),
					vehicle_shape_z(0.01),
					vehicle_line_width(2.0),
					draw_obstacles(true),
					log_msg_position(0,0,0),
					log_msg_scale(0.2)

				{
				}

				~TRenderPlannedPathOptions() {}
			};

			void renderMoveTree(
				mrpt::opengl::COpenGLScene &scene,
				const TPlannerInput  &pi,
				const TPlannerResult &result,
				const TRenderPlannedPathOptions &options
				);

			void setRenderTreeVisualization();

			mrpt::utils::CTimeLogger & getProfiler() { return m_timelogger; }
			const mrpt::nav::TListPTGPtr & getPTGs() const { return m_PTGs;}

		protected:
			mrpt::utils::CTimeLogger m_timelogger;
			bool  m_initialized; 
			mrpt::nav::TListPTGPtr m_PTGs;
			mrpt::maps::CSimplePointsMap m_local_obs; // Temporary map. Defined as a member to save realloc time between calls

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


		}; // end class PlannerRRT_SE2_TPS
		
	  /** @} */
	}
}
