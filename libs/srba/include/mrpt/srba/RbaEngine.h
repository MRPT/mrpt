/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

/** \file RbaEngine.h
  * \brief This file exposes the public API and data types of libmrpt-srba (it requires also including srba/models/{*.h} to have a complete SLAM/RBA system)
  */

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include "srba_types.h"
#include "srba_options.h"
#include "landmark_jacob_families.h"

#define VERBOSE_LEVEL(_LEVEL) if (m_verbose_level>=_LEVEL) std::cout

namespace mrpt
{
/** An heavily template-based implementation of Relative Bundle Adjustment (RBA) - See \ref mrpt_srba_grp */
namespace srba
{
	using namespace std;

	/** The set of default settings for RbaEngine */
	struct RBA_OPTIONS_DEFAULT
	{
		typedef options::sensor_pose_on_robot_none      sensor_pose_on_robot_t;  //!< The sensor pose coincides with the robot pose
		typedef options::observation_noise_identity     obs_noise_matrix_t;      //!< The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
		typedef options::solver_LM_schur_dense_cholesky solver_t;                //!< Solver algorithm (Default: Lev-Marq, with Schur, with dense Cholesky)
	};

	/** The main class for the mrpt-srba: it defines a Relative Bundle-Adjustment (RBA) problem with (optionally, partially known) landmarks,
	  *   the methods to update it with new observations and to optimize the relative poses with least squares optimizers.
	  *
	  *   The unknowns to be solved are:
	  *		- Relative poses among keyframes.
	  *		- Relative positions of landmarks wrt to their base frame (or no landmarks for graph-SLAM-like problems)
	  *
	  *   The set of known data used to run the optimization comprises:
	  *		- Sequence of all observations.
	  *		- Optional sensor parameters (e.g. camera calibration)
	  *		- Optionally, the relative positions of a subset of landmarks wrt to their base frame (these are the "fixed" or "known" landmarks).
	  *
	  *  See http://www.mrpt.org/srba and the <a href="srba-guide.pdf" >library guide</a> for a list of possible template arguments, code examples, etc.
	  *
	  * \tparam KF2KF_POSE_TYPE The parameterization of keyframe-to-keyframe relative poses (edges, problem unknowns).
	  * \tparam LM_TYPE The parameterization of relative positions of landmarks relative poses (edges).
	  * \tparam OBS_TYPE The type of observations.
	  * \tparam RBA_OPTIONS A struct with nested typedefs which can be used to tune and customize the behavior of this class.
	  */
	template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE, class RBA_OPTIONS = RBA_OPTIONS_DEFAULT>
	class RbaEngine
	{
	public:
		/** @name Templatized typedef's
		    @{ */
		typedef RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS> rba_engine_t;

		typedef KF2KF_POSE_TYPE kf2kf_pose_type;
		typedef LM_TYPE         lm_type;
		typedef OBS_TYPE        obs_type;
		typedef RBA_OPTIONS     rba_options_type;

		static const size_t REL_POSE_DIMS = KF2KF_POSE_TYPE::REL_POSE_DIMS;
		static const size_t LM_DIMS       = LM_TYPE::LM_DIMS;
		static const size_t OBS_DIMS      = OBS_TYPE::OBS_DIMS;

		typedef typename KF2KF_POSE_TYPE::se_traits_t  se_traits_t; //!< The SE(2) or SE(3) traits struct (for Lie algebra log/exp maps, etc.)

		typedef rba_joint_parameterization_traits_t<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>  traits_t;
		typedef jacobian_traits<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>                      jacobian_traits_t;
		typedef hessian_traits<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>                       hessian_traits_t;
		typedef kf2kf_pose_traits<KF2KF_POSE_TYPE>                                     kf2kf_pose_traits_t;
		typedef landmark_traits<LM_TYPE>                                               landmark_traits_t;
		typedef observation_traits<OBS_TYPE>                                           observation_traits_t;

		typedef sensor_model<LM_TYPE,OBS_TYPE>   sensor_model_t; //!< The sensor model for the specified combination of LM parameterization + observation type.

		typedef typename KF2KF_POSE_TYPE::pose_t  pose_t; //!< The type of relative poses (e.g. mrpt::poses::CPose3D)
		typedef TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS> rba_problem_state_t;

		typedef typename rba_problem_state_t::k2f_edge_t k2f_edge_t;
		typedef typename rba_problem_state_t::k2k_edge_t k2k_edge_t;
		typedef typename rba_problem_state_t::k2k_edges_deque_t  k2k_edges_deque_t;  //!< A list (deque) of KF-to-KF edges (unknown relative poses).

		typedef typename kf2kf_pose_traits_t::pose_flag_t pose_flag_t;
		typedef typename kf2kf_pose_traits_t::frameid2pose_map_t  frameid2pose_map_t;
		typedef typename kf2kf_pose_traits_t::TRelativePosesForEachTarget TRelativePosesForEachTarget;
		typedef typename landmark_traits_t::TRelativeLandmarkPosMap TRelativeLandmarkPosMap;  //!< An index of feature IDs and their relative locations
		typedef typename landmark_traits_t::TRelativeLandmarkPos    TRelativeLandmarkPos; //!< One landmark position (relative to its base KF)
		typedef typename traits_t::keyframe_info   keyframe_info;
		typedef typename traits_t::new_kf_observation_t   new_kf_observation_t;
		typedef typename traits_t::new_kf_observations_t  new_kf_observations_t;

		typedef typename kf2kf_pose_traits_t::array_pose_t         array_pose_t;
		typedef typename landmark_traits_t::array_landmark_t       array_landmark_t;
		typedef typename observation_traits_t::array_obs_t         array_obs_t;
		typedef typename observation_traits_t::residual_t          residual_t;
		typedef typename observation_traits_t::vector_residuals_t  vector_residuals_t;
		/** @} */

		/** Default constructor */
		RbaEngine();

		/** All the information returned by the local area optimizer \sa define_new_keyframe() */
		struct TOptimizeExtraOutputInfo
		{
			TOptimizeExtraOutputInfo()
			{
				clear();
			}
			
			size_t  num_observations;     //!< Number of individual feature observations taken into account in the optimization
			size_t  num_jacobians;        //!< Number of Jacobian blocks which had been to be evaluated for each relinearization step.
			size_t  num_kf2kf_edges_optimized; //!< Number of solved unknowns of type "kf-to-kf edge".
			size_t  num_kf2lm_edges_optimized; //!< Number of solved unknowns of type "kf-to-landmark".
			size_t  num_total_scalar_optimized;  //!< The total number of dimensions (scalar values) in all the optimized unknowns.
			size_t  num_span_tree_numeric_updates; //!< Number of poses updated in the spanning tree numeric-update stage.
			double  total_sqr_error_init, total_sqr_error_final; //!< Initial and final total squared error for all the observations
			double  HAp_condition_number; //!< To be computed only if enabled in parameters.compute_condition_number
			
			std::vector<size_t> optimized_k2k_edge_indices; //!< The 0-based indices of all kf-to-kf edges which were considered in the optimization
			std::vector<size_t> optimized_landmark_indices; //!< The 0-based indices of all landmarks whose relative positions were considered as unknowns in the optimization

			/** Other solver-specific output information */
			typename RBA_OPTIONS::solver_t::extra_results_t   extra_results; 

			void clear()
			{
				num_observations = 0;
				num_jacobians = 0;
				num_kf2kf_edges_optimized = 0;
				num_kf2lm_edges_optimized = 0;
				num_total_scalar_optimized = 0;
				num_span_tree_numeric_updates=0;
				total_sqr_error_init=0.;
				total_sqr_error_final=0.;
				HAp_condition_number=0.;
				optimized_k2k_edge_indices.clear();
				optimized_landmark_indices.clear();
				extra_results.clear();
			}
		};

		/** Information returned by RbaEngine::define_new_keyframe() */
		struct TNewKeyFrameInfo
		{
			TKeyFrameID                kf_id;         //!< The ID of the newly created KF.
			std::vector<TNewEdgeInfo>  created_edge_ids;  //!< The newly created edges (minimum: 1 edge)
			TOptimizeExtraOutputInfo   optimize_results;  //!< Results from the least-squares optimization

			void clear()
			{
				kf_id = static_cast<TKeyFrameID>(-1);
				created_edge_ids.clear();
				optimize_results.clear();
			}
		};

		/** @name Main API methods
		    @{ */

		/** The most common entry point for SRBA: append a new keyframe (KF) to the map, automatically creates the required edges
		  *  and optimize the local area around this new KF.
		  *
		  * \param[in]  obs All the landmark observations gathered from this new KF (with data association already solved).
		  * \param[out] out_new_kf_info Returned information about the newly created KF.
		  * \param[in]  run_local_optimization If set to true (default), the local map around the new KF will be optimized (i.e. optimize_local_area() will be called automatically).
		  */
		void define_new_keyframe(
			const typename traits_t::new_kf_observations_t  & obs,
			TNewKeyFrameInfo   & out_new_kf_info,
			const bool           run_local_optimization = true
			);

		/** Parameters for optimize_local_area() */
		struct TOptimizeLocalAreaParams
		{
			bool        optimize_k2k_edges;
			bool        optimize_landmarks;
			TKeyFrameID max_visitable_kf_id; //!< While exploring around the root KF, stop the BFS when KF_ID>=this number (default:infinity)
			size_t      dont_optimize_landmarks_seen_less_than_n_times;  //!< Set to 1 to try to optimize all landmarks even if they're just observed once, which may makes sense depending on the sensor type (default: 2)

			TOptimizeLocalAreaParams() :
				optimize_k2k_edges(true),
				optimize_landmarks(true),
				max_visitable_kf_id( static_cast<TKeyFrameID>(-1) ),
				dont_optimize_landmarks_seen_less_than_n_times( 2 )
			{}
		};

		/** Runs least-squares optimization for all the unknowns within a given topological distance to a given KeyFrame.
		  *
		  * \param[in] observation_indices_to_optimize Indices wrt \a rba_state.all_observations. An empty vector means use ALL the observations involving the selected unknowns.
		  * \note Extra options are available in \a parameters
		  *  \sa define_new_keyframe, add_observation, optimize_edges
		  */
		void optimize_local_area(
			const TKeyFrameID  root_id,
			const unsigned int win_size,
			TOptimizeExtraOutputInfo & out_info,
			const TOptimizeLocalAreaParams &params = TOptimizeLocalAreaParams(),
			const std::vector<size_t> & observation_indices_to_optimize = std::vector<size_t>()
			);


		struct TOpenGLRepresentationOptions : public LM_TYPE::render_mode_t::TOpenGLRepresentationOptionsExtra
		{
			TOpenGLRepresentationOptions() :
				span_tree_max_depth(static_cast<size_t>(-1)),
				draw_unknown_feats(true),
				draw_unknown_feats_ellipses(true),
				draw_unknown_feats_ellipses_quantiles(1),
				show_unknown_feats_ids(true)
			{
			}

			size_t span_tree_max_depth; //!< Maximum spanning tree depth for reconstructing relative poses (default=-1 : infinity)
			bool   draw_unknown_feats;  //!< Draw features with non-fixed rel.pos as well?
			bool   draw_unknown_feats_ellipses;
			double draw_unknown_feats_ellipses_quantiles;
			bool   show_unknown_feats_ids;
		};

		/** Build an opengl representation of the current state of this RBA problem
		  * One of different representations can be generated: those opengl objects passed as NULL smart pointers will not be generated.
		  * \param[out] out_scene If not a NULL smart pointer, at return will hold a 3D view of the current KF, neighbor KFs and landmarks.
		  * \param[out] out_root_tree If not a NULL smart pointer, at return will hold a schematic 2D view of the current KF and its spanning tree.
		  */
		void build_opengl_representation(
			const srba::TKeyFrameID root_keyframe,
			const TOpenGLRepresentationOptions &options,
			mrpt::opengl::CSetOfObjectsPtr out_scene,
			mrpt::opengl::CSetOfObjectsPtr out_root_tree = mrpt::opengl::CSetOfObjectsPtr()
			) const;

		/** Exports all the keyframes (and optionally all landmarks) as a directed graph in DOT (graphviz) format.
		  * \return false on any error writing to target file */
		bool save_graph_as_dot(
			const std::string &targetFileName,
			const bool all_landmarks = false
			) const;

		/** Evaluates the quality of the overall map/landmark estimations, by computing the sum of the squared
		  *  error contributions for all observations. For this, this method may have to compute *very long* shortest paths
		  *  between distant keyframes if no loop-closure edges exist in order to evaluate the best approximation of relative
		  *  coordinates between observing KFs and features' reference KFs.
		  *
		  * The worst-case time consumed by this method is O(M*log(N) + N^2 + N E), N=# of KFs, E=# of edges, M=# of observations.
		  */
		double eval_overall_squared_error() const;

		/** @} */  // End of main API methods
		

		/** @name Extra API methods (for debugging, etc.)
		    @{ */

		/** Reset the entire problem to an empty state (automatically called at construction) */
		void clear();

		/** Users normally won't need to call this directly. Use define_new_keyframe() instead.
		  * This method appends an empty new keyframe to the data structures
		  * \return The ID of the new KF.
		  * \note Runs in O(1)
		  */
		TKeyFrameID alloc_keyframe();

		/** Users normally won't need to call this directly. Use define_new_keyframe() instead.
		  * This method calls:
		  *  1) alloc_kf2kf_edge() for creating the data structures
		  *  2) spanning_tree.update_symbolic_new_node() for updating the spanning trees.
		  *
		  * \note obs is passed just for fixing missing spanning trees when using the policy of pure linear graph.
		  */
		size_t create_kf2kf_edge(
			const TKeyFrameID        new_kf_id,
			const TPairKeyFrameID  & new_edge,
			const typename traits_t::new_kf_observations_t   & obs,
			const pose_t &init_inv_pose_val = pose_t() );


		/** Creates a numeric spanning tree between a given root keyframe and the entire graph, returning it into a user-supplied data structure
		  *  Note that this method does NOT use the depth-limited spanning trees which are built incrementally with the graph. So, it takes an extra cost to
		  *  call this method. For the other trees, see get_rba_state()
		  * \param[in] root_id The root keyframe
		  * \param[out] span_tree The output with all found relative poses. Its previous contents are cleared.
		  * \param[in] max_depth Defaults to std::numeric_limits<size_t>::max() for infinity depth, can be set to a maximum desired depth from the root.
		  * \param[in] aux_ws Auxiliary working space: Set to an uninitialized vector<bool> (it'll automatically initialized) if you want to call this method simultaneously from several threads. Otherwise, setting to NULL will automatically use one working space, reusable between succesive calls.
		  * \sa find_path_bfs
		  */
		void create_complete_spanning_tree(
			const TKeyFrameID   root_id,
			frameid2pose_map_t & span_tree,
			const size_t        max_depth = std::numeric_limits<size_t>::max(),
			std::vector<bool>  * aux_ws = NULL
			) const;

		/** An unconstrained breadth-first search (BFS) for the shortest path between two keyframes.
		  *  Note that this method does NOT use the depth-limited spanning trees which are built incrementally with the graph. So, it takes the extra cost of
		  *  really running a BFS algorithm. For the other precomputed trees, see get_rba_state()
		  *  Edge direction is ignored during the search, i.e. as if we had an undirected graph of Keyframes.
		  *  If both source and target KF coincide, an empty path is returned.
		  * \return true if a path was found.
		  * \note Worst-case computational complexity is that of a BFS over the entire graph: O(V+E), V=number of nodes, E=number of edges.
		  * \sa create_complete_spanning_tree
		  */
		bool find_path_bfs(
			const TKeyFrameID           src_kf,
			const TKeyFrameID           trg_kf,
			std::vector<TKeyFrameID>    & found_path) const
		{
			return rba_state.find_path_bfs(src_kf,trg_kf,&found_path);
		}

		/** Visits all k2k & k2f edges following a BFS starting at a given starting node and up to a given maximum depth.
		  * Only k2k edges are considered for BFS paths.
		  */
		template <
			class KF_VISITOR,
			class FEAT_VISITOR,
			class K2K_EDGE_VISITOR,
			class K2F_EDGE_VISITOR
			>
		void bfs_visitor(
			const TKeyFrameID  root_id,
			const topo_dist_t  max_distance,
			const bool rely_on_prebuilt_spanning_trees,
			KF_VISITOR       & kf_visitor,
			FEAT_VISITOR     & feat_visitor,
			K2K_EDGE_VISITOR & k2k_edge_visitor,
			K2F_EDGE_VISITOR & k2f_edge_visitor ) const;

		/** @} */  // End of Extra API methods


		/** @name Public data fields
			@{ */

		/** Different parameters for the SRBA methods */
		struct TSRBAParameters : public mrpt::utils::CLoadableOptions
		{
			TSRBAParameters();

			/** See docs of mrpt::utils::CLoadableOptions */
			virtual void  loadFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section);
			/** See docs of mrpt::utils::CLoadableOptions */
			virtual void  saveToConfigFile(mrpt::utils::CConfigFileBase &out,const std::string & section) const;


			/** Parameters for determine_kf2kf_edges_to_create(); custom user-defined policies can be also defined (see tutorials) */
			TEdgeCreationPolicy  edge_creation_policy;

			/** Maximum depth for maintained spanning trees. */
			topo_dist_t          max_tree_depth;

			/** The maximum topological distance of keyframes to be optimized around the most recent keyframe. */
			topo_dist_t          max_optimize_depth;

			size_t              submap_size;
			size_t              min_obs_to_loop_closure; //!< Default:6, reduce to 1 for relative graph-slam
			// -------------------------------------------------------

			// Parameters for optimize_*()
			// -------------------------------------
			bool   optimize_new_edges_alone; //!< (Default:true) Before running a whole "local area" optimization, try to optimize new edges one by one to have a better starting point.
			bool   use_robust_kernel;
			double kernel_param;
			size_t max_iters;
			double max_error_per_obs_to_stop; //!< default: 1e-9
			double max_rho; //!< default: 1.0
			double max_lambda; //!< default: 1e20
			double min_error_reduction_ratio_to_relinearize; //!< default 0.01
			bool   numeric_jacobians; //!< (Default:false) Use a numeric approximation of the Jacobians (very slow!) instead of analytical ones.
			void (*feedback_user_iteration)(unsigned int iter, const double total_sq_err, const double mean_sqroot_error);
			bool   compute_condition_number; //!< Compute and return to the user the Hessian condition number of k2k edges (default=false)

			TCovarianceRecoveryPolicy  cov_recovery; //!< Recover covariance? What method to use? (Default: crpLandmarksApprox)
			// -------------------------------------

		};

		/** The unique struct which hold all the parameters from the different SRBA modules (sensors, optional features, optimizers,...) */
		struct TAllParameters
		{
			/** Different parameters for the SRBA methods \sa sensor_params */
			TSRBAParameters  srba;

			/** Sensor-specific parameters (sensor calibration, etc.) \sa parameters */
			typename OBS_TYPE::TObservationParams   sensor;

			/** Parameters related to the relative pose of sensors wrt the robot (if applicable) */
			typename RBA_OPTIONS::sensor_pose_on_robot_t::parameters_t  sensor_pose;

			/** Parameters related to the sensor noise covariance matrix */
			typename RBA_OPTIONS::obs_noise_matrix_t::parameters_t      obs_noise;
		};

		TAllParameters parameters;

		/** @} */  // End of data fields


		/** Enable or disables time profiling of all operations (default=enabled), which will be reported upon destruction */
		void inline enable_time_profiler(bool enable=true) { m_profiler.enable(enable); }

		const k2k_edges_deque_t & get_k2k_edges() const { return rba_state.k2k_edges; }

		const TRelativeLandmarkPosMap & get_known_feats()   const { return rba_state.known_lms; }
		const TRelativeLandmarkPosMap & get_unknown_feats() const { return rba_state.unknown_lms; }

		const rba_problem_state_t & get_rba_state() const { return rba_state; }
		rba_problem_state_t       & get_rba_state()       { return rba_state; }

		/** Access to the time profiler */
		inline mrpt::utils::CTimeLogger & get_time_profiler() { return m_profiler; }

		/** Changes the verbosity level: 0=None (only critical msgs), 1=verbose, 2=so verbose you'll have to say "Stop!" */
		inline void setVerbosityLevel(int level) { m_verbose_level = level; }

		/** Rebuild the Hessian symbolic information from the given Jacobians.
		  *  \param[out] H Output hessian (must be empty at input)
		  */
		template <class HESS_Ap, class HESS_f,class HESS_Apf, class JACOB_COLUMN_dh_dAp,class JACOB_COLUMN_dh_df>
		static void sparse_hessian_build_symbolic(
			HESS_Ap & HAp,
			HESS_f & Hf,
			HESS_Apf & HApf,
			const std::vector<JACOB_COLUMN_dh_dAp*> & dh_dAp,
			const std::vector<JACOB_COLUMN_dh_df*>  & dh_df);

		/** Rebuild the Hessian symbolic information from the internal pointers to blocks of Jacobians.
			*  Only the upper triangle is filled-in (all what is needed for Cholesky) for square Hessians, in whole for rectangular ones (it depends on the symbolic decomposition, done elsewhere).
			* \tparam SPARSEBLOCKHESSIAN can be: TSparseBlocksHessian_6x6, TSparseBlocksHessian_3x3 or TSparseBlocksHessian_6x3
			* \return The number of Jacobian multiplications skipped due to its observation being marked as "invalid"
			*/
		template <class SPARSEBLOCKHESSIAN>
		size_t sparse_hessian_update_numeric( SPARSEBLOCKHESSIAN & H ) const;


	protected:
		int m_verbose_level; //!< 0: None (only critical msgs), 1: verbose, 2:even more verbose, 3: even more

		typedef std::multimap<size_t,TKeyFrameID,std::greater<size_t> > base_sorted_lst_t;

		/** @name (Protected) Sub-algorithms
		    @{ */

		/** Make a list of base KFs of my new observations, ordered in descending order by # of shared observations: */
		void make_ordered_list_base_kfs(
			const typename traits_t::new_kf_observations_t & obs,
			base_sorted_lst_t            & obs_for_each_base_sorted,
			map<TKeyFrameID,size_t>       *out_obs_for_each_base =NULL ) const;

		/** This method will call edge_creation_policy(), which has predefined algorithms but could be re-defined by the user in a derived class */
		void determine_kf2kf_edges_to_create(
			const TKeyFrameID               new_kf_id,
			const typename traits_t::new_kf_observations_t   & obs,
			std::vector<TNewEdgeInfo> &new_k2k_edge_ids );

		/** Implements the edge-creation policy, by default depending on "parameters.edge_creation_policy" if the user doesn't re-implement this virtual method.
		  * See tutorials for examples of how to implement custom policies. */
		virtual void edge_creation_policy(
			const TKeyFrameID               new_kf_id,
			const typename traits_t::new_kf_observations_t   & obs,
			std::vector<TNewEdgeInfo> &new_k2k_edge_ids );

		/**
		  * \param observation_indices_to_optimize Indices wrt \a rba_state.all_observations. An empty vector means use ALL the observations involving the selected unknowns.
		  * \sa optimize_local_area
		  */
		void optimize_edges(
			const std::vector<size_t> & run_k2k_edges,
			const std::vector<size_t> & run_feat_ids_in,
			TOptimizeExtraOutputInfo & out_info,
			const std::vector<size_t> & observation_indices_to_optimize = std::vector<size_t>()
			);

		/** @} */

		/** Aux visitor struct, used in optimize_local_area() */
		struct VisitorOptimizeLocalArea
		{
			VisitorOptimizeLocalArea(const rba_problem_state_t & rba_state_, const TOptimizeLocalAreaParams &params_) :
				rba_state(rba_state_),
				params(params_)
			{ }

			const rba_problem_state_t & rba_state;
			const TOptimizeLocalAreaParams &params;

			vector<size_t> k2k_edges_to_optimize, lm_IDs_to_optimize;
			map<TLandmarkID,size_t>  lm_times_seen;

			/* Implementation of FEAT_VISITOR */
			inline bool visit_filter_feat(const TLandmarkID lm_ID,const topo_dist_t cur_dist)
			{
				return false; // Don't need to visit landmark nodes.
			}
			inline void visit_feat(const TLandmarkID lm_ID,const topo_dist_t cur_dist)
			{
				// Nothing to do
			}

			/* Implementation of KF_VISITOR */
			inline bool visit_filter_kf(const TKeyFrameID kf_ID,const topo_dist_t cur_dist)
			{
				return (kf_ID<=params.max_visitable_kf_id);
			}
			inline void visit_kf(const TKeyFrameID kf_ID,const topo_dist_t cur_dist)
			{
				// Nothing to do.
			}

			/* Implementation of K2K_EDGE_VISITOR */
			inline bool visit_filter_k2k(const TKeyFrameID current_kf, const TKeyFrameID next_kf,const k2k_edge_t* edge, const topo_dist_t cur_dist)
			{
				return true; // Visit all k2k edges
			}
			inline void visit_k2k(const TKeyFrameID current_kf, const TKeyFrameID next_kf,const k2k_edge_t* edge, const topo_dist_t cur_dist)
			{
				if (params.optimize_k2k_edges)
					k2k_edges_to_optimize.push_back(edge->id);
			}

			/* Implementation of K2F_EDGE_VISITOR */
			inline bool visit_filter_k2f(const TKeyFrameID current_kf, const k2f_edge_t* edge, const topo_dist_t cur_dist)
			{
				return params.optimize_landmarks; // Yes: visit all feature nodes if we're asked to
			}
			inline void visit_k2f(const TKeyFrameID current_kf, const k2f_edge_t* edge, const topo_dist_t cur_dist)
			{
				if (!edge->feat_has_known_rel_pos)
				{
					const TLandmarkID lm_ID = edge->obs.obs.feat_id;
					// Use an "==" so we only add the LM_ID to the list ONCE, just when it passes the threshold
					if (++lm_times_seen[lm_ID] == params.dont_optimize_landmarks_seen_less_than_n_times)
						lm_IDs_to_optimize.push_back(lm_ID);
				}
			}
		};

	private:
		rba_problem_state_t  rba_state;  //!< All the beef is here.

		mutable std::vector<bool> m_complete_st_ws; //!< Temporary working space used in \a create_complete_spanning_tree()

		/** Profiler for all SRBA operations
		  *  Enabled by default, can be disabled with \a enable_time_profiler(false)
		  */
		mutable mrpt::utils::CTimeLogger  m_profiler;

		/** Creates a new known/unknown position landmark (upon first LM observation ), and expands Jacobians with new observation
		  * \param[in] new_obs The basic data on the observed landmark: landmark ID, keyframe from which it's observed and parameters ("z" vector) of the observation itself (e.g. pixel coordinates).
		  * \param[in] fixed_relative_position If not NULL, this is the first observation of a landmark with a fixed, known position. Each such feature can be created only once, next observations MUST have this field set to NULL as with normal ("unfixed") landmarks.
		  * \param[in] unknown_relative_position_init_val Can be set to not-NULL only upon the first observation of a landmark with an unknown relative position. The feature will be created with this initial value for its relative position wrt the KF (further optimization will refine that value).
		  *
		  * \return The 0-based index of the new observation
		  *
		  * \note Both \a fixed_relative_position and \a unknown_relative_position_init_val CAN'T be set to !=NULL at once.
		  *
		  * \note If new edges had been introduced before this observation, make sure the symbolic spanning trees are up-to-date!
		  *
		  * \note Runs in O(P+log C), with:
		  *   - C=typical amount of KFs which all see the same landmark,
		  *   - P=typical number of kf2kf edges between observing and the base KF of the observed landmark.
		  */
		size_t add_observation(
			const TKeyFrameID         observing_kf_id,
			const typename observation_traits_t::observation_t     & new_obs,
			const array_landmark_t * fixed_relative_position = NULL,
			const array_landmark_t * unknown_relative_position_init_val = NULL
			);

		typedef typename jacobian_traits<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSparseBlocksJacobians_dh_dAp TSparseBlocksJacobians_dh_dAp;
		typedef typename jacobian_traits<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSparseBlocksJacobians_dh_df TSparseBlocksJacobians_dh_df;

		/** Prepare the list of all required KF roots whose spanning trees need numeric updates with each optimization iteration */
		void prepare_Jacobians_required_tree_roots(
			std::set<TKeyFrameID>  & kfs_num_spantrees_to_update,
			const std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*> &lst_JacobCols_dAp,
			const std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  &lst_JacobCols_df );


		/** Re-evaluate all Jacobians numerically using their symbolic info. Return overall number of block Jacobians */
		size_t recompute_all_Jacobians(
			std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*> &lst_JacobCols_dAp,
			std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  &lst_JacobCols_df,
			std::vector<const pose_flag_t*>    * out_list_of_required_num_poses = NULL );

	protected:
		/** Auxiliary template for evaluating the dh_df part in \a recompute_all_Jacobians().
		// The extra complexity of adding this auxiliary template with specializations is required to avoid 
		//  the compiler trying to evaluate the jacobians dh_df in relative SLAM problems, where the Jacobian does not exist. */
		template <landmark_jacob_family_t LM_JACOB_FAMILY>
		struct recompute_all_Jacobians_dh_df;

		// Specialization for "normal SLAM" (SLAM with real landmarks)
		template <> struct recompute_all_Jacobians_dh_df<jacob_point_landmark> {
			static size_t eval(
				rba_engine_t  &rba,
				std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  &lst_JacobCols_df,
				std::vector<const typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::pose_flag_t*>    * out_list_of_required_num_poses )
			{
				const size_t nUnknowns_k2f = lst_JacobCols_df.size();
				size_t nJacobs = 0;
				for (size_t i=0;i<nUnknowns_k2f;i++)
				{
					// For each column, process each nonzero block:
					typename TSparseBlocksJacobians_dh_df::col_t *col = lst_JacobCols_df[i];

					for (typename TSparseBlocksJacobians_dh_df::col_t::iterator it=col->begin();it!=col->end();++it)
					{
						const size_t obs_idx = it->first;
						typename TSparseBlocksJacobians_dh_df::TEntry & jacob_entry = it->second;
						rba.compute_jacobian_dh_df(
							jacob_entry,
			#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
							*
			#endif
							rba.get_rba_state().all_observations[obs_idx],
							out_list_of_required_num_poses );
						nJacobs++;
					}
				}
				return nJacobs;
			}

		};

		// Specialization for relative graph-SLAM (no real landmarks)
		template <> struct recompute_all_Jacobians_dh_df<jacob_relpose_landmark> {
			static size_t eval(
				rba_engine_t  &rba,
				std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  &lst_JacobCols_df,
				std::vector<const typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::pose_flag_t*>    * out_list_of_required_num_poses ) 
			{
				// Nothing to do: this will never be actually called.
				return 0;
			}
		};


	public:

		/** Private aux structure for BFS searches. */
		struct TBFSEntryEdges
		{
			TBFSEntryEdges() : dist( numeric_limits<topo_dist_t>::max() ), edge(NULL)
			{}

			TKeyFrameID prev;
			topo_dist_t dist;
			const k2k_edge_t* edge;
		};


		/** ====================================================================
		                         j,i                    lm_id,base_id
		             \partial  h            \partial  h
		                         l                      obs_frame_id
		   dh_dp = ------------------ = ---------------------------------
		                         d+1                    cur_id
		             \partial  p            \partial  p
		                         d                      stp.next_node

		    See tech report:
		     "A tutorial on SE(3) transformation parameterizations and
		      on-manifold optimization", Jose-Luis Blanco, 2010.
		   ==================================================================== */
		void compute_jacobian_dh_dp(
			typename TSparseBlocksJacobians_dh_dAp::TEntry  &jacob,
			const k2f_edge_t & observation,
			const k2k_edges_deque_t  &k2k_edges,
			std::vector<const pose_flag_t*>    *out_list_of_required_num_poses) const;

		/** ====================================================================
		                         j,i                    lm_id,base_id
		             \partial  h            \partial  h
		                         l                      obs_frame_id
		   dh_df = ------------------ = ---------------------------------

		             \partial  f            \partial  f

		     Note: f=relative position of landmark with respect to its base kf
		   ==================================================================== */
		void compute_jacobian_dh_df(
			typename TSparseBlocksJacobians_dh_df::TEntry  &jacob,
			const k2f_edge_t & observation,
			std::vector<const pose_flag_t*> *out_list_of_required_num_poses) const;

		void gl_aux_draw_node(mrpt::opengl::CSetOfObjects &soo, const std::string &label, const float x, const float y) const;

		const pose_t aux_null_pose; //!< A fixed SE(3) pose at the origin (used when we need a pointer or a reference to a "null transformation").

		struct TNumeric_dh_dAp_params
		{
			TNumeric_dh_dAp_params(
				const size_t  _k2k_edge_id,
				const pose_t * _pose_d1_wrt_obs,
				const pose_t & _pose_base_wrt_d1,
				const array_landmark_t & _xji_i,
				const bool _is_inverse_dir,
				const k2k_edges_deque_t  &_k2k_edges,
				const typename OBS_TYPE::TObservationParams   & _sensor_params,
				const typename RBA_OPTIONS::sensor_pose_on_robot_t::parameters_t  & _sensor_pose
				) :
			k2k_edge_id(_k2k_edge_id),
			pose_d1_wrt_obs(_pose_d1_wrt_obs),
			pose_base_wrt_d1(_pose_base_wrt_d1),
			xji_i(_xji_i),
			is_inverse_dir(_is_inverse_dir),
			k2k_edges(_k2k_edges),
			sensor_params(_sensor_params),
			sensor_pose(_sensor_pose)
			{
			}

			const size_t k2k_edge_id;
			const pose_t * pose_d1_wrt_obs;
			const pose_t & pose_base_wrt_d1;
			const array_landmark_t & xji_i;
			const bool is_inverse_dir;
			const k2k_edges_deque_t  &k2k_edges;
			const typename OBS_TYPE::TObservationParams   & sensor_params;
			const typename RBA_OPTIONS::sensor_pose_on_robot_t::parameters_t  & sensor_pose;
		};

		struct TNumeric_dh_df_params
		{
			TNumeric_dh_df_params(
				const pose_t * _pose_base_wrt_obs,
				const array_landmark_t & _xji_i,
				const typename OBS_TYPE::TObservationParams   & _sensor_params,
				const typename RBA_OPTIONS::sensor_pose_on_robot_t::parameters_t  & _sensor_pose
				) :
			pose_base_wrt_obs(_pose_base_wrt_obs),
			xji_i(_xji_i),
			sensor_params(_sensor_params),
			sensor_pose(_sensor_pose)
			{
			}

			const pose_t * pose_base_wrt_obs;
			const array_landmark_t & xji_i;
			const typename OBS_TYPE::TObservationParams   & sensor_params;
			const typename RBA_OPTIONS::sensor_pose_on_robot_t::parameters_t  & sensor_pose;
		};

		/** Auxiliary method for numeric Jacobian: numerically evaluates the new observation "y" for a small increment "x" in a relative KF-to-KF pose */
		static void numeric_dh_dAp(const array_pose_t &x, const TNumeric_dh_dAp_params& params, array_obs_t &y);
		/** Auxiliary method for numeric Jacobian: numerically evaluates the new observation "y" for a small increment "x" in a landmark position  */
		static void numeric_dh_df(const array_landmark_t &x, const TNumeric_dh_df_params& params, array_obs_t &y);

		static inline void add_edge_ij_to_list_needed_roots(std::set<TKeyFrameID>  & lst, const TKeyFrameID i, const TKeyFrameID j)
		{
			lst.insert(i);
			lst.insert(j);
		}

		void compute_minus_gradient(
			mrpt::vector_double & minus_grad,
			const std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*> & sparse_jacobs_Ap,
			const std::vector<typename TSparseBlocksJacobians_dh_df::col_t*> & sparse_jacobs_f,
			const vector_residuals_t  & residuals,
			const std::map<size_t,size_t> &obs_global_idx2residual_idx
			) const;

		/** Each of the observations used during the optimization */
		struct TObsUsed
		{
			TObsUsed(const size_t obs_idx_, k2f_edge_t * const k2f_) : obs_idx(obs_idx_),k2f(k2f_)
			{ }

			size_t      obs_idx; //!< Global index in all_observations
			k2f_edge_t*  k2f;     //!< Obs data

		private:
			// Don't use this constructor
			TObsUsed() : obs_idx(0), k2f(NULL) {}
		}; // end of TObsUsed


		inline double reprojection_residuals(
			vector_residuals_t & residuals, // Out:
			const std::vector<TObsUsed> & observations // In:
			) const;

		/** pseudo-huber cost function */
		static inline double huber_kernel(double delta, const double kernel_param)
		{
			return std::abs(2*mrpt::utils::square(kernel_param)*(std::sqrt(1+mrpt::utils::square(delta/kernel_param))-1));
		}

	}; // end of class "RbaEngine"


} // end of namespace "srba"
} // end of namespace "mrpt"

// -----------------------------------------------------------------
//          Include all template implementation files
// -----------------------------------------------------------------
#include "impl/add-observations.h"
#include "impl/alloc_keyframe.h"
#include "impl/alloc_kf2kf_edge.h"
#include "impl/create_kf2kf_edge.h"
#include "impl/define_new_keyframe.h"
#include "impl/jacobians.h"
#include "impl/rba_problem_common.h"
#include "impl/schur.h"
#include "impl/sparse_hessian_build_symbolic.h"
#include "impl/sparse_hessian_update_numeric.h"

#include "impl/spantree_create_complete.h"
#include "impl/spantree_update_symbolic.h"
#include "impl/spantree_update_numeric.h"
#include "impl/spantree_misc.h"

#include "impl/jacobians.h"

#include "impl/export_opengl.h"
#include "impl/export_dot.h"

#include "impl/eval_overall_error.h"

#include "impl/determine_kf2kf_edges_to_create.h"
#include "impl/edge_creation_policy.h"

#include "impl/reprojection_residuals.h"
#include "impl/compute_minus_gradient.h"
#include "impl/optimize_edges.h"
#include "impl/lev-marq_solvers.h"

#include "impl/bfs_visitor.h"
#include "impl/optimize_local_area.h"
// -----------------------------------------------------------------
//            ^^ End of implementation files ^^
// -----------------------------------------------------------------

// Undefine temporary macro for verbose output
#undef VERBOSE_LEVEL

