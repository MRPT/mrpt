/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#pragma once

/** \file RbaEngine.h
  * \brief This file exposes the public API and data types of libmrpt-srba (it requires also including srba/models/{*.h} to have a complete SLAM/RBA system)
  */

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/opengl/CSetOfObjects.h>

#define VERBOSE_LEVEL(_LEVEL) if (m_verbose_level>=_LEVEL) std::cout

namespace mrpt
{
namespace srba
{
	using namespace std;

	/** The main class for this library: it defines a Relative Bundle-Adjustment (RBA) problem with (partially known) landmarks,
	  *   plus the methods to update it with new observations and to optimize the relative poses with least squares optimizers.
	  *
	  *   The unknowns to be solved are:
	  *		- Relative 6D poses among keyframes.
	  *		- Relative 3D positions of landmarks wrt to their base frame.
	  *
	  *   The set of known data used to run the optimization comprises:
	  *		- Camera calibration parameters. In \a camera_calib
	  *		- Sequence of all observations, i.e. feature (x,y) coordinates. In \a all_obs_by_lm and also \a all_obs_by_frame.
	  *		- Relative 3D positions of a subset of landmarks wrt to their base frame (in \a known_lms)
	  *
	  */
	template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
	class RBA_Problem
	{
	public:
		/** @name Templatized typedef's
		    @{ */
		typedef RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE> rba_problem_t;

		typedef KF2KF_POSE_TYPE kf2kf_pose_type;
		typedef LM_TYPE         lm_type;
		typedef OBS_TYPE        obs_type;

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
		typedef TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE> rba_problem_state_t;

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


		/** @name Main API methods
		    @{ */

		/** The most common entry point for SRBA: append a new keyframe (KF) to the map, automatically creates the required edges
		  *  and optimize the local area around this new KF.
		  *
		  * \param[in]  obs All the landmark observations gathered from this new KF (with data association already solved).
		  * \param[out] out_new_kf_info Returned information about the newly created KF.
		  * \param[in]  run_local_optimization If set to true (default), the local map around the new KF will be optimized.
		  */
		void define_new_keyframe(
			const typename traits_t::new_kf_observations_t  & obs,
			TNewKeyFrameInfo             & out_new_kf_info,
			const bool                     run_local_optimization = true
			);

		/** Runs the least-squares optimization procedure to optimize the neighborhood of a given root KeyFrame.
		  *
		  * \param observation_indices_to_optimize Indices wrt \a rba_state.all_observations. An empty vector means use ALL the observations involving the selected unknowns.
		  * \note Extra options are available in \a parameters
		  *  \sa add_observation, optimize_edges
		  */
		void optimize_sliding_window(
			const TKeyFrameID  root_id,
			const unsigned int win_size,
			const bool         optimize_k2k_edges,
			const bool         optimize_k2f_edges,
			TOptimizeExtraOutputInfo & out_info,
			const TKeyFrameID max_visitable_kf_id =  static_cast<TKeyFrameID>(-1),
			const std::vector<size_t> & observation_indices_to_optimize = std::vector<size_t>()
			)
		{
			m_profiler.enter("optimize_sliding_window");

			// 1st) Find list of edges to optimize:
			m_profiler.enter("optimize_sliding_window.find_edges2opt");

			MRPT_TODO("Reuse existing spanning trees?")

			vector<size_t> k2k_edges_to_optimize, k2f_edges_to_optimize;
			this->find_sliding_window_edges(
				k2k_edges_to_optimize, k2f_edges_to_optimize,
				root_id,
				win_size,
				optimize_k2k_edges, optimize_k2f_edges,
				max_visitable_kf_id);

			m_profiler.leave("optimize_sliding_window.find_edges2opt");

			// 2nd) Optimize them:
			if (!k2k_edges_to_optimize.empty() || !k2f_edges_to_optimize.empty())
			{
				this->optimize_edges(k2k_edges_to_optimize,k2f_edges_to_optimize, out_info, observation_indices_to_optimize);
			}

			m_profiler.leave("optimize_sliding_window");
		}

		struct TOpenGLRepresentationOptions
		{
			TOpenGLRepresentationOptions() :
				span_tree_max_depth(static_cast<size_t>(-1)),
				draw_unknown_feats(true),
				draw_unknown_feats_ellipses(true),
				draw_unknown_feats_ellipses_quantiles(1)
			{
			}

			size_t span_tree_max_depth; //!< Maximum spanning tree depth for reconstructing relative poses (default=-1 : infinity)
			bool   draw_unknown_feats;  //!< Draw features with non-fixed rel.pos as well?
			bool   draw_unknown_feats_ellipses;
			double draw_unknown_feats_ellipses_quantiles;
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
			return rba_state.find_path_bfs(src_kf,trg_kf,found_path);
		}


		/** @} */  // End of Extra API methods

	private:
		/** @name Sub-algorithms
		    @{ */

		void determine_kf2kf_edges_to_create(
			const TKeyFrameID               new_kf_id,
			const typename traits_t::new_kf_observations_t   & obs,
			std::vector<TNewEdgeInfo> &new_k2k_edge_ids );

		/**
		  * \param observation_indices_to_optimize Indices wrt \a rba_state.all_observations. An empty vector means use ALL the observations involving the selected unknowns.
		  * \sa optimize_sliding_window
		  */
		void optimize_edges(
			const std::vector<size_t> & run_k2k_edges,
			const std::vector<size_t> & run_k2f_edges,
			TOptimizeExtraOutputInfo & out_info,
			const std::vector<size_t> & observation_indices_to_optimize = std::vector<size_t>()
			);

		/** Aux struct made for find_sliding_window_edges(). Always returns true, so all edges pass the filter. */
		struct Default_Filter
		{
			inline bool operator()(const size_t idx) const { return true; }
		};

		/** Makes a list of edges (unknowns) using a spanning tree from a given root keyframe ID.
		  * Version without an ID filter
		  */
		void find_sliding_window_edges(
			std::vector<size_t> &out_k2k_edges,
			std::vector<size_t> &out_k2f_edges,
			const TKeyFrameID  root_id,
			const unsigned int win_size,
			const bool         include_k2k_edges,
			const bool         include_k2f_edges,
			const TKeyFrameID  max_visitable_kf_id  =  static_cast<TKeyFrameID>(-1)
			)  const
		{
			Default_Filter k2k_filter, k2f_filter;
			this->find_sliding_window_edges<Default_Filter,Default_Filter>(
				out_k2k_edges,out_k2f_edges,
				root_id, win_size,
				include_k2k_edges, include_k2f_edges,
				k2k_filter, k2f_filter,
				max_visitable_kf_id);
		}


		/** Makes a list of edges (unknowns) using a spanning tree from a given root keyframe ID.
		  * Version with an ID filter
		  */
		template <class K2K_EDGE_FILTER,class K2F_EDGE_FILTER>
		void find_sliding_window_edges(
			std::vector<size_t> &out_k2k_edges,
			std::vector<size_t> &out_k2f_edges,
			const TKeyFrameID  root_id,
			const unsigned int win_size,
			const bool         include_k2k_edges,
			const bool         include_k2f_edges,
			const K2K_EDGE_FILTER & k2k_filter,
			const K2F_EDGE_FILTER & k2f_filter,
			const TKeyFrameID  max_visitable_kf_id  =  static_cast<TKeyFrameID>(-1)
			) const
		{
			ASSERT_(win_size>=1)
			ASSERT_(include_k2k_edges || include_k2f_edges)
			m_profiler.enter("find_sliding_window_edges");
			std::set<size_t> kfs_visited, k2k_edges_visited;
			std::map<size_t,bool> k2f_edges_visited;

			recursive_find_slidwindow(
				kfs_visited,
				k2k_edges_visited, k2f_edges_visited,
				include_k2k_edges, include_k2f_edges,
				root_id,
				rba_state.keyframes[root_id],
				rba_state, win_size, max_visitable_kf_id);

			// Convert into a vector:
			out_k2k_edges.clear(); out_k2k_edges.reserve(k2k_edges_visited.size());
			out_k2f_edges.clear(); out_k2f_edges.reserve(k2f_edges_visited.size());
			for (std::set<size_t>::const_iterator it=k2k_edges_visited.begin();it!=k2k_edges_visited.end();++it)
				if (k2k_filter(*it))
					out_k2k_edges.push_back(*it);

			for (std::map<size_t,bool>::const_iterator it=k2f_edges_visited.begin();it!=k2f_edges_visited.end();++it)
				if (!it->second && k2f_filter(it->first))
					out_k2f_edges.push_back(it->first);
			m_profiler.leave("find_sliding_window_edges");
		}


		/** @} */


		/** @name Public data fields
			@{ */
	public:

		/** Different parameters for the SRBA methods */
		struct TSRBAParameters : public mrpt::utils::CLoadableOptions
		{
			TSRBAParameters();

			/** See docs of mrpt::utils::CLoadableOptions */
			virtual void  loadFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section);
			/** See docs of mrpt::utils::CLoadableOptions */
			virtual void  saveToConfigFile(mrpt::utils::CConfigFileBase &out,const std::string & section) const;


			// Parameters for determine_kf2kf_edges_to_create(), etc.
			// -------------------------------------------------------
			TEdgeCreationPolicy  edge_creation_policy;

			/** Maximum depth for maintained spanning trees. */
			topo_dist_t          max_tree_depth;

			/** The maximum topological distance of keyframes to be optimized around the most recent keyframe. */
			topo_dist_t          max_optimize_depth;

			size_t              submap_size;
			// -------------------------------------------------------

			// Parameters for optimize_*()
			// -------------------------------------
			bool   use_robust_kernel;
			double kernel_param;
			size_t max_iters;
			double max_error_per_obs_px; //!< default: 1e-3
			bool   numeric_jacobians;
			void (*feedback_user_iteration)(unsigned int iter, const double total_sq_err, const double mean_sqroot_error);
			bool   compute_condition_number; //!< Compute and return to the user the Hessian condition number of k2k edges (default=false)
			double std_noise_pixels; //!< default: 1, the standard deviation assumed for feature coordinates (this parameter is only needed to scale the uncertainties of reconstructed LMs with unknown locations).
			// -------------------------------------

		};
		/** Different parameters for the SRBA methods \sa sensor_params */
		TSRBAParameters  parameters;

		/** Sensor-specific parameters (sensor calibration, etc.) \sa parameters */
		typename OBS_TYPE::TObservationParams   sensor_params;

		/** @} */  // End of data fields

		/** Default constructor */
		RBA_Problem();

		/** Reset the entire problem to an empty state (automatically called at construction) */
		void clear();

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


		struct TSizeFlag
		{
			TSizeFlag() : value(), valid(false) { }

			size_t value;
			bool    valid;
		};
		typedef mrpt::utils::map_as_vector<
			size_t,
			TSizeFlag,
			std::deque<std::pair<size_t,TSizeFlag> > > TMyMapSize2Size;

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
		static size_t sparse_hessian_update_numeric( SPARSEBLOCKHESSIAN & H );


	private:
		rba_problem_state_t  rba_state;  //!< All the beef is here.

		/** Aux. vector used during optimization.
		  * Made a class member to avoid reallocating the memory with each optimization
		  */
		TMyMapSize2Size m_cached_obs_global_idx2residual_idx;
		std::vector<size_t>  m_used_indices_in_obs_map; //!< Used indices in \a m_cached_obs_global_idx2residual_idx, to be cleared.

		mutable std::vector<bool> m_complete_st_ws; //!< Temporary working space used in \a create_complete_spanning_tree()

		/** Profiler for all SRBA operations
		  *  Enabled by default, can be disabled with \a enable_time_profiler(false)
		  */
		mutable mrpt::utils::CTimeLogger  m_profiler;

		int m_verbose_level; //!< 0: None (only critical msgs), 1: verbose, 2:even more verbose

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

		static void recursive_find_slidwindow(
			std::set<size_t> &kf_visited, std::set<size_t> &edges_visited, std::map<size_t,bool> &k2f_edges_visited,
			const bool   optimize_k2k_edges,const bool optimize_k2f_edges,
			const TKeyFrameID    kf_id,
			const keyframe_info &kfi,
			const rba_problem_state_t &rba,unsigned int win_size,const TKeyFrameID max_visitable_kf_id )
		{
			MRPT_TODO("Replace with non-assoc containers")

			ASSERTDEB_(kf_id< rba.keyframes.size())

			// k2f edges:
			if (optimize_k2f_edges)
			{
				const deque<k2f_edge_t*> & vf = kfi.adjacent_k2f_edges;

				for (typename deque<k2f_edge_t*>::const_iterator it=vf.begin();it!=vf.end();++it)
				{
					const k2f_edge_t* ed = *it;

					// Only count as k2f_edges the first observation of a feature with an unknown relative position:
					if (!ed->feat_has_known_rel_pos)
					{
						const TLandmarkID feat_id = ed->obs.obs.feat_id;

						map<size_t,bool>::iterator it_visited = k2f_edges_visited.find(feat_id);

						if (it_visited != k2f_edges_visited.end())
							it_visited->second = false; // Seen once = false
						else
							k2f_edges_visited[feat_id] = true; // First observation:
					}
				}
			}

			// explore next edges?
			if (!win_size--) return;

			kf_visited.insert(kf_id);

			// k2k edges:
			const std::deque<k2k_edge_t*> & vc = kfi.adjacent_k2k_edges;

			for (size_t i=0;i<vc.size();i++)
			{
				if (kf_id != vc[i]->from)
				{
					if (vc[i]->from<=max_visitable_kf_id && !kf_visited.count(vc[i]->from))
					{
						if (optimize_k2k_edges) edges_visited.insert(vc[i]->id);
						recursive_find_slidwindow(kf_visited,edges_visited,k2f_edges_visited,optimize_k2k_edges,optimize_k2f_edges,
							vc[i]->from, rba.keyframes[vc[i]->from],
							rba,win_size,max_visitable_kf_id);
					}
				}
				else
				{
					if (vc[i]->to<=max_visitable_kf_id &&!kf_visited.count(vc[i]->to))
					{
						if (optimize_k2k_edges) edges_visited.insert(vc[i]->id);
						recursive_find_slidwindow(kf_visited,edges_visited,k2f_edges_visited,optimize_k2k_edges,optimize_k2f_edges,
							vc[i]->to,rba.keyframes[vc[i]->to],
							rba,win_size,max_visitable_kf_id);
					}
				}
			}
		}

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
				const typename OBS_TYPE::TObservationParams   & _sensor_params
				) :
			k2k_edge_id(_k2k_edge_id),
			pose_d1_wrt_obs(_pose_d1_wrt_obs),
			pose_base_wrt_d1(_pose_base_wrt_d1),
			xji_i(_xji_i),
			is_inverse_dir(_is_inverse_dir),
			k2k_edges(_k2k_edges),
			sensor_params(_sensor_params)
			{
			}

			const size_t k2k_edge_id;
			const pose_t * pose_d1_wrt_obs;
			const pose_t & pose_base_wrt_d1;
			const array_landmark_t & xji_i;
			const bool is_inverse_dir;
			const k2k_edges_deque_t  &k2k_edges;
			const typename OBS_TYPE::TObservationParams   & sensor_params;
		};

		struct TNumeric_dh_df_params
		{
			TNumeric_dh_df_params(
				const pose_t * _pose_base_wrt_obs,
				const array_landmark_t & _xji_i,
				const typename OBS_TYPE::TObservationParams   & _sensor_params
				) :
			pose_base_wrt_obs(_pose_base_wrt_obs),
			xji_i(_xji_i),
			sensor_params(_sensor_params)
			{
			}

			const pose_t * pose_base_wrt_obs;
			const array_landmark_t & xji_i;
			const typename OBS_TYPE::TObservationParams   & sensor_params;
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

		typedef std::multimap<size_t,TKeyFrameID,std::greater<size_t> > base_sorted_lst_t;

		/** Make a list of base KFs of my new observations, ordered in descending order by # of shared observations: */
		void make_ordered_list_base_kfs(
			const typename traits_t::new_kf_observations_t & obs,
			base_sorted_lst_t            & obs_for_each_base_sorted,
			map<TKeyFrameID,size_t>       *out_obs_for_each_base =NULL ) const;

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

	}; // end of class "RBA_Problem"


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

#include "impl/reprojection_residuals.h"
#include "impl/compute_minus_gradient.h"
#include "impl/optimize_edges.h"
// -----------------------------------------------------------------
//            ^^ End of implementation files ^^
// -----------------------------------------------------------------
