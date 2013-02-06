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

namespace mrpt { namespace srba {

/** Default constructor */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::RBA_Problem() :
	rba_state(),
	m_profiler(true)
{
	clear();
}

/** Reset the entire problem to an empty state (automatically called at construction) */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::clear()
{
	this->rba_state.clear();
}

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSRBAParameters::TSRBAParameters() :
	// -------------------------------
	edge_creation_policy ( ecpICRA2013 ),
	max_tree_depth       ( 4 ),
	max_optimize_depth   ( 4 ),
	submap_size          ( 15 ),
	// -------------------------------
	use_robust_kernel    ( false ),
	kernel_param         ( 3. ),
	max_iters            ( 30 ),
	max_error_per_obs_to_stop    ( 1e-9 ),
	numeric_jacobians    ( false ),
	feedback_user_iteration(NULL),
	compute_condition_number(false)	
{
}

/** See docs of mrpt::utils::CLoadableOptions */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSRBAParameters::loadFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section)
{
	edge_creation_policy = source.read_enum(section, "edge_creation_policy", edge_creation_policy);
	MRPT_LOAD_CONFIG_VAR(max_tree_depth,uint64_t,source,section)
	MRPT_LOAD_CONFIG_VAR(max_optimize_depth,uint64_t,source,section)
	MRPT_LOAD_CONFIG_VAR(submap_size,uint64_t,source,section)

	MRPT_LOAD_CONFIG_VAR(use_robust_kernel,bool,source,section)
	MRPT_LOAD_CONFIG_VAR(kernel_param,double,source,section)
	MRPT_LOAD_CONFIG_VAR(max_iters,uint64_t,source,section)
	MRPT_LOAD_CONFIG_VAR(max_error_per_obs_to_stop,double,source,section)

}

/** See docs of mrpt::utils::CLoadableOptions */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSRBAParameters::saveToConfigFile(mrpt::utils::CConfigFileBase & out,const std::string & section) const
{
	out.write(section,"edge_creation_policy", mrpt::utils::TEnumType<TEdgeCreationPolicy>::value2name(edge_creation_policy) ,  /* text width */ 30, 30, "Arc creation policy");
	out.write(section,"max_tree_depth",max_tree_depth,  /* text width */ 30, 30, "Maximum depth of all spanning trees");
	out.write(section,"max_optimize_depth",max_optimize_depth, /* text width */ 30, 30, "Max. local optimization distance");
	out.write(section,"submap_size",submap_size, /* text width */ 30, 30, "Max. local optimization distance");

	out.write(section,"use_robust_kernel",use_robust_kernel,  /* text width */ 30, 30, "Use pseudo-Huber kernel?");
	out.write(section,"kernel_param",kernel_param,  /* text width */ 30, 30, "robust kernel parameter");
	out.write(section,"max_iters",max_iters,  /* text width */ 30, 30, "Max. iterations for optimization");
	out.write(section,"max_error_per_obs_to_stop",max_error_per_obs_to_stop,  /* text width */ 30, 30, "Another criterion for stopping optimization");


}


/** Computes stats on the degree (# of adjacent nodes) of all the nodes in the graph. Runs in O(N) with N=# of keyframes */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::compute_all_node_degrees(
	double &out_mean_degree,
	double &out_std_degree,
	double &out_max_degree) const
{
	out_mean_degree = 0;
	out_std_degree  = 0;
	out_max_degree  = 0;

	const size_t nKFs = keyframes.size();

	std::vector<size_t> degs;
	degs.reserve(nKFs);

	for(size_t i=0;i<nKFs;i++)
		degs.push_back( keyframes[i].adjacent_k2k_edges.size() );

	mrpt::math::meanAndStd(degs,out_mean_degree,out_std_degree);
	out_max_degree = mrpt::math::maximum(degs);

}

/** Returns true if the pair of KFs are connected thru a kf2kf edge, no matter the direction of the edge.
  * Runs in worst-case O(D) with D the degree of the KF graph (that is, the maximum number of edges adjacent to one KF) */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
bool TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::are_keyframes_connected(const TKeyFrameID id1, const TKeyFrameID id2) const
{
	ASSERT_BELOW_(id1, keyframes.size())
	ASSERT_BELOW_(id2, keyframes.size())

	const std::deque<k2k_edge_t*> & id1_adj = keyframes[id1].adjacent_k2k_edges;

	for (size_t i=0;i<id1_adj.size();i++)
		if ( id2== getTheOtherFromPair2(id1, *id1_adj[i]) )
			return true;

	return false;
}

} } // end NS
