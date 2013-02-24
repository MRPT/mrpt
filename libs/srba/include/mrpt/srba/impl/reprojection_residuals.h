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

/** reprojection_residuals */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
double RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::reprojection_residuals(
	vector_residuals_t & residuals, // Out:
	const std::vector<TObsUsed> & observations // In:
	) const
{
	const size_t nObs = observations.size();
	if (residuals.size()!=nObs) residuals.resize(nObs);

	double total_sqr_err = 0;

	for (size_t i=0;i<nObs;i++)
	{
		// Actually measured pixel coords: observations[i]->obs.px
		const TKeyFrameID  obs_frame_id = observations[i].k2f->obs.kf_id; // Observed from here.
		const TRelativeLandmarkPos *feat_rel_pos = observations[i].k2f->feat_rel_pos;

		ASSERTDEB_(feat_rel_pos!=NULL)

		const TKeyFrameID  base_id  = feat_rel_pos->id_frame_base;

		pose_t const * base_pose_wrt_observer=NULL;

		// This case can occur with feats with unknown rel.pos:
		if (base_id==obs_frame_id)
		{
			base_pose_wrt_observer = &aux_null_pose;
		}
		else
		{
			// num[SOURCE] |--> map[TARGET] = CPose3D of TARGET as seen from SOURCE
			const typename TRelativePosesForEachTarget::const_iterator itPoseMap_for_base_id = rba_state.spanning_tree.num.find(obs_frame_id);
			ASSERT_( itPoseMap_for_base_id != rba_state.spanning_tree.num.end() )

			const typename frameid2pose_map_t::const_iterator itRelPose = itPoseMap_for_base_id->second.find(base_id);
			ASSERT_( itRelPose != itPoseMap_for_base_id->second.end() )

			base_pose_wrt_observer = &itRelPose->second.pose;
		}

		// pose_robot2sensor(): pose wrt sensor = pose_wrt_robot (-) sensor_pose_on_the_robot
		typename options::internal::resulting_pose_t<typename RBA_OPTIONS::sensor_pose_on_robot_t,REL_POSE_DIMS>::pose_t base_pose_wrt_sensor(mrpt::poses::UNINITIALIZED_POSE);
		RBA_OPTIONS::sensor_pose_on_robot_t::pose_robot2sensor( *base_pose_wrt_observer, base_pose_wrt_sensor, this->parameters.sensor_pose );

		const array_obs_t & real_obs = observations[i].k2f->obs.obs_arr;
		residual_t &delta = residuals[i];

		// Generate observation and compare to real obs:
		sensor_model_t::observe_error(delta,real_obs, base_pose_wrt_sensor,feat_rel_pos->pos, this->parameters.sensor);

		const double sum_2 = delta.squaredNorm();
		if (this->parameters.srba.use_robust_kernel)
		{
			const double nrm = std::max(1e-11,std::sqrt(sum_2));
			const double w = std::sqrt(huber_kernel(nrm,parameters.srba.kernel_param))/nrm;
			delta *= w;
			total_sqr_err += (w*w)*sum_2;
		}
		else
		{
			// nothing else to do:
			total_sqr_err += sum_2;
		}
	} // end for i

	return total_sqr_err;
}

} }  // end namespaces
