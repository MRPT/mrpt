/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
