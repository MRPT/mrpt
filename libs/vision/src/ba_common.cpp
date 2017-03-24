/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/bundle_adjustment.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/math/robust_kernels.h>
#include "ba_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;


/* -------------------------------------------------------------------------------------
                            ba_initial_estimate

 Fills the frames & landmark points maps with an initial gross estimate
 from the sequence \a observations, so they can be fed to bundle adjustment methods.
   ------------------------------------------------------------------------------------ */
void mrpt::vision::ba_initial_estimate(
	const TSequenceFeatureObservations   & observations,
	const TCamera                        & camera_params,
	TFramePosesMap                       & frame_poses,
	TLandmarkLocationsMap                & landmark_points )
{
	MRPT_UNUSED_PARAM(camera_params);
	MRPT_START
	// VERY CRUDE APPROACH: All camera poses at the origin, all points at (0,0,1)
	// TODO: Improve with the data in "observations"...

	// Clear previous contents:
	frame_poses.clear();
	landmark_points.clear();

	// Go thru the obs list and insert poses:
	for (TSequenceFeatureObservations::const_iterator it=observations.begin();it!=observations.end();++it)
	{
		const TFeatureID     feat_ID  = it->id_feature;
		const TCameraPoseID  frame_ID = it->id_frame;

		frame_poses[frame_ID]    = CPose3D(0,0,0,0,0,0);
		landmark_points[feat_ID] = TPoint3D(0,0,1);
	}
	MRPT_END
}

void mrpt::vision::ba_initial_estimate(
	const TSequenceFeatureObservations   & observations,
	const TCamera                        & camera_params,
	TFramePosesVec                       & frame_poses,
	TLandmarkLocationsVec                & landmark_points )
{
	MRPT_UNUSED_PARAM(camera_params);
	MRPT_START
	// VERY CRUDE APPROACH: All camera poses at the origin, all points at (0,0,1)
	// TODO: Improve with the data in "observations"...

	// Clear previous contents:
	frame_poses.clear();
	landmark_points.clear();

	if (observations.empty()) return;

	// Go thru the obs list and insert poses:

	TFeatureID 		max_pt_id=0;
	TCameraPoseID   max_fr_id=0;

	for (TSequenceFeatureObservations::const_iterator it=observations.begin();it!=observations.end();++it)
	{
		const TFeatureID     feat_ID  = it->id_feature;
		const TCameraPoseID  frame_ID = it->id_frame;
		keep_max(max_fr_id, frame_ID);
		keep_max(max_pt_id, feat_ID);
	}

	// Insert N copies of the same values:
	frame_poses.assign(max_fr_id+1, CPose3D(0,0,0,0,0,0) );
	landmark_points.assign(max_pt_id+1, TPoint3D(0,0,1) );

	MRPT_END
}

// This function is what to do for each feature in the reprojection loops below.
// -> residual: the raw residual, even if using robust kernel
// -> sum+= scaled squared norm of the residual (which != squared norm if using robust kernel)
template <bool POSES_INVERSE>
inline void reprojectionResidualsElement(
	const TCamera  & camera_params,
	const TFeatureObservation & OBS,
	CArray<double,2>  & out_residual,
	const TFramePosesVec::value_type        & frame,
	const TLandmarkLocationsVec::value_type & point,
	double &sum,
	const bool  use_robust_kernel,
	const double kernel_param,
	double * out_kernel_1st_deriv
	)
{
	const TPixelCoordf  z_pred = mrpt::vision::pinhole::projectPoint_no_distortion<POSES_INVERSE>(camera_params, frame, point);
	const TPixelCoordf &z_meas = OBS.px;

	out_residual[0] = z_meas.x-z_pred.x;
	out_residual[1] = z_meas.y-z_pred.y;
	
	const double sum_2= square(out_residual[0])+square(out_residual[1]);

	if (use_robust_kernel)
	{
		RobustKernel<rkPseudoHuber> kernel;
		kernel.param_sq = square(kernel_param);
		double kernel_1st_deriv, kernel_2nd_deriv;

		sum += kernel.eval(sum_2, kernel_1st_deriv,kernel_2nd_deriv);
		if (out_kernel_1st_deriv) *out_kernel_1st_deriv = kernel_1st_deriv;
	}
	else
	{
		sum += sum_2;
	}
}


/** Compute reprojection error vector (used from within Bundle Adjustment methods, but can be used in general)
  *  See mrpt::vision::bundle_adj_full for a description of most parameters.
  *
  *  \return Overall squared reprojection error.
  */
double mrpt::vision::reprojectionResiduals(
	const TSequenceFeatureObservations   & observations,
	const TCamera                        & camera_params,
	const TFramePosesMap                 & frame_poses,
	const TLandmarkLocationsMap          & landmark_points,
	std::vector<CArray<double,2> > & out_residuals,
	const bool  frame_poses_are_inverse,
	const bool  use_robust_kernel,
	const double kernel_param,
	std::vector<double> * out_kernel_1st_deriv
	)
{
	MRPT_START

	double sum = 0;

	const size_t N = observations.size();
	out_residuals.resize(N);
	if (out_kernel_1st_deriv) out_kernel_1st_deriv->resize(N);

	for (size_t i=0;i<N;i++)
	{
		const TFeatureObservation & OBS = observations[i];

		const TFeatureID     i_p  = OBS.id_feature;
		const TCameraPoseID  i_f  = OBS.id_frame;

		TFramePosesMap::const_iterator        itF = frame_poses.find(i_f);
		TLandmarkLocationsMap::const_iterator itP = landmark_points.find(i_p);
		ASSERTMSG_(itF!=frame_poses.end(), "Frame ID is not in list!")
		ASSERTMSG_(itP!=landmark_points.end(), "Landmark ID is not in list!")

		const TFramePosesMap::mapped_type        & frame = itF->second;
		const TLandmarkLocationsMap::mapped_type & point = itP->second;

		double *ptr_1st_deriv = out_kernel_1st_deriv ? &((*out_kernel_1st_deriv)[i]) : NULL;

		if (frame_poses_are_inverse)
			reprojectionResidualsElement<true>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param,ptr_1st_deriv);
		else
			reprojectionResidualsElement<false>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param,ptr_1st_deriv);
	}

	return sum;

	MRPT_END
}

double mrpt::vision::reprojectionResiduals(
	const TSequenceFeatureObservations   & observations,
	const TCamera                        & camera_params,
	const TFramePosesVec                 & frame_poses,
	const TLandmarkLocationsVec          & landmark_points,
	std::vector<CArray<double,2> > & out_residuals,
	const bool  frame_poses_are_inverse,
	const bool  use_robust_kernel,
	const double kernel_param,
	std::vector<double> * out_kernel_1st_deriv
	)
{
	MRPT_START

	double sum = 0;

	const size_t N = observations.size();
	out_residuals.resize(N);
	if (out_kernel_1st_deriv) out_kernel_1st_deriv->resize(N);

	for (size_t i=0;i<N;i++)
	{
		const TFeatureObservation & OBS = observations[i];

		const TFeatureID     i_p  = OBS.id_feature;
		const TCameraPoseID  i_f  = OBS.id_frame;

		ASSERT_BELOW_(i_p,landmark_points.size())
		ASSERT_BELOW_(i_f,frame_poses.size())

		const TFramePosesVec::value_type        & frame = frame_poses[i_f];
		const TLandmarkLocationsVec::value_type & point = landmark_points[i_p];

		double *ptr_1st_deriv = out_kernel_1st_deriv ? &((*out_kernel_1st_deriv)[i]) : NULL;

		if (frame_poses_are_inverse)
			reprojectionResidualsElement<true>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param,ptr_1st_deriv);
		else
			reprojectionResidualsElement<false>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param,ptr_1st_deriv);
	}

	return sum;
	MRPT_END
}


// Compute gradients & Hessian blocks
void mrpt::vision::ba_build_gradient_Hessians(
	const TSequenceFeatureObservations          & observations,
	const vector<CArray<double,2> >              & residual_vec,
	const mrpt::aligned_containers<JacData<6,3,2> >::vector_t & jac_data_vec,
	mrpt::aligned_containers<CMatrixFixedNumeric<double,6,6> >::vector_t  & U,
	mrpt::aligned_containers<CArrayDouble<6> >::vector_t & eps_frame,
	mrpt::aligned_containers<CMatrixFixedNumeric<double,3,3> >::vector_t & V,
	mrpt::aligned_containers<CArrayDouble<3> >::vector_t & eps_point,
	const size_t                                  num_fix_frames,
	const size_t                                  num_fix_points,
	const vector<double>   * kernel_1st_deriv
	)
{
	MRPT_START

	const size_t N = observations.size();
	const bool use_robust_kernel = (kernel_1st_deriv!=NULL);

	for (size_t i=0;i<N;i++)
	{
		const TFeatureObservation & OBS = observations[i];

		const TFeatureID     i_p  = OBS.id_feature;
		const TCameraPoseID  i_f  = OBS.id_frame;

		const Eigen::Matrix<double,2,1> RESID( &residual_vec[i][0] );
		const JacData<6,3,2>  & JACOB = jac_data_vec[i];
		ASSERTDEB_(JACOB.frame_id==i_f && JACOB.point_id==i_p)

		if (i_f >= num_fix_frames)
		{
			const size_t frame_id = i_f - num_fix_frames;
			ASSERTDEB_(JACOB.J_frame_valid)
			ASSERT_BELOW_(frame_id,U.size())

			CMatrixDouble66 JtJ(UNINITIALIZED_MATRIX);
			JtJ.multiply_AtA(JACOB.J_frame);

			CArrayDouble<6> eps_delta;
			JACOB.J_frame.multiply_Atb(RESID, eps_delta); // eps_delta = J^t * RESID
			if (!use_robust_kernel)
			{
				eps_frame[frame_id] += eps_delta;
			}
			else  
			{
				const double rho_1st_der = (*kernel_1st_deriv)[i];
				eps_frame[frame_id] += eps_delta * rho_1st_der;
			}
			U[frame_id]+=JtJ;
		}
		if (i_p >= num_fix_points)
		{
			const size_t point_id = i_p - num_fix_points;
			ASSERTDEB_(JACOB.J_point_valid)
			ASSERT_BELOW_(point_id,V.size())

			CMatrixDouble33 JtJ(UNINITIALIZED_MATRIX);
			JtJ.multiply_AtA(JACOB.J_point);

			CArrayDouble<3> eps_delta;
			JACOB.J_point.multiply_Atb(RESID, eps_delta); // eps_delta = J^t * RESID
			if (!use_robust_kernel)
			{
				eps_point[point_id] += eps_delta;
			}
			else  
			{
				const double rho_1st_der = (*kernel_1st_deriv)[i];
				eps_point[point_id] += eps_delta  * rho_1st_der;
			}
				
			V[point_id]+=JtJ;
		}
	}

	MRPT_END
}

void mrpt::vision::add_se3_deltas_to_frames(
    const TFramePosesVec  & frame_poses,
    const CVectorDouble &delta,
    const size_t         delta_first_idx,
    const size_t         delta_num_vals,
    TFramePosesVec      & new_frame_poses,
	const size_t         num_fix_frames  )
{
	MRPT_START

	new_frame_poses.resize(frame_poses.size());

	for (size_t i=0;i<num_fix_frames;++i)
		new_frame_poses[i] = frame_poses[i];

	size_t delta_used_vals = 0;
	const double *delta_val = &delta[delta_first_idx];

	for (size_t i=num_fix_frames;i<frame_poses.size();i++)
	{
		const CPose3D &old_pose = frame_poses[i];
		CPose3D       &new_pose = new_frame_poses[i];

		// Use the Lie Algebra methods for the increment:
		const CArrayDouble<6> incr(delta_val);
		const CPose3D         incrPose = CPose3D::exp(incr);

		//new_pose =  old_pose  [+] delta
		//         = exp(delta) (+) old_pose
		new_pose.composeFrom(incrPose, old_pose);

		// Move to the next entry in delta:
		delta_val+=6;
		delta_used_vals+=6;
	}

	ASSERT_(delta_used_vals==delta_num_vals)

	MRPT_END
}
void mrpt::vision::add_3d_deltas_to_points(
    const TLandmarkLocationsVec  & landmark_points,
    const CVectorDouble       & delta,
    const size_t                delta_first_idx,
    const size_t                delta_num_vals,
    TLandmarkLocationsVec     & new_landmark_points,
	const size_t                num_fix_points )
{
	MRPT_START

	new_landmark_points.resize(landmark_points.size());

	for (size_t i=0;i<num_fix_points;++i)
		new_landmark_points[i] = landmark_points[i];

	size_t delta_used_vals = 0;
	const double *delta_val = &delta[delta_first_idx];

	for (size_t i=num_fix_points;i<landmark_points.size();i++)
	{
		const TPoint3D &old_point = landmark_points[i];
		TPoint3D       &new_point = new_landmark_points[i];

		for (size_t j=0;j<3;j++)
			new_point[j] = old_point[j] + delta_val[j];

		// Move to the next entry in delta:
		delta_val+=3;
		delta_used_vals+=3;
	}

	ASSERT_(delta_used_vals==delta_num_vals)

	MRPT_END
}


