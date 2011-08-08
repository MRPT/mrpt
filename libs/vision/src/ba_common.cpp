/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/vision/bundle_adjustment.h>
#include <mrpt/vision/pinhole.h>
#include "ba_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;


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


/** pseudo-huber cost function */
inline double kernel(double delta, const double kernel_param)
{
	return std::abs(2*square(kernel_param)*(std::sqrt(1+square(delta/kernel_param))-1));
}

// This function is what to do for each feature in the reprojection loops below.
template <bool POSES_INVERSE>
inline void reprojectionResidualsElement(
	const TCamera  & camera_params,
	const TFeatureObservation & OBS,
	CArray<double,2>  & out_residual,
	const TFramePosesVec::value_type        & frame,
	const TLandmarkLocationsVec::value_type & point,
	double &sum,
	const bool  use_robust_kernel,
	const double kernel_param )
{
	const TPixelCoordf  z_pred = mrpt::vision::pinhole::projectPoint_no_distortion<POSES_INVERSE>(camera_params, frame, point);
	const TPixelCoordf &z_meas = OBS.px;

	CArray<double,2> delta;
	delta[0] = z_meas.x-z_pred.x;
	delta[1] = z_meas.y-z_pred.y;

	const double sum_2= square(delta[0])+square(delta[1]);
	if (use_robust_kernel)
	{
		const double nrm = std::max(1e-11,std::sqrt(sum_2));
		const double w = std::sqrt(kernel(nrm,kernel_param))/nrm;
		delta[0] *= w;
		delta[1] *= w;
		out_residual = delta;
		sum += square(delta[0])+square(delta[1]);
	}
	else
	{
		out_residual = delta;
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
	const double kernel_param
	)
{
	MRPT_START

	double sum = 0;

	const size_t N = observations.size();
	out_residuals.resize(N);

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

		if (frame_poses_are_inverse)
			reprojectionResidualsElement<true>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param);
		else
			reprojectionResidualsElement<false>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param);
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
	const double kernel_param
	)
{
	MRPT_START

	double sum = 0;

	const size_t N = observations.size();
	out_residuals.resize(N);

	for (size_t i=0;i<N;i++)
	{
		const TFeatureObservation & OBS = observations[i];

		const TFeatureID     i_p  = OBS.id_feature;
		const TCameraPoseID  i_f  = OBS.id_frame;

		ASSERT_BELOW_(i_p,landmark_points.size())
		ASSERT_BELOW_(i_f,frame_poses.size())

		const TFramePosesVec::value_type        & frame = frame_poses[i_f];
		const TLandmarkLocationsVec::value_type & point = landmark_points[i_p];

		if (frame_poses_are_inverse)
			reprojectionResidualsElement<true>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param);
		else
			reprojectionResidualsElement<false>(camera_params, OBS, out_residuals[i], frame, point, sum, use_robust_kernel,kernel_param);
	}

	return sum;
	MRPT_END
}


// Compute temporary matrices during BA:
void mrpt::vision::ba_calcUVeps(
	const TSequenceFeatureObservations          & observations,
	const vector<CArray<double,2> >              & residual_vec,
	const vector<JacData<6,3,2>, Eigen::aligned_allocator<JacData<6,3,2> > >               & jac_data_vec,
	vector<CMatrixFixedNumeric<double,6,6>, Eigen::aligned_allocator<CMatrixFixedNumeric<double,6,6> > >    & U,
	vector<CArrayDouble<6>, Eigen::aligned_allocator<CArrayDouble<6> > >                    & eps_frame,
	vector<CMatrixFixedNumeric<double,3,3>, Eigen::aligned_allocator<CMatrixFixedNumeric<double,3,3> > >    & V,
	vector<CArrayDouble<3>, Eigen::aligned_allocator<CArrayDouble<3> > >                    & eps_point,
	const size_t                                  num_fix_frames,
	const size_t                                  num_fix_points )
{
	MRPT_START

	const size_t N = observations.size();

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
			U[frame_id]+=JtJ;

			CArrayDouble<6> eps_delta;
			JACOB.J_frame.multiply_Atb(RESID, eps_delta); // eps_delta = J^t * RESID
			eps_frame[frame_id] += eps_delta;
		}
		if (i_p >= num_fix_points)
		{
			const size_t point_id = i_p - num_fix_points;
			ASSERTDEB_(JACOB.J_point_valid)
			ASSERT_BELOW_(point_id,V.size())

			CMatrixDouble33 JtJ(UNINITIALIZED_MATRIX);
			JtJ.multiply_AtA(JACOB.J_point);
			V[point_id]+=JtJ;

			CArrayDouble<3> eps_delta;
			JACOB.J_point.multiply_Atb(RESID, eps_delta); // eps_delta = J^t * RESID
			eps_point[point_id] += eps_delta;
		}
	}

	MRPT_END
}

void mrpt::vision::add_se3_deltas_to_frames(
    const TFramePosesVec  & frame_poses,
    const vector_double &delta,
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
    const vector_double       & delta,
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


