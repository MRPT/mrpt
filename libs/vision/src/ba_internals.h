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

#ifndef ba_internals_H
#define ba_internals_H

#include <mrpt/math/CMatrixFixedNumeric.h>

// Declarations shared between ba_*.cpp files, but which are private to MRPT
//  not to be seen by an MRPT API user.

namespace mrpt
{
	namespace vision
	{
		using mrpt::math::CArrayDouble;
		using mrpt::math::CMatrixFixedNumeric;
		using std::vector;

		#define VERBOSE_COUT  if (verbose) cout

		// Auxiliary struct for keeping the list of all Jacobians in a sparse, efficient way.
		template <int FrameDof, int PointDof, int ObsDim>
		struct JacData
		{
			inline JacData() :
				J_frame(mrpt::math::UNINITIALIZED_MATRIX),
				J_point(mrpt::math::UNINITIALIZED_MATRIX),
				J_frame_valid(false),
				J_point_valid(false)
			{}

			TCameraPoseID  frame_id;
			TLandmarkID    point_id;

			// Jacobians of the observation wrt the camera pose & the point:
			CMatrixFixedNumeric<double,ObsDim,FrameDof>  J_frame;
			CMatrixFixedNumeric<double,ObsDim,PointDof>  J_point;
			bool J_frame_valid, J_point_valid;

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Needed by any struct having Eigen::Matrix<> fields
		};


		/** The projective camera 2x6 Jacobian \f$ \frac{\partial h(f,p)}{\partial p} \f$ (wrt the 6D camera pose)
		  * \note Jacobians as described in http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty  (Appendix A)
		  */
		template <bool POSES_ARE_INVERSE>
		void frameJac(
			   const TCamera  & camera_params,
			   const CPose3D  & cam_pose,
			   const TPoint3D & landmark_global,
			   CMatrixFixedNumeric<double,2,6> & out_J)
		{
			double x,y,z; // wrt cam (local coords)
			if (POSES_ARE_INVERSE)
				cam_pose.composePoint(
					   landmark_global.x,landmark_global.y,landmark_global.z,
					   x,y,z);
			else
				cam_pose.inverseComposePoint(
					   landmark_global.x,landmark_global.y,landmark_global.z,
					   x,y,z);

			ASSERT_(z!=0)

			const double _z = 1.0/z;
			const double fx_z = camera_params.fx() *_z;
			const double fy_z = camera_params.fy() *_z;
			const double _z2 = square(_z);
			const double fx_z2 = camera_params.fx()*_z2;
			const double fy_z2 = camera_params.fy()*_z2;

			if (POSES_ARE_INVERSE)
			{
				const double xy = x*y;

				const double J_vals[] = {
					fx_z,	0,		-fx_z2*x,	-fx_z2*xy, 								camera_params.fx()*(1+square(x*_z)),	-fx_z*y,
					0,		fy_z,	-fy_z2*y,	-camera_params.fy()*(1+square(y*_z)),	fy_z2*xy, 	fy_z*x };
				out_J.loadFromArray(J_vals);
			}
			else
			{
				const mrpt::math::CMatrixDouble33 & R = cam_pose.getRotationMatrix();

				const double jac_proj_vals[] = {
					fx_z,	0,		-fx_z2*x,
					0,		fy_z,	-fy_z2*y };
				const mrpt::math::CMatrixFixedNumeric<double,2,3>  jac_proj(jac_proj_vals);

				const double p_x = cam_pose.x();
				const double p_y = cam_pose.y();
				const double p_z = cam_pose.z();
				const double tx = landmark_global.x - p_x;
				const double ty = landmark_global.y - p_y;
				const double tz = landmark_global.z - p_z;

				const double aux_vals[] = {
					-R(0,0), -R(1,0), -R(2,0), tz * R(1,0) - ty * R(2,0) + R(1,0) * p_z - R(2,0) * p_y, tx * R(2,0) - tz * R(0,0) - R(0,0) * p_z + R(2,0) * p_x, ty * R(0,0) - tx * R(1,0) + R(0,0) * p_y - R(1,0) * p_x,
					-R(0,1), -R(1,1), -R(2,1), tz * R(1,1) - ty * R(2,1) + R(1,1) * p_z - R(2,1) * p_y, tx * R(2,1) - tz * R(0,1) - R(0,1) * p_z + R(2,1) * p_x, ty * R(0,1) - tx * R(1,1) + R(0,1) * p_y - R(1,1) * p_x,
					-R(0,2), -R(1,2), -R(2,2), tz * R(1,2) - ty * R(2,2) + R(1,2) * p_z - R(2,2) * p_y, tx * R(2,2) - tz * R(0,2) - R(0,2) * p_z + R(2,2) * p_x, ty * R(0,2) - tx * R(1,2) + R(0,2) * p_y - R(1,2) * p_x
					};
				const mrpt::math::CMatrixFixedNumeric<double,3,6>  vals(aux_vals);
				out_J.multiply_AB(jac_proj, vals);
			}
		}

		/** Jacobians wrt the point
		  * \note Jacobians as described in http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty  (Appendix A)
		*/
		template <bool POSES_ARE_INVERSE>
		void pointJac(
			const TCamera  & camera_params,
			const CPose3D     & cam_pose,
			const TPoint3D & landmark_global,
			CMatrixFixedNumeric<double,2,3> & out_J )
		{
			TPoint3D l; // Local point, wrt camera

			CMatrixDouble33 dp_point(UNINITIALIZED_MATRIX);

			if (POSES_ARE_INVERSE)
				cam_pose.composePoint(
					landmark_global.x,landmark_global.y,landmark_global.z,
					l.x,l.y,l.z,
					&dp_point );
			else
				cam_pose.inverseComposePoint(
					landmark_global.x,landmark_global.y,landmark_global.z,
					l.x,l.y,l.z,
					&dp_point );

			ASSERT_(l.z!=0)

			const double _z = 1.0/l.z;
			const double _z2 = square(_z);

			const double tmp_vals[] = {
				camera_params.fx()*_z,                     0, -camera_params.fx()*l.x*_z2,
				0                    , camera_params.fy()*_z, -camera_params.fy()*l.y*_z2 };
			const CMatrixFixedNumeric<double,2,3> tmp(tmp_vals);

			out_J.multiply_AB(tmp, dp_point);
		}

		// === Compute sparse Jacobians ====
		// Case: 6D poses + 3D points + 2D (x,y) observations
		// For the case of *inverse* or *normal* frame poses being estimated.
		// Made inline so immediate values in "poses_are_inverses" are propragated by the compiler
		template <bool POSES_ARE_INVERSE>
		void ba_compute_Jacobians(
			const TFramePosesVec         & frame_poses,
			const TLandmarkLocationsVec  & landmark_points,
			const TCamera                & camera_params,
			vector<JacData<6,3,2>,Eigen::aligned_allocator<JacData<6,3,2> > >      & jac_data_vec,
			const size_t                   num_fix_frames,
			const size_t                   num_fix_points)
		{
			MRPT_START

			// num_fix_frames & num_fix_points: Are relative to the order in frame_poses & landmark_points
			ASSERT_(!frame_poses.empty() && !landmark_points.empty())

			const size_t N = jac_data_vec.size();

			for (size_t i=0;i<N;i++)
			{
				JacData<6,3,2> &D = jac_data_vec[i];

				const TCameraPoseID  i_f = D.frame_id;
				const TLandmarkID    i_p = D.point_id;

				ASSERTDEB_(i_f<frame_poses.size())
				ASSERTDEB_(i_p<landmark_points.size())

				if (i_f>=num_fix_frames)
				{
					frameJac<POSES_ARE_INVERSE>(camera_params, frame_poses[i_f], landmark_points[i_p], D.J_frame);
					D.J_frame_valid = true;
				}

				if (i_p>=num_fix_points)
				{
					pointJac<POSES_ARE_INVERSE>(camera_params, frame_poses[i_f], landmark_points[i_p], D.J_point);
					D.J_point_valid = true;
				}
			}
			MRPT_END
		}

		// Compute temporary matrices during BA:
		void ba_calcUVeps(
			const TSequenceFeatureObservations          & observations,
			const vector<CArray<double,2> >              & residual_vec,
			const vector<JacData<6,3,2>, Eigen::aligned_allocator<JacData<6,3,2> > >               & jac_data_vec,
			vector<CMatrixFixedNumeric<double,6,6>, Eigen::aligned_allocator<CMatrixFixedNumeric<double,6,6> > >    & U,
			vector<CArrayDouble<6>, Eigen::aligned_allocator<CArrayDouble<6> > >                    & eps_frame,
			vector<CMatrixFixedNumeric<double,3,3>, Eigen::aligned_allocator<CMatrixFixedNumeric<double,3,3> > >    & V,
			vector<CArrayDouble<3>, Eigen::aligned_allocator<CArrayDouble<3> > >                    & eps_point,
			const size_t                                  num_fix_frames,
			const size_t                                  num_fix_points );


	}
}

#endif
