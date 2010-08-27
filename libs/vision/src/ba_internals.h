/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
		};


		/** The projective camera 2x6 Jacobian \f$ \frac{\partial h(f,p)}{\partial p} \f$ (wrt the 6D camera pose)
		  * \note Jacobians as described in Ethan Eade's Phd thesis: http://mi.eng.cam.ac.uk/~ee231/thesis_revised.pdf (Appendix A)
		  */
		template <bool POSES_ARE_INVERSE>
		void frameJac(
			   const TCamera  & camera_params,
			   const CPose3D  & cam_pose,
			   const TPoint3D & landmark_global,
			   CMatrixFixedNumeric<double,2,6> & out_J)
		{
			// JL says: This BA implementation assumes that we're estimating the **INVERSE** camera poses
			double x,y,z; // wrt cam (local coords)
			if (POSES_ARE_INVERSE)
				cam_pose.composePoint(
					   landmark_global.x,landmark_global.y,landmark_global.z,
					   x,y,z);
			else
				cam_pose.inverseComposePoint(
					   landmark_global.x,landmark_global.y,landmark_global.z,
					   x,y,z);

			const double z_2 = square(z);

			ASSERT_(z!=0)

			CMatrixDouble22 J_intrinsic;
			J_intrinsic(0,0) = camera_params.intrinsicParams(0,0);
			J_intrinsic(1,1) = camera_params.intrinsicParams(1,1);

			if (POSES_ARE_INVERSE)
			{
				const double J_frame_vals[] = {
					   1./z, 0   , -x/z_2, -x*y/z_2, 1+(square(x)/z_2), -y/z,
					   0   , 1./z, -y/z_2, -(1+square(y)/z_2), x*y/z_2, x/z  };
				const CMatrixFixedNumeric<double,2,6> J_frame(J_frame_vals);
				out_J.multiply_AB(J_intrinsic, J_frame);
			}
			else
			{
				const double J_frame_vals[] = {
					   -1./z, 0   , x/z_2, x*y/z_2, -1-(square(x)/z_2), y/z,
					   0   , -1./z, y/z_2, (1+square(y)/z_2), -x*y/z_2, -x/z  };
				const CMatrixFixedNumeric<double,2,6> J_frame(J_frame_vals);
				out_J.multiply_AB(J_intrinsic, J_frame);
			}
		}

		/**
		* Jacobians as described in Ethan Eade's Phd thesis:
		* http://mi.eng.cam.ac.uk/~ee231/thesis_revised.pdf , Appendix A
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

			const double z_2 = square(l.z);

			ASSERT_(l.z!=0)
			const double tmp_vals[] = {
				1.0/l.z, 0    , -l.x/z_2,
				0    , 1.0/l.z, -l.y/z_2 };
			const CMatrixFixedNumeric<double,2,3> tmp(tmp_vals);

			CMatrixDouble22 J_intrinsic;
			J_intrinsic(0,0) = camera_params.intrinsicParams(0,0);
			J_intrinsic(1,1) = camera_params.intrinsicParams(1,1);

			// RET: cam_pars.jacobian() * Jx
			//    = cam_pars.jacobian() * tmp * T.get_rotation();
			//out_J.multiply_AB(J_intrinsic, J_x);
			out_J.multiply_ABC(J_intrinsic,tmp, dp_point);
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
			vector<JacData<6,3,2> >      & jac_data_vec,
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
			const vector<CArrayDouble<2> >              & residual_vec,
			const vector<JacData<6,3,2> >               & jac_data_vec,
			vector<CMatrixFixedNumeric<double,6,6> >    & U,
			vector<CArrayDouble<6> >                    & eps_frame,
			vector<CMatrixFixedNumeric<double,3,3> >    & V,
			vector<CArrayDouble<3> >                    & eps_point,
			const size_t                                  num_fix_frames,
			const size_t                                  num_fix_points );



		// new_val = val + delta[idx:(idx+num)]
		void ba_addToFrames(
			const TFramePosesVec & frame_poses,
			const vector_double &delta,
			const size_t         delta_first_idx,
			const size_t         delta_num_vals,
			TFramePosesVec       & new_frame_poses,
			const size_t         num_fix_frames,
			const size_t         num_fix_points );

		// new_val = val + delta[idx:(idx+num)]
		void ba_addToPoints(
			const TLandmarkLocationsVec & landmark_points,
			const vector_double       & delta,
			const size_t                delta_first_idx,
			const size_t                delta_num_vals,
			TLandmarkLocationsVec        & new_landmark_points,
			const size_t                num_fix_frames,
			const size_t                num_fix_points );

	}
}

#endif
