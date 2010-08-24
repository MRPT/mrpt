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


		// === Compute sparse Jacobians ====
		// Case: 6D poses + 3D points + 2D (x,y) observations
		// For the case of *inverse* frame poses being estimated.
		void ba_compute_Jacobians(
			const TFramePosesVec         & frame_poses,
			const TLandmarkLocationsVec  & landmark_points,
			const TCamera                & camera_params,
			vector<JacData<6,3,2> >      & jac_data_vec,
			const size_t                   num_fix_frames,
			const size_t                   num_fix_points );

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
