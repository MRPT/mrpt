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
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/math/CSparseMatrix.h>

#include <memory>  // std::auto_ptr

#include "ba_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;

using mrpt::aligned_containers;


MRPT_TODO("Check valid Hessians when enabling robustifying kernel")

// Define -> estimate the inverse of poses; Undefined -> estimate the camera poses (read the doc of mrpt::vision::bundle_adj_full)
#define USE_INVERSE_POSES

#ifdef USE_INVERSE_POSES
#	define INV_POSES_BOOL  true
#else
#	define INV_POSES_BOOL  false
#endif

/* ----------------------------------------------------------
                    bundle_adj_full

	See bundle_adjustment.h for docs.

  This implementation is almost entirely an adapted version from
  RobotVision (at OpenSLAM.org) (C) by Hauke Strasdat (Imperial
  College London), licensed under GNU LGPL.
  See the related paper:  H. Strasdat, J.M.M. Montiel,
  A.J. Davison: "Scale Drift-Aware Large Scale Monocular SLAM",
  RSS2010, http://www.roboticsproceedings.org/rss06/p10.html

   ---------------------------------------------------------- */
double mrpt::vision::bundle_adj_full(
	const TSequenceFeatureObservations   & observations,
	const TCamera                        & camera_params,
	TFramePosesVec                       & frame_poses,
	TLandmarkLocationsVec                & landmark_points,
	const mrpt::utils::TParametersDouble & extra_params,
	const TBundleAdjustmentFeedbackFunctor user_feedback )
{
	MRPT_START

	// Generic BA problem dimension numbers:
	static const unsigned int FrameDof = 6; // Poses: x y z yaw pitch roll
	static const unsigned int PointDof = 3; // Landmarks: x y z
	static const unsigned int ObsDim   = 2; // Obs: x y (pixels)

	// Typedefs for this specific BA problem:
	typedef JacData<FrameDof,PointDof,ObsDim>        MyJacData;
	typedef aligned_containers<MyJacData>::vector_t  MyJacDataVec;

	typedef CArray<double,ObsDim>              Array_O;
	typedef CArrayDouble<FrameDof>             Array_F;
	typedef CArrayDouble<PointDof>             Array_P;
	typedef CMatrixFixedNumeric<double,FrameDof,FrameDof> Matrix_FxF;
	typedef CMatrixFixedNumeric<double,PointDof,PointDof> Matrix_PxP;
	typedef CMatrixFixedNumeric<double,FrameDof,PointDof> Matrix_FxP;

	// Extra params:
	const bool use_robust_kernel  = 0!=extra_params.getWithDefaultVal("robust_kernel",1);
	const bool verbose            = 0!=extra_params.getWithDefaultVal("verbose",0);
	const double initial_mu       = extra_params.getWithDefaultVal("mu",-1);
	const size_t max_iters        = extra_params.getWithDefaultVal("max_iterations",50);
	const size_t num_fix_frames   = extra_params.getWithDefaultVal("num_fix_frames",1);
	const size_t num_fix_points   = extra_params.getWithDefaultVal("num_fix_points",0);
	const double kernel_param     = extra_params.getWithDefaultVal("kernel_param",3.0);

	const bool   enable_profiler  = 0!=extra_params.getWithDefaultVal("profiler",0);

	mrpt::utils::CTimeLogger  profiler(enable_profiler);

	profiler.enter("bundle_adj_full (complete run)");

	// Input data sizes:
	const size_t num_points = landmark_points.size();
	const size_t num_frames = frame_poses.size();
	const size_t num_obs    = observations.size();


	ASSERT_ABOVE_(num_frames,0)
	ASSERT_ABOVE_(num_points,0)
	ASSERT_(num_fix_frames>=1)
	ASSERT_ABOVEEQ_(num_frames,num_fix_frames);
	ASSERT_ABOVEEQ_(num_points,num_fix_points);

#ifdef USE_INVERSE_POSES
	// *Warning*: This implementation assumes inverse camera poses: inverse them at the entrance and at exit:
	profiler.enter("invert_poses");
	for (size_t i=0;i<num_frames;i++)
		frame_poses[i].inverse();
	profiler.leave("invert_poses");
#endif


	MyJacDataVec     jac_data_vec(num_obs);
	vector<Array_O>  residual_vec(num_obs);

	// prepare structure of sparse Jacobians:
	for (size_t i=0; i<num_obs; i++)
	{
		jac_data_vec[i].frame_id = observations[i].id_frame;
		jac_data_vec[i].point_id = observations[i].id_feature;
	}

	// Compute sparse Jacobians:
	profiler.enter("compute_Jacobians");
	ba_compute_Jacobians<INV_POSES_BOOL>(frame_poses, landmark_points, camera_params, jac_data_vec, num_fix_frames, num_fix_points);
	profiler.leave("compute_Jacobians");


	profiler.enter("reprojectionResiduals");
	double res = mrpt::vision::reprojectionResiduals(
					 observations, camera_params, frame_poses, landmark_points,
					 residual_vec,
					 INV_POSES_BOOL, // are poses inverse?
					 use_robust_kernel,
					 kernel_param );
	profiler.leave("reprojectionResiduals");

	MRPT_CHECK_NORMAL_NUMBER(res)

	VERBOSE_COUT << "res: " << res << endl;

	// Auxiliary vars:
	Array_F arrF_zeros;
	arrF_zeros.assign(0);
	Array_P arrP_zeros;
	arrP_zeros.assign(0);

	const size_t num_free_frames = num_frames-num_fix_frames;
	const size_t num_free_points = num_points-num_fix_points;
	const size_t len_free_frames = FrameDof * num_free_frames;
	const size_t len_free_points = PointDof * num_free_points;

	aligned_containers<Matrix_FxF>::vector_t    U         (num_free_frames);
	aligned_containers<Array_F>::vector_t       eps_frame (num_free_frames, arrF_zeros);
	aligned_containers<Matrix_PxP>::vector_t    V         (num_free_points);
	aligned_containers<Array_P>::vector_t       eps_point (num_free_points, arrP_zeros);

	profiler.enter("calcUVeps");
	ba_calcUVeps(observations,residual_vec,jac_data_vec, U,eps_frame,V,eps_point, num_fix_frames, num_fix_points );
	profiler.leave("calcUVeps");

	double nu = 2;
	double eps = 1e-16; // 0.000000000000001;
	bool   stop = false;
	double mu = initial_mu;

	// Automatic guess of "mu":
	if (mu<0)
	{
		double norm_max_A = 0;
		for (size_t j=0; j<num_free_frames; ++j)
			for (size_t dim=0; dim<FrameDof; dim++)
				keep_max(norm_max_A, U[j](dim,dim) );

		for (size_t i=0; i<num_free_points; ++i)
			for (size_t dim=0; dim<PointDof; dim++)
				keep_max(norm_max_A, V[i](dim,dim) );
		double tau = 1e-3;
		mu = tau*norm_max_A;
	}

	Matrix_FxF   I_muFrame(UNINITIALIZED_MATRIX);
	Matrix_PxP   I_muPoint(UNINITIALIZED_MATRIX);

	// Cholesky object, as a pointer to reuse it between iterations:
	std::auto_ptr<CSparseMatrix::CholeskyDecomp>  ptrCh; //  Ch(sS);


	for (size_t iter=0; iter<max_iters; iter++)
	{
		VERBOSE_COUT << "iteration: "<< iter << endl;

		// provide feedback to the user:
		if (user_feedback)
			(*user_feedback)(iter, res, max_iters, observations, frame_poses, landmark_points );

		double rho = 0;
		do
		{
			profiler.enter("COMPLETE_ITER");

			VERBOSE_COUT << "mu: " <<mu<< endl;

			I_muFrame.unit(FrameDof,mu);
			I_muPoint.unit(PointDof,mu);

			aligned_containers<Matrix_FxF>::vector_t   U_star(num_free_frames, I_muFrame);
			aligned_containers<Matrix_PxP>::vector_t   V_inv (num_free_points);

			for (size_t i=0; i<U_star.size(); ++i)
				U_star[i] += U[i];

			for (size_t i=0; i<V.size(); ++i)
				(V[i]+I_muPoint).inv_fast( V_inv[i] );


			typedef aligned_containers<pair<TCameraPoseID,TLandmarkID>,Matrix_FxP>::map_t  WMap;
			WMap   W,Y;

			// For quick look-up of entries in W affecting a given point ID:
			vector<vector<WMap::iterator> >   W_entries(num_points);  // Index is "TLandmarkID"

			MyJacDataVec::const_iterator jac_iter = jac_data_vec.begin();
			for (TSequenceFeatureObservations::const_iterator it_obs=observations.begin(); it_obs!=observations.end(); ++it_obs)
			{
				const TFeatureID feat_id     = it_obs->id_feature;
				const TCameraPoseID frame_id = it_obs->id_frame;

				if (jac_iter->J_frame_valid && jac_iter->J_point_valid)
				{
					const CMatrixFixedNumeric<double,ObsDim,FrameDof> & J_frame = jac_iter->J_frame;
					const CMatrixFixedNumeric<double,ObsDim,PointDof> & J_point = jac_iter->J_point;
					const pair<TCameraPoseID,TLandmarkID> id_pair = make_pair(frame_id, feat_id);

					Matrix_FxP tmp(UNINITIALIZED_MATRIX);
					tmp.multiply_AtB(J_frame, J_point);

					// W[ids] = J_f^T * J_p
					// Was: W[id_pair] = tmp;
					const pair<WMap::iterator,bool> &retInsert = W.insert( make_pair(id_pair,tmp) );
					ASSERT_(retInsert.second==true)
					W_entries[feat_id].push_back(retInsert.first); // Keep the iterator

					// Y[ids] = W[ids] * V^{-1}
					Y[id_pair].multiply_AB(tmp, V_inv[feat_id-num_fix_points]);
				}
				++jac_iter;
			}

			aligned_containers<pair<TCameraPoseID,TLandmarkID>,Matrix_FxF>::map_t  YW_map;
			for (size_t i=0; i<U.size(); ++i)
				YW_map[make_pair<TCameraPoseID,TLandmarkID>(i,i)] = U_star[i];

			vector_double  delta( len_free_frames + len_free_points );
			vector_double  e    ( len_free_frames );


			profiler.enter("e");
			for (size_t j=0; j<num_free_frames; ++j)
				::memcpy( &e[j*FrameDof], &eps_frame[j][0], sizeof(e[0])*FrameDof ); // e.slice(j*FrameDof,FrameDof) = AT(eps_frame,j);

			for (WMap::iterator Y_ij=Y.begin(); Y_ij!=Y.end(); ++Y_ij)
			{
				const TLandmarkID  point_id = Y_ij->first.second;
				const size_t i = Y_ij->first.second - num_fix_points; // point index
				const size_t j = Y_ij->first.first  - num_fix_frames; // frame index

				const vector<WMap::iterator> &iters = W_entries[point_id]; //->second;

				for (size_t itIdx=0; itIdx<iters.size(); itIdx++)
					//for (size_t k=0; k<num_free_frames; ++k)
				{
					const WMap::iterator &W_ik = iters[itIdx];
					const TLandmarkID k = W_ik->first.first-num_fix_frames;

					Matrix_FxF  YWt(UNINITIALIZED_MATRIX);		//-(Y_ij->second) * (W_ik->second).T();
					YWt.multiply_ABt( Y_ij->second, W_ik->second );
					//YWt*=-1.0; // The "-" sign is taken into account below:

					const pair<TCameraPoseID,TLandmarkID> ids_jk = make_pair<TCameraPoseID,TLandmarkID>(j,k);

					aligned_containers<pair<TCameraPoseID,TLandmarkID>,Matrix_FxF>::map_t::iterator it = YW_map.find(ids_jk);
					if(it!=YW_map.end())
						it->second -= YWt;  // += (-YWt);
					else
						YW_map[ids_jk] = YWt*(-1.0);
				}

				CArrayDouble<FrameDof>  r;
				Y_ij->second.multiply_Ab( eps_point[i] ,r);
				for (size_t k=0; k<FrameDof; k++)
					e[j*FrameDof+k]-=r[k];
			}
			profiler.leave("e");


			profiler.enter("sS:ALL");
			profiler.enter("sS:fill");

			VERBOSE_COUT << "Entries in YW_map:" << YW_map.size() << endl;

			CSparseMatrix sS(len_free_frames, len_free_frames);

			for (aligned_containers<pair<TCameraPoseID,TLandmarkID>,Matrix_FxF>::map_t::const_iterator it= YW_map.begin(); it!=YW_map.end(); ++it)
			{
				const pair<TCameraPoseID,TLandmarkID> & ids = it->first;
				const Matrix_FxF & YW = it->second;
				sS.insert_submatrix(ids.first*FrameDof,ids.second*FrameDof, YW);
			}
			profiler.leave("sS:fill");

			// Compress the sparse matrix:
			profiler.enter("sS:compress");
			sS.compressFromTriplet();
			profiler.leave("sS:compress");

			try
			{
				profiler.enter("sS:chol");
				if (!ptrCh.get())
						ptrCh = std::auto_ptr<CSparseMatrix::CholeskyDecomp>(new CSparseMatrix::CholeskyDecomp(sS) );
				else ptrCh.get()->update(sS);
				profiler.leave("sS:chol");

				profiler.enter("sS:backsub");
				vector_double  bck_res;
				ptrCh->backsub(e,bck_res);
				::memcpy(&delta[0],&bck_res[0],bck_res.size()*sizeof(bck_res[0]));	// delta.slice(0,...) = Ch.backsub(e);
				profiler.leave("sS:backsub");
				profiler.leave("sS:ALL");
			}
			catch (CExceptionNotDefPos &)
			{
				profiler.leave("sS:ALL");
				// not positive definite so increase mu and try again
				mu *= nu;
				nu *= 2.;
				stop = (mu>999999999.f);
				continue;
			}

			//VERBOSE_COUT << "delta: " << delta << endl;

			profiler.enter("g");

			vector_double g(len_free_frames+len_free_points);
			::memcpy(&g[0],&e[0],len_free_frames*sizeof(g[0])); //g.slice(0,FrameDof*(num_frames-num_fix_frames)) = e;

			for (size_t i=0; i<num_free_points; ++i)
			{
				Array_P tmp = eps_point[i]; // eps_point.slice(PointDof*i,PointDof);

				for (size_t j=0; j<num_free_frames; ++j)
				{
					WMap::iterator W_ij;
					W_ij = W.find(make_pair<TCameraPoseID,TLandmarkID>(j+num_fix_frames,i+num_fix_points));

					if (W_ij!=W.end())
					{
						//tmp -= W_ij->second.T() * delta.slice(j*FrameDof,FrameDof);
						const Array_F  v( &delta[j*FrameDof] );
						Array_P  r;
						W_ij->second.multiply_Atb(v, r); // r= A^t * v
						tmp-=r;
					}
				}
				Array_P Vi_tmp;
				V_inv[i].multiply_Ab(tmp, Vi_tmp); // Vi_tmp = V_inv[i] * tmp

				::memcpy(&delta[len_free_frames + i*PointDof], &Vi_tmp[0], sizeof(Vi_tmp[0])*PointDof ); // delta.slice((num_frames-num_fix_frames)*FrameDof + i*PointDof, PointDof) = V_inv[i] * tmp;
				::memcpy(&g[len_free_frames + i*PointDof], &eps_point[i][0], sizeof(eps_point[0][0])*PointDof ); // g.slice(, PointDof) = eps_point[i]; // .slice(PointDof*i,PointDof);
			}
			profiler.leave("g");


			// Vars for temptative new estimates:
			TFramePosesVec        new_frame_poses;
			TLandmarkLocationsVec new_landmark_points;

			profiler.enter("add_se3_deltas_to_frames");
			add_se3_deltas_to_frames(
				frame_poses,
				delta, 0,len_free_frames,
				new_frame_poses, num_fix_frames );
			profiler.leave("add_se3_deltas_to_frames");

			profiler.enter("add_3d_deltas_to_points");
			add_3d_deltas_to_points(
				landmark_points,
				delta, len_free_frames, len_free_points,
				new_landmark_points, num_fix_points );
			profiler.leave("add_3d_deltas_to_points");


			profiler.enter("reprojectionResiduals");
			double res_new = mrpt::vision::reprojectionResiduals(
								 observations, camera_params,
								 new_frame_poses, new_landmark_points,
								 residual_vec,
								 INV_POSES_BOOL, // are poses inverse?
								 use_robust_kernel );
			profiler.leave("reprojectionResiduals");

			MRPT_CHECK_NORMAL_NUMBER(res_new)

			rho = (res-res_new)/ (delta.array()*(mu*delta + g).array() ).sum();

			if(rho>0)
			{
				// Good: Accept new values
				VERBOSE_COUT << "res_new: " << res_new << ", avr. err in px=" <<  sqrt(res_new)/num_obs <<  " rho: " << rho << endl;

				// swap is faster than "="
				frame_poses.swap(new_frame_poses);
				landmark_points.swap(new_landmark_points);

				res = res_new;

				profiler.enter("compute_Jacobians");
				ba_compute_Jacobians<INV_POSES_BOOL>(frame_poses, landmark_points, camera_params, jac_data_vec, num_fix_frames, num_fix_points);
				profiler.leave("compute_Jacobians");


				// Reset to zeros:
				U.assign(num_free_frames,Matrix_FxF() );
				V.assign(num_free_points,Matrix_PxP() );
				eps_frame.assign(num_free_frames, arrF_zeros);
				eps_point.assign(num_free_points, arrP_zeros);

				profiler.enter("calcUVeps");
				ba_calcUVeps(observations,residual_vec,jac_data_vec,U,eps_frame,V,eps_point, num_fix_frames, num_fix_points );
				profiler.leave("calcUVeps");

				stop = norm_inf(g)<=eps;
				mu *= max(1.0/3.0, 1-std::pow(2*rho-1,3.0) );
				nu = 2.0;
			}
			else
			{
				VERBOSE_COUT << "no update: res vs.res_new "
				<< res
				<< " vs. "
				<< res_new
				<< endl;
				mu *= nu;
				nu *= 2.0;
				stop = (mu>1e9);
			}

			profiler.leave("COMPLETE_ITER");
		}
		while(rho<=0 && !stop);

		if (stop)
			break;
	}

	// *Warning*: This implementation assumes inverse camera poses: inverse them at the entrance and at exit:
#ifdef USE_INVERSE_POSES
	profiler.enter("invert_poses");
	for (size_t i=0;i<num_frames;i++)
		frame_poses[i].inverse();
	profiler.leave("invert_poses");
#endif

	profiler.leave("bundle_adj_full (complete run)");

	return res;
	MRPT_END
}
