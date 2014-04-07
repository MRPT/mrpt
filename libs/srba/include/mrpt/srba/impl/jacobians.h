/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/jacobians.h>
#include <mrpt/poses/SE_traits.h>
#include <mrpt/srba/landmark_jacob_families.h>

namespace mrpt { namespace srba {

#ifndef SRBA_USE_NUMERIC_JACOBIANS
#  define SRBA_USE_NUMERIC_JACOBIANS             0
#endif
#ifndef SRBA_VERIFY_AGAINST_NUMERIC_JACOBIANS
#  define SRBA_VERIFY_AGAINST_NUMERIC_JACOBIANS  0
#endif

#define SRBA_COMPUTE_NUMERIC_JACOBIANS   (SRBA_VERIFY_AGAINST_NUMERIC_JACOBIANS || SRBA_USE_NUMERIC_JACOBIANS)
#define SRBA_COMPUTE_ANALYTIC_JACOBIANS  (SRBA_VERIFY_AGAINST_NUMERIC_JACOBIANS || !SRBA_USE_NUMERIC_JACOBIANS)

#define DEBUG_JACOBIANS_SUPER_VERBOSE  0
#define DEBUG_NOT_UPDATED_ENTRIES      0   // Extremely slow, just for debug during development! This checks that all the expected Jacobians are actually updated


namespace internal {
    /** Auxiliary template for evaluating the dh_df part in \a recompute_all_Jacobians().
    // The extra complexity of adding this auxiliary template with specializations is required to avoid
    //  the compiler trying to evaluate the jacobians dh_df in relative SLAM problems, where the Jacobian does not exist. */
    template <landmark_jacob_family_t LM_JACOB_FAMILY>
    struct recompute_all_Jacobians_dh_df;

    // Specialization for "normal SLAM" (SLAM with real landmarks)
    template <> struct recompute_all_Jacobians_dh_df<jacob_point_landmark> {
        template <class RBAENGINE,class LSTJACOBCOLS,class LSTPOSES>
        static size_t eval(
            RBAENGINE &rba,
            LSTJACOBCOLS  &lst_JacobCols_df,  // std::vector<typename RBAENGINE::TSparseBlocksJacobians_dh_df::col_t*>
            LSTPOSES * out_list_of_required_num_poses ) // std::vector<const typename RBAENGINE::kf2kf_pose_traits<RBAENGINE::KF2KF_POSE_TYPE>::pose_flag_t*>
        {
            const size_t nUnknowns_k2f = lst_JacobCols_df.size();
            size_t nJacobs = 0;
            for (size_t i=0;i<nUnknowns_k2f;i++)
            {
                // For each column, process each nonzero block:
                typename RBAENGINE::TSparseBlocksJacobians_dh_df::col_t *col = lst_JacobCols_df[i];

                for (typename RBAENGINE::TSparseBlocksJacobians_dh_df::col_t::iterator it=col->begin();it!=col->end();++it)
                {
                    const size_t obs_idx = it->first;
                    typename RBAENGINE::TSparseBlocksJacobians_dh_df::TEntry & jacob_entry = it->second;
                    rba.compute_jacobian_dh_df(
                        jacob_entry,
        #ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
                        *
        #endif
                        rba.get_rba_state().all_observations[obs_idx],
                        out_list_of_required_num_poses );
                    nJacobs++;
                }
            }
            return nJacobs;
        }

    };

    // Specialization for relative graph-SLAM (no real landmarks)
    template <> struct recompute_all_Jacobians_dh_df<jacob_relpose_landmark> {
        template <class RBAENGINE,class LSTJACOBCOLS,class LSTPOSES>
        static size_t eval(
            RBAENGINE &rba,
            LSTJACOBCOLS  &lst_JacobCols_df,  // std::vector<typename RBAENGINE::TSparseBlocksJacobians_dh_df::col_t*>
            LSTPOSES * out_list_of_required_num_poses ) // std::vector<const typename RBAENGINE::kf2kf_pose_traits<RBAENGINE::KF2KF_POSE_TYPE>::pose_flag_t*>
        {
            // Nothing to do: this will never be actually called.
            return 0;
        }
    };
} // end "internal" ns


/** Numeric implementation of the partial Jacobian dh_dAp */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::numeric_dh_dAp(const array_pose_t &x, const TNumeric_dh_dAp_params& params, array_obs_t &y)
{
	pose_t incr(mrpt::poses::UNINITIALIZED_POSE);
	mrpt::poses::SE_traits<pose_t::rotation_dimensions>::pseudo_exp(x,incr);

	pose_t base_from_obs(mrpt::poses::UNINITIALIZED_POSE);
	if (!params.is_inverse_dir)
	{
		if (params.pose_d1_wrt_obs) // "A" in papers
				base_from_obs.composeFrom(*params.pose_d1_wrt_obs,incr);
		else	base_from_obs = incr;
		base_from_obs.composeFrom(base_from_obs,params.pose_base_wrt_d1);
	}
	else
	{
		// Changes due to the inverse pose:
		// D becomes D' = p_d^{d+1} (+) D
		ASSERT_(params.k2k_edge_id<params.k2k_edges.size())
		const pose_t & p_d_d1 = params.k2k_edges[params.k2k_edge_id]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
		->
#else
		.
#endif
		inv_pose;

		pose_t pose_base_wrt_d_prime(mrpt::poses::UNINITIALIZED_POSE);  // D' in papers
		pose_base_wrt_d_prime.composeFrom( p_d_d1, params.pose_base_wrt_d1 );

		pose_t p_d_d1_mod(mrpt::poses::UNINITIALIZED_POSE);
		p_d_d1_mod.composeFrom(incr, p_d_d1);
		p_d_d1_mod.inverse();

		// total pose: base from obs =  pose_d1_wrt_obs (+) inv(p_d_d1_mod) (+) D'
		if (params.pose_d1_wrt_obs) // "A" in papers
				base_from_obs = *params.pose_d1_wrt_obs + p_d_d1_mod + pose_base_wrt_d_prime;
		else	base_from_obs =  p_d_d1_mod + pose_base_wrt_d_prime;
	}

	// pose_robot2sensor(): pose wrt sensor = pose_wrt_robot (-) sensor_pose_on_the_robot
	typename options::internal::resulting_pose_t<typename RBA_OPTIONS::sensor_pose_on_robot_t,REL_POSE_DIMS>::pose_t base_pose_wrt_sensor(mrpt::poses::UNINITIALIZED_POSE);
	RBA_OPTIONS::sensor_pose_on_robot_t::pose_robot2sensor( base_from_obs, base_pose_wrt_sensor, params.sensor_pose );

	// Generate observation:
	array_obs_t z_zero;
	z_zero.setZero();

	sensor_model_t::observe_error(y,z_zero,base_pose_wrt_sensor,params.xji_i, params.sensor_params);
	y=-y; // because the method above evals: "z_zero - h(x)"
}

#if DEBUG_NOT_UPDATED_ENTRIES

struct TNumSTData
{
	TKeyFrameID from, to;
};

TNumSTData check_num_st_entry_exists(
	const pose_flag_t * entry,
	const TRBA_Problem_state::TSpanningTree & st)
{
	for (TRelativePosesForEachTarget::const_iterator it_tree=st.num.begin();it_tree!=st.num.end();++it_tree)
	{
		const TKeyFrameID id_from = it_tree->first;
		const frameid2pose_map_t & tree = it_tree->second;

		for (frameid2pose_map_t::const_iterator it=tree.begin();it!=tree.end();++it)
		{
			const TKeyFrameID id_to = it->first;
			const pose_flag_t & e = it->second;
			if (&e == entry)
			{
				TNumSTData d;
				d.from = id_from;
				d.to = id_to;
				return d;
			}
		}
	}
	ASSERT_(false)
}

#endif

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::numeric_dh_df(const array_landmark_t &x, const TNumeric_dh_df_params& params, array_obs_t &y)
{
	static const pose_t my_aux_null_pose;

	const array_landmark_t x_local = params.xji_i + x;
	const pose_t * pos_cam = params.pose_base_wrt_obs!=NULL ? params.pose_base_wrt_obs : &my_aux_null_pose;

	// pose_robot2sensor(): pose wrt sensor = pose_wrt_robot (-) sensor_pose_on_the_robot
	typename options::internal::resulting_pose_t<typename RBA_OPTIONS::sensor_pose_on_robot_t,REL_POSE_DIMS>::pose_t  base_pose_wrt_sensor(mrpt::poses::UNINITIALIZED_POSE);
	RBA_OPTIONS::sensor_pose_on_robot_t::pose_robot2sensor( *pos_cam, base_pose_wrt_sensor, params.sensor_pose );

	// Generate observation:
	array_obs_t z_zero;
	z_zero.setZero();
	sensor_model_t::observe_error(y,z_zero,base_pose_wrt_sensor,x_local, params.sensor_params);
	y=-y; // because the method above evals: "z_zero - h(x)"
}


/** Auxiliary sub-jacobian used in compute_jacobian_dh_dp() (it's a static method within specializations of this struct) */
template <landmark_jacob_family_t JACOB_FAMILY, size_t POINT_DIMS, size_t POSE_DIMS, class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps;

// ====================================================================
//                       j,i                    lm_id,base_id
//           \partial  h            \partial  h
//                       l                      obs_frame_id
// dh_dp = ------------------ = ---------------------------------
//                       d+1                    cur_id
//           \partial  p            \partial  p
//                       d                      stp.next_node
//
//  See tech report:
//   "A tutorial on SE(3) transformation parameterizations and
//    on-manifold optimization", Jose-Luis Blanco.
// ====================================================================
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::compute_jacobian_dh_dp(
	typename TSparseBlocksJacobians_dh_dAp::TEntry  &jacob,
	const k2f_edge_t & observation,
	const k2k_edges_deque_t  &k2k_edges,
	std::vector<const pose_flag_t*>    *out_list_of_required_num_poses) const
{
	ASSERT_(observation.obs.kf_id!=jacob.sym.kf_base)

	if (! *jacob.sym.is_valid )
		return; // Another block of the same Jacobian row said this observation was invalid for some reason.

	// And x^{j,i}_l = pose_of_i_wrt_l (+) x^{j,i}_i

	// Handle the special case when d==obs, so rel_pose_d1_from_obs==NULL, and its pose is the origin:
	const pose_flag_t * pose_d1_wrt_obs  =  jacob.sym.rel_pose_d1_from_obs; // "A" in papers
	const pose_flag_t & pose_base_wrt_d1 = *jacob.sym.rel_pose_base_from_d1;
	const bool is_inverse_edge_jacobian = !jacob.sym.edge_normal_dir;  // If edge points in the opposite direction than as assumed in mathematical derivation.

	if (out_list_of_required_num_poses)
	{
		if (jacob.sym.rel_pose_d1_from_obs)
			out_list_of_required_num_poses->push_back(jacob.sym.rel_pose_d1_from_obs);
		out_list_of_required_num_poses->push_back(jacob.sym.rel_pose_base_from_d1);
	}

	// make sure the numeric spanning tree is working and updating all that we need:
#if DEBUG_NOT_UPDATED_ENTRIES
	TNumSTData d1 = check_num_st_entry_exists(&pose_base_wrt_d1, rba_state.spanning_tree);
#endif

	if (!pose_base_wrt_d1.updated)
	{
		std::cerr << " kf_d+1: " << jacob.sym.kf_d << ", base_id: "<< jacob.sym.kf_base << std::endl;
		rba_state.spanning_tree.save_as_dot_file("_debug_jacob_error_all_STs.dot");
		ASSERT_(pose_base_wrt_d1.updated)
	}

	if (pose_d1_wrt_obs)
	{
#if DEBUG_NOT_UPDATED_ENTRIES
		TNumSTData d2 = check_num_st_entry_exists(pose_d1_wrt_obs, rba_state.spanning_tree);
#endif
		if (!pose_d1_wrt_obs->updated)
		{
			std::cerr << " kf_d+1: " << jacob.sym.kf_d << ", obs_frame_id: "<< observation.obs.kf_id << std::endl;
			rba_state.spanning_tree.save_as_dot_file("_debug_jacob_error_all_STs.dot");
		}
		ASSERT_(pose_d1_wrt_obs->updated)
	}


	// i<-l = d <- obs/l  (+)  base/i <- d
	pose_t pose_i_wrt_l(mrpt::poses::UNINITIALIZED_POSE);
	if (pose_d1_wrt_obs!=NULL)
			pose_i_wrt_l.composeFrom( pose_d1_wrt_obs->pose, pose_base_wrt_d1.pose);
	else	pose_i_wrt_l = pose_base_wrt_d1.pose;

	// First, we need x^{j,i}_i:
	// LM parameters in: jacob.sym.feat_rel_pos->pos[0:N-1]
	const array_landmark_t &xji_i = jacob.sym.feat_rel_pos->pos;

	// xji_l = pose_i_wrt_l (+) xji_i
	array_landmark_t xji_l = xji_i; //
	LM_TYPE::composePosePoint(xji_l, pose_i_wrt_l);

#if DEBUG_JACOBIANS_SUPER_VERBOSE  // Debug:
	{
		const TKeyFrameID obs_frame_id = observation.obs.kf_id;
		const TKeyFrameID base_id      = jacob.sym.kf_base;
		const TKeyFrameID d_plus_1_id  = jacob.sym.kf_d;
		cout << "compute_jacobian_dh_dp: "
			<< " obs_frame_id: " << obs_frame_id
			<< " base_id: " << base_id
			<< " kf_d+1: " << d_plus_1_id
			<< " k2k_edge_id: " << jacob.sym.k2k_edge_id
			<< " is_inverse: " << is_inverse_edge_jacobian << endl
			<< " pose_base_wrt_d1 (D): "<<pose_base_wrt_d1.pose << endl;
		if (pose_d1_wrt_obs) cout << " pose_d1_wrt_obs (A): "<< pose_d1_wrt_obs->pose<< endl;
		// Observations from the "base_id" of a fixed,known landmark are NOT considered observations for optimization:
		//mrpt::system::pause();
	}
#endif

#if SRBA_COMPUTE_NUMERIC_JACOBIANS
	// Numeric jacobians
	typename TSparseBlocksJacobians_dh_dAp::matrix_t  num_jacob;

	array_pose_t x;
	x.setZero(); // Evaluate Jacobian at manifold incr around origin
	array_pose_t x_incrs;
	x_incrs.setConstant(1e-4);

	const TNumeric_dh_dAp_params num_params(jacob.sym.k2k_edge_id,&pose_d1_wrt_obs->pose, pose_base_wrt_d1.pose,jacob.sym.feat_rel_pos->pos, is_inverse_edge_jacobian,k2k_edges,this->parameters.sensor,this->parameters.sensor_pose);

	mrpt::math::jacobians::jacob_numeric_estimate(x,&numeric_dh_dAp,x_incrs,num_params,num_jacob);

#endif // SRBA_COMPUTE_NUMERIC_JACOBIANS


#if SRBA_COMPUTE_ANALYTIC_JACOBIANS
	//  d h(x^{j,i}_l)    d h(x')       d x^{j,i}_l
	// --------------- = --------- * ----------------
	//  d p^{d+1}_d        d x'         d epsilon
	//
	//                  With: x' = x^{j,i}_l

	// First jacobian: (uses xji_l)
	// -----------------------------
	Eigen::Matrix<double,OBS_DIMS,LM_DIMS>  dh_dx;

	// Converts a point relative to the robot coordinate frame (P) into a point relative to the sensor (RES = P \ominus POSE_IN_ROBOT )
	RBA_OPTIONS::sensor_pose_on_robot_t::template point_robot2sensor<LM_TYPE,array_landmark_t>(xji_l,xji_l,this->parameters.sensor_pose );

	// Invoke sensor model:
	if (!sensor_model_t::eval_jacob_dh_dx(dh_dx,xji_l, this->parameters.sensor))
	{
		// Invalid Jacobian:
		*jacob.sym.is_valid = 0;
		jacob.num.setZero();
		return;
	}

	// take into account the possible displacement of the sensor wrt the keyframe:
	RBA_OPTIONS::sensor_pose_on_robot_t::jacob_dh_dx_rotate( dh_dx, this->parameters.sensor_pose );

	// Second Jacobian: (uses xji_i)
	// ------------------------------
	compute_jacobian_dAepsDx_deps<LM_TYPE::jacob_family,LM_DIMS,REL_POSE_DIMS,rba_engine_t>::eval(jacob.num,dh_dx,is_inverse_edge_jacobian,xji_i, pose_d1_wrt_obs, pose_base_wrt_d1,jacob.sym,k2k_edges,rba_state.all_observations);

#endif // SRBA_COMPUTE_ANALYTIC_JACOBIANS


#if SRBA_VERIFY_AGAINST_NUMERIC_JACOBIANS
	// Check jacob.num vs. num_jacob
	const double MAX_REL_ERROR = 0.1;
	if ((jacob.num-num_jacob).array().abs().maxCoeff()>MAX_REL_ERROR*num_jacob.array().maxCoeff())
	{
		std::cerr << "NUMERIC VS. ANALYTIC JACOBIAN dh_dAp FAILED:"
			<< "\njacob.num:\n" << jacob.num
			<< "\nnum_jacob:\n" << num_jacob
			<< "\nDiff:\n" << jacob.num-num_jacob << endl << endl;
	}
#endif

#if SRBA_USE_NUMERIC_JACOBIANS
	jacob.num = num_jacob;
#endif
}

// ====================================================================
//  Auxiliary sub-Jacobian used in compute_jacobian_dh_dp()
// These are specializations of the template, for each of the cases:
//template <size_t , size_t , class MATRIX, class POINT>
// ====================================================================

// Case: 3D point, SE(3) poses:
template <class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps<jacob_point_landmark /* Jacobian family: this LM is a point */, 3 /*POINT_DIMS*/,6 /*POSE_DIMS*/,RBA_ENGINE_T>
{
	template <class MATRIX, class MATRIX_DH_DX,class POINT,class pose_flag_t,class JACOB_SYM_T,class K2K_EDGES_T,class OBS_VECTOR>
	static void eval(
		MATRIX      & jacob,
		const MATRIX_DH_DX  & dh_dx,
		const bool is_inverse_edge_jacobian,
		const POINT & xji_i,
		const pose_flag_t * pose_d1_wrt_obs,  // "A" in handwritten notes
		const pose_flag_t & pose_base_wrt_d1, // "D" in handwritten notes
		const JACOB_SYM_T & jacob_sym,
		const K2K_EDGES_T & k2k_edges,
		const OBS_VECTOR  & all_obs
		)
	{
		// See section 10.3.7 of technical report on SE(3) poses [http://mapir.isa.uma.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf]
		if (!is_inverse_edge_jacobian)
		{	// Normal formulation: unknown is pose "d+1 -> d"

			// This is "D" in my handwritten notes:
			// pose_i_wrt_dplus1  -> pose_base_wrt_d1

			const mrpt::math::CMatrixDouble33 & ROTD = pose_base_wrt_d1.pose.getRotationMatrix();

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0,0,0,0):
			Eigen::Matrix<double,RBA_ENGINE_T::OBS_DIMS,3> H_ROTA;
			if (pose_d1_wrt_obs!=NULL)
			{
				// pose_d_plus_1_wrt_l  -> pose_d1_wrt_obs

				// 3x3 term: H*R(A)
				H_ROTA = dh_dx * pose_d1_wrt_obs->pose.getRotationMatrix();
			}
			else
			{
				// 3x3 term: H*R(A)
				H_ROTA = dh_dx;
			}

			// First 2x3 block:
			jacob.block(0,0,RBA_ENGINE_T::OBS_DIMS,3).noalias() = H_ROTA;

			// Second 2x3 block:
			// compute aux vector "v":
			Eigen::Matrix<double,3,1> v;
			v[0] =  -pose_base_wrt_d1.pose.x()  - xji_i[0]*ROTD.coeff(0,0) - xji_i[1]*ROTD.coeff(0,1) - xji_i[2]*ROTD.coeff(0,2);
			v[1] =  -pose_base_wrt_d1.pose.y()  - xji_i[0]*ROTD.coeff(1,0) - xji_i[1]*ROTD.coeff(1,1) - xji_i[2]*ROTD.coeff(1,2);
			v[2] =  -pose_base_wrt_d1.pose.z()  - xji_i[0]*ROTD.coeff(2,0) - xji_i[1]*ROTD.coeff(2,1) - xji_i[2]*ROTD.coeff(2,2);

			Eigen::Matrix<double,3,3> aux;

			aux.coeffRef(0,0)=0;
			aux.coeffRef(1,1)=0;
			aux.coeffRef(2,2)=0;

			aux.coeffRef(1,2)=-v[0];
			aux.coeffRef(2,1)= v[0];
			aux.coeffRef(2,0)=-v[1];
			aux.coeffRef(0,2)= v[1];
			aux.coeffRef(0,1)=-v[2];
			aux.coeffRef(1,0)= v[2];

			jacob.block(0,3,RBA_ENGINE_T::OBS_DIMS,3).noalias() = H_ROTA * aux;
		}
		else
		{	// Inverse formulation: unknown is pose "d -> d+1"

			// Changes due to the inverse pose:
			// D becomes D' = p_d^{d+1} (+) D

			ASSERT_(jacob_sym.k2k_edge_id<k2k_edges.size())
			const typename RBA_ENGINE_T::pose_t & p_d_d1 = k2k_edges[jacob_sym.k2k_edge_id]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			->
#else
			.
#endif
			inv_pose;

			typename RBA_ENGINE_T::pose_t pose_base_wrt_d1_prime(mrpt::poses::UNINITIALIZED_POSE);
			pose_base_wrt_d1_prime.composeFrom( p_d_d1 , pose_base_wrt_d1.pose );

			const mrpt::math::CMatrixDouble33 & ROTD = pose_base_wrt_d1_prime.getRotationMatrix();

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0,0,0,0):
			Eigen::Matrix<double,RBA_ENGINE_T::OBS_DIMS,3> H_ROTA;
			if (pose_d1_wrt_obs!=NULL)
			{
				// pose_d_plus_1_wrt_l  -> pose_d1_wrt_obs

				// In inverse edges, A (which is "pose_d1_wrt_obs") becomes A * (p_d_d1)^-1 =>
				//   So: ROT_A' = ROT_A * ROT_d_d1^t

				// 3x3 term: H*R(A')
				H_ROTA = dh_dx * pose_d1_wrt_obs->pose.getRotationMatrix() * p_d_d1.getRotationMatrix().transpose();
			}
			else
			{
				// Was in the normal edge: H_ROTA = dh_dx;

				// 3x3 term: H*R(A')
				H_ROTA = dh_dx * p_d_d1.getRotationMatrix().transpose();
			}

			// First 2x3 block:
			jacob.block(0,0,RBA_ENGINE_T::OBS_DIMS,3).noalias() = H_ROTA;

			// Second 2x3 block:
			// compute aux vector "v":
			Eigen::Matrix<double,3,1> v;
			v[0] =  -pose_base_wrt_d1_prime.x()  - xji_i[0]*ROTD.coeff(0,0) - xji_i[1]*ROTD.coeff(0,1) - xji_i[2]*ROTD.coeff(0,2);
			v[1] =  -pose_base_wrt_d1_prime.y()  - xji_i[0]*ROTD.coeff(1,0) - xji_i[1]*ROTD.coeff(1,1) - xji_i[2]*ROTD.coeff(1,2);
			v[2] =  -pose_base_wrt_d1_prime.z()  - xji_i[0]*ROTD.coeff(2,0) - xji_i[1]*ROTD.coeff(2,1) - xji_i[2]*ROTD.coeff(2,2);

			Eigen::Matrix<double,3,3> aux;

			aux.coeffRef(0,0)=0;
			aux.coeffRef(1,1)=0;
			aux.coeffRef(2,2)=0;

			aux.coeffRef(1,2)=-v[0];
			aux.coeffRef(2,1)= v[0];
			aux.coeffRef(2,0)=-v[1];
			aux.coeffRef(0,2)= v[1];
			aux.coeffRef(0,1)=-v[2];
			aux.coeffRef(1,0)= v[2];

			jacob.block(0,3,RBA_ENGINE_T::OBS_DIMS,3).noalias() = H_ROTA * aux;

			// And this comes from: d exp(-epsilon)/d epsilon = - d exp(epsilon)/d epsilon
			jacob = -jacob;

		} // end inverse edge case

	}
}; // end of specialization of "compute_jacobian_dAepsDx_deps"


/** Case: 2D or 3D points, SE(2) poses
  * Both cases are grouped because a SE(2) pose doesn't transform the "z" of 3D points, so both sets of Jacobians are almost identical.
  */
template <size_t POINT_DIMS, class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps_SE2
{
	template <class MATRIX, class MATRIX_DH_DX,class POINT,class pose_flag_t,class JACOB_SYM_T,class K2K_EDGES_T,class OBS_VECTOR>
	static void eval(
		MATRIX      & jacob,
		const MATRIX_DH_DX  & dh_dx,
		const bool is_inverse_edge_jacobian,
		const POINT & xji_i,
		const pose_flag_t * pose_d1_wrt_obs,  // "A" in handwritten notes
		const pose_flag_t & pose_base_wrt_d1, // "D" in handwritten notes
		const JACOB_SYM_T & jacob_sym,
		const K2K_EDGES_T & k2k_edges,
		const OBS_VECTOR  & all_obs
		)
	{
		MRPT_COMPILE_TIME_ASSERT(POINT_DIMS==2 || POINT_DIMS==3)

		if (!is_inverse_edge_jacobian)
		{	// Normal formulation: unknown is pose "d+1 -> d"

			const double Xd=pose_base_wrt_d1.pose.x();
			const double Yd=pose_base_wrt_d1.pose.y();

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			double PHIa=0; // Xa, Ya: Are not really needed, since they don't appear in the Jacobian.
			mrpt::poses::CPose2D AD(mrpt::poses::UNINITIALIZED_POSE); // AD = A(+)D
			if (pose_d1_wrt_obs!=NULL)
			{
				// pose_d_plus_1_wrt_l  -> pose_d1_wrt_obs
				//Xa = pose_d1_wrt_obs->pose.x(); Ya = pose_d1_wrt_obs->pose.y(); // Not needed
				PHIa = pose_d1_wrt_obs->pose.phi();
				AD.composeFrom(pose_d1_wrt_obs->pose,pose_base_wrt_d1.pose);
			}
			else
			{
				AD = pose_base_wrt_d1.pose;  // A=0 -> A(+)D is simply D
			}

			// d(P (+) x) / dP, P = A*D
			Eigen::Matrix<double,POINT_DIMS/* 2 or 3*/,3 /* x,y,phi*/> dPx_P;
			const double ccos_ad = cos(AD.phi());
			const double ssin_ad = sin(AD.phi());
			dPx_P(0,0) = 1;  dPx_P(0,1) = 0; dPx_P(0,2) = -xji_i[0]*ssin_ad - xji_i[1]*ccos_ad;
			dPx_P(1,0) = 0;  dPx_P(1,1) = 1; dPx_P(1,2) =  xji_i[0]*ccos_ad - xji_i[1]*ssin_ad;
			if (POINT_DIMS==3) {
				dPx_P(2,0) = 0;  dPx_P(2,1) = 0; dPx_P(2,2) =  1;
			}

			// d(A*exp(eps)*D) / deps
			Eigen::Matrix<double,3,3> dAD_deps;
			const double ccos_a = cos(PHIa);
			const double ssin_a = sin(PHIa);
			dAD_deps(0,0) = ccos_a; dAD_deps(0,1) = -ssin_a;
			dAD_deps(1,0) = ssin_a; dAD_deps(1,1) =  ccos_a;
			dAD_deps(2,0) = 0;    dAD_deps(2,1) =  0;

			dAD_deps(0,2) = -ssin_a*Xd - ccos_a*Yd;
			dAD_deps(1,2) =  ccos_a*Xd - ssin_a*Yd;
			dAD_deps(2,2) = 1;

			// Chain rule:
			jacob.noalias() = dh_dx * dPx_P * dAD_deps;
		}
		else
		{	// Inverse formulation: unknown is pose "d -> d+1"

			// Changes due to the inverse pose:
			// D becomes D' = p_d^{d+1} (+) D
			// and A (which is "pose_d1_wrt_obs") becomes A' = A (+) (p_d_d1)^-1

			ASSERT_(jacob_sym.k2k_edge_id<k2k_edges.size())
			const typename RBA_ENGINE_T::pose_t & p_d_d1 = k2k_edges[jacob_sym.k2k_edge_id]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			->
#else
			.
#endif
			inv_pose;

			typename RBA_ENGINE_T::pose_t pose_base_wrt_d1_prime(mrpt::poses::UNINITIALIZED_POSE);
			pose_base_wrt_d1_prime.composeFrom( p_d_d1 , pose_base_wrt_d1.pose );

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			const typename RBA_ENGINE_T::pose_t p_d_d1_inv = -p_d_d1;

			typename RBA_ENGINE_T::pose_t A_prime = (pose_d1_wrt_obs!=NULL) ?
				(pose_d1_wrt_obs->pose + p_d_d1_inv)
				:
				p_d_d1_inv;

			mrpt::poses::CPose2D AD(mrpt::poses::UNINITIALIZED_POSE); // AD = A(+)D
			AD.composeFrom(A_prime,pose_base_wrt_d1_prime);

			const double Xd=pose_base_wrt_d1_prime.x();
			const double Yd=pose_base_wrt_d1_prime.y();

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			const double PHIa=A_prime.phi();


			// d(P (+) x) / dP, P = A*D
			Eigen::Matrix<double,POINT_DIMS/* 2 or 3*/,3 /* x,y,phi*/> dPx_P;
			const double ccos_ad = cos(AD.phi());
			const double ssin_ad = sin(AD.phi());
			dPx_P(0,0) = 1;  dPx_P(0,1) = 0; dPx_P(0,2) = -xji_i[0]*ssin_ad - xji_i[1]*ccos_ad;
			dPx_P(1,0) = 0;  dPx_P(1,1) = 1; dPx_P(1,2) =  xji_i[0]*ccos_ad - xji_i[1]*ssin_ad;
			if (POINT_DIMS==3) {
				dPx_P(2,0) = 0;  dPx_P(2,1) = 0; dPx_P(2,2) =  1;
			}

			// d(A*exp(eps)*D) / deps
			Eigen::Matrix<double,3,3> dAD_deps;
			const double ccos_a = cos(PHIa);
			const double ssin_a = sin(PHIa);
			dAD_deps(0,0) = ccos_a; dAD_deps(0,1) = -ssin_a;
			dAD_deps(1,0) = ssin_a; dAD_deps(1,1) =  ccos_a;
			dAD_deps(2,0) = 0;    dAD_deps(2,1) =  0;

			dAD_deps(0,2) = -ssin_a*Xd - ccos_a*Yd;
			dAD_deps(1,2) =  ccos_a*Xd - ssin_a*Yd;
			dAD_deps(2,2) = 1;

			// Chain rule:
			jacob.noalias() = dh_dx * dPx_P * dAD_deps;

			// And this comes from: d exp(-epsilon)/d epsilon = - d exp(epsilon)/d epsilon
			jacob = -jacob;

		} // end inverse edge case

	}
}; // end of "compute_jacobian_dAepsDx_deps_SE2"

// Case: 2D point, SE(2) poses: (derived from generic SE2 implementation above)
template <class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps<jacob_point_landmark /* Jacobian family: this LM is a point */, 2 /*POINT_DIMS*/,3 /*POSE_DIMS*/,RBA_ENGINE_T>
	: public compute_jacobian_dAepsDx_deps_SE2<2 /*POINT_DIMS*/,RBA_ENGINE_T>
{
};

// Case: 3D point, SE(2) poses: (derived from generic SE2 implementation above)
template <class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps<jacob_point_landmark /* Jacobian family: this LM is a point */, 3 /*POINT_DIMS*/,3 /*POSE_DIMS*/,RBA_ENGINE_T>
	: public compute_jacobian_dAepsDx_deps_SE2<3 /*POINT_DIMS*/,RBA_ENGINE_T>
{
};

// Case: SE(2) relative-poses, SE(2) poses:
template <class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps<jacob_relpose_landmark /* Jacobian family: this LM is a relative pose (graph-slam) */, 3 /*POINT_DIMS*/,3 /*POSE_DIMS*/,RBA_ENGINE_T>
{
	template <class MATRIX, class MATRIX_DH_DX,class POINT,class pose_flag_t,class JACOB_SYM_T,class K2K_EDGES_T,class OBS_VECTOR>
	static void eval(
		MATRIX      & jacob,
		const MATRIX_DH_DX  & dh_dx,
		const bool is_inverse_edge_jacobian,
		const POINT & xji_i,
		const pose_flag_t * pose_d1_wrt_obs,  // "A" in handwritten notes
		const pose_flag_t & pose_base_wrt_d1, // "D" in handwritten notes
		const JACOB_SYM_T & jacob_sym,
		const K2K_EDGES_T & k2k_edges,
		const OBS_VECTOR  & all_obs
		)
	{
		double Xd,Yd,PHIa;
		mrpt::poses::CPose2D  base_wrt_obs(mrpt::poses::UNINITIALIZED_POSE); // A(+)D

		if (!is_inverse_edge_jacobian)
		{	// Normal formulation: unknown is pose "d+1 -> d"

			Xd=pose_base_wrt_d1.pose.x();
			Yd=pose_base_wrt_d1.pose.y();

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			PHIa=0; // Xa, Ya: Are not really needed, since they don't appear in the Jacobian.
			if (pose_d1_wrt_obs!=NULL)
			{
				// pose_d_plus_1_wrt_l  -> pose_d1_wrt_obs
				PHIa = pose_d1_wrt_obs->pose.phi();
				base_wrt_obs.composeFrom(pose_d1_wrt_obs->pose, pose_base_wrt_d1.pose);  // A (+) D
			}
			else
			{
				base_wrt_obs = pose_base_wrt_d1.pose;  // A (+) D
			}
		}
		else
		{	// Inverse formulation: unknown is pose "d -> d+1"

			// Changes due to the inverse pose:
			// D becomes D' = p_d^{d+1} (+) D
			// and A (which is "pose_d1_wrt_obs") becomes A' = A (+) (p_d_d1)^-1

			ASSERT_(jacob_sym.k2k_edge_id<k2k_edges.size())
			const typename RBA_ENGINE_T::pose_t & p_d_d1 = k2k_edges[jacob_sym.k2k_edge_id]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			->
#else
			.
#endif
			inv_pose;

			typename RBA_ENGINE_T::pose_t pose_base_wrt_d1_prime(mrpt::poses::UNINITIALIZED_POSE);
			pose_base_wrt_d1_prime.composeFrom( p_d_d1 , pose_base_wrt_d1.pose );

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			const typename RBA_ENGINE_T::pose_t p_d_d1_inv = -p_d_d1;

			typename RBA_ENGINE_T::pose_t A_prime = (pose_d1_wrt_obs!=NULL) ?
				(pose_d1_wrt_obs->pose + p_d_d1_inv)
				:
				p_d_d1_inv;

			Xd=pose_base_wrt_d1_prime.x();
			Yd=pose_base_wrt_d1_prime.y();

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			PHIa=A_prime.phi();

			base_wrt_obs.composeFrom(A_prime, pose_base_wrt_d1_prime);  // A (+) D
		}

		//const mrpt::poses::CPose2D base_wrt_obs_inv = -base_wrt_obs;
		const double PHIad = base_wrt_obs.phi();

		const double ccos_ad = cos(PHIad), ssin_ad=sin(PHIad);
		const double ccos_a = cos(PHIa), ssin_a=sin(PHIa);

		Eigen::Matrix<double,3,3> J0; // -d(\ominus p)_dp, with p=A*D
		J0(0,0)= ccos_ad; J0(0,1)= ssin_ad; J0(0,2)= 0;
		J0(1,0)=-ssin_ad; J0(1,1)= ccos_ad; J0(1,2)= 0;
		J0(2,0)=0;        J0(2,1)=0;        J0(2,2)= 1;

		Eigen::Matrix<double,3,3> J1; // dAD_dA;
		J1(0,0)=1; J1(0,1)=0; J1(0,2)= -Xd*ssin_a-Yd*ccos_a;
		J1(1,0)=0; J1(1,1)=1; J1(1,2)=  Xd*ccos_a-Yd*ssin_a;
		J1(2,0)=0; J1(2,1)=0; J1(2,2)= 1;

		Eigen::Matrix<double,3,3> J2; // dAe_de
		J2(0,0)=ccos_a;  J2(0,1)=-ssin_a;  J2(0,2)=0;
		J2(1,0)=ssin_a;  J2(1,1)= ccos_a;  J2(1,2)=0;
		J2(2,0)=0;       J2(2,1)=0;        J2(2,2)=1;

		// Chain rule:
		jacob.noalias() = dh_dx * J0* J1 * J2;

		if (is_inverse_edge_jacobian)
		{
			// And this comes from: d exp(-epsilon)/d epsilon = - d exp(epsilon)/d epsilon
			jacob = -jacob;
		}
	}
}; // end of "compute_jacobian_dAepsDx_deps", Case: SE(2) relative-poses, SE(2) poses:

// Case: SE(3) relative-poses, SE(3) poses:
template <class RBA_ENGINE_T>
struct compute_jacobian_dAepsDx_deps<jacob_relpose_landmark /* Jacobian family: this LM is a relative pose (graph-slam) */, 6 /*POINT_DIMS*/,6 /*POSE_DIMS*/,RBA_ENGINE_T>
{
	template <class MATRIX, class MATRIX_DH_DX,class POINT,class pose_flag_t,class JACOB_SYM_T,class K2K_EDGES_T,class OBS_VECTOR>
	static void eval(
		MATRIX      & jacob,
		const MATRIX_DH_DX  & dh_dx,
		const bool is_inverse_edge_jacobian,
		const POINT & xji_i,
		const pose_flag_t * pose_d1_wrt_obs,  // "A" in handwritten notes
		const pose_flag_t & pose_base_wrt_d1, // "D" in handwritten notes
		const JACOB_SYM_T & jacob_sym,
		const K2K_EDGES_T & k2k_edges,
		const OBS_VECTOR  & all_obs
		)
	{
		//
		//  d ps-log(p^obs_base)      d ps-log(p)       d A*e^eps*D
		// --------------------  =  --------------- * -----------------
		//    d eps^d+1_d                d p             d eps
		//                   6x6                 6x12                12x6
		//
		//                              ^: (1)              ^: (2)
		// See section 10.3.7 of technical report on SE(3) poses [http://mapir.isa.uma.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf]
		mrpt::math::CMatrixDouble33 ROTA;  // A.rotationMatrix
		typename RBA_ENGINE_T::pose_t  D(mrpt::poses::UNINITIALIZED_POSE);
		typename RBA_ENGINE_T::pose_t  base_wrt_obs(mrpt::poses::UNINITIALIZED_POSE); // A(+)D

		if (!is_inverse_edge_jacobian)
		{	// Normal formulation: unknown is pose "d+1 -> d"
			D = pose_base_wrt_d1.pose;

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			if (pose_d1_wrt_obs!=NULL)
			{
				// pose_d_plus_1_wrt_l  -> pose_d1_wrt_obs
				base_wrt_obs.composeFrom(pose_d1_wrt_obs->pose, D);  // A (+) D
				ROTA = pose_d1_wrt_obs->pose.getRotationMatrix();
			}
			else
			{
				base_wrt_obs = D;  // A (+) D
				ROTA.setIdentity();
			}
		}
		else
		{	// Inverse formulation: unknown is pose "d -> d+1"

			// Changes due to the inverse pose:
			// D becomes D' = p_d^{d+1} (+) D
			// and A (which is "pose_d1_wrt_obs") becomes A' = A (+) (p_d_d1)^-1

			ASSERT_(jacob_sym.k2k_edge_id<k2k_edges.size())
			const typename RBA_ENGINE_T::pose_t & p_d_d1 = k2k_edges[jacob_sym.k2k_edge_id]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			->
#else
			.
#endif
			inv_pose;

			typename RBA_ENGINE_T::pose_t pose_base_wrt_d1_prime(mrpt::poses::UNINITIALIZED_POSE);
			pose_base_wrt_d1_prime.composeFrom( p_d_d1 , pose_base_wrt_d1.pose );

			// We need to handle the special case where "d+1"=="l", so A=Pose(0,0,0):
			const typename RBA_ENGINE_T::pose_t p_d_d1_inv = -p_d_d1;

			typename RBA_ENGINE_T::pose_t A_prime = (pose_d1_wrt_obs!=NULL) ?
				(pose_d1_wrt_obs->pose + p_d_d1_inv)
				:
				p_d_d1_inv;

			ROTA=A_prime.getRotationMatrix();

			D=pose_base_wrt_d1_prime;

			base_wrt_obs.composeFrom(A_prime, pose_base_wrt_d1_prime);  // A (+) D
		}

		const mrpt::math::CMatrixDouble44 & HM_D = D.getHomogeneousMatrixVal(); // [ROT(D) | Trans(D) ]


		// (1): Common part: d_Ln(R)_dR:
		//
		const mrpt::math::CMatrixDouble33 & ROT_AplusB = base_wrt_obs.getRotationMatrix(); // The rotation matrix.
		Eigen::Matrix<double,6,12>  dLnRelPose_deps;
		dLnRelPose_deps.setZero();
		dLnRelPose_deps.block<3,3>(0,9).setIdentity();
		{
			mrpt::math::CMatrixFixedNumeric<double,3,9> dLnRot_dRot(mrpt::math::UNINITIALIZED_MATRIX);
			mrpt::poses::CPose3D::ln_rot_jacob( ROT_AplusB, dLnRot_dRot);
			dLnRelPose_deps.block<3,9>(3,0) = dLnRot_dRot;
		}

		// (2): d A*e^eps*D / d eps =
		//
		//  [ 0_{3x3}  -R(A)*[dc1]_x ]
		//  [ 0_{3x3}  -R(A)*[dc2]_x ]
		//  [ 0_{3x3}  -R(A)*[dc3]_x ]
		//  [ R(A)     -R(A)*[dt]_x  ]
		//
		Eigen::Matrix<double,12,6>  dAeD_de;
		// Blocks: (1-3,1)
		dAeD_de.block<9,3>(0,0).setZero();
		// Block (4,1)
		dAeD_de.block<3,3>(9,0) = (ROTA * HM_D.block<3,3>(0,0)).transpose();

		//cout << "========= ROTA ========= :\n" << ROTA << endl;
		//cout << "========= HM_D ========= :\n" << HM_D << endl;

		// Blocks (1-4,2)
		for (int i=0;i<4;i++)
		{
			EIGEN_ALIGN16 const double aux_vals[] = {
				        .0, -HM_D(2,i),  HM_D(1,i),
				 HM_D(2,i),         .0, -HM_D(0,i),
				-HM_D(1,i),  HM_D(0,i),         .0  };
			dAeD_de.block<3,3>(i*3,3) = -ROTA * mrpt::math::CMatrixDouble33(aux_vals);
		}

		// (3): Apply chain rule:
		//
		jacob.noalias() = dLnRelPose_deps * dAeD_de;

		if (is_inverse_edge_jacobian)
		{
			// And this comes from: d exp(-epsilon)/d epsilon = - d exp(epsilon)/d epsilon
			jacob = -jacob;
		}
	}
}; // end of "compute_jacobian_dAepsDx_deps", Case: SE(3) relative-poses, SE(3) poses:



// ====================================================================
//                       j,i                    lm_id,base_id
//           \partial  h            \partial  h
//                       l                      obs_frame_id
// dh_df = ------------------ = ---------------------------------
//
//           \partial  f            \partial  f
//
//   Note: f=relative position of landmark with respect to its base kf
// ====================================================================
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::compute_jacobian_dh_df(
	typename TSparseBlocksJacobians_dh_df::TEntry  &jacob,
	const k2f_edge_t & observation,
	std::vector<const pose_flag_t*> *out_list_of_required_num_poses) const
{
	if (! *jacob.sym.is_valid )
		return; // Another block of the same Jacobian row said this observation was invalid for some reason.

	// Handle the special case when obs==base, for which rel_pose_base_from_obs==NULL
	const pose_flag_t * rel_pose_base_from_obs = jacob.sym.rel_pose_base_from_obs;

	if (out_list_of_required_num_poses && rel_pose_base_from_obs)
		out_list_of_required_num_poses->push_back(jacob.sym.rel_pose_base_from_obs);

	// make sure the numeric spanning tree is working and updating all that we need:
	if (rel_pose_base_from_obs)
	{
#if DEBUG_NOT_UPDATED_ENTRIES
		TNumSTData d1 = check_num_st_entry_exists(rel_pose_base_from_obs, rba_state.spanning_tree);
#endif
		if (!rel_pose_base_from_obs->updated)
		{
#if DEBUG_NOT_UPDATED_ENTRIES
			cout << "not updated ST entry for: from=" << d1.from << ", to=" << d1.to << endl;
#endif
			rba_state.spanning_tree.save_as_dot_file("_debug_jacob_error_all_STs.dot");
			ASSERT_(rel_pose_base_from_obs->updated)
		}
	}

	// First, we need x^{j,i}_i:
	//const TPoint3D & xji_i = jacob.sym.feat_rel_pos->pos;
	const array_landmark_t &xji_i = jacob.sym.feat_rel_pos->pos;

	array_landmark_t xji_l = xji_i; //

	if (rel_pose_base_from_obs!=NULL)
	{
#if 0
		cout << "dh_df(ft_id="<< observation.obs.obs.feat_id << ", obs_kf="<< observation.obs.kf_id << "): o2b=" << *rel_pose_base_from_obs << endl;
#endif
		// xji_l = rel_pose_base_from_obs (+) xji_i
		//rel_pose_base_from_obs->pose.composePoint(xji_i, xji_l);
		LM_TYPE::composePosePoint(xji_l, rel_pose_base_from_obs->pose);
	}
	else
	{
		// I'm observing from the same base key-frame: xji_l = xji_i (already done above)
	}


#if SRBA_COMPUTE_NUMERIC_JACOBIANS
	// Numeric jacobians
	typename TSparseBlocksJacobians_dh_df::matrix_t  num_jacob;

	array_landmark_t x;
	x.setZero(); // Evaluate Jacobian at incr around origin
	array_landmark_t x_incrs;
	x_incrs.setConstant(1e-3);

	const TNumeric_dh_df_params num_params(&rel_pose_base_from_obs->pose,jacob.sym.feat_rel_pos->pos,this->parameters.sensor,this->parameters.sensor_pose);

	mrpt::math::jacobians::jacob_numeric_estimate(x,&numeric_dh_df,x_incrs,num_params,num_jacob);

#endif // SRBA_COMPUTE_NUMERIC_JACOBIANS


#if SRBA_COMPUTE_ANALYTIC_JACOBIANS
	//  d h(x^{j,i}_l)    d h(x')       d x'
	// --------------- = --------- * ----------
	//  d f                d x'         d x
	//
	//                  With: x' = x^{j,i}_l

	// First jacobian: (uses xji_l)
	// -----------------------------
	Eigen::Matrix<double,OBS_DIMS,LM_DIMS>  dh_dx;

	// Converts a point relative to the robot coordinate frame (P) into a point relative to the sensor (RES = P \ominus POSE_IN_ROBOT )
	RBA_OPTIONS::sensor_pose_on_robot_t::template point_robot2sensor<LM_TYPE,array_landmark_t>(xji_l,xji_l,this->parameters.sensor_pose );

	// Invoke sensor model:
	if (!sensor_model_t::eval_jacob_dh_dx(dh_dx,xji_l, this->parameters.sensor))
	{
		// Invalid Jacobian:
		*jacob.sym.is_valid = 0;
		jacob.num.setZero();
		return;
	}

	// take into account the possible displacement of the sensor wrt the keyframe:
	RBA_OPTIONS::sensor_pose_on_robot_t::jacob_dh_dx_rotate( dh_dx, this->parameters.sensor_pose );

	// Second Jacobian: Simply the 2x2 or 3x3 rotation matrix of base wrt observing
	// ------------------------------
	if (rel_pose_base_from_obs!=NULL)
	{
		mrpt::math::CMatrixFixedNumeric<double,LM_DIMS,LM_DIMS> R(mrpt::math::UNINITIALIZED_MATRIX);
		rel_pose_base_from_obs->pose.getRotationMatrix(R);
		jacob.num.noalias() = dh_dx * R;
	}
	else
	{
		// if observing from the same base kf, we're done:
		jacob.num.noalias() = dh_dx;
	}
#endif // SRBA_COMPUTE_ANALYTIC_JACOBIANS


#if SRBA_VERIFY_AGAINST_NUMERIC_JACOBIANS
	// Check jacob.num vs. num_jacob
	const double MAX_REL_ERROR = 0.1;
	if ((jacob.num-num_jacob).array().abs().maxCoeff()>MAX_REL_ERROR*num_jacob.array().maxCoeff())
	{
		std::cerr << "NUMERIC VS. ANALYTIC JACOBIAN dh_df FAILED:"
			<< "\njacob.num:\n" << jacob.num
			<< "\nnum_jacob:\n" << num_jacob
			<< "\nDiff:\n" << jacob.num-num_jacob << endl << endl;
	}
#endif

#if SRBA_USE_NUMERIC_JACOBIANS
	jacob.num = num_jacob;
#endif
}


// ------------------------------------------------------------------------
//   prepare_Jacobians_required_tree_roots()
// ------------------------------------------------------------------------
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::prepare_Jacobians_required_tree_roots(
	std::set<TKeyFrameID>  & lst,
	const std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*> &lst_JacobCols_dAp,
	const std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  &lst_JacobCols_df )
{
	lst.clear();

	const size_t nUnknowns_k2k = lst_JacobCols_dAp.size();
	const size_t nUnknowns_k2f = lst_JacobCols_df.size();

	// k2k edges ------------------------------------------------------
	for (size_t i=0;i<nUnknowns_k2k;i++)
	{
		// For each column, process each nonzero block:
		const typename TSparseBlocksJacobians_dh_dAp::col_t *col = lst_JacobCols_dAp[i];

		for (typename TSparseBlocksJacobians_dh_dAp::col_t::const_iterator it=col->begin();it!=col->end();++it)
		{
			// For each dh_dAp block we need:
			//  *  (d+1) -> obs
			//  *  (d+1) -> base

			const size_t obs_idx = it->first;
			const typename TSparseBlocksJacobians_dh_dAp::TEntry & jacob_entry = it->second;

#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			const TKeyFrameID obs_id  = rba_state.all_observations[obs_idx]->obs.kf_id;
#else
			const TKeyFrameID obs_id  = rba_state.all_observations[obs_idx].obs.kf_id;
#endif
			const TKeyFrameID d1_id   = jacob_entry.sym.kf_d;
			const TKeyFrameID base_id = jacob_entry.sym.kf_base;

			add_edge_ij_to_list_needed_roots(lst, d1_id  , obs_id );
			add_edge_ij_to_list_needed_roots(lst, base_id, d1_id );
		}
	}

	// k2f edges ------------------------------------------------------
	for (size_t i=0;i<nUnknowns_k2f;i++)
	{
		// For each column, process each nonzero block:
		const typename TSparseBlocksJacobians_dh_df::col_t *col = lst_JacobCols_df[i];

		for (typename TSparseBlocksJacobians_dh_df::col_t::const_iterator it=col->begin();it!=col->end();++it)
		{
			// For each dh_df block we need:
			//  *  obs -> base
			const size_t obs_idx = it->first;
			const k2f_edge_t &k2f =
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			*
#endif
			rba_state.all_observations[obs_idx];

			ASSERT_(k2f.feat_rel_pos)

			const TKeyFrameID obs_id  = k2f.obs.kf_id;
			const TKeyFrameID base_id = k2f.feat_rel_pos->id_frame_base;

			add_edge_ij_to_list_needed_roots(lst, base_id, obs_id );
		}
	}
}


// ------------------------------------------------------------------------
//   recompute_all_Jacobians(): Re-evaluate all Jacobians numerically
// ------------------------------------------------------------------------
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
size_t RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::recompute_all_Jacobians(
	std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*> &lst_JacobCols_dAp,
	std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  &lst_JacobCols_df,
	std::vector<const typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::pose_flag_t*>    * out_list_of_required_num_poses )
{
	size_t nJacobs=0;
	if (out_list_of_required_num_poses) out_list_of_required_num_poses->clear();

	const size_t nUnknowns_k2k = lst_JacobCols_dAp.size();

	// k2k edges ------------------------------------------------------
	for (size_t i=0;i<nUnknowns_k2k;i++)
	{
		// For each column, process each nonzero block:
		typename TSparseBlocksJacobians_dh_dAp::col_t *col = lst_JacobCols_dAp[i];

		for (typename TSparseBlocksJacobians_dh_dAp::col_t::iterator it=col->begin();it!=col->end();++it)
		{
			const size_t obs_idx = it->first;
			typename TSparseBlocksJacobians_dh_dAp::TEntry & jacob_entry = it->second;
			compute_jacobian_dh_dp(
				jacob_entry,
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				*rba_state.all_observations[obs_idx],
#else
				rba_state.all_observations[obs_idx],
#endif
				rba_state.k2k_edges,
				out_list_of_required_num_poses );
			nJacobs++;
		}
	}

	// k2f edges ------------------------------------------------------
	// Only if we are in landmarks-based SLAM, not in graph-SLAM:
	nJacobs += internal::recompute_all_Jacobians_dh_df<LM_TYPE::jacob_family>::eval(*this, lst_JacobCols_df,out_list_of_required_num_poses);

	return nJacobs;
} // end of recompute_all_Jacobians()


} } // end of namespace
