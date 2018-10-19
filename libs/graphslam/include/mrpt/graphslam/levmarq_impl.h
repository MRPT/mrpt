/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{
namespace graphslam
{
/// Internal auxiliary classes
namespace detail
{
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::graphslam;
using namespace mrpt::math;
using namespace std;

// Auxiliary struct to update the oplus increments after each iteration
// Specializations are below.
template <class POSE, class gst>
struct AuxPoseOPlus;

// Nodes: CPose2D
template <class gst>
struct AuxPoseOPlus<CPose2D, gst>
{
	static inline void sumIncr(CPose2D& p, const typename gst::Array_O& delta)
	{
		p.x_incr(delta[0]);
		p.y_incr(delta[1]);
		p.phi_incr(delta[2]);
		p.normalizePhi();
	}
};

// Nodes: CPosePDFGaussianInf
template <class gst>
struct AuxPoseOPlus<CPosePDFGaussianInf, gst>
{
	template <class POSE>
	static inline void sumIncr(POSE& p, const typename gst::Array_O& delta)
	{
		p.x_incr(delta[0]);
		p.y_incr(delta[1]);
		p.phi_incr(delta[2]);
		p.normalizePhi();
	}
};

// Nodes: CPose2D
template <class gst>
struct AuxPoseOPlus<CPose3D, gst>
{
	static inline void sumIncr(CPose3D& p, const typename gst::Array_O& delta)
	{
		// exp_delta_i = Exp_SE( delta_i )
		typename gst::graph_t::constraint_t::type_value exp_delta_pose(
			UNINITIALIZED_POSE);
		gst::SE_TYPE::exp(delta, exp_delta_pose);
		p = p + exp_delta_pose;
	}
};

// Nodes: CPosePDFGaussianInf
template <class gst>
struct AuxPoseOPlus<CPose3DPDFGaussianInf, gst>
{
	template <class POSE>
	static inline void sumIncr(POSE& p, const typename gst::Array_O& delta)
	{
		// exp_delta_i = Exp_SE( delta_i )
		typename gst::graph_t::constraint_t::type_value exp_delta_pose(
			UNINITIALIZED_POSE);
		gst::SE_TYPE::exp(delta, exp_delta_pose);
		p = p + exp_delta_pose;
	}
};

// An auxiliary struct to compute the pseudo-ln of a pose error, possibly
// modified with an information matrix.
//  Specializations are below.
template <class EDGE, class gst>
struct AuxErrorEval;

// For graphs of 2D constraints (no information matrix)
template <class gst>
struct AuxErrorEval<CPose2D, gst>
{
	template <class POSE, class VEC, class EDGE_ITERATOR>
	static inline void computePseudoLnError(
		const POSE& DinvP1invP2, VEC& err, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		gst::SE_TYPE::pseudo_ln(DinvP1invP2, err);
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJtLambdaJ(
		const MAT& J1, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		JtJ = J1.transpose() * J1;
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJ1tLambdaJ2(
		const MAT& J1, const MAT& J2, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		JtJ = J1.transpose() * J2;
	}

	template <class JAC, class EDGE_ITERATOR, class VEC1, class VEC2>
	static inline void multiply_Jt_W_err(
		const JAC& J, const EDGE_ITERATOR& edge, const VEC1& ERR, VEC2& OUT)
	{
		MRPT_UNUSED_PARAM(edge);
		const auto grad_incr = (J.transpose() * ERR).eval();
		OUT += grad_incr;
	}
};

// For graphs of 3D constraints (no information matrix)
template <class gst>
struct AuxErrorEval<CPose3D, gst>
{
	template <class POSE, class VEC, class EDGE_ITERATOR>
	static inline void computePseudoLnError(
		const POSE& DinvP1invP2, VEC& err, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		gst::SE_TYPE::pseudo_ln(DinvP1invP2, err);
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJtLambdaJ(
		const MAT& J1, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		JtJ = J1.transpose() * J1;
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJ1tLambdaJ2(
		const MAT& J1, const MAT& J2, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		JtJ = J1.transpose() * J2;
	}

	template <class JAC, class EDGE_ITERATOR, class VEC1, class VEC2>
	static inline void multiply_Jt_W_err(
		const JAC& J, const EDGE_ITERATOR& edge, const VEC1& ERR, VEC2& OUT)
	{
		MRPT_UNUSED_PARAM(edge);
		OUT += J.transpose() * ERR;
	}
};

// For graphs of 2D constraints (with information matrix)
template <class gst>
struct AuxErrorEval<CPosePDFGaussianInf, gst>
{
	template <class POSE, class VEC, class EDGE_ITERATOR>
	static inline void computePseudoLnError(
		const POSE& DinvP1invP2, VEC& err, const EDGE_ITERATOR& edge)
	{
		gst::SE_TYPE::pseudo_ln(DinvP1invP2, err);
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJtLambdaJ(
		const MAT& J1, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		JtJ = (J1.transpose() * edge->second.cov_inv) * J1;
	}
	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJ1tLambdaJ2(
		const MAT& J1, const MAT& J2, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		JtJ = (J1.transpose() * edge->second.cov_inv) * J2;
	}

	template <class JAC, class EDGE_ITERATOR, class VEC1, class VEC2>
	static inline void multiply_Jt_W_err(
		const JAC& J, const EDGE_ITERATOR& edge, const VEC1& ERR, VEC2& OUT)
	{
		OUT += (J.transpose() * edge->second.cov_inv) * ERR;
	}
};

// For graphs of 3D constraints (with information matrix)
template <class gst>
struct AuxErrorEval<CPose3DPDFGaussianInf, gst>
{
	template <class POSE, class VEC, class EDGE_ITERATOR>
	static inline void computePseudoLnError(
		const POSE& DinvP1invP2, VEC& err, const EDGE_ITERATOR& edge)
	{
		MRPT_UNUSED_PARAM(edge);
		gst::SE_TYPE::pseudo_ln(DinvP1invP2, err);
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJtLambdaJ(
		const MAT& J1, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		JtJ = (J1.transpose() * edge->second.cov_inv) * J1;
	}

	template <class MAT, class EDGE_ITERATOR>
	static inline void multiplyJ1tLambdaJ2(
		const MAT& J1, const MAT& J2, MAT& JtJ, const EDGE_ITERATOR& edge)
	{
		JtJ = (J1.transpose() * edge->second.cov_inv) * J2;
	}

	template <class JAC, class EDGE_ITERATOR, class VEC1, class VEC2>
	static inline void multiply_Jt_W_err(
		const JAC& J, const EDGE_ITERATOR& edge, const VEC1& ERR, VEC2& OUT)
	{
		OUT += (J.transpose() * edge->second.cov_inv) * ERR;
	}
};

}  // namespace detail

// Compute, at once, jacobians and the error vectors for each constraint in
// "lstObservationData", returns the overall squared error.
template <class GRAPH_T>
double computeJacobiansAndErrors(
	const GRAPH_T& graph,
	const std::vector<typename graphslam_traits<GRAPH_T>::observation_info_t>&
		lstObservationData,
	typename graphslam_traits<GRAPH_T>::map_pairIDs_pairJacobs_t& lstJacobians,
	mrpt::aligned_std_vector<typename graphslam_traits<GRAPH_T>::Array_O>& errs)
{
	MRPT_UNUSED_PARAM(graph);
	using gst = graphslam_traits<GRAPH_T>;

	lstJacobians.clear();
	errs.clear();

	const size_t nObservations = lstObservationData.size();

	for (size_t i = 0; i < nObservations; i++)
	{
		const typename gst::observation_info_t& obs = lstObservationData[i];
		auto e = obs.edge;
		const typename gst::graph_t::constraint_t::type_value* EDGE_POSE =
			obs.edge_mean;
		typename gst::graph_t::constraint_t::type_value* P1 = obs.P1;
		typename gst::graph_t::constraint_t::type_value* P2 = obs.P2;

		const auto& ids = e->first;
		const auto& edge = e->second;

		// Compute the residual pose error of these pair of nodes + its
		// constraint:
		// DinvP1invP2 = inv(EDGE) * inv(P1) * P2 = (P2 \ominus P1) \ominus EDGE
		typename gst::graph_t::constraint_t::type_value DinvP1invP2 =
			((*P2) - (*P1)) - *EDGE_POSE;

		// Add to vector of errors:
		errs.resize(errs.size() + 1);
		detail::AuxErrorEval<typename gst::edge_t, gst>::computePseudoLnError(
			DinvP1invP2, errs.back(), edge);

		// Compute the jacobians:
		alignas(MRPT_MAX_ALIGN_BYTES)
			std::pair<mrpt::graphs::TPairNodeIDs, typename gst::TPairJacobs>
				newMapEntry;
		newMapEntry.first = ids;
		gst::SE_TYPE::jacobian_dDinvP1invP2_depsilon(
			-(*EDGE_POSE), *P1, *P2, &newMapEntry.second.first,
			&newMapEntry.second.second);

		// And insert into map of jacobians:
		lstJacobians.insert(lstJacobians.end(), newMapEntry);
	}

	// return overall square error:  (Was:
	// std::accumulate(...,mrpt::squareNorm_accum<>), but led to GCC
	// errors when enabling parallelization)
	double ret_err = 0.0;
	for (size_t i = 0; i < errs.size(); i++) ret_err += errs[i].squaredNorm();
	return ret_err;
}

}  // namespace graphslam
}  // namespace mrpt
