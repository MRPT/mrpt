/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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

// An auxiliary struct to compute the pseudo-ln of a pose error, possibly
// modified with an information matrix.
//  Specializations are below.
template <class EDGE, class gst>
struct AuxErrorEval;

// For graphs of 2D constraints (no information matrix)
template <class gst>
struct AuxErrorEval<CPose2D, gst>
{
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
		// const auto& edge = e->second;

		// Compute the residual pose error of these pair of nodes + its
		// constraint:
		// DinvP1invP2 = inv(EDGE) * inv(P1) * P2 = (P2 \ominus P1) \ominus EDGE
		typename gst::graph_t::constraint_t::type_value DinvP1invP2 =
			((*P2) - (*P1)) - *EDGE_POSE;

		// Add to vector of errors:
		errs.resize(errs.size() + 1);
		errs.back() = gst::SE_TYPE::log(DinvP1invP2);

		// Compute the jacobians:
		alignas(MRPT_MAX_ALIGN_BYTES)
			std::pair<mrpt::graphs::TPairNodeIDs, typename gst::TPairJacobs>
				newMapEntry;
		newMapEntry.first = ids;
		gst::SE_TYPE::jacob_dDinvP1invP2_de1e2(
			-(*EDGE_POSE), *P1, *P2, newMapEntry.second.first,
			newMapEntry.second.second);

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
